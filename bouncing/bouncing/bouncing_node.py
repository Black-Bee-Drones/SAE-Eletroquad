import rclpy
import time
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from typing import Tuple, List
from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing.camera.image_calculus import ImageCalculus
import numpy as np
from ultralytics import YOLO


class BouncingNode(Node):
    """
    ROS 2 node that manages a drone's search and detection routine within a predefined GPS area.
    It uses YOLO-based object detection on real-time camera feeds and navigates based on detection results.
    """


    def __init__(
            self, 
            figure: str,
            p1_lat: float, p1_lon: float,
            p2_lat: float, p2_lon: float,
            p3_lat: float, p3_lon: float,
            p4_lat: float, p4_lon: float
        ) -> None:

        """
        Initializes the BouncingNode with the specified target figure and GPS area corners.

        Args:
            figure (str): Name of the target figure to detect (e.g., "circle").
            p1_lat, p1_lon, ..., p4_lat, p4_lon (float): GPS coordinates of the rectangular search area corners, 
                ordered as top-left, top-right, bottom-left, bottom-right.
        """


        super().__init__('bouncing_node')

        # Configura QoS com apenas 1 imagem no buffer
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(Image, 'camera/image_raw', self.camera_cb, qos_profile)

        self.last_frame = None


        figure_map: dict[str, int] = {
            "circle": 0,
            "square": 1,
            "triangle": 2,
            "hexagon": 3,
            "pentagon": 4,
            "star": 5,
            "cross": 6,
            "house": 7
        }

        figure_size_map: dict[int, float] = {
            0: 0.793,
            1: 0.554,
            2: 0.596,
            3: 0.687,
            4: 0.710,
            5: 0.715,
            6: 0.790,
            7: 0.715
        }

        self.figure_class = figure_map.get(figure, None)

        self.figure_size = figure_size_map.get(self.figure_class, None)
        
        if self.figure_class is None:
            raise ValueError(f"Figura '{figure}' inválida. Opções válidas: {list(figure_map.keys())}")

        self.bridge = CvBridge()

        self.threshold = 164

        self.image_height = 320
        model_path = os.path.join(os.path.dirname(__file__), "ai", "yolo", "yolov11n.onnx")
        self.model = YOLO(model_path, task='detect')
        
        self.drone: MavDrone = MavDrone(self, False)

        self.corner_top_left: Tuple[float, float] = (p1_lat, p1_lon)     
        self.corner_top_right: Tuple[float, float] = (p2_lat, p2_lon)    
        self.corner_bottom_left: Tuple[float, float] = (p3_lat, p3_lon)  
        self.corner_bottom_right: Tuple[float, float] = (p4_lat, p4_lon) 

        self.search_point_1a: Tuple[float, float]
        self.search_point_1b: Tuple[float, float]
        self.search_point_2a: Tuple[float, float]
        self.search_point_2b: Tuple[float, float]
        self.search_point_3a: Tuple[float, float]
        self.search_point_3b: Tuple[float, float]
        self.search_point_4a: Tuple[float, float]
        self.search_point_4b: Tuple[float, float]

        self.photos_heading: float

        self.points_calculation()

        self.points_to_visit: List[Tuple[float, float]] = [
            self.search_point_1a, 
            self.search_point_1b, 
            self.search_point_2a, 
            self.search_point_2b, 
            self.search_point_3a, 
            self.search_point_3b,
            self.search_point_4a,
            self.search_point_4b
        ]

    def camera_cb(self, msg: Image):
        """
        ROS callback that receives and stores the latest camera frame.

        Args:
            msg (Image): ROS image message containing the camera frame.
        """

        try:
            self.last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erro ao converter imagem: {e}")

    def run_inference(self) -> Tuple[int, int]:
        """
        Runs YOLO object detection on the latest camera frame.

        Returns:
            Tuple[int, int]: Coordinates (x1, y1, x2, y2) of the bounding box if the target is detected;
                            (-1, -1, -1, -1) if not found.
        """

        rclpy.spin_once(self, timeout_sec=0.2)

        cv2.imwrite("inference.jpg", self.last_frame)
        
        results = self.model(self.last_frame, conf=0.5)[0]
        x1, y1, x2, y2 = -1, -1, -1, -1

        if results.boxes is None or len(results.boxes) == 0:
            return -1, -1, -1, -1

        for box in results.boxes:
            cls = int(box.cls.item())
            if cls == self.figure_class:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                break

        if x1 > 0:
            self.get_logger().info("Detected!")

        return x1, y1, x2, y2
    

    def run(self) -> None:
        """
        Performs the complete mission routine:
        - Takes off.
        - Visits predefined GPS waypoints.
        - Runs inference on each location.
        - If the target is detected, attempts to navigate and land on it.
        """

        self.drone.arm_takeoff(6.5)

        time.sleep(8.0)

        #running first inference for pre-compiling the model
        x1, y1, x2, y2 = self.run_inference()

        x = (x1 + x2) // 2
        y = (y1 + y2) // 2

        if x1 != -1:
            self.visit_detection(x, y)

        while(len(self.points_to_visit) > 0):

            self.drone.offboard_gps_position(
                lat_setpoint=self.points_to_visit[0][0], 
                lon_setpoint=self.points_to_visit[0][1], 
                alt_setpoint=6.5, 
                heading=self.photos_heading,
                precision_radius=0.1
            )
            
            self.get_logger().info(" --Running Inference--")
            x1, y1, x2, y2 = self.run_inference()

            x = (x1 + x2) // 2
            y = (y1 + y2) // 2
            rclpy.spin_once(self, timeout_sec=0.2)

            if x != -1:
                if self.visit_detection(x, y): break

            self.points_to_visit.pop(0)

            self.points_to_visit.sort(
                key=lambda point: self.drone.gps_controller.haversine_distance(lat=point[0], lon=point[1])
            )


        
    def visit_detection(self, coord_x: int, coord_y: int) -> bool:
        """
        Converts the pixel coordinates of a detected object into estimated GPS coordinates,
        then navigates the drone to that location for validation.

        Args:
            coord_x (int): X pixel coordinate of the detected object.
            coord_y (int): Y pixel coordinate of the detected object.

        Returns:
            bool: True if the object was successfully validated and landing was initiated; False otherwise.
        """

        camera_displacement = ImageCalculus.calculate_offset_pixels(
            0.12, 6.5, 43.3, self.image_height
        )

        self.get_logger().info(f"drone center: {(self.image_height / 2) + camera_displacement} | x:{coord_x} | y:{coord_y}")

        
        lat, lon = ImageCalculus.estimate_pixel_gps(
            origin_lat=self.drone.get_gps.latitude,
            origin_lon=self.drone.get_gps.longitude,
            origin_row=(self.image_height / 2) + camera_displacement,
            origin_col=(self.image_height / 2),
            target_row=coord_y,
            target_col=coord_x,
            gsd= 1.1 / 145,
            image_bearing=self.drone.get_heading.data
        )
        
        self.drone.offboard_gps_position(
            lat_setpoint=lat,
            lon_setpoint=lon,
            alt_setpoint=5.0,
            heading=self.drone.gps_controller.calculate_bearing(lat, lon),
            precision_radius=0.1
        )

        return self.adjust_position()

    def calculate_error(self) -> Tuple[float, float]:
        """
        Analyzes the bounding box of the detected object and computes positional errors
        (front and side) based on the drone's image center and the object’s estimated size.

        Returns:
            Tuple[float, float]: Positional errors in meters (front, side) from the target.
        """

        x1, y1, x2, y2 = self.run_inference()

        gsd = 0

        if x1 >= 0:

            x = (x1 + x2) // 2
            y = (y1 + y2) // 2

            crop = self.last_frame[int(y1):int(y2), int(x1):int(x2)]
            lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB)
            l_channel, _, _ = cv2.split(lab)
            _, thresh = cv2.threshold(l_channel, self.threshold, 255, cv2.THRESH_BINARY_INV)

            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            cnt = max(contours, key=cv2.contourArea)

            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            width, height = rect[1]
            side_length = max(width, height)

            gsd = self.figure_size / side_length

            drone_height = (self.image_height / 2) * gsd / np.tan(np.radians(43.3/2))

            self.get_logger().info(f"drone height calculated pixel: {drone_height}")

            camera_displacement = ImageCalculus.calculate_offset_pixels(
                0.12, drone_height, 43.3, self.image_height
            )

            self.get_logger().info(f"cam_dis: {camera_displacement * gsd}")

            error_sides = (self.image_height / 2) - x
            error_front = (self.image_height / 2) + camera_displacement - y
            self.get_logger().info(f"--- X:{x} | Y:{y} | ERROR FRONT: {error_front*gsd}m | ERROR SIDES: {error_sides*gsd}m")

            cv2.drawContours(self.last_frame, [box], 0, (255, 0, 0), 2)

            # Desenha ponto da inferência (azul)
            cv2.circle(self.last_frame, (int(x), int(y)), 5, (255, 0, 0), -1)

            # Desenha ponto do centro do drone (verde)
            center_x = (self.image_height // 2)
            center_y = (self.image_height // 2) + int(camera_displacement)
            cv2.circle(self.last_frame, (center_x, center_y), 5, (0, 255, 0), -1)

            # Desenha ponto do centro do drone (verde)
            center_x = (self.image_height // 2)
            center_y = (self.image_height // 2)
            cv2.circle(self.last_frame, (center_x, center_y), 5, (255, 0, 0), -1)   

            # Salva a imagem
            cv2.imwrite("contorno.jpg", self.last_frame)

        return gsd * error_front, gsd * error_sides


    def adjust_position(self) -> bool:
        """
        Performs fine position adjustment using calculated error, then attempts a second correction.
        Initiates landing if the object remains in view.

        Returns:
            bool: True if landing was initiated after adjustments; False otherwise.
        """


        error_front, error_sides = self.calculate_error()
        if error_front != None:
            kpy, kpx = 0.08, 0.03
            start_time = time.time()
            self.get_logger().info(f"Moving drone with: x:{error_front*kpy} | y:{error_sides*kpx}")
            
            self.drone.offboard_velocity_timer(error_front*kpy, error_sides*kpx, 0.0, 0.0, time=1.0)

            self.get_logger().info(f"Finished first adjust")

            return self.adjust_and_land()
        else:
            return False

    def adjust_and_land(self):
        """
        Performs a second fine adjustment based on updated inference data.
        If successful, lands the drone.

        Returns:
            bool: True if drone landed successfully; False otherwise.
        """

        error_sides, error_front = 1, 1

        while error_front > 0.1 or error_sides > 0.1:
            error_front, error_sides = self.calculate_error()

            if error_front != None:
                kpy, kpx = 0.1, 0.05
                start_time = time.time()
                self.get_logger().info(f"Moving drone with: x:{error_front*kpy} | y:{error_sides*kpx}")

                self.drone.offboard_velocity_timer(error_front*kpy, error_sides*kpx, 0.0, 0.0, time=1)
            
            else:
                self.get_logger().info("Not detected!!! --- Moving UP.")
                self.drone.offboard_velocity_timer(0.0, 0.0, 1.0, 0.0, time=1)

        self.get_logger().info(f"Finished second adjust, landing...")

        self.drone.land()

        return True

    def points_calculation(self) -> None:
        """
        Calculates 8 internal GPS waypoints evenly spaced across the defined rectangular area.
        These points are used during the search routine.

        Also computes the camera heading angle for consistent orientation during image capture.
        """


        upper_quarter_left = self.drone.gps_controller.interp_geo(self.corner_top_left, self.corner_bottom_left, 1/4)
        lower_quarter_left = self.drone.gps_controller.interp_geo(self.corner_top_left, self.corner_bottom_left, 3/4)

        upper_quarter_right = self.drone.gps_controller.interp_geo(self.corner_top_right, self.corner_bottom_right, 1/4)
        lower_quarter_right = self.drone.gps_controller.interp_geo(self.corner_top_right, self.corner_bottom_right, 3/4)

        self.search_point_1a = self.drone.gps_controller.interp_geo(upper_quarter_left, upper_quarter_right, 1/8)
        self.search_point_2a = self.drone.gps_controller.interp_geo(upper_quarter_left, upper_quarter_right, 3/8)
        self.search_point_3a = self.drone.gps_controller.interp_geo(upper_quarter_left, upper_quarter_right, 5/8)
        self.search_point_4a = self.drone.gps_controller.interp_geo(upper_quarter_left, upper_quarter_right, 7/8)

        self.search_point_1b = self.drone.gps_controller.interp_geo(lower_quarter_left, lower_quarter_right, 1/8)
        self.search_point_2b = self.drone.gps_controller.interp_geo(lower_quarter_left, lower_quarter_right, 3/8)
        self.search_point_3b = self.drone.gps_controller.interp_geo(lower_quarter_left, lower_quarter_right, 5/8)
        self.search_point_4b = self.drone.gps_controller.interp_geo(lower_quarter_left, lower_quarter_right, 7/8)

        lat, lon, lat1, lon1 = map(np.radians, [self.search_point_4a[0], self.search_point_4a[1], self.search_point_1a[0], self.search_point_1a[1]])

        dlon = lon - lon1

        x = np.sin(dlon) * np.cos(lat)
        y = np.cos(lat1) * np.sin(lat) - (np.sin(lat1) * np.cos(lat) * np.cos(dlon))
        bearing = np.arctan2(x, y)

        bearing = np.degrees(bearing)

        self.photos_heading = (bearing + 360) % 360


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BouncingNode(
        "cross",
        # -22.4136107,
        # -45.44662,
        # -22.4135038,
        # -45.4465338,
        # -22.4136517,
        # -45.4465533,
        # -22.413543,
        # -45.4464616
        
        -22.4152503,
        -45.4479286,
        -22.4153305,
        -45.4478092,
        -22.4153082,
        -45.4479674,
        -22.4153965,
        -45.4478671
          )
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
