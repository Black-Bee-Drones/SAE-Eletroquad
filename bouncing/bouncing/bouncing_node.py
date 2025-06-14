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
    ROS 2 node that manages the drone search routine over a mapped area using GPS navigation
    and computer vision to detect specific objects in real-time.
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
        Initializes the drone node, loads the target figure class, camera, and map coordinates.

        Args:
            figure (str): Name of the figure class to detect (e.g., "circle").
            p1_lat, p1_lon ... p4_lat, p4_lon: GPS coordinates defining the corners of the search area.
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
            0: 0.8,
            1: 0.8,
            2: 0.8,
            3: 0.8,
            4: 0.8,
            5: 0.8,
            6: 0.8,
            7: 0.715
        }

        self.figure_class = figure_map.get(figure, None)

        self.figure_size = figure_size_map.get(self.figure_class, None)
        
        if self.figure_class is None:
            raise ValueError(f"Figura '{figure}' inválida. Opções válidas: {list(figure_map.keys())}")

        self.brigde = CvBridge()

        self.image_height = 320
        model_path = os.path.join(os.path.dirname(__file__), "ai", "yolo", "YOLOv11p.onnx")
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
        try:
            self.last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erro ao converter imagem: {e}")

    def run_inference(self) -> Tuple[int, int]:
        """
        Performs object detection using the YOLO model on the captured frame.

        Returns:
            Tuple[int, int]: Pixel coordinates (x, y) of the detected object center, or (-1, -1) if not found.
        """

        rclpy.spin_once(self)
        
        results = self.model(self.last_frame)[0]
        x1, y1, x2, y2 = -1, -1, -1, -1

        if results.boxes is None or len(results.boxes) == 0:
            return -1, -1, -1, -1

        for box in results.boxes:
            cls = int(box.cls.item())
            if cls == self.figure_class:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                break

        return x1, y1, x2, y2
    

    def run(self) -> None:
        """
        Executes the full search routine:
        - Takes off
        - Visits each precomputed point
        - Runs detection
        - If the object is detected, navigates toward it and attempts to land.
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
            rclpy.spin_once(self)

            if x != -1:
                self.get_logger().info("Detected!")
                if self.visit_detection(x, y): break

            self.points_to_visit.pop(0)

            self.points_to_visit.sort(
                key=lambda point: self.drone.gps_controller.haversine_distance(lat=point[0], lon=point[1])
            )


        
    def visit_detection(self, coord_x: int, coord_y: int) -> bool:
        """
        Converts the pixel coordinates of the detection into GPS coordinates,
        then navigates the drone to the target location.

        Args:
            coord_x (int): X pixel coordinate of the detection.
            coord_y (int): Y pixel coordinate of the detection.

        Returns:
            bool: True if the object was successfully re-identified and landed on, False otherwise.
        """
        camera_displacement = ImageCalculus.calculate_offset_pixels(
            0.12, 6.5, 43.3, self.model_output_size
        )

        self.get_logger().info(f"drone center: {(self.model_output_size / 2) + camera_displacement} | x:{coord_x} | y:{coord_y}")

        
        lat, lon = ImageCalculus.estimate_pixel_gps(
            origin_lat=self.drone.get_gps.latitude,
            origin_lon=self.drone.get_gps.longitude,
            origin_row=(self.model_output_size / 2) + camera_displacement,
            origin_col=(self.model_output_size / 2),
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
        x1, y1, x2, y2 = self.run_inference()

        error_front, error_sides = None, None

        if x1 >= 0:
            x = (x1 + x2) // 2
            y = (y1 + y2) // 2

            side_length = max(x2 - x1, y2 - y1)
            
            gsd = self.figure_size / side_length

            drone_height = (self.model_output_size / 2) * gsd / np.tan(np.radians(43.3/2))

            self.get_logger().info(f"drone height calculated pixel: {drone_height}")

            camera_displacement = ImageCalculus.calculate_offset_pixels(
                0.12, drone_height, 43.3, self.model_output_size
            )

            error_sides = (self.model_output_size / 2) - x
            error_front = (self.model_output_size / 2) + camera_displacement - y
            self.get_logger().info(f"--- X:{x} | Y:{y} | ERROR FRONT: {error_front} | ERROR SIDES: {error_sides}")

        return gsd * error_front, gsd * error_sides


    def adjust_position(self) -> bool:
        """
        Performs a final detection and velocity adjustment toward the target.
        If detection is still valid, lands the drone.

        Returns:
            bool: True if target still detected and drone lands, False otherwise.
        """

        error_front, error_sides = self.calculate_error()
        if error_front != None:
            kp = 0.5
            start_time = time.time()
            self.get_logger().info(f"Moving drone with: x:{error_front*kp} | y:{error_sides*kp}")
            while time.time() - start_time < 0.2:
                #self.drone.offboard_velocity(error_front*kp, error_sides*kp, -0.5, 0.0)
                break
            self.get_logger().info(f"Finished first adjust")

            self.drone.land()

            return True

            #return self.adjust_and_land()
        else:
            return False

    def adjust_and_land(self):

        error_front, error_sides = self.calculate_error()

        if error_front != None:
            kp = 0.5
            start_time = time.time()
            self.get_logger().info(f"Moving drone with: x:{error_front*kp} | y:{error_sides*kp}")
            while time.time() - start_time < 0.2:
                #self.drone.offboard_velocity(error_front*kp, error_sides*kp, -0.5, 0.0)
                break
            self.get_logger().info(f"Finished second adjust, landing...")

            self.drone.land()

            return True
        else:
            return False

    def points_calculation(self) -> None:
        """
        Computes 8 intermediate GPS points within the search area based on the input corners.
        The search points are evenly distributed along horizontal bands across the area.
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

        lat, lon, lat1, lon1 = map(np.radians, [self.search_point_1a[0], self.search_point_1a[1], self.search_point_4a[0], self.search_point_4a[1]])

        dlon = lon - lon1

        x = np.sin(dlon) * np.cos(lat)
        y = np.cos(lat1) * np.sin(lat) - (np.sin(lat1) * np.cos(lat) * np.cos(dlon))
        bearing = np.arctan2(x, y)

        bearing = np.degrees(bearing)

        self.photos_heading = (bearing + 360) % 360


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BouncingNode(
        "house",
        -22.4136107,
        -45.44662,
        -22.4135038,
        -45.4465338,
        -22.4136517,
        -45.4465533,
        -22.413543,
        -45.4464616
        
        # -22.4152503,
        # -45.4479286,
        # -22.4153305,
        # -45.4478092,
        # -22.4153082,
        # -45.4479674,
        # -22.4153965,
        # -45.4478671
          )
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
