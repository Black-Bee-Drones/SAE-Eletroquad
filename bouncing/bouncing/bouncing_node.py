
import rclpy
import time
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from typing import Tuple, List, Optional
from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing.camera.image_calculus import ImageCalculus
import numpy as np
from ultralytics import YOLO
from geopy.distance import distance
from geopy.point import Point


class PID:
    def __init__(self, kp: float, ki: float, kd: float) -> None:
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd

        self.integral: float = 0.0
        self.prev_error: Optional[float] = None

    def compute(self, error: float, dt: float) -> float:
        self.integral += error * dt
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    
    def restart(self) -> None:
        self.integral = 0.0
        self.prev_error = None


class BouncingNode(Node):
    """
    ROS 2 node that manages a drone's search and detection routine within a predefined GPS area.
    It uses YOLO-based object detection on real-time camera feeds and navigates based on detection results.
    """


    def __init__(self, figure: str) -> None:
        """
        Initializes the BouncingNode with the specified target figure and GPS area corners.

        Args:
            figure (str): Name of the target figure to detect (e.g., "circle").
            ordered as top-left, top-right, bottom-left, bottom-right.
        """

        super().__init__('bouncing_node')

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

        self.figure_class = figure_map.get(figure, None)

        if self.figure_class is None:
            raise ValueError(f"Figura '{figure}' inválida. Opções válidas: {list(figure_map.keys())}")

        self.image_height = 320

        model_path = os.path.join(os.path.dirname(__file__), "ai", "yolo", "yolov11n.onnx")
        self.model = YOLO(model_path, task='detect')
        
        self.drone: MavDrone = MavDrone(self, False)

        self.corner_top_left: Tuple[float, float]     
        self.corner_top_right: Tuple[float, float]    
        self.corner_bottom_left: Tuple[float, float] 
        self.corner_bottom_right: Tuple[float, float]

        self.search_point_1a: Tuple[float, float]
        self.search_point_1b: Tuple[float, float]
        self.search_point_2a: Tuple[float, float]
        self.search_point_2b: Tuple[float, float]
        self.search_point_3a: Tuple[float, float]
        self.search_point_3b: Tuple[float, float]
        self.search_point_4a: Tuple[float, float]
        self.search_point_4b: Tuple[float, float]

        self.search_point_m1: Tuple[float, float]
        self.search_point_m2: Tuple[float, float]
        self.search_point_m3: Tuple[float, float]
        self.search_point_m4: Tuple[float, float]

        self.photos_heading: float

        self.points_calculation()

        self.points_to_visit: List[Tuple[float, float]] = [
            self.search_point_m4,
            self.search_point_4a, 
            self.search_point_4b, 
            self.search_point_3a, 
            self.search_point_3b, 
            self.search_point_2a, 
            self.search_point_2b,
            self.search_point_1a,
            self.search_point_1b,
            self.search_point_m2,
            self.search_point_m3,
            self.search_point_m4
        ]

    def run_inference(self) -> Tuple[int, int]:
        """
        Runs YOLO object detection on the latest camera frame.

        Returns:
            Tuple[int, int]: Coordinates (x1, y1, x2, y2) of the bounding box if the target is detected;
                            (-1, -1, -1, -1) if not found.
        """

        self.last_frame = cv2.imread('photo.jpg')

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

        detect = False

        error_front, error_sides = 0.0, 0.0

        if x1 >= 0:

            detect = True

            x = (x1 + x2) // 2
            y = (y1 + y2) // 2

            error_sides = (self.image_height / 2) - x
            error_front = (self.image_height / 2) - y + 10
            self.get_logger().info(f"--- X:{x} | Y:{y} | ERROR FRONT: {error_front} | ERROR SIDES: {error_sides}")

            # Desenha ponto da inferência (azul)
            cv2.circle(self.last_frame, (int(x), int(y)), 5, (255, 0, 0), -1)

            # Desenha ponto do centro do drone (verde)
            center_x = (self.image_height // 2)
            center_y = (self.image_height // 2)
            cv2.circle(self.last_frame, (center_x, center_y), 5, (0, 255, 0), -1)

            # Desenha ponto do centro do drone (verde)
            center_x = (self.image_height // 2)
            center_y = (self.image_height // 2)
            cv2.circle(self.last_frame, (center_x, center_y), 5, (255, 0, 0), -1)   

            # Salva a imagem
            cv2.imwrite("contorno.jpg", self.last_frame)

        
        return error_front, error_sides, detect

    def adjust_position(self) -> bool:
        """
        Performs fine position adjustment using calculated error, then attempts a second correction.
        Initiates landing if the object remains in view.

        Returns:
            bool: True if landing was initiated after adjustments; False otherwise.
        """


        error_front, error_sides, detect = self.calculate_error()
        if detect:
            kpy, kpx = 0.001, 0.001
            start_time = time.time()
            self.get_logger().info(f"Moving drone with: x:{error_front*kpy} | y:{error_sides*kpx}")
            
            self.drone.offboard_velocity_timer(error_front*kpy, error_sides*kpx, 0.0, 0.0, time=0.5)

            self.get_logger().info(f"Finished first adjust")

            return self.adjust_and_land()
        else:
            
            error_front, error_sides, detect = self.calculate_error()
            if detect:
                kpy, kpx = 0.001, 0.001
                start_time = time.time()
                self.get_logger().info(f"Moving drone with: x:{error_front*kpy} | y:{error_sides*kpx}")
                
                self.drone.offboard_velocity_timer(error_front*kpy, error_sides*kpx, 0.0, 0.0, time=0.5)

                self.get_logger().info(f"Finished first adjust")

                return self.adjust_position2()
            
            else:
                return False

    def adjust_position2(self) -> bool:
        """
        Performs fine position adjustment using calculated error, then attempts a second correction.
        Initiates landing if the object remains in view.

        Returns:
            bool: True if landing was initiated after adjustments; False otherwise.
        """


        error_front, error_sides, detect = self.calculate_error()
        if detect:
            kpy, kpx = 0.001, 0.001
            start_time = time.time()
            self.get_logger().info(f"Moving drone with: x:{error_front*kpy} | y:{error_sides*kpx}")
            
            self.drone.offboard_velocity_timer(min(0.3, error_front*kpy), min(0.3, error_sides*kpx), 0.0, 0.0, time=0.5)

            self.get_logger().info(f"Finished first adjust")

            return self.adjust_and_land()
        else:

            error_front, error_sides, detect = self.calculate_error()
            if detect:
                kpy, kpx = 0.001, 0.001
                start_time = time.time()
                self.get_logger().info(f"Moving drone with: x:{error_front*kpy} | y:{error_sides*kpx}")

                self.drone.offboard_velocity_timer(min(0.3, error_front*kpy), min(0.3, error_sides*kpx), 0.0, 0.0, time=0.5)

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

        error_front, error_sides = 50, 50

        fails = 0
        count_drops = 0

        while (abs(error_front) > 10 or abs(error_sides) > 10) or count_drops < 7:
            error_front, error_sides, detect = self.calculate_error()

            ci_x, ci_y = 0.0, 0.0

            if detect:
                kpx, kpy = 0.00218, 0.00218
                
                ci_y += error_front * 0.000015
                ci_x += error_sides * 0.000015
                
                self.get_logger().info(f"Moving drone with: x:{error_front*kpy} + {ci_y} | y:{error_sides*kpx} + {ci_x}")

                self.drone.offboard_velocity_timer(error_front*kpy + ci_y, error_sides*kpx + ci_x, 0.0, 0.0, time=0.3)

                if abs(error_front) < 30 and abs(error_sides) < 30:
                    ci_x, ci_y = 0.0, 0.0
                    count_drops += 1
                    self.get_logger().info(f"Count drops: {count_drops}")
                    self.drone.offboard_velocity_timer(0.0, 0.0, -0.3, 0.0, time=0.5)
                    if count_drops >= 5:
                        self.get_logger().info("Count drops reached 4, landing...")
                        break
            
            else:
                fails += 1

                if fails > 3:
                    self.get_logger().info("Failed 3 times, landing...")
                    break

            start_t = time.time()

        self.drone.land()

        self.get_logger().info(f"Finished second adjust, landing...")

        return True
    

    def gerar_poligono_com_heading(self, lat_central, lon_central, heading, frente=20, tras=20, lado=15):
        """
        Gera os 4 vértices de um polígono retangular orientado por um heading.

        Args:
            lat_central (float): Latitude do ponto central.
            lon_central (float): Longitude do ponto central.
            heading (float): Direção em graus (0 = norte, 90 = leste, etc.).
            frente (float): Distância em metros para frente (na direção do heading).
            tras (float): Distância em metros para trás (oposto ao heading).
            lado (float): Distância em metros para os lados (perpendicular ao heading).

        Returns:
            List[Tuple[float, float]]: Lista com 4 tuplas (lat, lon), no sentido horário.
        """
        centro = Point(lat_central, lon_central)

        # Ponto frontal (à frente do centro, na direção do heading)
        frente_pt = distance(meters=frente).destination(centro, heading % 360)
        # Ponto traseiro (na direção oposta)
        tras_pt = distance(meters=tras).destination(centro, (heading + 180) % 360)

        # Cantos do retângulo
        # Vértice frontal esquerdo
        frente_esq = distance(meters=lado).destination(frente_pt, (heading - 90) % 360)
        # Vértice frontal direito
        frente_dir = distance(meters=lado).destination(frente_pt, (heading + 90) % 360)

        # Vértice traseiro direito
        tras_dir = distance(meters=lado).destination(tras_pt, (heading + 90) % 360)
        # Vértice traseiro esquerdo
        tras_esq = distance(meters=lado).destination(tras_pt, (heading - 90) % 360)

        return [
            (tras_esq.latitude, tras_esq.longitude),
            (frente_esq.latitude, frente_esq.longitude),
            (tras_dir.latitude, tras_dir.longitude),
            (frente_dir.latitude, frente_dir.longitude)
        ]


    def points_calculation(self) -> None:
        """
        Calculates 8 internal GPS waypoints evenly spaced across the defined rectangular area.
        These points are used during the search routine.

        Also computes the camera heading angle for consistent orientation during image capture.
        """

        rclpy.spin_once(self, timeout_sec=1.0)

        coords = self.gerar_poligono_com_heading(self.drone.get_gps.latitude, self.drone.get_gps.longitude, self.drone.get_heading.data, 7.0, 7.0, 3.0)

        self.corner_top_left = coords[0]
        self.corner_top_right = coords[1]
        self.corner_bottom_left = coords[2]
        self.corner_bottom_right = coords[3]

        upper_quarter_left = self.drone.gps_controller.interp_geo(self.corner_top_left, self.corner_bottom_left, 1/4)
        lower_quarter_left = self.drone.gps_controller.interp_geo(self.corner_top_left, self.corner_bottom_left, 3/4)

        middle_left = self.drone.gps_controller.interp_geo(self.corner_top_left, self.corner_bottom_left, 1/2)
        middle_right = self.drone.gps_controller.interp_geo(self.corner_top_right, self.corner_bottom_right, 1/2)

        upper_quarter_right = self.drone.gps_controller.interp_geo(self.corner_top_right, self.corner_bottom_right, 1/4)
        lower_quarter_right = self.drone.gps_controller.interp_geo(self.corner_top_right, self.corner_bottom_right, 3/4)

        self.search_point_1a = self.drone.gps_controller.interp_geo(upper_quarter_left, upper_quarter_right, 1/8)
        self.search_point_2a = self.drone.gps_controller.interp_geo(upper_quarter_left, upper_quarter_right, 3/8)
        self.search_point_3a = self.drone.gps_controller.interp_geo(upper_quarter_left, upper_quarter_right, 5/8)
        self.search_point_4a = self.drone.gps_controller.interp_geo(upper_quarter_left, upper_quarter_right, 7/8)

        self.search_point_m1 = self.drone.gps_controller.interp_geo(middle_left, middle_right, 1/8)
        self.search_point_m2 = self.drone.gps_controller.interp_geo(middle_left, middle_right, 3/8)
        self.search_point_m3 = self.drone.gps_controller.interp_geo(middle_left, middle_right, 5/8)
        self.search_point_m4 = self.drone.gps_controller.interp_geo(middle_left, middle_right, 7/8)

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
    node = BouncingNode("star")
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
