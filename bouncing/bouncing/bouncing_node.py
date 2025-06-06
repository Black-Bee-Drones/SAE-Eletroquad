import rclpy
import time
import cv2
import os
import subprocess
import re
from rclpy.node import Node
from typing import Tuple, List
from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing.camera.image_calculus import ImageCalculus
import numpy as np
from ultralytics import YOLO

class CameraFeed():
    """
    Handles camera initialization, configuration, image capture, and inference using a YOLO model.
    """

    def __init__(self, target_class):
        """
        Initializes the camera feed, configures USB camera settings using v4l2-ctl,
        and loads a YOLOv8 model for object detection.

        Args:
            target_class (int): The target class index to detect during inference.
        """

        self.target_class = target_class

        C920_CTRL_MAP = {
            'HD Pro Webcam C920': 'focus_auto=0',
            'Logi Webcam C920e': 'focus_auto=0',
            }
        
        result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True)
        lines = result.stdout.splitlines()
        device = None
        ctrl_param = None

        for i, line in enumerate(lines):
            for model_name, param in C920_CTRL_MAP.items():
                if model_name in line:
                    ctrl_param = param
                    j = i + 1
                    while j < len(lines) and lines[j].startswith('\t'):
                        match = re.search(r'(/dev/video\d+)', lines[j])
                        if match:
                            device = match.group(1)
                            break
                        j += 1
                    break
            if device and ctrl_param:
                subprocess.run(['v4l2-ctl', '-d', device, '--set-ctrl=' + ctrl_param])
                break

        if device is None:
            raise RuntimeError("C920 camera not detected. Please ensure the device is connected and that 'v4l2-ctl' is installed.")
        
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        success = True
        success &= self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        success &= self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        success &= self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        success &= self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not success:
            print(
                "Failed to apply all camera settings. Continuing, but performance may be degraded."
            )

        model_path = os.path.join(os.path.dirname(__file__), "ai", "yolo", "best.onnx")
        self.model = YOLO(model_path, task='detect')

    def take_photo(self) -> np.ndarray:
        """
        Captures a frame from the camera, crops it to a square (1:1), and resizes to 640x640.

        Returns:
            np.ndarray: Processed image frame ready for inference.
        """

        ret, frame = self.cap.read()
        if not ret:
            time.sleep(0.5)
            ret, frame = self.cap.read()
            if not ret:
                raise RuntimeError("Falha ao capturar frame da câmera.")

        # Cortar para 1:1 centralizado (quadrado)
        side = 1080
        center_x, center_y = 1920 // 2, 1080 // 2
        half_side = side // 2
        crop = frame[center_y - half_side:center_y + half_side, center_x - half_side:center_x + half_side]

        # Resize para 640x640
        return cv2.resize(crop, (640, 640), interpolation=cv2.INTER_AREA)
    
    def run_inference(self) -> Tuple[int, int]:
        """
        Performs object detection using the YOLO model on the captured frame.

        Returns:
            Tuple[int, int]: Pixel coordinates (x, y) of the detected object center, or (-1, -1) if not found.
        """
        
        frame = self.take_photo()
        results = self.model(frame)[0]
        x, y = -1, -1
        for box in results.boxes:
            cls = int(box.cls.item())
            if cls == self.target_class:
                detected = True

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                x = int((x1 + x2) / 2)
                y = int((y1 + y2) / 2)
                break
        return x, y




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


        self.camera = CameraFeed(self.figure_class)

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

    def run(self) -> None:
        """
        Executes the full search routine:
        - Takes off
        - Visits each precomputed point
        - Runs detection
        - If the object is detected, navigates toward it and attempts to land.
        """

        self.drone.arm_takeoff(5.0)

        time.sleep(10)

        while(len(self.points_to_visit) > 0):

            self.drone.offboard_gps_position(
                lat_setpoint=self.points_to_visit[0][0], 
                lon_setpoint=self.points_to_visit[0][1], 
                alt_setpoint=5.0, 
                heading=self.drone.gps_controller.calculate_bearing(self.points_to_visit[0][0], self.points_to_visit[0][1]),
                precision_radius=0.1
            )

            x, y = self.camera.run_inference()

            rclpy.spin_once(self)

            if x != -1:
                if self.visit_detection(x, y): break

            self.points_to_visit.pop(0)

            self.points_to_visit.sort(
                key=lambda point: self.drone.gps_controller.haversine_distance(lat=point[0], lon=point[1])
            )


        
    def visit_detection(self, coord_x, coord_y) -> bool:
        """
        Converts the pixel coordinates of the detection into GPS coordinates,
        then navigates the drone to the target location.

        Args:
            coord_x (int): X pixel coordinate of the detection.
            coord_y (int): Y pixel coordinate of the detection.

        Returns:
            bool: True if the object was successfully re-identified and landed on, False otherwise.
        """

        lat, lon = ImageCalculus.find_coordinate(
            centerpixel_lat=self.drone.get_gps.latitude,
            centerpixel_lon=self.drone.get_gps.longitude,
            bearing=self.drone.get_heading.data,
            centerpixel_height=320,
            centerpixel_width=320,
            pixel2_height=coord_y,
            pixel2_width=coord_x,
            gdr= 1.1 / 161
            )
        
        self.drone.offboard_gps_position(
            lat_setpoint=lat,
            lon_setpoint=lon,
            alt_setpoint=5.0,
            heading=self.drone.gps_controller.calculate_bearing(lat, lon),
            precision_radius=0.1
        )

        return self.adjust_position()

    def adjust_position(self) -> bool:
        """
        Performs a final detection and velocity adjustment toward the target.
        If detection is still valid, lands the drone.

        Returns:
            bool: True if target still detected and drone lands, False otherwise.
        """

        x, y = self.camera.run_inference()

        if x != -1:
            kp = 0.0
            self.drone.offboard_velocity_timer(x*kp, y*kp, 0.0, 0.0, time=1)
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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BouncingNode(
        "house",
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
