import rclpy
import time
import cv2
import subprocess
import re
from rclpy.node import Node
from typing import Tuple, List
from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.image_processing.camera.image_calculus import ImageCalculus

class BouncingNode(Node):
    """
    ROS2 Node that controls drone movement using visual feedback and PID controllers.

    This node:
    - Receives mission commands via the "/mission_cmd" topic (e.g., "takeoff")
    - Receives visual alignment error (dx, dy) via the "/figure_error" topic
    - Applies PID controllers for visual alignment correction
    - Initiates automatic landing when alignment is within a threshold
    - Publishes landing status to the "/movement_status" topic
    """

    def __init__(
            self, 
            figure: str,
            p1_lat: float, p1_lon: float,
            p2_lat: float, p2_lon: float,
            p3_lat: float, p3_lon: float,
            p4_lat: float, p4_lon: float
        ) -> None:
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

        C920_CTRL_MAP = {
            'HD Pro Webcam C920': 'focus_automatic_continuous=0',
            'Logi Webcam C920e': 'focus_auto=0',
            }


        self.figure_class = figure_map.get(figure, None)

        result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True)
        lines = result.stdout.splitlines()
        device = None
        ctrl_param = None

        for i, line in enumerate(lines):
            for model_name, param in self.C920_CTRL_MAP.items():
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
            self.node.get_logger().error(
                "C920 camera not detected. Please ensure the device is connected and that 'v4l2-ctl' is installed."
            )
            self.node.get_logger().warn(
                f"Falling back to default camera: cv2.VideoCapture({self.cap_num})."
            )
        
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        success = True
        success &= self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        success &= self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        success &= self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        success &= self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not success:
            self.node.get_logger().warn(
                "Failed to apply all camera settings. Continuing, but performance may be degraded."
            )
            
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

        self.points_to_visit: List[float] = [
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
        self.drone.takeoff(5.0)

        time.sleep(5)

        while(len(self.points_to_visit) > 0):

            self.drone.offboard_gps_position(
                lat_setpoint=self.points_to_visit[0][0], 
                lon_setpoint=self.points_to_visit[0][1], 
                alt_setpoint=5.0, 
                heading=self.drone.gps_controller.calculate_bearing(self.points_to_visit[0][0], self.points_to_visit[0][1]),
                precision_radius=0.1
            )

            frame = self.take_photo()

            detected = False

            # AQUI RODA A IA -- self.figure_class já segura o index da figura desejada.
            x, y = 0, 0 # Pixels do centro da figura

            rclpy.spin_once(self)

            if detected:
                break
            else:
                self.points_to_visit.pop(0)
                self.points_to_visit.sort(key = lambda a, b: self.drone.gps_controller.haversine_distance(lat=a, lon=b))

        lat, lon = ImageCalculus.find_coordinate(
            centerpixel_lat=self.drone.get_gps.latitude,
            centerpixel_lon=self.drone.get_gps.longitude,
            bearing=self.drone.get_heading.data,
            centerpixel_height=320,
            centerpixel_width=320,
            pixel2_height=y,
            pixel2_width=x,
            gdr= 1.1 / 161
            )

            

    def points_calculation(self) -> None:
        """
        Computes intermediate search points across the mapped region using geodesic interpolation.
        The points are evenly spaced along the center axis of the area, and divide the area in 3 sections, which will map the entire area.
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

    def take_photo(self) -> cv2.MatLike:
        ret, frame = self.cap.read()
        if not ret:
            try:
                time.sleep(0.5)
                ret, frame = self.cap.read()
            
            except:
                raise RuntimeError(f"Falha ao capturar frame.")

        # Cortar para 1:1 centralizado (quadrado)
        side = 1080
        center_x, center_y = 1920 // 2, 1080 // 2
        half_side = side // 2
        crop = frame[center_y - half_side:center_y + half_side, center_x - half_side:center_x + half_side]

        # Resize para 640x640
        return cv2.resize(crop, (640, 640), interpolation=cv2.INTER_AREA)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BouncingNode("circle", 0, 0, 0, 0, 0, 0, 0, 0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
