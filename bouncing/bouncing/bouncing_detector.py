import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from mirela_sdk.image_processing.camera.image_handler import ImageHandler
import cv2
import numpy as np


class BouncingDetector(Node):
    def __init__(self, image_source: str = "c920"):
        super().__init__("bouncing_detector_node")
        self.image_handler: ImageHandler = ImageHandler(node=self, image_source=image_source, show_result='', c920_config=2) 
        self.figure_sub = self.create_subscription(String, "/current_figure", self.figure_callback, 10)
        self.error_pub = self.create_publisher(Float32MultiArray, "/figure_error", 10)
        self.status_pub = self.create_publisher(String, "/detector_status", 10)
        self.image_handler.run()
        self.image_handler.image_processing_callback = None

        self.callback_map: dict[str, callable] = {
            "circle": self.findCircle,
            "square": self.findSquare,
            "triangle": self.findTriangle,
            "hexagon": self.findHexagon,
            "pentagon": self.findPentagon,
            "star": self.findStar,
            "cross": self.findCross,
            "house": self.findHouse,
            "none": None,
        }

        self.publish_ready()

    def figure_callback(self, msg: String) -> None:
        figure: str = msg.data
        self.get_logger().info(f"New figure: {figure}")
        func = self.callback_map.get(figure, None)
        self.image_handler.image_processing_callback = func

    def publish_ready(self):
        msg = String()
        msg.data = "ready"
        self.status_pub.publish(msg)

    def publish_error(self, dx: float, dy: float) -> None:
        msg: Float32MultiArray = Float32MultiArray()
        msg.data = [dx, dy]
        self.error_pub.publish(msg)

    def findCircle(self, img: np.ndarray) -> None:
        pass

    def findSquare(self, img: np.ndarray) -> None:
        pass

    def findTriangle(self, img: np.ndarray) -> None:
        pass

    def findHexagon(self, img: np.ndarray) -> None:
        pass

    def findPentagon(self, img: np.ndarray) -> None:
        pass

    def findStar(self, img: np.ndarray) -> None:
        pass

    def findCross(self, img: np.ndarray) -> None:
        pass

    def findHouse(self, img: np.ndarray) -> None:
        pass


def main(args=None) -> None:    
    rclpy.init(args=args)
    node = BouncingDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()