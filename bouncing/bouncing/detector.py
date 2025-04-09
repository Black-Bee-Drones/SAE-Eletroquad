import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from mirela_sdk.image_processing.camera import ImageHandler
import cv2
import numpy as np


class BouncingDetector(Node):
    def __init__(self, image_source: str = "webcam"):
        super().__init__("bouncing_detector_node")
        self.image_handler: ImageHandler = ImageHandler(node=self, image_source=image_source, show_result='') 
        self.state_sub = self.create_subscription(String, "/current_state", self.state_callback, 10)
        self.error_pub = self.create_publisher(Float32MultiArray, "/figure_error", 10)
        self.image_handler.run()

    def state_callback(self, msg: String) -> None:
        state: str = msg.data
        self.get_logger().info(f"New state: {state}")

        callback_map: dict[str, callable] = {
            "circle": self.findCircle,
            "square": self.findSquare,
            "triangle": self.findTriangle,
            "hexagon": self.findHexagon,
            "pentagon": self.findPentagon,
            "star": self.findStar,
            "cross": self.findCross,
            "house": self.findHouse,
        }

        self.image_handler.image_processing_callback = callback_map.get(state, None)

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