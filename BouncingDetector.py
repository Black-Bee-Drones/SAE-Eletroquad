import rclpy
from rclpy.node import Node
from mirela_sdk.image_processing.camera import ImageHandler
import cv2
import numpy as np


class BouncingDetector(Node):
    def __init__(self, image_source: str = "webcam", state_list: list[str] = ["start"]):
        super().__init__("camera_viewer_node")
        self.image_handler = ImageHandler(node=self, image_source=image_source, show_result='')
        self.state_list: list[str] = state_list
        self._state_index: int = -1
        self.nextState()
        self.image_handler.run()

    @property
    def state(self) -> str:
        return self.state_list[self._state_index]

    @state.setter
    def state(self, value: int) -> None:
        if not (0 <= value < len(self.state_list)):
            raise IndexError("The given state index is out of range.")
        self._state_index = value

    def nextState(self) -> None:
        if self._state_index + 1 < len(self.state_list):
            self._state_index += 1
        else:
            raise IndexError("The state_list has been completed, there is no next state.")

        # Map state names to processing callbacks
        if self.state == "circle":
            self.image_handler.image_processing_callback = self.findCircle

        elif self.state == "square":
            self.image_handler.image_processing_callback = self.findSquare

        elif self.state == "triangle":
            self.image_handler.image_processing_callback = self.findTriangle

        elif self.state == "hexagon":
            self.image_handler.image_processing_callback = self.findHexagon

        elif self.state == "pentagon":
            self.image_handler.image_processing_callback = self.findPentagon

        elif self.state == "star":
            self.image_handler.image_processing_callback = self.findStar

        elif self.state == "cross":
            self.image_handler.image_processing_callback = self.findCross

        elif self.state == "house":
            self.image_handler.image_processing_callback = self.findHouse

        else:
            self.image_handler.image_processing_callback = None

        

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


def main():
    rclpy.init()
    test = BouncingDetector(state_list=["circle"])
    rclpy.spin(test)
    rclpy.shutdown()

if __name__ == "__main__":
    main()