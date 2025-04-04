import rclpy
import time

from yasmin import StateMachine, CbState
from yasmin_ros.basic_outcomes import SUCCEED

from mirela_sdk.utils.process import ProcessUtils

class StartLineDetection(CbState):
    def __init__(self, line_color: str, image_source: str = "webcam", method: str = "RotatedRect,") -> None:
        super().__init__([SUCCEED], self.start_detection)
        self.line_color = line_color
        self.image_source = image_source
        self.method = method
    
    def start_detection(self, blackboard) -> str:
        ProcessUtils.start_process(
            f"ros2 run mirela_sdk line_detection_node --ros-args -p image_source:={self.image_source} -p line_color:={self.line_color} -p method:={self.method}",
            "line_detect_node"
        )
        time.sleep(2)
        return SUCCEED

class StopLineDetection(CbState):
    def __init__(self):
        super().__init__([SUCCEED], self.stop_detection)
    
    def stop_detection(self, blackboard):
        """
        Stop the line detection.
        """
        ProcessUtils.kill_process("line_detect_node")
        return SUCCEED