import rclpy

import yasmin
from yasmin import State, StateMachine, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from mirela_sdk.utils.process import ProcessUtils
from std_msgs.msg import Bool

from time import sleep
import time

from hook.states.constants import (
    FORWARD_SPEED,
    MIN_BLUE_LINE_DETECTIONS,
    LINE_DETECT_NODE_NAME,
)


class StartBlueLineDetection(State):
    """Start the blue line detection process."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Starting blue line detection process...")

        # Kill any existing process with the same name first
        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)

        # Start line detection node with blue color
        line_detection_cmd = (
            "ros2 run mirela_sdk line_detection_node "
            "--ros-args "
            "-p line_colors:=blue "  
            "-p show_visualization:=True " 
            "-p image_source:=webcam "  
            "-p visualization_name:='Blue Line Search'"
        )

        if not ProcessUtils.start_process(
            line_detection_cmd, LINE_DETECT_NODE_NAME, False
        ):
            yasmin.YASMIN_LOG_ERROR("Failed to start blue line detection process.")
            return ABORT

        yasmin.YASMIN_LOG_INFO(
            "Line detection node started successfully for blue line search."
        )
        sleep(2)  # Give node time to start

        return SUCCEED


class SearchForBlueLine(State):
    """Move forward and detect blue line until confirmed."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.mavdrone = None
        self.line_detected_sub = None
        self.detection_count = 0
        self.line_detected = False
        self.node = YasminNode.get_instance()

    def line_detect_callback(self, msg: Bool):
        if msg.data:
            self.detection_count += 1
        else:
            self.detection_count = 0  # Reset count if detection lost
        self.line_detected = msg.data  # Store last state
        yasmin.YASMIN_LOG_INFO(
            f"Line detected: {msg.data}, Count: {self.detection_count}"
        )

    def execute(self, blackboard: Blackboard):
        if not "mavdrone" in blackboard:
            yasmin.YASMIN_LOG_ERROR(
                "MavDrone not available in SearchForBlueLine state."
            )
            return ABORT

        self.mavdrone = blackboard["mavdrone"]

        yasmin.YASMIN_LOG_INFO("Searching for blue line...")
        self.detection_count = 0
        self.line_detected = False

        # Subscribe to line detection status for blue
        self.line_detected_sub = self.node.create_subscription(
            Bool,
            "/line_detect/blue",  # Topic with color name
            self.line_detect_callback,
            10,
        )

        start_time = time.time()
        timeout = 60  # seconds

        # Main search loop
        while time.time() - start_time < timeout:
            # Command forward velocity
            self.mavdrone.offboard_velocity(
                linear_x=FORWARD_SPEED, linear_y=0.0, linear_z=0.0, angular_z=0.0
            )

            rclpy.spin_once(
                YasminNode.get_instance(), timeout_sec=0.05
            )  # Process callbacks

            if self.detection_count >= MIN_BLUE_LINE_DETECTIONS:
                yasmin.YASMIN_LOG_INFO(
                    f"Blue line confirmed after {self.detection_count} detections."
                )
                # Cleanup subscriber
                if self.line_detected_sub:
                    self.node.destroy_subscription(self.line_detected_sub)
                    self.line_detected_sub = None

                return SUCCEED

        yasmin.YASMIN_LOG_ERROR("Search for blue line timed out.")

        # Cleanup subscriber
        if self.line_detected_sub:
            self.node.destroy_subscription(self.line_detected_sub)
            self.line_detected_sub = None

        return ABORT


class CleanupResources(State):
    """Clean up resources after the search process."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Cleaning up SearchBlueLine resources...")

        # Kill the line detection process
        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)

        yasmin.YASMIN_LOG_INFO("SearchBlueLine cleanup completed")
        return SUCCEED


class SearchBlueLine(StateMachine):
    """StateMachine that handles the blue line search process."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        # Define states
        self.add_state(
            "START_DETECTION",
            StartBlueLineDetection(),
            transitions={SUCCEED: "SEARCH_LINE", ABORT: "CLEANUP_RESOURCES"},
        )

        self.add_state(
            "SEARCH_LINE",
            SearchForBlueLine(),
            transitions={SUCCEED: "CLEANUP_RESOURCES", ABORT: "CLEANUP_RESOURCES"},
        )

        self.add_state(
            "CLEANUP_RESOURCES",
            CleanupResources(),
            transitions={SUCCEED: SUCCEED},
        )

    def execute(self, blackboard):
        """Execute the state machine with outcome tracking."""
        # Execute the standard StateMachine execution with outcome tracking
        outcome = super().execute(blackboard)
        self._last_outcome = outcome
        return outcome
