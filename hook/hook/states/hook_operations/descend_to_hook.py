import rclpy

import yasmin
from yasmin import State, StateMachine, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from mirela_sdk.utils.process import ProcessUtils
from mirela_interfaces.msg import LineInfo
from std_msgs.msg import Bool

from time import sleep
import time

from hook.states.constants import (
    DESCEND_SPEED,
    MIN_DESCEND_ALTITUDE,
    LINE_DETECT_NODE_NAME,
    LINE_DETECTION_RED_COLOR_NAME,
    LINE_DETECTION_RED_SPACE,
    LINE_DETECTION_IMAGE_SOURCE,
    LINE_DETECTION_SHOW_VISUALIZATION,
    LINE_DETECTION_DESCENT_TITLE,
)


class StartRedLineDetection(State):
    """Start the red line detection process for descent."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Starting red line detection process for descent...")
        
        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)

        line_detection_cmd = (
            "ros2 run mirela_sdk line_detection_node "
            "--ros-args "
            f"-p line_colors:={LINE_DETECTION_RED_COLOR_NAME} "
            f"-p spaces:={LINE_DETECTION_RED_SPACE} "
            f"-p show_visualization:={LINE_DETECTION_SHOW_VISUALIZATION} "
            f"-p image_source:={LINE_DETECTION_IMAGE_SOURCE} "
            f"-p visualization_name:='{LINE_DETECTION_DESCENT_TITLE}'"
        )

        if not ProcessUtils.start_process(line_detection_cmd, LINE_DETECT_NODE_NAME):
            yasmin.YASMIN_LOG_ERROR(
                "Failed to start red line detection node for descent."
            )
            return ABORT

        yasmin.YASMIN_LOG_INFO("Line detection node started successfully for descent.")
        sleep(2)

        return SUCCEED


class PerformDescent(State):
    """Perform the descent operation while tracking the red line."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.red_line_info_sub = None
        self.red_detected_sub = None
        self.max_area = 0
        self.last_area = 0
        self.last_detected = False
        self.area_decreasing_count = 0
        self.node = YasminNode.get_instance()

    def red_line_info_callback(self, msg: LineInfo):
        """We track area information from the red line to determine proximity"""
        # Note: LineInfo doesn't currently include area, but this could be added
        # Here we're just using the callback to know we're still receiving data
        pass

    def red_detect_callback(self, msg: Bool):
        """Use the detection status to track if we're still seeing the red line"""
        self.last_detected = msg.data

    def execute(self, blackboard: Blackboard):
        if not "mavdrone" in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in PerformDescent state.")
            return ABORT

        mavdrone = blackboard["mavdrone"]

        yasmin.YASMIN_LOG_INFO("Descending towards hook...")
        self.max_area = 0
        self.last_area = 0
        self.last_detected = False
        self.area_decreasing_count = 0

        self.red_detected_sub = self.node.create_subscription(
            Bool,
            f"/line_detect/{LINE_DETECTION_RED_COLOR_NAME}",
            self.red_detect_callback,
            10,
        )

        self.red_line_info_sub = self.node.create_subscription(
            LineInfo,
            f"/line_state/{LINE_DETECTION_RED_COLOR_NAME}",
            self.red_line_info_callback,
            10,
        )

        start_time = time.time()
        timeout = 30
        consecutive_not_detected = 0
        max_consecutive_not_detected = 5  # Allow brief detection losses

        # Main descent loop
        while time.time() - start_time < timeout:
            rel_alt = mavdrone.get_rel_alt.data

            # Check if we're still detected - if we lose detection too long, we've likely gone too far down
            if not self.last_detected:
                consecutive_not_detected += 1
                yasmin.YASMIN_LOG_DEBUG(
                    f"Red line not detected: {consecutive_not_detected}/{max_consecutive_not_detected}"
                )
            else:
                consecutive_not_detected = 0
                yasmin.YASMIN_LOG_DEBUG(f"Red line detected, altitude: {rel_alt}m")

            if consecutive_not_detected > max_consecutive_not_detected:
                yasmin.YASMIN_LOG_INFO(
                    "Red line no longer detected consistently, likely at drop position."
                )
                self._cleanup_subscribers()
                return SUCCEED
            
            mavdrone.offboard_velocity(
                linear_x=0.0, linear_y=0.0, linear_z=DESCEND_SPEED, angular_z=0.0
            )

            rclpy.spin_once(self.node, timeout_sec=0.05)

            if rel_alt < MIN_DESCEND_ALTITUDE:
                yasmin.YASMIN_LOG_INFO(
                    f"Reached minimum safe altitude ({MIN_DESCEND_ALTITUDE}m), ready to drop hook."
                )
                mavdrone.offboard_velocity(
                    linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0
                )
                self._cleanup_subscribers()
                return SUCCEED

        yasmin.YASMIN_LOG_ERROR("Failed to descend to hook (timeout).")
        self._cleanup_subscribers()
        return ABORT

    def _cleanup_subscribers(self):
        """Clean up subscribers"""
        yasmin.YASMIN_LOG_INFO("Cleaning up PerformDescent subscribers...")

        # Clean up subscribers
        if self.red_line_info_sub:
            self.node.destroy_subscription(self.red_line_info_sub)
            self.red_line_info_sub = None

        if self.red_detected_sub:
            self.node.destroy_subscription(self.red_detected_sub)
            self.red_detected_sub = None

        yasmin.YASMIN_LOG_INFO("PerformDescent subscribers cleanup completed")


class CleanupProcesses(State):
    """Clean up all processes started for descent."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Cleaning up descent processes...")

        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)

        yasmin.YASMIN_LOG_INFO("Descent process cleanup completed")
        return SUCCEED


class DescendToHook(StateMachine):
    """StateMachine that manages the descent to hook operation."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "START_RED_LINE_DETECTION",
            StartRedLineDetection(),
            transitions={SUCCEED: "PERFORM_DESCENT", ABORT: "CLEANUP_PROCESSES"},
        )

        self.add_state(
            "PERFORM_DESCENT",
            PerformDescent(),
            transitions={SUCCEED: "CLEANUP_PROCESSES", ABORT: "CLEANUP_PROCESSES"},
        )

        self.add_state(
            "CLEANUP_PROCESSES",
            CleanupProcesses(),
            transitions={SUCCEED: SUCCEED},
        )

    def execute(self, blackboard):
        """Execute the state machine with outcome tracking."""
        outcome = super().execute(blackboard)
        return outcome
