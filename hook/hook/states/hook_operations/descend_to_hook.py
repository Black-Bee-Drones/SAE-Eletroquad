import rclpy

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.utils.process import ProcessUtils
from mirela_interfaces.msg import LineInfo
from std_msgs.msg import Bool

from time import sleep
import time

from hook.states.constants import (
    DESCEND_SPEED,
    MIN_DESCEND_ALTITUDE,
    LINE_DETECT_NODE_NAME,
)


class DescendToHook(State):
    """Descends to the hook target position and monitors when the drone is close enough"""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.red_line_info_sub = None
        self.red_detected_sub = None
        self.max_area = 0
        self.last_area = 0
        self.last_detected = False
        self.area_decreasing_count = 0
        self.running = True

    def red_line_info_callback(self, msg: LineInfo):
        """We track area information from the red line to determine proximity"""
        # Note: LineInfo doesn't currently include area, but this could be added
        # Here we're just using the callback to know we're still receiving data
        pass

    def red_detect_callback(self, msg: Bool):
        """Use the detection status to track if we're still seeing the red line"""
        self.last_detected = msg.data

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Descending towards hook...")
        self.max_area = 0
        self.last_area = 0
        self.last_detected = False
        self.area_decreasing_count = 0
        self.running = True

        # Clean up any previously running processes
        self._cleanup_resources()

        # Start line detection node with red color only
        line_detection_cmd = (
            "ros2 run mirela_sdk line_detection_node "
            "--ros-args "
            "-p line_colors:=red "
            "-p show_visualization:=True "
            "-p image_source:=webcam "
            "-p visualization_name:='Descent Tracking'"
        )

        if not ProcessUtils.start_process(line_detection_cmd, LINE_DETECT_NODE_NAME):
            yasmin.YASMIN_LOG_ERROR("Failed to start red line detection node for descent.")
            return ABORT

        yasmin.YASMIN_LOG_INFO("Line detection node started successfully for descent.")
        sleep(2)  # Give node time to start

        # Subscribe to red line detection status
        self.red_detected_sub = self.node.create_subscription(
            Bool,
            "/line_detect/red",
            self.red_detect_callback,
            10,
        )

        # Subscribe to red line info if needed for future enhancements
        self.red_line_info_sub = self.node.create_subscription(
            LineInfo,
            "/line_state/red",
            self.red_line_info_callback,
            10,
        )

        start_time = time.time()
        timeout = 30
        consecutive_not_detected = 0
        max_consecutive_not_detected = 5  # Allow brief detection losses

        # Main descent loop
        while time.time() - start_time < timeout:
            # Get current relative altitude
            rel_alt = blackboard.mavdrone.get_rel_alt.data

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
                self._cleanup_resources()
                return SUCCEED

            # Control lateral position to stay centered while descending
            blackboard.mavdrone.offboard_velocity(
                linear_x=0.0, linear_y=0.0, linear_z=DESCEND_SPEED, angular_z=0.0
            )

            rclpy.spin_once(self.node, timeout_sec=0.05)

            # Safety check for minimum altitude
            if rel_alt < MIN_DESCEND_ALTITUDE:
                yasmin.YASMIN_LOG_INFO(
                    f"Reached minimum safe altitude ({MIN_DESCEND_ALTITUDE}m), ready to drop hook."
                )
                self._cleanup_resources()
                return SUCCEED

        yasmin.YASMIN_LOG_ERROR("Failed to descend to hook (timeout).")
        self._cleanup_resources()
        return ABORT

    def _cleanup_resources(self):
        """Clean up subscribers and processes"""
        yasmin.YASMIN_LOG_INFO("Cleaning up DescendToHook resources...")

        # Clean up subscribers
        if self.red_line_info_sub:
            self.node.destroy_subscription(self.red_line_info_sub)
            self.red_line_info_sub = None

        if self.red_detected_sub:
            self.node.destroy_subscription(self.red_detected_sub)
            self.red_detected_sub = None

        # Kill the line detection process
        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)

        yasmin.YASMIN_LOG_INFO("DescendToHook cleanup completed")
