import rclpy

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.utils.process import ProcessUtils
from mirela_interfaces.msg import LineInfo
from std_msgs.msg import Float64

from time import sleep
import time

from hook.states.constants import (
    CENTERING_THRESHOLD_RADIUS,
    CENTERING_CONFIRMATIONS,
    LINE_DETECT_NODE_NAME,
    CENTERING_PID_PROCESS,
    IMAGE_CENTER_X,
    CENTERING_P,
    CENTERING_I,
    CENTERING_D,
    CENTERING_OUTPUT_MIN,
    CENTERING_OUTPUT_MAX,
)


class CenterRedBlob(State):
    """Centers the drone over the red blob (hose target)"""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.red_line_info_sub = None
        self.control_effort_y_sub = None
        self.centering_confirmations = 0
        self.current_red_center_x = None
        self.current_y_velocity = 0.0
        self.running = True
        self.last_error_x = 0
        self.last_error_y = 0

        # Define topic names for improved organization
        self.red_center_state_topic = "/line_state/red/center_x"
        self.red_center_setpoint_topic = "/red/center_setpoint"
        self.red_vel_y_topic = "/red/velocity_y"

    def red_line_info_callback(self, msg: LineInfo):
        """Callback for red line state updates"""
        # Calculate error from center of image
        error_x = msg.center_x - IMAGE_CENTER_X

        # For this state, we consider the line centered when it's within the threshold
        if abs(error_x) < CENTERING_THRESHOLD_RADIUS:
            self.centering_confirmations += 1
        else:
            self.centering_confirmations = 0

        self.last_error_x = error_x
        self.current_red_center_x = msg.center_x

    def control_effort_y_callback(self, msg: Float64):
        self.current_y_velocity = msg.data

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Centering on red blob...")
        self.centering_confirmations = 0
        self.running = True
        self.last_error_x = 0
        self.last_error_y = 0
        self.current_y_velocity = 0.0

        # Clean up any existing processes
        self._cleanup_resources()

        # Ensure line detection node is running with red color
        line_detection_cmd = (
            "ros2 run mirela_sdk line_detection_node "
            "--ros-args "
            "-p line_colors:=red "
            "-p show_visualization:=True "
            "-p image_source:=webcam "
            "-p visualization_name:='Red Centering'"
        )

        if not ProcessUtils.start_process(line_detection_cmd, LINE_DETECT_NODE_NAME):
            yasmin.YASMIN_LOG_ERROR("Failed to start red line detection node.")
            return ABORT

        yasmin.YASMIN_LOG_INFO("Line detection node started successfully for red detection.")
        sleep(2)  # Give node time to start

        # Initialize setpoint publisher
        center_setpoint_pub = self.node.create_publisher(
            Float64, self.red_center_setpoint_topic, 10
        )

        # Publish initial setpoint
        center_msg = Float64()
        center_msg.data = IMAGE_CENTER_X
        center_setpoint_pub.publish(center_msg)

        # Start PID controller for centering the red blob
        centering_pid_cmd = (
            "ros2 run pid_controller pid_controller_standalone "
            "--ros-args "
            f"-p p_gain:={CENTERING_P} "
            f"-p i_gain:={CENTERING_I} "
            f"-p d_gain:={CENTERING_D} "
            f"-p output_min:={CENTERING_OUTPUT_MIN} "
            f"-p output_max:={CENTERING_OUTPUT_MAX} "
            f"-p state_topic:={self.red_center_state_topic} "
            f"-p setpoint_topic:={self.red_center_setpoint_topic} "
            f"-p control_effort_topic:={self.red_vel_y_topic} "
            "-p publish_rate:=20.0 "
            "-p auto_start:=true "
            "-r __node:=red_center_pid"
        )

        # Start PID controller process
        if not ProcessUtils.start_process(centering_pid_cmd, CENTERING_PID_PROCESS):
            yasmin.YASMIN_LOG_ERROR("Failed to start centering PID controller.")
            self._cleanup_resources()
            return ABORT

        yasmin.YASMIN_LOG_INFO("Red centering PID controller started successfully.")
        sleep(1)  # Give controller time to initialize

        # Subscribe to red line info
        self.red_line_info_sub = self.node.create_subscription(
            LineInfo,
            "/line_state/red",
            self.red_line_info_callback,
            10,
        )

        # Subscribe to control effort from PID controller
        self.control_effort_y_sub = self.node.create_subscription(
            Float64, self.red_vel_y_topic, self.control_effort_y_callback, 10
        )

        start_time = time.time()
        timeout = 30

        # Main control loop
        while time.time() - start_time < timeout:
            # Use the PID controller output for y velocity
            # Note that we negate the output since we want to move in the opposite direction of the error
            blackboard.mavdrone.offboard_velocity(
                linear_x=0.0,
                linear_y=-self.current_y_velocity,  # Negate to move toward the center
                linear_z=0.05,  # Small upward velocity to maintain altitude
                angular_z=0.0,
            )

            rclpy.spin_once(self.node, timeout_sec=0.05)

            if self.centering_confirmations >= CENTERING_CONFIRMATIONS:
                yasmin.YASMIN_LOG_INFO("Red blob centered.")
                self._cleanup_resources()
                return SUCCEED

        yasmin.YASMIN_LOG_ERROR("Failed to center on red blob.")
        self._cleanup_resources()
        return ABORT

    def _cleanup_resources(self):
        """Clean up resources"""
        yasmin.YASMIN_LOG_INFO("Cleaning up CenterRedBlob resources...")

        if self.red_line_info_sub:
            self.node.destroy_subscription(self.red_line_info_sub)
            self.red_line_info_sub = None

        if self.control_effort_y_sub:
            self.node.destroy_subscription(self.control_effort_y_sub)
            self.control_effort_y_sub = None

        # Kill the PID controller process
        ProcessUtils.kill_process(CENTERING_PID_PROCESS)
        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)

        yasmin.YASMIN_LOG_INFO("CenterRedBlob cleanup completed")
