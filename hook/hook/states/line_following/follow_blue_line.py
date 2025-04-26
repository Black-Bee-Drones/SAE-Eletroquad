import rclpy

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from mirela_sdk.utils.process import ProcessUtils
from mirela_interfaces.msg import LineInfo
from std_msgs.msg import Bool, Float64

from time import sleep
import time

from hook.states.constants import (
    FORWARD_SPEED,
    MIN_RED_AREA_CONFIRMATIONS,
    LINE_DETECT_NODE_NAME,
    CENTER_PID_PROCESS,
    ANGLE_PID_PROCESS,
    IMAGE_CENTER_X,
    CENTER_P,
    CENTER_I,
    CENTER_D,
    CENTER_OUTPUT_MIN,
    CENTER_OUTPUT_MAX,
    ANGLE_P,
    ANGLE_I,
    ANGLE_D,
    ANGLE_OUTPUT_MIN,
    ANGLE_OUTPUT_MAX,
)


class FollowBlueLineWithRedDetection(State):
    """Follows the blue line using PID and checks for red hose detection in parallel."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "red_detected"])
        self.mavdrone = None
        self.red_area_confirmations = 0
        self.red_detected = False
        self.blue_line_info_sub = None
        self.red_line_info_sub = None
        self.red_detected_sub = None
        self.control_effort_y_sub = None
        self.control_effort_angular_z_sub = None
        self.current_blue_center_x = None
        self.current_blue_angle = None
        self.current_red_center_x = None
        self.current_y_velocity = 0.0
        self.current_angular_z = 0.0
        self.running = True

        # Define topic names for improved organization
        self.blue_center_state_topic = "/line_state/blue/center_x"
        self.blue_angle_state_topic = "/line_state/blue/angle"
        self.blue_center_setpoint_topic = "/blue/center_setpoint"
        self.blue_angle_setpoint_topic = "/blue/angle_setpoint"
        self.vel_y_topic = "/blue/velocity_y"
        self.angular_z_topic = "/blue/angular_velocity_z"

    def blue_line_info_callback(self, msg: LineInfo):
        self.current_blue_center_x = msg.center_x
        self.current_blue_angle = msg.angle

    def red_line_info_callback(self, msg: LineInfo):
        self.current_red_center_x = msg.center_x
        # We track the center_x but can also use other properties if needed

    def red_detect_callback(self, msg: Bool):
        if msg.data:
            self.red_area_confirmations += 1
        else:
            self.red_area_confirmations = 0  # Reset count if detection lost

        # Check if we've detected the red line consistently
        if self.red_area_confirmations >= MIN_RED_AREA_CONFIRMATIONS:
            self.red_detected = True
            yasmin.YASMIN_LOG_INFO(
                f"Red hose detection confirmed after {self.red_area_confirmations} detections"
            )

    def control_effort_y_callback(self, msg: Float64):
        self.current_y_velocity = msg.data

    def control_effort_angular_z_callback(self, msg: Float64):
        self.current_angular_z = msg.data

    def execute(self, blackboard: Blackboard):
        if not "mavdrone" in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in FollowBlueLineWithRedDetection state.")
            return ABORT
        
        self.mavdrone = blackboard["mavdrone"]

        yasmin.YASMIN_LOG_INFO("Following blue line with red detection...")
        self.red_area_confirmations = 0
        self.red_detected = False
        self.current_blue_center_x = None
        self.current_blue_angle = None
        self.current_red_center_x = None
        self.current_y_velocity = 0.0
        self.current_angular_z = 0.0
        self.running = True

        # Clean up any existing processes first
        self._cleanup()

        # Start line detection node with both blue and red detection
        line_detection_cmd = (
            "ros2 run mirela_sdk line_detection_node "
            "--ros-args "
            "-p line_colors:=blue,red "
            "-p show_visualization:=True "
            "-p image_source:=webcam "
            "-p visualization_name:='Line Following'"
        )

        if not ProcessUtils.start_process(line_detection_cmd, LINE_DETECT_NODE_NAME):
            yasmin.YASMIN_LOG_ERROR("Failed to start multi-color line detection node.")
            return ABORT

        yasmin.YASMIN_LOG_INFO("Line detection node started successfully.")
        # Give node time to start
        sleep(2)

        # Initialize setpoint publishers
        center_setpoint_pub = self.node.create_publisher(
            Float64, self.blue_center_setpoint_topic, 10
        )
        angle_setpoint_pub = self.node.create_publisher(
            Float64, self.blue_angle_setpoint_topic, 10
        )

        # Publish initial setpoints
        center_msg = Float64()
        center_msg.data = IMAGE_CENTER_X
        center_setpoint_pub.publish(center_msg)

        angle_msg = Float64()
        angle_msg.data = 0.0  # Target angle is 0 (straight)
        angle_setpoint_pub.publish(angle_msg)

        # Start PID controller for line center (lateral position control)
        center_pid_cmd = (
            "ros2 run pid_controller pid_controller_standalone "
            "--ros-args "
            f"-p p_gain:={CENTER_P} "
            f"-p i_gain:={CENTER_I} "
            f"-p d_gain:={CENTER_D} "
            f"-p output_min:={CENTER_OUTPUT_MIN} "
            f"-p output_max:={CENTER_OUTPUT_MAX} "
            f"-p state_topic:={self.blue_center_state_topic} "
            f"-p setpoint_topic:={self.blue_center_setpoint_topic} "
            f"-p control_effort_topic:={self.vel_y_topic} "
            "-p publish_rate:=20.0 "
            "-p auto_start:=true "
            "-r __node:=blue_center_pid"
        )

        # Start PID controller for line angle (heading control)
        angle_pid_cmd = (
            "ros2 run pid_controller pid_controller_standalone "
            "--ros-args "
            f"-p p_gain:={ANGLE_P} "
            f"-p i_gain:={ANGLE_I} "
            f"-p d_gain:={ANGLE_D} "
            f"-p output_min:={ANGLE_OUTPUT_MIN} "
            f"-p output_max:={ANGLE_OUTPUT_MAX} "
            f"-p state_topic:={self.blue_angle_state_topic} "
            f"-p setpoint_topic:={self.blue_angle_setpoint_topic} "
            f"-p control_effort_topic:={self.angular_z_topic} "
            "-p publish_rate:=20.0 "
            "-p auto_start:=true "
            "-r __node:=blue_angle_pid"
        )

        # Start PID controller processes
        if not ProcessUtils.start_process(center_pid_cmd, CENTER_PID_PROCESS):
            yasmin.YASMIN_LOG_ERROR("Failed to start center PID controller.")
            self._cleanup()
            return ABORT

        if not ProcessUtils.start_process(angle_pid_cmd, ANGLE_PID_PROCESS):
            yasmin.YASMIN_LOG_ERROR("Failed to start angle PID controller.")
            self._cleanup()
            return ABORT

        yasmin.YASMIN_LOG_INFO("PID controllers started successfully.")
        # Give controllers time to initialize
        sleep(1)

        # Subscribe to line info for detection
        self.blue_line_info_sub = self.node.create_subscription(
            LineInfo,
            "/line_state/blue",
            self.blue_line_info_callback,
            10,
        )

        # Subscribe to red line info and detection status
        self.red_line_info_sub = self.node.create_subscription(
            LineInfo,
            "/line_state/red",
            self.red_line_info_callback,
            10,
        )

        self.red_detected_sub = self.node.create_subscription(
            Bool,
            "/line_detect/red",
            self.red_detect_callback,
            10,
        )

        # Subscribe to control efforts from PID controllers
        self.control_effort_y_sub = self.node.create_subscription(
            Float64, self.vel_y_topic, self.control_effort_y_callback, 10
        )

        self.control_effort_angular_z_sub = self.node.create_subscription(
            Float64, self.angular_z_topic, self.control_effort_angular_z_callback, 10
        )

        start_time = time.time()
        timeout = 60  # seconds

        # Main control loop
        while time.time() - start_time < timeout and not self.red_detected:
            # Use the PID controller outputs for velocity commands
            self.mavdrone.offboard_velocity(
                linear_x=FORWARD_SPEED,
                linear_y=self.current_y_velocity,
                linear_z=0.0,
                angular_z=self.current_angular_z,
            )

            rclpy.spin_once(self.node, timeout_sec=0.05)

            if self.red_detected:
                yasmin.YASMIN_LOG_INFO("Red hose detected!")
                self._cleanup()
                return "red_detected"

        self._cleanup()

        if self.red_detected:
            return "red_detected"
        else:
            yasmin.YASMIN_LOG_ERROR("Follow blue line timed out.")
            return ABORT

    def _cleanup(self):
        """Clean up all subscribers and processes to avoid resource leaks"""
        yasmin.YASMIN_LOG_INFO("Cleaning up resources...")

        # Destroy all subscribers to avoid resource leaks
        if self.blue_line_info_sub:
            self.node.destroy_subscription(self.blue_line_info_sub)
            self.blue_line_info_sub = None

        if self.red_line_info_sub:
            self.node.destroy_subscription(self.red_line_info_sub)
            self.red_line_info_sub = None

        if self.red_detected_sub:
            self.node.destroy_subscription(self.red_detected_sub)
            self.red_detected_sub = None

        if self.control_effort_y_sub:
            self.node.destroy_subscription(self.control_effort_y_sub)
            self.control_effort_y_sub = None

        if self.control_effort_angular_z_sub:
            self.node.destroy_subscription(self.control_effort_angular_z_sub)
            self.control_effort_angular_z_sub = None

        # Kill the PID controller processes
        ProcessUtils.kill_process(CENTER_PID_PROCESS)
        ProcessUtils.kill_process(ANGLE_PID_PROCESS)
        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)

        yasmin.YASMIN_LOG_INFO("Cleanup completed")
