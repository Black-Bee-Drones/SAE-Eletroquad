import rclpy

import yasmin
from yasmin import State, StateMachine, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from mirela_sdk.utils.process import ProcessUtils
from mirela_interfaces.msg import LineInfo
from std_msgs.msg import Float64

from time import sleep
import time

from hook.constants import (
    CENTERING_CONFIRMATIONS,
    LINE_DETECT_NODE_NAME,
    CENTERING_PID_PROCESS,
    IMAGE_CENTER_X,
    IMAGE_CENTER_Y,
    CENTERING_P,
    CENTERING_I,
    CENTERING_D,
    CENTERING_OUTPUT_MIN,
    CENTERING_OUTPUT_MAX,
    LINE_DETECTION_RED_COLOR_NAME,
    LINE_DETECTION_RED_SPACE,
    LINE_DETECTION_IMAGE_SOURCE,
    LINE_DETECTION_SHOW_VISUALIZATION,
    LINE_DETECTION_RED_CENTERING_TITLE,
)


class StartRedLineDetection(State):
    """Start the red line detection process for centering."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Starting red line detection process for centering...")

        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)

        line_detection_cmd = (
            "ros2 run mirela_sdk line_detection_node "
            "--ros-args "
            f"-p line_colors:={LINE_DETECTION_RED_COLOR_NAME} "
            f"-p spaces:={LINE_DETECTION_RED_SPACE} "
            f"-p show_visualization:={LINE_DETECTION_SHOW_VISUALIZATION} "
            f"-p image_source:={LINE_DETECTION_IMAGE_SOURCE} "
            f"-p visualization_name:='{LINE_DETECTION_RED_CENTERING_TITLE}'"
        )

        if not ProcessUtils.start_process(line_detection_cmd, LINE_DETECT_NODE_NAME):
            yasmin.YASMIN_LOG_ERROR("Failed to start red line detection node.")
            return ABORT

        yasmin.YASMIN_LOG_INFO(
            "Line detection node started successfully for red centering."
        )
        sleep(2)

        return SUCCEED


class SetupRedLineStateRepublisher(State):
    """
    Sets up publishers and subscribers to republish the red line's center_x value to a separate topic.
    Also publishes the setpoint value continuously alongside the state.
    """

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()
        self.red_line_info_sub = None
        self.red_center_pub = None
        self.center_setpoint_pub = None
        self.should_continue = True

        self.red_center_state_topic = (
            f"/line_state/{LINE_DETECTION_RED_COLOR_NAME}/center_y"
        )
        self.red_center_setpoint_topic = (
            f"/{LINE_DETECTION_RED_COLOR_NAME}/center_setpoint"
        )
        self.center_setpoint = IMAGE_CENTER_Y

    def line_info_callback(self, msg: LineInfo):
        """Callback for LineInfo messages, republishes center_y and setpoint"""
        self.center_setpoint_pub.publish(Float64(data=self.center_setpoint))
        self.red_center_pub.publish(Float64(data=msg.center_y))

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO(
            "Setting up red line state republisher and setpoint publisher..."
        )

        # Create publisher for the center_x state
        self.red_center_pub = self.node.create_publisher(
            Float64, self.red_center_state_topic, 10
        )
        # Create publisher for setpoint
        self.center_setpoint_pub = self.node.create_publisher(
            Float64, self.red_center_setpoint_topic, 10
        )
        # Subscribe to the original LineInfo topic
        self.red_line_info_sub = self.node.create_subscription(
            LineInfo,
            f"/line_state/{LINE_DETECTION_RED_COLOR_NAME}",
            self.line_info_callback,
            10,
        )
        # Store the topic names in blackboard for later states to use
        blackboard["red_center_state_topic"] = self.red_center_state_topic
        blackboard["red_center_setpoint_topic"] = self.red_center_setpoint_topic
        # Store subscribers in blackboard for cleanup later
        if "subscribers_to_clean" not in blackboard:
            blackboard["subscribers_to_clean"] = []
        blackboard["subscribers_to_clean"].append(
            {"node": self.node, "subscription": self.red_line_info_sub}
        )
        # Publish initial setpoint before any callbacks
        center_setpoint_msg = Float64()
        center_setpoint_msg.data = self.center_setpoint
        self.center_setpoint_pub.publish(center_setpoint_msg)
        sleep(1)
        yasmin.YASMIN_LOG_INFO(
            f"Red line state republisher and setpoint publisher set up successfully.\n"
            f"State topic: {self.red_center_state_topic}\n"
            f"Setpoint topic: {self.red_center_setpoint_topic}"
        )
        return SUCCEED


class StartCenteringPID(State):
    """Start the PID controller for centering on the red blob."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

        # Define topic names
        self.red_center_state_topic = (
            f"/line_state/{LINE_DETECTION_RED_COLOR_NAME}/center_y"
        )
        self.red_center_setpoint_topic = (
            f"/{LINE_DETECTION_RED_COLOR_NAME}/center_setpoint"
        )
        self.red_vel_y_topic = f"/{LINE_DETECTION_RED_COLOR_NAME}/velocity_y"

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Starting centering PID controller...")

        ProcessUtils.kill_process(CENTERING_PID_PROCESS)

        center_setpoint_pub = self.node.create_publisher(
            Float64, self.red_center_setpoint_topic, 10
        )

        # Publish initial setpoint
        center_msg = Float64()
        center_msg.data = IMAGE_CENTER_Y
        center_setpoint_pub.publish(center_msg)

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
            f"-r __node:={LINE_DETECTION_RED_COLOR_NAME}_center_pid"
        )

        if not ProcessUtils.start_process(centering_pid_cmd, CENTERING_PID_PROCESS):
            yasmin.YASMIN_LOG_ERROR("Failed to start centering PID controller.")
            return ABORT

        yasmin.YASMIN_LOG_INFO("Red centering PID controller started successfully.")
        sleep(1)

        blackboard["red_center_state_topic"] = self.red_center_state_topic
        blackboard["red_center_setpoint_topic"] = self.red_center_setpoint_topic
        blackboard["red_vel_y_topic"] = self.red_vel_y_topic

        return SUCCEED


class PerformCentering(State):
    """Center the drone over the red blob using PID control."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.red_line_info_sub = None
        self.control_effort_y_sub = None
        self.centering_confirmations = 0
        self.current_red_center_x = None
        self.current_y_velocity = 0.0
        self.last_error_x = 0
        self.update_control_effort = True
        self.node = YasminNode.get_instance()

    def control_effort_y_callback(self, msg: Float64):
        self.current_y_velocity = msg.data
        self.update_control_effort = True
        if self.current_y_velocity <= 0.05:
            self.centering_confirmations += 1
        else:
            self.centering_confirmations -= 0

    def execute(self, blackboard: Blackboard):
        if not "mavdrone" in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in PerformCentering state.")
            return ABORT

        mavdrone = blackboard["mavdrone"]

        red_center_state_topic = blackboard.get(
            "red_center_state_topic",
            f"/line_state/{LINE_DETECTION_RED_COLOR_NAME}/center_y",
        )
        red_vel_y_topic = blackboard.get(
            "red_vel_y_topic", f"/{LINE_DETECTION_RED_COLOR_NAME}/velocity_y"
        )

        yasmin.YASMIN_LOG_INFO("Centering on red blob...")
        self.centering_confirmations = 0
        self.last_error_x = 0
        self.current_y_velocity = 0.0

        self.control_effort_y_sub = self.node.create_subscription(
            Float64, red_vel_y_topic, self.control_effort_y_callback, 10
        )

        start_time = time.time()
        timeout = 30

        # Main control loop
        while time.time() - start_time < timeout:
            if self.update_control_effort:
                mavdrone.offboard_velocity(
                    linear_x=self.current_y_velocity,
                    linear_y=0.0,
                    linear_z=0.0,
                    angular_z=0.0,
                )
            self.update_control_effort = False

            rclpy.spin_once(self.node)

            if self.centering_confirmations >= CENTERING_CONFIRMATIONS:
                mavdrone.offboard_velocity_timer(
                    linear_x=0.0,
                    linear_y=0.0,
                    linear_z=0.0,
                    angular_z=0.0,
                    time=1.0,
                )
                yasmin.YASMIN_LOG_INFO("Red blob centered.")
                self._cleanup_subscribers()
                return SUCCEED

        yasmin.YASMIN_LOG_ERROR("Failed to center on red blob (timeout).")
        self._cleanup_subscribers()
        return ABORT

    def _cleanup_subscribers(self):
        """Clean up subscribers"""
        yasmin.YASMIN_LOG_INFO("Cleaning up PerformCentering subscribers...")

        if self.red_line_info_sub:
            self.node.destroy_subscription(self.red_line_info_sub)
            self.red_line_info_sub = None

        if self.control_effort_y_sub:
            self.node.destroy_subscription(self.control_effort_y_sub)
            self.control_effort_y_sub = None

        yasmin.YASMIN_LOG_INFO("PerformCentering subscribers cleanup completed")


class CleanupProcesses(State):
    """Clean up all processes started for centering."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Cleaning up centering processes...")

        ProcessUtils.kill_process(CENTERING_PID_PROCESS)
        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)

        yasmin.YASMIN_LOG_INFO("Centering process cleanup completed")
        return SUCCEED


class CenterRedBlob(StateMachine):
    """StateMachine that manages centering on the red blob."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        # Define states
        self.add_state(
            "START_RED_LINE_DETECTION",
            StartRedLineDetection(),
            transitions={SUCCEED: "START_CENTERING_PID", ABORT: "CLEANUP_PROCESSES"},
        )

        self.add_state(
            "SETUP_RED_LINE_STATE_REPUBLISHER",
            SetupRedLineStateRepublisher(),
            transitions={SUCCEED: "START_CENTERING_PID", ABORT: "CLEANUP_PROCESSES"},
        )

        self.add_state(
            "START_CENTERING_PID",
            StartCenteringPID(),
            transitions={SUCCEED: "PERFORM_CENTERING", ABORT: "CLEANUP_PROCESSES"},
        )

        self.add_state(
            "PERFORM_CENTERING",
            PerformCentering(),
            transitions={SUCCEED: "CLEANUP_PROCESSES", ABORT: "CLEANUP_PROCESSES"},
        )

        self.add_state(
            "CLEANUP_PROCESSES", CleanupProcesses(), transitions={SUCCEED: SUCCEED}
        )

    def execute(self, blackboard):
        """Execute the state machine with outcome tracking."""
        # Execute the standard StateMachine execution
        outcome = super().execute(blackboard)
        return outcome
