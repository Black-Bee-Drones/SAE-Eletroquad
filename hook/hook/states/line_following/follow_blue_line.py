import rclpy

import yasmin
from yasmin import State, StateMachine, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from mirela_sdk.utils.process import ProcessUtils
from mirela_interfaces.msg import LineInfo
from std_msgs.msg import Bool, Float64

from time import sleep
import time

from hook.constants import (
    FORWARD_SPEED_FOLLOW_BLUE_LINE,
    MIN_RED_COUNT_CONFIRMATIONS,
    LINE_DETECT_NODE_NAME,
    CENTER_PID_PROCESS,
    ANGLE_PID_PROCESS,
    IMAGE_CENTER_X,
    IMAGE_CENTER_Y,
    ANGLE_SETPOINT,
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
    LINE_DETECTION_BLUE_COLOR_NAME,
    LINE_DETECTION_RED_COLOR_NAME,
    LINE_DETECTION_IMAGE_SOURCE,
    LINE_DETECTION_SHOW_VISUALIZATION,
    LINE_DETECTION_LINE_FOLLOWING_TITLE,
    LINE_DETECTION_BLUE_SPACE,
    LINE_DETECTION_RED_SPACE,
    LINE_DETECTION_METHOD,
)


class StartLineDetection(State):
    """Start the line detection process for both blue and red lines."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Starting multi-color line detection process...")

        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)

        line_detection_cmd = (
            "ros2 run mirela_sdk line_detection_node "
            "--ros-args "
            f"-p line_colors:={LINE_DETECTION_BLUE_COLOR_NAME},{LINE_DETECTION_RED_COLOR_NAME} "
            f"-p spaces:={LINE_DETECTION_BLUE_SPACE},{LINE_DETECTION_RED_SPACE} "
            f"-p show_visualization:={LINE_DETECTION_SHOW_VISUALIZATION} "
            f"-p image_source:={LINE_DETECTION_IMAGE_SOURCE} "
            f"-p method:={LINE_DETECTION_METHOD} "
            f"-p visualization_name:='{LINE_DETECTION_LINE_FOLLOWING_TITLE}'"
        )

        if not ProcessUtils.start_process(line_detection_cmd, LINE_DETECT_NODE_NAME):
            yasmin.YASMIN_LOG_ERROR("Failed to start multi-color line detection node.")
            return ABORT

        yasmin.YASMIN_LOG_INFO("Line detection node started successfully.")
        sleep(2)

        return SUCCEED


class SetupLineStateRepublisher(State):
    """
    Sets up publishers and subscribers to republish LineInfo values to separate topics.
    This state creates the bridge between /line_state/blue (LineInfo) and the
    separate center_x and angle topics needed by the PID controllers.
    Also publishes setpoint values continuously alongside the state.
    """

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()
        self.blue_line_info_sub = None
        self.blue_center_pub = None
        self.blue_angle_pub = None
        self.center_setpoint_pub = None
        self.angle_setpoint_pub = None
        self.should_continue = True

        self.blue_center_state_topic = (
            f"/line_state/{LINE_DETECTION_BLUE_COLOR_NAME}/center_x"
        )
        self.blue_angle_state_topic = (
            f"/line_state/{LINE_DETECTION_BLUE_COLOR_NAME}/angle"
        )
        self.blue_center_setpoint_topic = (
            f"/{LINE_DETECTION_BLUE_COLOR_NAME}/center_setpoint"
        )
        self.blue_angle_setpoint_topic = (
            f"/{LINE_DETECTION_BLUE_COLOR_NAME}/angle_setpoint"
        )

        # Define setpoint values
        self.center_setpoint = IMAGE_CENTER_X
        self.angle_setpoint = ANGLE_SETPOINT

    def line_info_callback(self, msg: LineInfo):
        """Callback for LineInfo messages, republishes to separate topics"""

        self.angle_setpoint_pub.publish(Float64(data=self.angle_setpoint))
        self.center_setpoint_pub.publish(Float64(data=self.center_setpoint))

        # Publish to the separate topics (states)
        self.blue_center_pub.publish(Float64(data=msg.center_x))
        self.blue_angle_pub.publish(Float64(data=msg.angle))

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO(
            "Setting up line state republishers and setpoint publishers..."
        )

        # Create publishers for the separate topics (states)
        self.blue_center_pub = self.node.create_publisher(
            Float64, self.blue_center_state_topic, 10
        )
        self.blue_angle_pub = self.node.create_publisher(
            Float64, self.blue_angle_state_topic, 10
        )

        # Create publishers for setpoints
        self.center_setpoint_pub = self.node.create_publisher(
            Float64, self.blue_center_setpoint_topic, 10
        )
        self.angle_setpoint_pub = self.node.create_publisher(
            Float64, self.blue_angle_setpoint_topic, 10
        )

        # Subscribe to the original LineInfo topic
        self.blue_line_info_sub = self.node.create_subscription(
            LineInfo,
            f"/line_state/{LINE_DETECTION_BLUE_COLOR_NAME}",
            self.line_info_callback,
            10,
        )

        # Store the topic names in blackboard for later states to use
        blackboard["blue_center_state_topic"] = self.blue_center_state_topic
        blackboard["blue_angle_state_topic"] = self.blue_angle_state_topic
        blackboard["blue_center_setpoint_topic"] = self.blue_center_setpoint_topic
        blackboard["blue_angle_setpoint_topic"] = self.blue_angle_setpoint_topic

        # Store subscribers in blackboard for cleanup later
        if "subscribers_to_clean" not in blackboard:
            blackboard["subscribers_to_clean"] = []

        blackboard["subscribers_to_clean"].append(
            {"node": self.node, "subscription": self.blue_line_info_sub}
        )

        # Publish initial setpoints before any callbacks
        center_setpoint_msg = Float64()
        center_setpoint_msg.data = self.center_setpoint
        self.center_setpoint_pub.publish(center_setpoint_msg)

        angle_setpoint_msg = Float64()
        angle_setpoint_msg.data = self.angle_setpoint
        self.angle_setpoint_pub.publish(angle_setpoint_msg)

        sleep(1)

        yasmin.YASMIN_LOG_INFO(
            f"Line state republishers and setpoint publishers set up successfully.\n"
            f"State topics: {self.blue_center_state_topic}, {self.blue_angle_state_topic}\n"
            f"Setpoint topics: {self.blue_center_setpoint_topic}, {self.blue_angle_setpoint_topic}"
        )

        return SUCCEED


class StartPIDControllers(State):
    """Start the PID controllers for line following."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

        # Only define control effort topics, others come from blackboard
        self.vel_y_topic = f"/{LINE_DETECTION_BLUE_COLOR_NAME}/velocity_y"
        self.angular_z_topic = f"/{LINE_DETECTION_BLUE_COLOR_NAME}/angular_velocity_z"

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Starting PID controllers...")

        # Get topic names from blackboard
        blue_center_state_topic = blackboard["blue_center_state_topic"]
        blue_angle_state_topic = blackboard["blue_angle_state_topic"]
        blue_center_setpoint_topic = blackboard["blue_center_setpoint_topic"]
        blue_angle_setpoint_topic = blackboard["blue_angle_setpoint_topic"]

        center_pid_cmd = (
            "ros2 run pid_controller pid_controller_standalone "
            "--ros-args "
            f"-p p_gain:={CENTER_P} "
            f"-p i_gain:={CENTER_I} "
            f"-p d_gain:={CENTER_D} "
            f"-p output_min:={CENTER_OUTPUT_MIN} "
            f"-p output_max:={CENTER_OUTPUT_MAX} "
            f"-p state_topic:={blue_center_state_topic} "
            f"-p setpoint_topic:={blue_center_setpoint_topic} "
            f"-p control_effort_topic:={self.vel_y_topic} "
            "-p publish_rate:=10.0 "
            "-p auto_start:=true "
            f"-r __node:={LINE_DETECTION_BLUE_COLOR_NAME}_center_pid"
        )

        angle_pid_cmd = (
            "ros2 run pid_controller pid_controller_standalone "
            "--ros-args "
            f"-p p_gain:={ANGLE_P} "
            f"-p i_gain:={ANGLE_I} "
            f"-p d_gain:={ANGLE_D} "
            f"-p output_min:={ANGLE_OUTPUT_MIN} "
            f"-p output_max:={ANGLE_OUTPUT_MAX} "
            f"-p state_topic:={blue_angle_state_topic} "
            f"-p setpoint_topic:={blue_angle_setpoint_topic} "
            f"-p control_effort_topic:={self.angular_z_topic} "
            "-p publish_rate:=10.0 "
            "-p auto_start:=true "
            f"-r __node:={LINE_DETECTION_BLUE_COLOR_NAME}_angle_pid"
        )

        if not ProcessUtils.start_process(center_pid_cmd, CENTER_PID_PROCESS):
            yasmin.YASMIN_LOG_ERROR("Failed to start center PID controller.")
            return ABORT

        if not ProcessUtils.start_process(angle_pid_cmd, ANGLE_PID_PROCESS):
            yasmin.YASMIN_LOG_ERROR("Failed to start angle PID controller.")
            # Kill the center PID process that was started
            return ABORT

        yasmin.YASMIN_LOG_INFO("PID controllers started successfully.")

        sleep(1)

        blackboard["vel_y_topic"] = self.vel_y_topic
        blackboard["angular_z_topic"] = self.angular_z_topic

        return SUCCEED


class FollowLineWithDetection(State):
    """Follow the blue line using PID and detect red objects."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "red_detected"])
        self.mavdrone = None
        self.red_count_confirmations = 0
        self.red_detected = False
        self.red_detected_sub = None
        self.control_effort_y_sub = None
        self.control_effort_angular_z_sub = None
        self.current_blue_center_x = None
        self.current_blue_angle = None
        self.current_red_center_x = None
        self.current_y_velocity = 0.0
        self.current_angular_z = 0.0
        self.filtered_angular_z = 0.0
        self.angular_z_buffer = []
        self.buffer_size = 4
        self.max_angular_change = 0.5
        self.update_center = False
        self.update_angle = False
        self.node = YasminNode.get_instance()

    def red_detect_callback(self, msg: LineInfo):
        self.red_count_confirmations += 1

        yasmin.YASMIN_LOG_INFO(
            f"Red hose count: {self.red_count_confirmations} detections"
        )

        if self.red_count_confirmations >= MIN_RED_COUNT_CONFIRMATIONS:
            self.red_detected = True
            yasmin.YASMIN_LOG_INFO(
                f"Red hose detection confirmed after {self.red_count_confirmations} detections"
            )

    def control_effort_y_callback(self, msg: Float64):
        self.current_y_velocity = msg.data
        self.update_center = True

    def control_effort_angular_z_callback(self, msg: Float64):
        raw_angular_z = msg.data

        if len(self.angular_z_buffer) == 0:
            self.filtered_angular_z = raw_angular_z
        else:
            change = abs(raw_angular_z - self.filtered_angular_z)
            if change > self.max_angular_change and len(self.angular_z_buffer) >= 2:
                recent_avg = sum(self.angular_z_buffer[-2:]) / 2
                if abs(raw_angular_z - recent_avg) > self.max_angular_change:
                    self.filtered_angular_z = recent_avg
                else:
                    self.filtered_angular_z = raw_angular_z
            else:
                self.filtered_angular_z = raw_angular_z

        self.angular_z_buffer.append(self.filtered_angular_z)
        if len(self.angular_z_buffer) > self.buffer_size:
            self.angular_z_buffer.pop(0)

        self.current_angular_z = self.filtered_angular_z
        self.update_angle = True

    def execute(self, blackboard: Blackboard):
        if not "mavdrone" in blackboard:
            yasmin.YASMIN_LOG_ERROR(
                "MavDrone not available in FollowLineWithDetection state."
            )
            return ABORT

        self.mavdrone = blackboard["mavdrone"]

        vel_y_topic = blackboard["vel_y_topic"]
        angular_z_topic = blackboard["angular_z_topic"]

        yasmin.YASMIN_LOG_INFO("Following blue line with red detection...")
        self.red_count_confirmations = 0
        self.red_detected = False
        self.current_blue_center_x = None
        self.current_blue_angle = None
        self.current_red_center_x = None
        self.current_y_velocity = 0.0
        self.current_angular_z = 0.0
        self.filtered_angular_z = 0.0
        self.angular_z_buffer = []

        self.red_detected_sub = self.node.create_subscription(
            Bool,
            f"/line_state/{LINE_DETECTION_RED_COLOR_NAME}",
            self.red_detect_callback,
            10,
        )

        self.control_effort_y_sub = self.node.create_subscription(
            Float64, vel_y_topic, self.control_effort_y_callback, 10
        )

        self.control_effort_angular_z_sub = self.node.create_subscription(
            Float64, angular_z_topic, self.control_effort_angular_z_callback, 10
        )

        start_time = time.time()
        timeout = 120

        while time.time() - start_time < timeout and not self.red_detected:
            if self.update_center or self.update_angle:
                self.mavdrone.offboard_velocity(
                    linear_x=FORWARD_SPEED_FOLLOW_BLUE_LINE,
                    linear_y=self.current_y_velocity,
                    linear_z=0.0,
                    angular_z=self.current_angular_z,
                )

            self.update_center = False
            self.update_angle = False

            rclpy.spin_once(self.node)

            if self.red_detected:
                yasmin.YASMIN_LOG_INFO("Red hose detected!")
                self.mavdrone.offboard_velocity(
                    linear_x=0.0,
                    linear_y=0.0,
                    linear_z=0.0,
                    angular_z=0.0,
                )
                self._cleanup_subscribers()
                return "red_detected"

        self._cleanup_subscribers()

        self.mavdrone.offboard_velocity(
            linear_x=0.0,
            linear_y=0.0,
            linear_z=0.0,
            angular_z=0.0,
        )

        if self.red_detected:
            return "red_detected"
        else:
            yasmin.YASMIN_LOG_ERROR("Follow blue line timed out.")
            return ABORT

    def _cleanup_subscribers(self):
        """Clean up all subscribers to avoid resource leaks"""
        yasmin.YASMIN_LOG_INFO("Cleaning up subscribers...")

        if self.red_detected_sub:
            self.node.destroy_subscription(self.red_detected_sub)
            self.red_detected_sub = None

        if self.control_effort_y_sub:
            self.node.destroy_subscription(self.control_effort_y_sub)
            self.control_effort_y_sub = None

        if self.control_effort_angular_z_sub:
            self.node.destroy_subscription(self.control_effort_angular_z_sub)
            self.control_effort_angular_z_sub = None

        yasmin.YASMIN_LOG_INFO("Subscribers cleanup completed")


class CleanupProcesses(State):
    """Clean up all processes started for line following."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Cleaning up processes...")

        ProcessUtils.kill_process(CENTER_PID_PROCESS)
        ProcessUtils.kill_process(ANGLE_PID_PROCESS)
        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)

        # Clean up any subscribers stored in the blackboard
        if "subscribers_to_clean" in blackboard:
            for sub_info in blackboard["subscribers_to_clean"]:
                node = sub_info["node"]
                subscription = sub_info["subscription"]
                node.destroy_subscription(subscription)
            blackboard["subscribers_to_clean"] = []

        yasmin.YASMIN_LOG_INFO("Process cleanup completed")
        return SUCCEED


class FollowBlueLineWithRedDetection(StateMachine):
    """StateMachine that manages blue line following and red object detection."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "red_detected"])

        self.add_state(
            "START_LINE_DETECTION",
            StartLineDetection(),
            transitions={SUCCEED: "SETUP_REPUBLISHER", ABORT: "CLEANUP_PROCESSES"},
        )

        self.add_state(
            "SETUP_REPUBLISHER",
            SetupLineStateRepublisher(),
            transitions={SUCCEED: "START_PID_CONTROLLERS", ABORT: "CLEANUP_PROCESSES"},
        )

        self.add_state(
            "START_PID_CONTROLLERS",
            StartPIDControllers(),
            transitions={SUCCEED: "FOLLOW_LINE", ABORT: "CLEANUP_PROCESSES"},
        )

        self.add_state(
            "FOLLOW_LINE",
            FollowLineWithDetection(),
            transitions={
                SUCCEED: "CLEANUP_PROCESSES",
                ABORT: "CLEANUP_PROCESSES",
                "red_detected": "CLEANUP_PROCESSES",
            },
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
