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

from hook.states.constants import (
    FORWARD_SPEED,
    MIN_RED_AREA_CONFIRMATIONS,
    LINE_DETECT_NODE_NAME,
    CENTER_PID_PROCESS,
    ANGLE_PID_PROCESS,
    IMAGE_CENTER_X,
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
)


class StartLineDetection(State):
    """Start the line detection process for both blue and red lines."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Starting multi-color line detection process...")

        # Kill any existing process with the same name first
        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)

        # Start line detection node with both blue and red detection
        line_detection_cmd = (
            "ros2 run mirela_sdk line_detection_node "
            "--ros-args "
            "-p line_colors:=blue,red "  
            "-p show_visualization:=true "  
            "-p image_source:=webcam "  
            "-p visualization_name:='Line Following'"
        )

        if not ProcessUtils.start_process(line_detection_cmd, LINE_DETECT_NODE_NAME):
            yasmin.YASMIN_LOG_ERROR("Failed to start multi-color line detection node.")
            return ABORT

        yasmin.YASMIN_LOG_INFO("Line detection node started successfully.")
        # Give node time to start
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

        # Define topic names
        self.blue_center_state_topic = "/line_state/blue/center_x"
        self.blue_angle_state_topic = "/line_state/blue/angle"
        self.blue_center_setpoint_topic = "/blue/center_setpoint"
        self.blue_angle_setpoint_topic = "/blue/angle_setpoint"

        # Define setpoint values
        self.center_setpoint = IMAGE_CENTER_X
        self.angle_setpoint = ANGLE_SETPOINT

    def line_info_callback(self, msg: LineInfo):
        """Callback for LineInfo messages, republishes to separate topics"""
        # Publish setpoints continuously with each state update
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
            LineInfo, "/line_state/blue", self.line_info_callback, 10
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

        # Give the republisher a moment to connect
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
        self.vel_y_topic = "/blue/velocity_y"
        self.angular_z_topic = "/blue/angular_velocity_z"

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Starting PID controllers...")

        # Get topic names from blackboard
        blue_center_state_topic = blackboard["blue_center_state_topic"]
        blue_angle_state_topic = blackboard["blue_angle_state_topic"]
        blue_center_setpoint_topic = blackboard["blue_center_setpoint_topic"]
        blue_angle_setpoint_topic = blackboard["blue_angle_setpoint_topic"]

        # Start PID controller for line center (lateral position control)
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
            "-p publish_rate:=5.0 "
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
            f"-p state_topic:={blue_angle_state_topic} "
            f"-p setpoint_topic:={blue_angle_setpoint_topic} "
            f"-p control_effort_topic:={self.angular_z_topic} "
            "-p publish_rate:=5.0 "
            "-p auto_start:=true "
            "-r __node:=blue_angle_pid"
        )

        # Start PID controller processes
        if not ProcessUtils.start_process(center_pid_cmd, CENTER_PID_PROCESS):
            yasmin.YASMIN_LOG_ERROR("Failed to start center PID controller.")
            return ABORT

        if not ProcessUtils.start_process(angle_pid_cmd, ANGLE_PID_PROCESS):
            yasmin.YASMIN_LOG_ERROR("Failed to start angle PID controller.")
            # Kill the center PID process that was started
            ProcessUtils.kill_process(CENTER_PID_PROCESS)
            return ABORT

        yasmin.YASMIN_LOG_INFO("PID controllers started successfully.")
        # Give controllers time to initialize
        sleep(1)

        # Store the topic names in blackboard for the follow state to use
        blackboard["vel_y_topic"] = self.vel_y_topic
        blackboard["angular_z_topic"] = self.angular_z_topic

        return SUCCEED


class FollowLineWithDetection(State):
    """Follow the blue line using PID and detect red objects."""

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
        self.node = YasminNode.get_instance()

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
            yasmin.YASMIN_LOG_ERROR(
                "MavDrone not available in FollowLineWithDetection state."
            )
            return ABORT

        self.mavdrone = blackboard["mavdrone"]

        # Get topic names from blackboard
        blue_center_state_topic = blackboard["blue_center_state_topic"]
        blue_angle_state_topic = blackboard["blue_angle_state_topic"]
        vel_y_topic = blackboard["vel_y_topic"]
        angular_z_topic = blackboard["angular_z_topic"]

        yasmin.YASMIN_LOG_INFO("Following blue line with red detection...")
        self.red_area_confirmations = 0
        self.red_detected = False
        self.current_blue_center_x = None
        self.current_blue_angle = None
        self.current_red_center_x = None
        self.current_y_velocity = 0.0
        self.current_angular_z = 0.0

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
            Float64, vel_y_topic, self.control_effort_y_callback, 10
        )

        self.control_effort_angular_z_sub = self.node.create_subscription(
            Float64, angular_z_topic, self.control_effort_angular_z_callback, 10
        )

        start_time = time.time()
        timeout = 600  # seconds

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
                self._cleanup_subscribers()
                return "red_detected"

        self._cleanup_subscribers()

        if self.red_detected:
            return "red_detected"
        else:
            yasmin.YASMIN_LOG_ERROR("Follow blue line timed out.")
            return ABORT

    def _cleanup_subscribers(self):
        """Clean up all subscribers to avoid resource leaks"""
        yasmin.YASMIN_LOG_INFO("Cleaning up subscribers...")

        # Destroy all subscribers
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

        yasmin.YASMIN_LOG_INFO("Subscribers cleanup completed")


class CleanupProcesses(State):
    """Clean up all processes started for line following."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Cleaning up processes...")

        # Kill the PID controller processes
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

        # Define states
        self.add_state(
            "START_LINE_DETECTION",
            StartLineDetection(),
            transitions={SUCCEED: "SETUP_REPUBLISHER", ABORT: "CLEANUP_PROCESSES"},
        )

        # Add the new republisher state
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
