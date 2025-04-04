import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from std_msgs.msg import Float32, Bool

import sys
import time
from yasmin import Blackboard
from yasmin_ros import MonitorState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

from mirela_interfaces.msg import LineInfo
from mirela_sdk.image_processing.line.line_detection_node import LineDetectionNode

from mirela_sdk.control.mavros.mavros_api import MavDrone


class FollowLineState(MonitorState):
    # Default values (consider moving to a config file or launch parameters)
    DEFAULT_FORWARD_SPEED = 0.1
    DEFAULT_KP_CENTER_X = -0.0006  # P gain for lateral velocity based on center_x error
    DEFAULT_KI_CENTER_X = -0.00005  # I gain for lateral velocity
    DEFAULT_KD_CENTER_X = -0.0001  # D gain for lateral velocity
    DEFAULT_KP_ANGULAR_Z = 0.009  # P gain for yaw rate based on angle error
    DEFAULT_KI_ANGULAR_Z = 0.0005  # I gain for yaw rate
    DEFAULT_KD_ANGULAR_Z = 0.001  # D gain for yaw rate
    DEFAULT_MAX_VEL_Y = 0.3  # Max lateral velocity
    DEFAULT_MAX_ANGULAR_Z = 0.5  # Max yaw rate
    DEFAULT_INTEGRAL_LIMIT_Y = 0.1  # Integral saturation limit for vel_y
    DEFAULT_INTEGRAL_LIMIT_Z = 0.1  # Integral saturation limit for angular_z

    def __init__(self, drone: MavDrone) -> None:
        super().__init__(
            msg_type=LineInfo,
            topic_name=LineDetectionNode.LINE_STATE_TOPIC,
            outcomes=["adjusting"],
            monitor_handler=self._state_callback,
            timeout=5,  # Consider adjusting timeout based on expected detection rate
        )

        self.drone = drone
        # Use the drone's node for parameters and logging
        self._node = self.drone.node
        self._logger = self._node.get_logger()

        # --- Declare ROS Parameters for PID Gains ---
        self._node.declare_parameter(
            "follow_line.forward_speed", self.DEFAULT_FORWARD_SPEED
        )
        self._node.declare_parameter(
            "follow_line.kp_center_x", self.DEFAULT_KP_CENTER_X
        )
        self._node.declare_parameter(
            "follow_line.ki_center_x", self.DEFAULT_KI_CENTER_X
        )
        self._node.declare_parameter(
            "follow_line.kd_center_x", self.DEFAULT_KD_CENTER_X
        )
        self._node.declare_parameter(
            "follow_line.kp_angular_z", self.DEFAULT_KP_ANGULAR_Z
        )
        self._node.declare_parameter(
            "follow_line.ki_angular_z", self.DEFAULT_KI_ANGULAR_Z
        )
        self._node.declare_parameter(
            "follow_line.kd_angular_z", self.DEFAULT_KD_ANGULAR_Z
        )
        self._node.declare_parameter("follow_line.max_vel_y", self.DEFAULT_MAX_VEL_Y)
        self._node.declare_parameter(
            "follow_line.max_angular_z", self.DEFAULT_MAX_ANGULAR_Z
        )
        self._node.declare_parameter(
            "follow_line.integral_limit_y", self.DEFAULT_INTEGRAL_LIMIT_Y
        )
        self._node.declare_parameter(
            "follow_line.integral_limit_z", self.DEFAULT_INTEGRAL_LIMIT_Z
        )

        # Setpoint values
        self.setpoint_center_x = LineDetectionNode.IMG_SIZE[0] / 2
        self.setpoint_angle = 0.0

        # --- PID State Variables ---
        self.integral_center_x: float = 0.0
        self.prev_error_center_x: float = 0.0
        self.integral_angle: float = 0.0
        self.prev_error_angle: float = 0.0
        self.last_time = self._node.get_clock().now()

        # --- Visualization/Logging Publishers (Optional) ---
        # Example: Publish error for rqt_plot
        self.error_center_x_pub = self._node.create_publisher(
            Float32, "follow_line/error_center_x", 10
        )
        self.error_angle_pub = self._node.create_publisher(
            Float32, "follow_line/error_angle", 10
        )
        self.vel_y_pub = self._node.create_publisher(
            Float32, "follow_line/vel_y", 10
        )
        self.angular_z_pub = self._node.create_publisher(
            Float32, "follow_line/angular_z", 10
        )

        self.is_activate = False
        self._logger.info("Follow Line PID Controller State initialized.")

    def _get_params(self):
        """Retrieve current PID parameters from ROS parameter server."""
        params = {}
        params["forward_speed"] = self._node.get_parameter(
            "follow_line.forward_speed"
        ).value
        params["kp_center_x"] = self._node.get_parameter(
            "follow_line.kp_center_x"
        ).value
        params["ki_center_x"] = self._node.get_parameter(
            "follow_line.ki_center_x"
        ).value
        params["kd_center_x"] = self._node.get_parameter(
            "follow_line.kd_center_x"
        ).value
        params["kp_angular_z"] = self._node.get_parameter(
            "follow_line.kp_angular_z"
        ).value
        params["ki_angular_z"] = self._node.get_parameter(
            "follow_line.ki_angular_z"
        ).value
        params["kd_angular_z"] = self._node.get_parameter(
            "follow_line.kd_angular_z"
        ).value
        params["max_vel_y"] = self._node.get_parameter("follow_line.max_vel_y").value
        params["max_angular_z"] = self._node.get_parameter(
            "follow_line.max_angular_z"
        ).value
        params["integral_limit_y"] = self._node.get_parameter(
            "follow_line.integral_limit_y"
        ).value
        params["integral_limit_z"] = self._node.get_parameter(
            "follow_line.integral_limit_z"
        ).value
        return params

    def _reset_pid(self):
        """Resets the PID controller's integral and derivative states."""
        self.integral_center_x = 0.0
        self.prev_error_center_x = 0.0
        self.integral_angle = 0.0
        self.prev_error_angle = 0.0
        self.last_time = self._node.get_clock().now()
        self._logger.info("PID controller state reset.")

    def _state_callback(self, blackboard: Blackboard, msg: LineInfo) -> str:
        """
        Callback function for the line state message.
        Applies PID control based on the received line info.
        """
        # Ignore potentially invalid initial values
        if msg.center_x == 0.0 and msg.angle == 0.0:
            self._logger.warn(
                "Received potentially invalid LineInfo (0, 0), skipping control step."
            )
            return "adjusting"  # Stay in adjusting state

        return self.apply_control(msg.center_x, msg.angle)

    def apply_control(self, state_center_x: float, state_angle: float) -> str:
        """
        Apply PID control values (linear y, angular z) to the drone.
        """
        current_time = self._node.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Duration in seconds

        # Avoid division by zero or huge derivatives if dt is too small or first run
        if dt <= 0.001:
            dt = 0.001  # Set a minimum dt or handle appropriately

        params = self._get_params()

        # --- Lateral Control (center_x -> linear_y) ---
        error_center_x = self.setpoint_center_x - state_center_x
        self.integral_center_x += error_center_x * dt
        # Anti-windup: Clamp integral term
        self.integral_center_x = max(
            min(self.integral_center_x, params["integral_limit_y"]),
            -params["integral_limit_y"],
        )

        derivative_center_x = (error_center_x - self.prev_error_center_x) / dt

        p_term_y = params["kp_center_x"] * error_center_x
        i_term_y = params["ki_center_x"] * self.integral_center_x
        d_term_y = params["kd_center_x"] * derivative_center_x
        vel_y = p_term_y + i_term_y + d_term_y

        # Clamp output velocity
        vel_y = max(min(vel_y, params["max_vel_y"]), -params["max_vel_y"])

        # --- Yaw Control (angle -> angular_z) ---
        error_angle = self.setpoint_angle - state_angle
        # Handle angle wrap-around if necessary (e.g., if angle can jump from -89 to 89)
        # For now, assume angle is continuous around 0 from the detector

        self.integral_angle += error_angle * dt
        # Anti-windup: Clamp integral term
        self.integral_angle = max(
            min(self.integral_angle, params["integral_limit_z"]),
            -params["integral_limit_z"],
        )

        derivative_angle = (error_angle - self.prev_error_angle) / dt

        p_term_z = params["kp_angular_z"] * error_angle
        i_term_z = params["ki_angular_z"] * self.integral_angle
        d_term_z = params["kd_angular_z"] * derivative_angle
        angular_z = p_term_z + i_term_z + d_term_z

        # Clamp output angular velocity
        angular_z = max(
            min(angular_z, params["max_angular_z"]), -params["max_angular_z"]
        )

        # --- Update PID State ---
        self.prev_error_center_x = error_center_x
        self.prev_error_angle = error_angle
        self.last_time = current_time

        # --- Logging and Publishing ---
        self._logger.debug(
            f"dt: {dt:.4f}, "
            f"ErrX: {error_center_x:.2f}, IntX: {self.integral_center_x:.2f}, DerX: {derivative_center_x:.2f} -> VelY: {vel_y:.3f} (P:{p_term_y:.3f}, I:{i_term_y:.3f}, D:{d_term_y:.3f}) | "
            f"ErrA: {error_angle:.2f}, IntA: {self.integral_angle:.2f}, DerA: {derivative_angle:.2f} -> AngZ: {angular_z:.3f} (P:{p_term_z:.3f}, I:{i_term_z:.3f}, D:{d_term_z:.3f})"
        )

        # Publish debug values
        self.error_center_x_pub.publish(Float32(data=error_center_x))
        self.error_angle_pub.publish(Float32(data=error_angle))
        self.vel_y_pub.publish(Float32(data=vel_y))
        self.angular_z_pub.publish(Float32(data=angular_z))

        # --- Send Command to Drone ---
        self.drone.offboard_velocity(
            linear_x=params["forward_speed"],
            linear_y=vel_y,
            linear_z=0.0,
            angular_z=angular_z,
        )

        # Removed time.sleep(0.2) - control loop should run based on message arrival rate

        return "adjusting"

    def finish(self) -> None:
        """Called when the state machine transitions away from this state."""
        # Ensure drone stops movement when this state finishes
        self.drone.offboard_velocity(0.0, 0.0, 0.0, 0.0, False)
        self._logger.info("[95mFollow Line State finished. Stopping drone.[0m")

    # Removed should_apply_control method

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the Follow Line PID control state.
        Resets PID on first activation.
        """
        if not self.is_activate:
            self._logger.info("[94m - Follow Line PID state initiated[0m")
            self._reset_pid()  # Reset PID state on activation
            # Initial command might be zero velocity until first message arrives
            self.drone.offboard_velocity(0.0, 0.0, 0.0, 0.0, False)
            self._logger.info("Waiting for first LineInfo message to start control...")
            self.is_activate = True  # Set flag after first execution setup

        # Let the MonitorState handle waiting for the message and calling _state_callback
        return super().execute(blackboard)
