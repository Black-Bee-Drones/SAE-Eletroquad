import rclpy

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

import time

from hook.constants import (
    MIN_DESCEND_ALTITUDE,
    DESCEND_TIMEOUT,
    IMAGE_CENTER_X,
    IMAGE_CENTER_Y,
    LINE_DETECTION_RED_COLOR_NAME,
    DESCEND_KP_Z,
    DESCEND_KP_Y,
    DESCEND_KP_X,
    DESCEND_MAX_SPEED_Z,
    DESCEND_MAX_SPEED_XY,
)
from hook.utils.distance_parameters import (
    DISTANCE_CALIBRATION_CONST,
    TARGET_DISTANCE_CM,
    DISTANCE_TOLERANCE_CM,
)
from hook.utils.distance_estimation import DistanceEstimator, EstimationMethod
from mirela_interfaces.msg import LineInfo
from mirela_interfaces.msg import LineInfo
from mirela_sdk.image_processing.camera.image_calculus import ImageCalculus


class PerformDescent(State):
    """
    Perform the descent operation based on the detected red hose's height,
    while actively centering the drone on it.

    - Subscribes to line_info to get height, center_x, and center_y.
    - Estimates distance to the hose using its height.
    - Uses P-controllers to command velocity (x, y, z) to descend
      to a target distance while staying centered.
    """

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()
        self.line_info_sub = None
        self.distance_estimator = DistanceEstimator()

        # Data from subscriber
        self.hose_height = 0.0
        self.center_x = 0.0
        self.center_y = 0.0
        self.width_updates = 0

    def line_info_callback(self, msg: LineInfo):
        """Callback to update hose detection data."""
        if msg.height > 10.0:
            self.hose_height = msg.height
            self.center_x = msg.center_x
            self.center_y = msg.center_y
            self.width_updates += 1

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in PerformDescent state.")
            return ABORT

        mavdrone = blackboard["mavdrone"]
        self.width_updates = 0

        # Subscribe to line info
        line_info_topic = f"/line_state/{LINE_DETECTION_RED_COLOR_NAME}"
        self.line_info_sub = self.node.create_subscription(
            LineInfo, line_info_topic, self.line_info_callback, 10
        )
        yasmin.YASMIN_LOG_INFO(f"Subscribed to {line_info_topic} for descent control.")

        start_time = time.time()
        while time.time() - start_time < DESCEND_TIMEOUT:
            rclpy.spin_once(self.node)

            if self.width_updates < 3:  # Wait for a few messages
                yasmin.YASMIN_LOG_INFO("Waiting for hose detection to stabilize...")
                mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                time.sleep(0.1)
                continue

            # --- Main Control Logic ---

            # 1. Estimate current distance using distance estimator
            current_dist_cm = self.distance_estimator.estimate_distance(
                self.hose_height
            )
            distance_m = current_dist_cm / 100.0

            # 2. Check for success condition
            dist_error = current_dist_cm - TARGET_DISTANCE_CM
            if abs(dist_error) <= DISTANCE_TOLERANCE_CM:
                yasmin.YASMIN_LOG_INFO("Reached the target distance to the hose.")
                mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                self.node.destroy_subscription(self.line_info_sub)
                mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                return SUCCEED

            # 3. Calculate velocity commands
            # Z velocity (descent)
            vz = -DESCEND_KP_Z * dist_error
            vz = max(-DESCEND_MAX_SPEED_Z, min(DESCEND_MAX_SPEED_Z, vz))

            # Y velocity (lateral centering)
            error_x = IMAGE_CENTER_X - self.center_x
            vy = DESCEND_KP_Y * error_x
            vy = max(-DESCEND_MAX_SPEED_XY, min(DESCEND_MAX_SPEED_XY, vy))

            # X velocity (forward/backward centering with dynamic offset)
            offset_px = ImageCalculus.calculate_offset_pixels(
                0.089, distance_m, 43.3, 480
            )
            setpoint_y = IMAGE_CENTER_Y + offset_px
            error_y = setpoint_y - self.center_y
            vx = DESCEND_KP_X * error_y
            vx = max(-DESCEND_MAX_SPEED_XY, min(DESCEND_MAX_SPEED_XY, vx))

            yasmin.YASMIN_LOG_INFO(
                f"Dist: {current_dist_cm:.1f}cm, "
                f"Vel(x,y,z): ({vx:.2f}, {vy:.2f}, {vz:.2f})m/s"
            )

            # 4. Send velocity command
            mavdrone.offboard_velocity(
                linear_x=vx, linear_y=0.0, linear_z=vz, angular_z=0.0
            )

            rclpy.spin_once(self.node)

        yasmin.YASMIN_LOG_ERROR("Failed to descend to hook (timeout)")
        mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
        self.node.destroy_subscription(self.line_info_sub)
        return ABORT
