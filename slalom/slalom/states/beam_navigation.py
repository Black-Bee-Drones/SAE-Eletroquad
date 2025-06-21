import rclpy
import time

import yasmin
from yasmin import State, StateMachine
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from mirela_interfaces.msg import LineInfo
from std_msgs.msg import Float64

from slalom.constants import (
    BEAM_COLORS,
    SEARCH_SPEED_Y,
    SEARCH_TIMEOUT,
    CENTERING_KP,
    CENTERING_THRESHOLD,
    CENTERING_TIMEOUT,
    CENTERING_CONFIRMATIONS,
    APPROACH_DISTANCE,
    APPROACH_SPEED,
    APPROACH_TIMEOUT,
    SIDE_PASS_SPEED,
    SIDE_PASS_TIME,
    FORWARD_PASS_SPEED,
    FORWARD_PASS_TIME,
    IMAGE_CENTER_X,
)
from slalom.utils.distance_estimation import BeamDistanceEstimator, EstimationMethod


class SearchBeam(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()
        self.line_info_sub = None
        self.beam_found = False
        self.search_direction = 1

    def line_info_callback(self, msg: LineInfo):
        if msg.center_x > 0 and msg.width > 0:
            self.beam_found = True

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in SearchBeam state.")
            return ABORT

        mavdrone = blackboard["mavdrone"]
        current_beam_index = blackboard["current_beam_index"]
        current_color = BEAM_COLORS[current_beam_index]

        yasmin.YASMIN_LOG_INFO(f"Searching for {current_color} beam...")

        self.beam_found = False
        self.search_direction = 1

        self.line_info_sub = self.node.create_subscription(
            LineInfo,
            f"/line_state/{current_color}",
            self.line_info_callback,
            10,
        )

        start_time = time.time()
        direction_change_time = time.time()
        direction_duration = 3.0

        while time.time() - start_time < SEARCH_TIMEOUT:
            if time.time() - direction_change_time > direction_duration:
                self.search_direction *= -1
                direction_change_time = time.time()

            mavdrone.offboard_velocity(
                linear_x=0.0,
                linear_y=SEARCH_SPEED_Y * self.search_direction,
                linear_z=0.0,
                angular_z=0.0,
            )

            rclpy.spin_once(self.node, timeout_sec=0.1)

            if self.beam_found:
                mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                yasmin.YASMIN_LOG_INFO(f"{current_color} beam found!")
                self._cleanup_subscriber()
                return SUCCEED

        yasmin.YASMIN_LOG_ERROR(f"Search for {current_color} beam timed out.")
        self._cleanup_subscriber()
        return ABORT

    def _cleanup_subscriber(self):
        if self.line_info_sub:
            self.node.destroy_subscription(self.line_info_sub)
            self.line_info_sub = None


class CenterOnBeam(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()
        self.line_info_sub = None
        self.current_center_x = 0.0
        self.centering_confirmations = 0

    def line_info_callback(self, msg: LineInfo):
        self.current_center_x = msg.center_x

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in CenterOnBeam state.")
            return ABORT

        mavdrone = blackboard["mavdrone"]
        current_beam_index = blackboard["current_beam_index"]
        current_color = BEAM_COLORS[current_beam_index]

        yasmin.YASMIN_LOG_INFO(f"Centering on {current_color} beam...")

        self.current_center_x = 0.0
        self.centering_confirmations = 0

        self.line_info_sub = self.node.create_subscription(
            LineInfo,
            f"/line_state/{current_color}",
            self.line_info_callback,
            10,
        )

        start_time = time.time()

        while time.time() - start_time < CENTERING_TIMEOUT:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if self.current_center_x > 0:
                error = self.current_center_x - IMAGE_CENTER_X
                velocity_y = -CENTERING_KP * error

                mavdrone.offboard_velocity(
                    linear_x=0.0,
                    linear_y=velocity_y,
                    linear_z=0.0,
                    angular_z=0.0,
                )

                if abs(error) < CENTERING_THRESHOLD:
                    self.centering_confirmations += 1
                    yasmin.YASMIN_LOG_INFO(
                        f"Centering confirmation: {self.centering_confirmations}"
                    )
                else:
                    self.centering_confirmations = 0

                if self.centering_confirmations >= CENTERING_CONFIRMATIONS:
                    mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                    yasmin.YASMIN_LOG_INFO(f"{current_color} beam centered!")
                    self._cleanup_subscriber()
                    return SUCCEED

        yasmin.YASMIN_LOG_ERROR(f"Centering on {current_color} beam timed out.")
        self._cleanup_subscriber()
        return ABORT

    def _cleanup_subscriber(self):
        if self.line_info_sub:
            self.node.destroy_subscription(self.line_info_sub)
            self.line_info_sub = None


class ApproachBeam(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()
        self.line_info_sub = None
        self.current_width = 0.0
        self.distance_estimator = BeamDistanceEstimator(
            default_method=EstimationMethod.POLYNOMIAL,
            validate_inputs=False,
        )

    def line_info_callback(self, msg: LineInfo):
        self.current_width = msg.width

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in ApproachBeam state.")
            return ABORT

        mavdrone = blackboard["mavdrone"]
        current_beam_index = blackboard["current_beam_index"]
        current_color = BEAM_COLORS[current_beam_index]

        yasmin.YASMIN_LOG_INFO(f"Approaching {current_color} beam...")

        self.current_width = 0.0

        self.line_info_sub = self.node.create_subscription(
            LineInfo,
            f"/line_state/{current_color}",
            self.line_info_callback,
            10,
        )

        start_time = time.time()

        while time.time() - start_time < APPROACH_TIMEOUT:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if self.current_width > 0:
                try:
                    estimated_distance_cm = self.distance_estimator.estimate_distance(
                        self.current_width
                    )
                    estimated_distance_m = estimated_distance_cm / 100.0

                    yasmin.YASMIN_LOG_INFO(
                        f"Width: {self.current_width:.1f}px -> Distance: {estimated_distance_m:.2f}m ({estimated_distance_cm:.1f}cm)"
                    )

                    if estimated_distance_m <= APPROACH_DISTANCE:
                        mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                        yasmin.YASMIN_LOG_INFO(
                            f"Reached approach distance to {current_color} beam!"
                        )
                        self._cleanup_subscriber()
                        return SUCCEED

                except Exception as e:
                    yasmin.YASMIN_LOG_ERROR(f"Distance estimation error: {e}")

            mavdrone.offboard_velocity(
                linear_x=APPROACH_SPEED,
                linear_y=0.0,
                linear_z=0.0,
                angular_z=0.0,
            )

        yasmin.YASMIN_LOG_ERROR(f"Approach to {current_color} beam timed out.")
        self._cleanup_subscriber()
        return ABORT

    def _cleanup_subscriber(self):
        if self.line_info_sub:
            self.node.destroy_subscription(self.line_info_sub)
            self.line_info_sub = None


class PassThroughBeam(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in PassThroughBeam state.")
            return ABORT

        mavdrone = blackboard["mavdrone"]
        current_beam_index = blackboard["current_beam_index"]
        current_side = blackboard["current_side"]
        current_color = BEAM_COLORS[current_beam_index]

        yasmin.YASMIN_LOG_INFO(
            f"Passing through {current_color} beam on {current_side} side..."
        )

        side_multiplier = -1 if current_side == "left" else 1

        yasmin.YASMIN_LOG_INFO(f"Moving {current_side}...")
        mavdrone.offboard_velocity_timer(
            linear_x=0.0,
            linear_y=SIDE_PASS_SPEED * side_multiplier,
            linear_z=0.0,
            angular_z=0.0,
            time=SIDE_PASS_TIME,
        )

        yasmin.YASMIN_LOG_INFO("Moving forward...")
        mavdrone.offboard_velocity_timer(
            linear_x=FORWARD_PASS_SPEED,
            linear_y=0.0,
            linear_z=0.0,
            angular_z=0.0,
            time=FORWARD_PASS_TIME,
        )

        blackboard["current_beam_index"] += 1
        blackboard["current_side"] = "right" if current_side == "left" else "left"

        yasmin.YASMIN_LOG_INFO(f"Passed through {current_color} beam!")
        return SUCCEED


class CheckMissionComplete(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        current_beam_index = blackboard["current_beam_index"]

        if current_beam_index >= len(BEAM_COLORS):
            yasmin.YASMIN_LOG_INFO("All beams passed! Mission complete!")
            return SUCCEED
        else:
            next_color = BEAM_COLORS[current_beam_index]
            yasmin.YASMIN_LOG_INFO(f"Next beam: {next_color}")
            return ABORT


class BeamNavigationStateMachine(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "SEARCH_BEAM",
            SearchBeam(),
            transitions={SUCCEED: "CENTER_ON_BEAM", ABORT: ABORT},
        )

        self.add_state(
            "CENTER_ON_BEAM",
            CenterOnBeam(),
            transitions={SUCCEED: "APPROACH_BEAM", ABORT: ABORT},
        )

        self.add_state(
            "APPROACH_BEAM",
            ApproachBeam(),
            transitions={SUCCEED: "PASS_THROUGH_BEAM", ABORT: ABORT},
        )

        self.add_state(
            "PASS_THROUGH_BEAM",
            PassThroughBeam(),
            transitions={SUCCEED: "CHECK_MISSION_COMPLETE", ABORT: ABORT},
        )

        self.add_state(
            "CHECK_MISSION_COMPLETE",
            CheckMissionComplete(),
            transitions={SUCCEED: SUCCEED, ABORT: "SEARCH_BEAM"},
        )
