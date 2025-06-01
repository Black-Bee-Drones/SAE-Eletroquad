import rclpy

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

import time

from hook.constants import MIN_DESCEND_ALTITUDE, DESCEND_TIMEOUT


class PerformDescent(State):
    """
    Perform the descent operation based on height data from gps
    while tracking the red line

    - descend with constant linear_z
    - monitor the rel_alt
    - stop when reach min_descend_altitude

    """

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        if not "mavdrone" in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in PerformDescet state.")
            return ABORT

        mavdrone = blackboard["mavdrone"]

        yasmin.YASMIN_LOG_INFO("Descending towards red hose...")

        start_time = time.time()
        while time.time() - start_time < DESCEND_TIMEOUT:
            rclpy.spin_once(self.node)

            rel_alt = mavdrone.get_rel_alt.data
            yasmin.YASMIN_LOG_INFO(f"Current altitude: {rel_alt:.2f}m")

            diff = abs(rel_alt) - MIN_DESCEND_ALTITUDE
            if abs(diff) < 0.10:
                yasmin.YASMIN_LOG_INFO("Reached the mininum safe altitude.")
                mavdrone.offboard_velocity(
                    linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0
                )
                return SUCCEED

            if diff < 0.0:
                mavdrone.offboard_velocity(0.0, 0.0, -0.22 * diff, 0.0)
            else:
                mavdrone.offboard_velocity(0.0, 0.0, 0.22 * diff, 0.0)

        yasmin.YASMIN_LOG_ERROR("Failed to descend to hook (timeout)")
        return ABORT
