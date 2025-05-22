import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import yasmin
from yasmin import State, StateMachine, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from mirela_sdk.utils.process import ProcessUtils
from mirela_interfaces.msg import LineInfo
from std_msgs.msg import Bool

from time import sleep
import time

from hook.states.constants import DESCEND_SPEED, MIN_DESCEND_ALTITUDE, DESCEND_TIMEOUT


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

    def execute(self, blackboard: Blackboard):
        if not "mavdrone" in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in PerformDescet state.")
            return ABORT

        mavdrone = blackboard["mavdrone"]

        yasmin.YASMIN_LOG_INFO("Descending towards red hose...")

        start_time = time.time()
        while time.time() - start_time < DESCEND_TIMEOUT:
            rel_alt = mavdrone.get_rel_alt.data

            mavdrone.offboard_velocity(
                linear_x=0.0, linear_y=0.0, linear_z=DESCEND_SPEED, angular_z=0.0
            )

            if rel_alt < MIN_DESCEND_ALTITUDE:
                yasmin.YASMIN_LOG_WARN(
                    f"Reached the mininum safe altitude ({MIN_DESCEND_ALTITUDE}m), ready to drop the hook."
                )
                mavdrone.offboard_velocity(
                    linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0
                )
                return SUCCEED

            rclpy.spin_once(self.node, timeout_sec=0.05)

        yasmin.YASMIN_LOG_ERROR("Failed to descend to hook (timeout)")
        return ABORT
