import rclpy

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.utils.process import ProcessUtils

import time

from hook.constants import (
    TAKEOFF_ALTITUDE,
    RETURN_ALTITUDE,
    LINE_DETECT_NODE_NAME,
    CENTER_PID_PROCESS,
    ANGLE_PID_PROCESS,
    CENTERING_PID_PROCESS,
)


class Initialize(State):
    """Connects to the drone and checks initial status."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.mavdrone: MavDrone = None

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Initializing Drone Connection...")
        try:
            blackboard["mavdrone"] = MavDrone(node=YasminNode.get_instance())
            self.mavdrone: MavDrone = blackboard["mavdrone"]

            time.sleep(3)

            if not self.mavdrone.get_state.connected:
                yasmin.YASMIN_LOG_ERROR("MAVROS not connected!")
                return ABORT

            yasmin.YASMIN_LOG_INFO("Drone Initialized Successfully.")

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Initialization failed: {e}")
            return ABORT


class Takeoff(State):
    """Arms and takes off to a specified altitude."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.mavdrone: MavDrone = None
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        if not "mavdrone" in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in Takeoff state.")
            return ABORT

        self.mavdrone = blackboard["mavdrone"]

        yasmin.YASMIN_LOG_INFO(f"Attempting takeoff to {TAKEOFF_ALTITUDE}m...")
        try:
            self.mavdrone.arm_takeoff(TAKEOFF_ALTITUDE)

            time.sleep(3)

            start_time = time.time()
            timeout = 30
            while time.time() - start_time < timeout:
                rclpy.spin_once(self.node)

                alt = self.mavdrone.get_rel_alt.data
                yasmin.YASMIN_LOG_INFO(f"Current altitude: {alt:.2f}m")

                diff = abs(alt) - TAKEOFF_ALTITUDE
                if abs(diff) < 0.10:
                    yasmin.YASMIN_LOG_INFO("Takeoff altitude reached.")
                    self.mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                    time.sleep(1)
                    return SUCCEED

                if diff < 0.0:
                    self.mavdrone.offboard_velocity(0.0, 0.0, -0.25 * diff, 0.0)
                else:
                    self.mavdrone.offboard_velocity(0.0, 0.0, 0.25 * diff, 0.0)

            yasmin.YASMIN_LOG_ERROR("Takeoff timed out.")
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Takeoff failed: {e}")
            return ABORT


class ReturnToLaunch(State):
    """Returns the drone to the launch position."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard):
        if not "mavdrone" in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in ReturnToLaunch state.")
            return ABORT

        mavdrone: MavDrone = blackboard["mavdrone"]

        yasmin.YASMIN_LOG_INFO("Returning to launch...")

        try:
            mavdrone.rtl(rtl_alt=RETURN_ALTITUDE)
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"RTL failed: {e}")
            return ABORT


class End(State):
    """Cleans up any running processes."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED])

    def execute(self, blackboard):
        yasmin.YASMIN_LOG_INFO("Mission ended. Cleaning up all processes...")

        mavdrone: MavDrone = blackboard["mavdrone"]
        if mavdrone and mavdrone.get_state.armed:
            yasmin.YASMIN_LOG_INFO("Drone still armed, attempting to disarm...")
            try:
                # Ensure velocity is zeroed first
                mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                mavdrone.land()
            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(f"Failed to land: {e}")

        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)
        ProcessUtils.kill_process(CENTER_PID_PROCESS)
        ProcessUtils.kill_process(ANGLE_PID_PROCESS)
        ProcessUtils.kill_process(CENTERING_PID_PROCESS)

        yasmin.YASMIN_LOG_INFO("Mission cleanup completed.")
        return SUCCEED
