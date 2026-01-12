import rclpy
import time

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from mirela_sdk.control.mavros.mavros_api import MavDrone
from mirela_sdk.utils.process import ProcessUtils

from slalom.constants import (
    TAKEOFF_ALTITUDE,
    RETURN_ALTITUDE,
    LINE_DETECT_NODE_NAME,
    BEAM_COLORS,
    LINE_DETECTION_SPACE,
    LINE_DETECTION_IMAGE_SOURCE,
    LINE_DETECTION_SHOW_VISUALIZATION,
    LINE_DETECTION_VISUALIZATION_TITLE,
    LINE_DETECTION_METHOD,
)


class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Initializing Drone Connection...")
        try:
            blackboard["mavdrone"] = MavDrone(node=YasminNode.get_instance())
            mavdrone: MavDrone = blackboard["mavdrone"]

            time.sleep(2)

            blackboard["current_beam_index"] = 0
            blackboard["current_side"] = "left"

            yasmin.YASMIN_LOG_INFO("Drone Initialized Successfully.")
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Initialization failed: {e}")
            return ABORT


class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in Takeoff state.")
            return ABORT

        mavdrone = blackboard["mavdrone"]

        yasmin.YASMIN_LOG_INFO(f"Attempting takeoff to {TAKEOFF_ALTITUDE}m...")
        try:
            mavdrone.set_home(current_gps=True)
            mavdrone.arm_takeoff(TAKEOFF_ALTITUDE)

            time.sleep(3)

            start_time = time.time()
            timeout = 30
            while time.time() - start_time < timeout:
                rclpy.spin_once(self.node)

                alt = TAKEOFF_ALTITUDE  # mavdrone.get_rel_alt.data
                yasmin.YASMIN_LOG_INFO(f"Current altitude: {alt:.2f}m")

                diff = abs(alt) - TAKEOFF_ALTITUDE
                if abs(diff) < 0.10:
                    yasmin.YASMIN_LOG_INFO("Takeoff altitude reached.")
                    mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                    time.sleep(1)
                    return SUCCEED

                if diff < 0.0:
                    mavdrone.offboard_velocity(0.0, 0.0, -0.25 * diff, 0.0)
                else:
                    mavdrone.offboard_velocity(0.0, 0.0, -0.25 * diff, 0.0)

            yasmin.YASMIN_LOG_ERROR("Takeoff timed out.")
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Takeoff failed: {e}")
            return ABORT


class StartLineDetection(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Starting line detection for all colors...")

        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)

        colors_str = ",".join(BEAM_COLORS)
        spaces_str = ",".join([LINE_DETECTION_SPACE] * len(BEAM_COLORS))

        line_detection_cmd = (
            "ros2 run mirela_sdk line_detection_node "
            "--ros-args "
            f"-p line_colors:={colors_str} "
            f"-p spaces:={spaces_str} "
            f"-p show_visualization:={LINE_DETECTION_SHOW_VISUALIZATION} "
            f"-p image_source:={LINE_DETECTION_IMAGE_SOURCE} "
            f"-p visualization_name:='{LINE_DETECTION_VISUALIZATION_TITLE}' "
            f"-p method:={LINE_DETECTION_METHOD} "
        )

        if not ProcessUtils.start_process(line_detection_cmd, LINE_DETECT_NODE_NAME):
            yasmin.YASMIN_LOG_ERROR("Failed to start line detection node.")
            return ABORT

        yasmin.YASMIN_LOG_INFO("Line detection node started successfully.")
        time.sleep(2)

        return SUCCEED


class Land(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if "mavdrone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in Land state.")
            return ABORT

        mavdrone = blackboard["mavdrone"]

        yasmin.YASMIN_LOG_INFO("Landing drone...")
        try:
            mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
            mavdrone.land()
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Landing failed: {e}")
            return ABORT


class End(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Mission ended. Cleaning up all processes...")

        mavdrone: MavDrone = blackboard["mavdrone"]
        if mavdrone and mavdrone.get_state.armed:
            yasmin.YASMIN_LOG_INFO("Drone still armed, attempting to disarm...")
            try:
                mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
                mavdrone.land()
            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(f"Failed to land: {e}")

        ProcessUtils.kill_process(LINE_DETECT_NODE_NAME)

        yasmin.YASMIN_LOG_INFO("Mission cleanup completed.")
        return SUCCEED
