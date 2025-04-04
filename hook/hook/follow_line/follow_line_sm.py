from yasmin import StateMachine, CbState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, TIMEOUT, CANCEL

# from indoor.tasks.gate import AdjustToCenter
from hook.follow_line.line_process_states import StartLineDetection, StopLineDetection
from hook.follow_line.follow_controller_state import FollowLineState

from mirela_sdk.utils.process import ProcessUtils


class FollowStateMachine(StateMachine):
    """
    A state machine for following lines with a drone using visual detection.
    """

    def __init__(self, drone):
        """
        :param drone (mirela_sdk.control.mavros.mavros_api.MavDrone): The drone object.
        """
        super().__init__(outcomes=[SUCCEED])

        self.drone = drone

        self.add_state(
            "START_DETECTION",
            StartLineDetection(line_color="green"),
            transitions={SUCCEED: "FOLLOW_LINE"},
        )

        self.add_state(
            "FOLLOW_LINE",
            FollowLineState(self.drone),
            transitions={
                "adjusting": "FOLLOW_LINE",  # Keep following the line
                TIMEOUT: "STOP_DETECTION",  # If we lose the line, stop detection
            },
        )

        self.add_state(
            "STOP_DETECTION", StopLineDetection(), transitions={SUCCEED: "STOP"}
        )

        self.add_state(
            "STOP",
            CbState([SUCCEED], self.stop),
            transitions={SUCCEED: SUCCEED},
        )

        self.set_start_state("START_DETECTION")

    def start_detection(self, blackboard):
        ProcessUtils.start_process(
            "ros2 run indoor gate_detect --ros-args -p image_source:=webcam",
            "gate_detector",
            False,
        )

        return SUCCEED

    def stop_detection(self, blackboard) -> str:
        """
        Stop the gate detection.
        """
        ProcessUtils.kill_process("gate_detector")
        return SUCCEED

    def stop(self, blackboard) -> str:
        """Stop the drone movement when the state machine completes."""
        self.drone.offboard_velocity(0.0, 0.0, 0.0, 0.0, False)
        return SUCCEED
