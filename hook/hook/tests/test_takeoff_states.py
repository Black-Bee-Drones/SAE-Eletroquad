import rclpy
import yasmin
from yasmin import StateMachine, State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros import set_ros_loggers

from hook.states.basic_states import Initialize, Takeoff

from mirela_sdk.control.mavros.mavros_api import MavDrone


class TestTakeoffSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE",
            Initialize(),
            transitions={SUCCEED: "TAKEOFF", ABORT: "END"},
        )

        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED: "END", ABORT: "END"},
        )

        self.add_state(
            "END",
            End(),
            transitions={SUCCEED: SUCCEED, ABORT: ABORT},
        )


class End(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard):
        if not "mavdrone" in blackboard:
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in ReturnToLaunch state.")
            return ABORT

        mavdrone: MavDrone = blackboard["mavdrone"]
        mavdrone.land()

        yasmin.YASMIN_LOG_INFO("Ending test...")
        return SUCCEED


def main(args=None):
    yasmin.YASMIN_LOG_INFO("TEST TAKEOFF STATE MACHINE")
    rclpy.init(args=args)
    set_ros_loggers()

    sm = TestTakeoffSM()
    outcome = sm()
    print(f"TestTakeoffSM finished with outcome: {outcome}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
