import rclpy
import yasmin
from yasmin import StateMachine, State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

from hook.states.basic_states import Initialize, Takeoff
from hook.states.line_following import FollowBlueLineWithRedDetection

from mirela_sdk.control.mavros.mavros_api import MavDrone


class TestFollowBlueLinePIDSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE",
            Initialize(),
            transitions={SUCCEED: "FOLLOW_BLUE_LINE", ABORT: "END"},
        )

        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED: "FOLLOW_BLUE_LINE", ABORT: "END"},
        )

        self.add_state(
            "FOLLOW_BLUE_LINE",
            FollowBlueLineWithRedDetection(),
            transitions={
                SUCCEED: "END",
                ABORT: "END",
                "red_detected": "END",
            },
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
            yasmin.YASMIN_LOG_ERROR("MavDrone not available in End state.")
            return ABORT

        mavdrone: MavDrone = blackboard["mavdrone"]

        # Stop any movement before landing
        mavdrone.offboard_velocity_timer(
            linear_x=0.0,
            linear_y=0.0,
            linear_z=0.0,
            angular_z=0.0,
            time=2.0,
        )

        mavdrone.land()

        yasmin.YASMIN_LOG_INFO("Ending follow blue line PID test...")
        return SUCCEED


def main(args=None):
    yasmin.YASMIN_LOG_INFO("TEST FOLLOW BLUE LINE PID STATE MACHINE")
    rclpy.init(args=args)
    set_ros_loggers()

    sm = TestFollowBlueLinePIDSM()

    # Optional: Add viewer for debugging
    viewer = YasminViewerPub("test_follow_blue_line_pid", sm)

    outcome = sm()
    print(f"TestFollowBlueLinePIDSM finished with outcome: {outcome}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
