import rclpy

import yasmin
from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

from hook.states import (
    Initialize,
    Takeoff,
    SearchBlueLine,
    FollowBlueLineWithRedDetection,
    CenterRedBlob,
    PerformDescent,
    ReleaseHook,
    ReturnToLaunch,
    End,
)


class HangTheHookSM(StateMachine):
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
            transitions={SUCCEED: "CENTER_RED_BLOB", ABORT: "RETURN_TO_LAUNCH"},
        )

        self.add_state(
            "SEARCH_BLUE_LINE",
            SearchBlueLine(),
            transitions={SUCCEED: "FOLLOW_BLUE_LINE", ABORT: "RETURN_TO_LAUNCH"},
        )

        self.add_state(
            "FOLLOW_BLUE_LINE",
            FollowBlueLineWithRedDetection(),
            transitions={
                SUCCEED: "CENTER_RED_BLOB",
                ABORT: "RETURN_TO_LAUNCH",
            },
        )

        self.add_state(
            "CENTER_RED_BLOB",
            CenterRedBlob(),
            transitions={SUCCEED: "DESCEND_TO_HOOK", ABORT: "RETURN_TO_LAUNCH"},
        )

        self.add_state(
            "DESCEND_TO_HOOK",
            PerformDescent(),
            transitions={SUCCEED: "RELEASE_HOOK", ABORT: "RETURN_TO_LAUNCH"},
        )

        self.add_state(
            "RELEASE_HOOK",
            ReleaseHook(),
            transitions={
                SUCCEED: "RETURN_TO_LAUNCH",  # Go to RTL after release
                ABORT: "RETURN_TO_LAUNCH",
            },
        )

        self.add_state(
            "RETURN_TO_LAUNCH",
            ReturnToLaunch(),
            transitions={SUCCEED: "END", ABORT: "END"},
        )

        self.add_state("END", End(), transitions={SUCCEED: SUCCEED})


def main(args=None):
    yasmin.YASMIN_LOG_INFO("HANG THE HOOK STATE MACHINE")

    rclpy.init(args=args)

    set_ros_loggers()

    mangalarga = HangTheHookSM()

    viewer = YasminViewerPub("mangalarga_state_machine", mangalarga)

    avante = mangalarga()
    print(avante)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
