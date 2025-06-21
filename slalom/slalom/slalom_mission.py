#!/usr/bin/env python3

import rclpy

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode
from yasmin_viewer import YasminViewerPub

from slalom.states.basic_states import (
    Initialize,
    Takeoff,
    StartLineDetection,
    Land,
    End,
)
from slalom.states.beam_navigation import BeamNavigationStateMachine


class SlalomStateMachine(StateMachine):
    def __init__(self):
        super().__init__([SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE",
            Initialize(),
            transitions={SUCCEED: "TAKEOFF", ABORT: "END"},
        )

        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED: "START_LINE_DETECTION", ABORT: "END"},
        )

        self.add_state(
            "START_LINE_DETECTION",
            StartLineDetection(),
            transitions={SUCCEED: "BEAM_NAVIGATION", ABORT: "END"},
        )

        self.add_state(
            "BEAM_NAVIGATION",
            BeamNavigationStateMachine(),
            transitions={SUCCEED: "LAND", ABORT: "END"},
        )

        self.add_state(
            "LAND",
            Land(),
            transitions={SUCCEED: "END", ABORT: "END"},
        )

        self.add_state(
            "END",
            End(),
            transitions={SUCCEED: SUCCEED},
        )


def main() -> None:
    print("Slalom Mission State Machine")

    rclpy.init()

    slalom_sm = SlalomStateMachine()

    YasminViewerPub("slalom_state_machine", slalom_sm)

    outcome = slalom_sm()
    print(f"Mission outcome: {outcome}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
