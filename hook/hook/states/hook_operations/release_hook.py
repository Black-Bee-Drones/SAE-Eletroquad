import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

import time

from hook.constants import HOCK_SERVO_CHANNEL, HOCK_RELEASE_PWM, HOCK_HOLD_PWM


class ReleaseHook(State):
    """Activates the servo to release the hook."""

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Releasing hook...")
        mavdrone = blackboard["mavdrone"]
        try:
            mavdrone.offboard_velocity(0.0, 0.0, 0.0, 0.0)
            time.sleep(1)

            mavdrone.do_servo(HOCK_SERVO_CHANNEL, HOCK_HOLD_PWM)
            time.sleep(1)

            mavdrone.do_servo(HOCK_SERVO_CHANNEL, HOCK_RELEASE_PWM)
            time.sleep(2)

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Failed to release hook: {e}")
            return ABORT
