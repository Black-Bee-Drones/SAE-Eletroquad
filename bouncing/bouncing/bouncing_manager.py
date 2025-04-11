import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import List

TAKE_OFF_DELAY: float = 5.0 # This is the time in seconds to wait after takeoff before publishing the first state
WAIT_INIT_DELAY: float = 2.0 # This is the time in seconds to wait for the other nodes to start before takeoff

class BouncingManager(Node):
    def __init__(self) -> None:
        super().__init__('bouncing_manager')

        self.figures: List[str] = ["circle", "square", "cross"]
        self.current_index: int = 0

        self.state_pub = self.create_publisher(String, "/current_state", 10)
        self.cmd_pub = self.create_publisher(String, "/mission_cmd", 10)

        self.create_subscription(String, "/movement_status", self.movement_cb, 10)

        self.takeoff_timer = None
        self.state_timer = None

        self.get_logger().info("[Manager] Starting Bouncing Mission...")
        self.publish_stop_state()

        # Schedule takeoff shortly after starting
        self.takeoff_timer = self.create_timer(WAIT_INIT_DELAY, self.send_takeoff)

    def publish_stop_state(self) -> None:
        msg = String()
        msg.data = "none"
        self.state_pub.publish(msg)
        self.get_logger().info("[Manager] Sending No-State to Detector.")

    def send_takeoff(self) -> None:
        if self.takeoff_timer:
            self.takeoff_timer.cancel()

        msg = String()
        msg.data = "takeoff"
        self.cmd_pub.publish(msg)
        self.get_logger().info("[Manager] Publishing Take-off command...")

        # Schedule publishing of the current state
        self.state_timer = self.create_timer(TAKE_OFF_DELAY, self.publish_current_state)

    def publish_current_state(self) -> None:
        if self.state_timer:
            self.state_timer.cancel()

        if self.current_index < len(self.figures):
            state = self.figures[self.current_index]
            msg = String()
            msg.data = state
            self.state_pub.publish(msg)
            self.get_logger().info(f"[Manager] Changing to state: {state}")
        else:
            self.get_logger().info("[Manager] All states completed.")

    def movement_cb(self, msg: String) -> None:
        status = msg.data

        if status == "landed":
            self.get_logger().info(f"[Manager] State '{self.figures[self.current_index]}' finished.")

            self.current_index += 1
            if self.current_index >= len(self.figures):
                self.get_logger().info("[Manager] No more states to execute.")
                return

            self.get_logger().info("[Manager] Publishing Take-off before next state...")
            msg = String()
            msg.data = "takeoff"
            self.cmd_pub.publish(msg)

            # Schedule next state after takeoff delay
            self.state_timer = self.create_timer(TAKE_OFF_DELAY, self.publish_current_state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BouncingManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
