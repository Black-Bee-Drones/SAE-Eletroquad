import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import List
import time


class BouncingManager(Node):
    def __init__(self) -> None:
        super().__init__('bouncing_manager')

        self.figures: List[str] = ["circle", "square", "cross"]
        self.current_index: int = 0

        self.state_pub = self.create_publisher(String, "/current_state", 10)
        self.cmd_pub = self.create_publisher(String, "/mission_cmd", 10)

        self.create_subscription(String, "/movement_status", self.movement_cb, 10)

        self.publish_current_state()

    def publish_current_state(self) -> None:
        if self.current_index < len(self.figures):
            state: str = self.figures[self.current_index]
            msg: String = String()
            msg.data = state
            self.state_pub.publish(msg)
            self.get_logger().info(f"[Manager] Changing to state: {state}")

    def movement_cb(self, msg: String) -> None:
        status: str = msg.data

        if status == "landed":
            self.get_logger().info(f"[Manager] State '{self.figures[self.current_index]}' finished.")
            
            if self.current_index >= len(self.figures):
                self.get_logger().info("[Manager] All states finished.")
                return
            
            else:
                self.get_logger().info(f"[Manager] Taking off before moving to next state.")
                takeoff_msg: String = String()
                takeoff_msg.data = "takeoff"
                self.cmd_pub.publish(takeoff_msg)

                time.sleep(5)  # Wait for the drone to take off

                self.current_index += 1
                self.publish_current_state()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = BouncingManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
