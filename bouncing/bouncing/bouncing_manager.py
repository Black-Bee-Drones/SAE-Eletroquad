import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import List

TAKE_OFF_DELAY: float = 5.0 # This is the time in seconds to wait after takeoff before publishing the next figure

class BouncingManager(Node):
    def __init__(self) -> None:
        super().__init__('bouncing_manager')

        self.figures: List[str] = ["circle", "square", "cross"]
        self.current_index: int = 0

        self.figure_pub = self.create_publisher(String, "/current_figure", 10)
        self.cmd_pub = self.create_publisher(String, "/mission_cmd", 10)

        self.create_subscription(String, "/movement_status", self.movement_cb, 10)
        self.create_subscription(String, "/detector_status", self.detector_cb, 10)

        self.takeoff_timer = None
        self.figure_timer = None

        self.detector_ready: bool = False
        self.movement_ready: bool = False

        while rclpy.ok():
            rclpy.spin_once(self)
            if self.detector_ready and self.movement_ready:
                break

        self.get_logger().info("[Manager] Starting Bouncing Mission...")
        self.publish_stop_detection()

        self.send_takeoff()

    def publish_stop_detection(self) -> None:
        msg = String()
        msg.data = "none"
        self.figure_pub.publish(msg)
        self.get_logger().info("[Manager] Sending No-figure to Detector.")

    def send_takeoff(self) -> None:
        if self.takeoff_timer:
            self.takeoff_timer.cancel()

        msg = String()
        msg.data = "takeoff"
        self.cmd_pub.publish(msg)
        self.get_logger().info("[Manager] Publishing Take-off command...")

        # Schedule publishing of the current figure
        self.figure_timer = self.create_timer(TAKE_OFF_DELAY, self.publish_current_figure)

    def publish_current_figure(self) -> None:
        if self.figure_timer:
            self.figure_timer.cancel()

        if self.current_index < len(self.figures):
            figure = self.figures[self.current_index]
            msg = String()
            msg.data = figure
            self.figure_pub.publish(msg)
            self.get_logger().info(f"[Manager] Changing to figure: {figure}")
            self.detector_ready = False
        else:
            self.get_logger().info("[Manager] All figures completed.")

    def movement_cb(self, msg: String) -> None:
        status = msg.data

        if status == "ready":
            self.movement_ready = True

        if status == "landed":
            self.get_logger().info(f"[Manager] figure '{self.figures[self.current_index]}' finished.")

            self.current_index += 1
            if self.current_index >= len(self.figures):
                self.get_logger().info("[Manager] No more figures to execute.")
                rclpy.shutdown()
                return
            
            self.publish_stop_detection()
            self.get_logger().info("[Manager] Publishing Take-off before next figure...")
            msg = String()
            msg.data = "takeoff"
            self.cmd_pub.publish(msg)

            # Schedule next figure after takeoff delay
            self.figure_timer = self.create_timer(TAKE_OFF_DELAY, self.publish_current_figure)

    def detector_cb(self, msg: String):
        status = msg.data

        if status == "ready":
            self.detector_ready = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BouncingManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
