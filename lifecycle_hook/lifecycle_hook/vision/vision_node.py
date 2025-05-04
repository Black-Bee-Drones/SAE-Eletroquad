import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import Float32MultiArray
import random

class VisionNode(LifecycleNode):
    def __init__(self):
        super().__init__('vision_node')
        self.publisher = None
        self.timer = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:

        self.get_logger().info("Configuring Vision")
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/vision/target_position',
            10
        )
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Activating Vision")
        self.timer = self.create_timer(2.0, self.publish_target_position)
        return TransitionCallbackReturn.SUCCESS

    def publish_target_position(self):
        msg = Float32MultiArray()
        msg.data = [540 + random.uniform(-30, 30), 360 + random.uniform(-30, 30)]
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing target position: {msg.data}")

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating Vision")
        if self.timer:
            self.destroy_timer(self.timer)
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up Vision")
        return TransitionCallbackReturn.SUCCESS
    
def main():
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()