import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import String

class ActuatorNode(LifecycleNode):
    def __init__(self):
        super().__init__('actuator_node')
        self.command_subscriber = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring Actuator")
        self.command_subscriber = self.create_subscription(
            String,
            '/actuator/command',
            self.command_callback,
            10
        )
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Activating Actuator")
        return TransitionCallbackReturn.SUCCESS
    
    def command_callback(self, msg: String):
        if msg.data == "drop_hook":
            self.get_logger().info("Dropping hook")
            # mandar sinal pro servo
        else:
            self.get_logger().info(f"Unknown command: {msg.data}")

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating Actuator")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up Actuator")
        return TransitionCallbackReturn.SUCCESS

def main():
    rclpy.init()
    node = ActuatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
