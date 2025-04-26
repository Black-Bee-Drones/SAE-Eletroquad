import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import String

class MissionControlNode(LifecycleNode):   
    def __init__(self):
        super().__init__('mission_control_node')
        self.state = "takeoff" 
        self.publisher = None
        self.subscription = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.publisher = self.create_lifecycle_publisher(
            String,
            '/mission/command',
            10
        )

        self.subscription = self.create_subscription(
            String,
            '/mission/status',
            self.status_callback,
            10
        )

        self.get_logger().info("Mission control node configured.")
        return TransitionCallbackReturn.SUCCESS
    

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Mission control node activated.")
        self.timer = self.create_timer(1.0, self.mission_step)
        return TransitionCallbackReturn.SUCCESS
    
    def status_callback(self, msg):
        self.get_logger().info(f"Received status: {msg.data}")
        if msg.data == "takeoff_complete":
            self.state = "search_for_red_line"
        elif msg.data == "search_for_red_line_complete":
            self.state = "drop_hook"
        elif msg.data == "drop_hook_complete":
            self.state = "return_to_base"
        elif msg.data == "return_complete":
            self.state = "land"
        elif msg.data == "land_complete":
            self.state = "finished"

    def mission_step(self):
        if self.state == "takeoff":
            msg = String()
            msg.data = "takeoff"
            self.publisher.publish(msg)
        elif self.state == "search_for_red_line":
            msg = String()
            msg.data = "search_for_red_line"
            self.publisher.publish(msg)
        elif self.state == "drop_hook":
            msg = String()
            msg.data = "drop_hook"
            self.publisher.publish(msg)
        elif self.state == "return_to_base":
            msg = String()
            msg.data = "return_to_base"
            self.publisher.publish(msg)
        elif self.state == "land":
            msg = String()
            msg.data = "land"
            self.publisher.publish(msg)
        elif self.state == "finished":
            self.get_logger().info("Mission completed.")
            self.destroy_timer(self.timer)
        
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Mission control node deactivated.")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.destroy_publisher(self.publisher)
        self.destroy_subscription(self.subscription)
        return TransitionCallbackReturn.SUCCESS

def main():
    rclpy.init()
    node = MissionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    
    