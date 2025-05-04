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
        """ COnfigure the mission control node"""
        self.get_logger().info("Configuring mission control node.")
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
        """ Activate the mission control node"""
        try:
            self.publisher.on_activate("takeoff")
        except Exception as e:
            self.get_logger().error(f"Error activating publisher: {e}")
            return TransitionCallbackReturn.ERROR

        self.get_logger().info("Mission control node activated.")
        self.timer = self.create_timer(1.0, self.mission_step)
        return TransitionCallbackReturn.SUCCESS
    
    def status_callback(self, msg):
        """ Callback for the /mission/status topic
            It just receives the status of and changes the state of the mission control node"""
        self.get_logger().info(f"Received status: {msg.data}")
        
        match(msg.data):
            case "takeoff_complete":
                self.get_logger().info("State changed to search_for_red_line")
                self.state = "search_for_red_line"

            case "search_for_red_line_complete":
                self.get_logger().info("State changed to drop_hook")
                self.state = "drop_hook"

            case "drop_hook_complete":
                self.get_logger().info("State changed to return_to_launch")
                self.state = "return_to_launch"
            
            case "return_to_launch_complete":
                self.get_logger().info("State changed to land")
                self.state = "land"

            case "land_complete":
                self.get_logger().info("State changed to finished")
                self.state = "finished"

    def mission_step(self):

        """ This function is called every second and publishes the current state of the mission control node"""
        self.get_logger().info(f"Current state: {self.state}")

        # Publish the current state to the /mission/command TOPIC
        msg = String()
        msg.data = self.state
        self.publisher.publish(msg)

        
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
    