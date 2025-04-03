import rclpy
from rclpy.node import Node
from mission_manager import MissionManager

class MissionTestNode(Node):
    def __init__(self):
        super().__init__('mission_test_node')
        
        # Create mission manager (set is_lifecycle based on your needs)
        self.manager = MissionManager(
            node=self,
            is_lifecycle=False,  # Set True if testing with lifecycle node
            lifecycle_node_name='drone_lifecycle_node'  # Only needed if is_lifecycle=True
        )
        
        # Timer to start the mission after node initialization
        self.timer = self.create_timer(1.0, self.start_mission)
        
        # Add a state monitor timer
        self.state_monitor = self.create_timer(0.5, self.log_state)
    
    def start_mission(self):
        self.timer.cancel()  # Only run once
        self.get_logger().info("Starting mission...")
        self.manager.execute_mission()
    
    def log_state(self):
        self.get_logger().info(f"Current state: {self.manager.current_state.name}")

def main(args=None):
    rclpy.init(args=args)
    
    node = MissionTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mission test interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()