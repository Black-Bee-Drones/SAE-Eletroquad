import rclpy
from rclpy.node import Node
from time import sleep
from mirela_sdk.control.mavros.mavros_api import MavDrone
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

class SlalonMovement(Node):
    """
    ROS 2 Node for controlling the drone in a slalon movement.
    """
    
    def __init__(self):
        super().__init__("slalon_movement")
        self.drone = MavDrone(node=self, mavros=False)
        self.proximity_distance = float('inf')  # Default value when no obstacle detected
        
        self.subscription = self.create_subscription(
            Point,
            'obstacle_distance',
            self.obstacle_callback,
            10
        )
        
        # Publisher for the drone movement
        self.movement_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Slalon movement has been initialized")
        
        self.drone.check_driver_node()
    
    def run(self):

        self.get_logger().info("Arming and taking off")
        #self.drone.arm_takeoff(1.5)
        sleep(2)

        self.get_logger().info(f"ROS status: {rclpy.ok()}")
        self.get_logger().info("Starting main loop")
        sleep(2)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            self.get_logger().info(f"Current obstacle distance: {self.proximity_distance}")
            
            if self.proximity_distance == float('inf'):
                self.get_logger().info("No obstacles detected. Moving forward.")
                self.drone.offboard_velocity_timer(
                    linear_x=0.3,
                    ground_reference=False,
                    pub_rate=10,
                    time=5
                )
            elif self.proximity_distance < 0.8:
                self.get_logger().info("Obstacle detected. Turning left!")
                self.drone.offboard_velocity_timer(
                    linear_x=0.0,
                    linear_y=0.5,
                    ground_reference=False,
                    pub_rate=10,
                    time=5
                )
            else:
                self.get_logger().info("Path clear. Moving forward.")
                self.drone.offboard_velocity_timer(
                    linear_x=0.3,
                    ground_reference=False,
                    pub_rate=10,
                    time=5
                )

            sleep(1)  # To avoid spamming actions

        self.get_logger().info("Landing...")
        #self.drone.land()

    def obstacle_callback(self, msg):
        """
        Callback for obstacle distance. Updates proximity_distance based on the message received.
        :param msg: Point message containing the x, y, z spatial coordinates of the obstacle
        """
        self.proximity_distance = msg.z
        self.get_logger().info(f"Obstacle distance updated: {self.proximity_distance}")


def main(args=None):
    rclpy.init(args=args)
    slalon = SlalonMovement()
    try:
        slalon.run()
    except KeyboardInterrupt:
        pass
    finally:
        slalon.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
