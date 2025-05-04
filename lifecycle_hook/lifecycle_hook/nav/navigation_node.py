import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import String, Float32MultiArray, Float64
from mirela_sdk.control.mavros.mavros_api import MavDrone
from lifecycle_hook.nav.drone_command import DroneCommand
import time


class NavigationNode(LifecycleNode):
    def __init__(self):
        super().__init__('navigation_node')
        self.drone = MavDrone(node=self, mavros=False)
        self.status_pub = None
        self.command_sub = None
        self.target_pos_sub = None
        self.commander = None # will be initialized in on_configure
        #self.home = self.drone
        
        # Target Tracking
        self.target_x = None # posicao x da mangueira
        self.target_y = None # posicao y da mangueira
        self.aligned = False

        # Image DImensions
        self.image_center_x = 540
        self.image_center_y = 360

        # Tentativa de um PID
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.kp = 0.002 # ganho proporcional
        self.kd = 0.001 # ganho derivativo (sujeito a ajustes)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
            Configures the publisher and subscriber for the navigation node.
        """
        
        self.get_logger().info("Configuring Navigation Node")
        
        # Create publishers
        self.status_pub = self.create_publisher(
            String, '/mission/status', 10)
        
        # Initialize the command executor
        self.commander = DroneCommand(
            self.drone, self.status_pub, self.get_logger())

        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/mission/command', self.command_callback, 10)
        
        self.target_pos_sub = self.create_subscription(
            Float32MultiArray, '/vision/target_position', self.align_callback, 10)
        
        
        #self.drone.check_driver_node() # verificacao mavros
        
        self.get_logger().info("Navigation Node configured")
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """ Start timers and activate the node."""
        self.get_logger().info("Navigation Node: Activated!")
        self.timer = self.create_timer(1.0, self.update_status)
        return TransitionCallbackReturn.SUCCESS
    
    def command_callback(self, msg):
        """ Process incoming commands from the command topic.
            Args:
                msg: The incoming message containing the command.
        """
        command = msg.data
        self.get_logger().info(f"Command received: {command}")

        match(command):

            case "takeoff":
                self.get_logger().info("Takeoff command received")
                self.commander.execute_async(
                    self.execute_takeoff,
                    "takeoff_complete" 
                    )

            case "search_for_red_line":
                self.get_logger().info("Search for red line command received")
                self.reset_alignment_tracking()
            
            case "drop_hook":
                self.get_logger().info("Drop hook command received")
                self.commander.execute_async(
                    self.execute_hook_drop,
                    "drop_hook_complete"
                    )
                
            case "return_to_launch":
                self.get_logger().info("Return to launch command received")
                self.commander.execute_async(
                    self.execute_return_to_launch,
                    "return_to_launch_complete"
                    )
            
            case "land":
                self.get_logger().info("Land command received")
                self.commander.execute_async(
                    self.execute_land,
                    "land_complete"
                    )
            case "finished":
                self.get_logger().info("Finished command received")
                msg = String()
                msg.data = "finished"
                self.status_pub.publish(msg)
        
    def execute_takeoff(self):
        self.drone.arm_takeoff(takeoff_alt=5.0)
        self.get_logger().info("Takeoff command executed")
        time.sleep(5)

    def execute_hook_drop(self):
        self.drone.offboard_velocity_timer(
                linear_x=0.0,
                ground_reference=False,
                pub_rate=30,
                time=0.5
            )
        self.get_logger().info("Dropping hook...")
        time.sleep(3)
    
    def execute_return_to_launch(self):
        self.drone.rtl(
            rtl_alt=5.0
        )
        self.get_logger().info("Returning to launch...")
        time.sleep(5)

    def execute_land(self):
        self.drone.land()
        self.get_logger().info("Landing...")
        time.sleep(5)

    def reset_alignment_tracking(self):
        self.get_logger().info("Resetting alignment tracking")
        self.aligned = False
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

    def align_callback(self, msg):
        if self.target_x is None or self.target_y is None:
            self.target_x = msg.data[0]
            self.target_y = msg.data[1]
            self.get_logger().info(f"Target position: {self.target_x}, {self.target_y}")

        # Erro atual
        error_x = self.target_x - self.image_center_x
        error_y = self.target_y - self.image_center_y         

        # Erro derivativo
        deriv_x = error_x - self.prev_error_x
        deriv_y = error_y - self.prev_error_y
        self.prev_error_x = error_x
        self.prev_error_y = error_y

        # Controle PID -> velocidades proporcionais ao erro e a sua derivada
        vel_x = -self.kp * error_y - self.kd * deriv_y # Inverte Y
        vel_y = -self.kp * error_x + self.kd * deriv_x # X da imagem = Y drone

        vel_x = max(min(vel_x, 0.5), -0.5) 
        vel_y = max(min(vel_y, 0.5), -0.5)

        if not self.aligned:
            self.drone.offboard_velocity_timer(
                linear_x=vel_x,
                linear_y=vel_y,
                ground_reference=False,
                pub_rate=30,
                time=0.5
            )
            self.get_logger().info(f"Aligning: vel_x = {vel_x}, vel_y = {vel_y}")

        # Verifica se o drone está alinhado (tolerancia de 20 pixels)
        if abs(error_x) < 50 and abs(error_y) < 50:
            self.get_logger().info("Aligned!")
            self.aligned = True
            msg = String()
            msg.data = "search_for_red_line_complete"
            self.status_pub.publish(msg)
        else:
            self.aligned = False

    def update_status(self):    
        if self.drone.get_rel_alt.data >= 4.0:
            msg = String()
            msg.data = "takeoff_complete"
            self.status_pub.publish(msg)
            self.get_logger().info("Takeoff complete!")
        
    def on_deactivate(self, state):
        self.get_logger().info("Navigation Node: Deactivated!")
        if self.timer:
            self.destroy_timer(self.timer)
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state):
        self.get_logger().info("Navigation Node: Cleaned up!")
        return TransitionCallbackReturn.SUCCESS

def main():
    rclpy.init()
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
