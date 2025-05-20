from mirela_sdk.control.mavros.mavros_api import MavDrone
from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from time import sleep
import time

NOWHERE = 0
CENTER = 1
LEFT = 2
RIGHT = 3

class MovementStateMachine(Node):
    def __init__(self):
        super().__init__("movement")
        self.drone = MavDrone(self)

        self.state = 0
        self.side = LEFT  # controla de que lado o drone deve passar pela haste

        self.depth_st_pub = self.create_publisher(Int8, "switch_state", 10)

        self.depth_sub = self.create_subscription(Float32, "depth_topic", self.distance_callback, 10)
        self.find_sub = self.create_subscription(Int8, "where_is_it", self.find_object_callback, 10)

        self.object_location = 0
        self.distance_to_object = 0.0
        self.count_pipe = 0
        self.where_is_pipe = 0

        # inicia a máquina de estados
        self.get_logger().info("Iniciando máquina de estados...")
        

    def distance_callback(self, msg):
        self.distance_to_object = msg.data

    def find_object_callback(self, msg):
        self.object_location = msg.data

    def right_or_left(self):
        
        self.get_logger().info(f"{self.object_location}")

        if self.object_location == NOWHERE:
            found = False
            start = time.time()
            now = time.time()
            while now - start < 3:
                self.drone.offboard_velocity(0.0, 0.0, 0.0, 0.2, False)
                rclpy.spin_once(self)
                
                if self.object_location != NOWHERE:
                    self.get_logger().info("Object found! Left side")
                    found = True
                    self.drone.offboard_velocity_timer(0.0, 0.0, 0.0, -0.2, time=now - start)
                    return self.object_location
                now = time.time()

            if not found:
                start = time.time()
                now = time.time()
                while now - start < 3:
                    self.drone.offboard_velocity(0.0, 0.0, 0.0, -0.2)
                    rclpy.spin_once(self)
                    if self.object_location != NOWHERE:
                        self.get_logger().info("Object found! Right side")
                        self.drone.offboard_velocity_timer(0.0, 0.0, 0.0, 0.2, time=now - start)
                        return self.object_location
                    now = time.time()

        return self.object_location

    def centralize(self):
        self.get_logger().info(f"entrei center {self.where_is_pipe}")
        if self.where_is_pipe == NOWHERE:
            self.get_logger().info("NOWHERE")
            self.where_is_pipe = self.right_or_left()
            #self.centralize()

        if self.where_is_pipe == LEFT:
            self.get_logger().info(f"LEFT object location: {self.object_location}")
            while self.object_location != CENTER:
                self.get_logger().info("Moving drone to the left")
                self.drone.offboard_velocity(0.0, 0.2, 0.0, 0.0)
                rclpy.spin_once(self)

        elif self.where_is_pipe == RIGHT:
            self.get_logger().info("RIGHT")
            while self.object_location != CENTER:
                self.get_logger().info("Moving drone to the right")
                self.drone.offboard_velocity(0.0, -0.2, 0.0, 0.0)
                rclpy.spin_once(self)

    def move_foward(self):
        while self.distance_to_object > 200:
            self.get_logger().info("Drone going foward")
            self.drone.offboard_velocity(0.2, 0.0, 0.0, 0.0)
            rclpy.spin_once(self)

        self.drone.offboard_velocity(0.0, 0.0, 0.0, 0.0, False)

    def pass_by(self):
        if self.side == LEFT:
            self.get_logger().info("Moving drone to the left for 5 seconds")
            self.drone.offboard_velocity_timer(0.0, 0.2, 0.0, 0.0, time=5)
            self.side = RIGHT
        else:
            self.get_logger().info("Moving drone to the right for 5 seconds")
            self.drone.offboard_velocity_timer(0.0, -0.2, 0.0, 0.0, time=5)
            self.side = LEFT

        self.get_logger().info("Moving foward for 5 seconds")
        self.drone.offboard_velocity_timer(0.2, 0.0, 0.0, 0.0, time=5)

    def movement_st(self):
        self.get_logger().info(f"Executing movement state: {self.state}")

        rclpy.spin_once(self)

        if self.state == 0:
            # self.drone.arm_takeoff(1.5)
            sleep(5)
            self.state = 1
            self.movement_st()

        elif self.state == 1:
            self.where_is_pipe = self.right_or_left()
            self.state = 2
            self.movement_st()

        elif self.state == 2:
            self.centralize()
            self.state = 3
            self.movement_st()

        elif self.state == 3:
            self.move_foward()
            self.state = 4
            self.movement_st()

        elif self.state == 4:
            self.pass_by()
            msg = Int8()
            self.depth_st_pub.publish(msg)
            self.count_pipe += 1
            if self.count_pipe == 4:
                self.state = 5
            else:
                self.state = 1
                self.get_logger().info("Repeating states for next pipe...")
            self.movement_st()

        elif self.state == 5:
            self.get_logger().info("Landing drone...")
            self.drone.land()

def main():
    rclpy.init()
    st = MovementStateMachine()
     
    while rclpy.ok():
        rclpy.spin_once(st)
        st.movement_st()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
