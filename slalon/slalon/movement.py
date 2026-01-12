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
        self.drone = MavDrone(self, False)

        self.state = 0
        #First side the drone must pass by the pipe
        self.side = LEFT  

        self.depth_st_pub = self.create_publisher(Int8, "switch_state", 10)

        self.depth_sub = self.create_subscription(Float32, "depth_topic", self.distance_callback, 10)
        self.find_sub = self.create_subscription(Int8, "where_is_it", self.find_object_callback, 10)
        self.changed_color_sub = self.create_subscription(Int8, "color_changed", self.changed_color_callback, 10)

        #Saves if pipe is at the CENTER, LEFT, RIGHT or NOWHERE at all times
        self.object_location = 0
        #Distance from the depth_callback
        self.distance_to_object = 0.0
        #Counts how many pipes the drone already passed by
        self.count_pipe = 0
        #Saves if the pipe was found at the left or right side of the drone when it looks 
        #for the pipe
        self.where_is_pipe = 0
        #Saves how many seconds the drone moved to the right or left so it can get back
        #to the center where it started (+ left, -right)
        self.lateral_position = 0.0
        #True if distance to pipe < 250 cm
        self.too_close = False

        self.changed_color_ok = 0

        # inicia a máquina de estados
        self.get_logger().info("Iniciando máquina de estados...")
        
    def changed_color_callback(self, msg):
        self.changed_color_ok = msg.data

    def distance_callback(self, msg):
        self.distance_to_object = msg.data
        if self.distance_to_object < 250:
            self.too_close = True
        else:
            self.too_close = False
    

    def find_object_callback(self, msg):
        self.object_location = msg.data

    def right_or_left(self):
        
        self.get_logger().info(f"self.object location: {self.object_location}")

        if self.object_location == NOWHERE:
            found = False
            start = time.time()
            now = time.time()
            while now - start < 3:
                #Yaws drone to the left to look for pipe
                self.drone.offboard_velocity(0.0, 0.0, 0.0, 0.5, False)
                rclpy.spin_once(self)
                
                if self.object_location != NOWHERE:
                    self.get_logger().info("Object found! Left side")
                    found = True
                    self.drone.offboard_velocity_timer(0.0, 0.0, 0.0, -0.5, time=now - start)
                    return self.object_location
                now = time.time()

            if found == False:
                self.get_logger().warning("NOT FOUND")
                self.drone.offboard_velocity_timer(0.0, 0.0, 0.0, -0.5, time=3.0)
                start = time.time()
                now = time.time()
                while now - start < 3:
                    #Yaws drone to the right
                    self.drone.offboard_velocity(0.0, 0.0, 0.0, -0.5)
                    rclpy.spin_once(self)
                    if self.object_location != NOWHERE:
                        self.get_logger().info("Object found! Right side")
                        self.drone.offboard_velocity_timer(0.0, 0.0, 0.0, 0.5, time=now - start)
                        return self.object_location
                    now = time.time()
                if found == False:
                    self.drone.offboard_velocity_timer(0.0, 0.0, 0.0, 0.5, time=3.0)

        return self.object_location

    def search(self):
        #Drone goes foward, then left, then right looking for the pipe
        #Repeats if no found
        while self.where_is_pipe == NOWHERE:
            #Returns drone to the center where it started based on self.lateral_position 
            vel = (-1) * abs(self.lateral_position)/self.lateral_position if self.lateral_position != 0 else 0.0
            self.drone.offboard_velocity_timer(0.0, vel, 0.0, 0.0, time=abs(self.lateral_position))
            self.lateral_position = 0

            self.where_is_pipe = self.right_or_left()

            if self.where_is_pipe == NOWHERE:
                #Gets closer and searches again
                self.drone.offboard_velocity_timer(0.5, 0.0, 0.0, 0.0, time=3)
                self.where_is_pipe = self.right_or_left()

                if self.where_is_pipe == NOWHERE:
                    #Searches going to the left
                    self.drone.offboard_velocity_timer(0.0, 1.0, 0.0, 0.0, time=3)
                    self.lateral_position += 3
                    self.where_is_pipe = self.right_or_left()
                    
                    if self.where_is_pipe == NOWHERE:
                        #Searches going to the right
                        self.drone.offboard_velocity_timer(0.0, -1.0, 0.0, 0.0, time=6)
                        self.lateral_position -= 6
                        self.where_is_pipe = self.right_or_left()
            rclpy.spin_once(self)

    def centralize(self):
        self.get_logger().info(f"Centralizing, where_is_pipe: {self.where_is_pipe}")
        if self.where_is_pipe == NOWHERE:
            self.search()

        if self.where_is_pipe == LEFT:
            self.get_logger().info(f"LEFT object location: {self.object_location}")
            start = time.time()
            now = time.time()

            #Moves drone until it centralizes for max of 6 seconds
            while self.object_location != CENTER:
                if now - start > 6:
                    self.get_logger().info("Timeout to centralize!")
                    return False
                
                self.get_logger().info("Moving drone to the left")
                self.drone.offboard_velocity(0.0, 0.3, 0.0, 0.0)
                now = time.time()
                rclpy.spin_once(self)
            self.lateral_position += now - start

        elif self.where_is_pipe == RIGHT:
            self.get_logger().info("RIGHT")
            start = time.time()
            now = time.time()

            #Moves drone until it centralizes or for max of 6 seconds
            while self.object_location != CENTER:
                if now - start > 6:
                    self.get_logger().info("Timeout to centralize!")
                    return False
                
                self.get_logger().info("Moving drone to the right")
                self.drone.offboard_velocity(0.0, -0.3, 0.0, 0.0)
                now = time.time()
                rclpy.spin_once(self)
            self.lateral_position -= now - start

        return True
        

    def move_foward(self):
        self.get_logger().info("Move_foward")
        while not self.too_close:
            self.get_logger().info("Drone going foward")
            self.drone.offboard_velocity(1.0, 0.0, 0.0, 0.0)
            rclpy.spin_once(self)

        self.drone.offboard_velocity(0.0, 0.0, 0.0, 0.0, False)

    def pass_by(self):
        if self.side == LEFT:
            self.get_logger().info("Moving drone to the left for 2 seconds")
            self.drone.offboard_velocity_timer(0.0, 1.0, 0.0, 0.0, time=2)
            self.lateral_position += 2
            self.side = RIGHT
        else:
            self.get_logger().info("Moving drone to the right for 2 seconds")
            self.drone.offboard_velocity_timer(0.0, -1.0, 0.0, 0.0, time=2)
            self.lateral_position -= 2
            self.side = LEFT

        self.get_logger().info("Moving foward for 3 seconds")
        self.drone.offboard_velocity_timer(1.0, 0.0, 0.0, 0.0, time=5)

    def movement_st(self):
        self.get_logger().info(f"Executing movement state: {self.state}")

        rclpy.spin_once(self)

        if self.state == 0:
            #Takes off and goes to the next state
            self.drone.arm_takeoff(1.5)
            sleep(8)
            self.change_state()

        elif self.state == 1:
            #Checks if pipe is at the left or at the right side by yawing the drone
            self.where_is_pipe = self.right_or_left()
            self.change_state()

        elif self.state == 2:
            #Sends drone to the side the pipe was found until ir centralizes with it
            #Times out if can't centralize and returns state to look for pipe again
            if self.centralize() == True:
                self.change_state()
            else:
                self.get_logger().info("Repeating search for pipe!")
                self.state = 1

        elif self.state == 3:
            #Moves the drone foward until no too close
            self.move_foward()
            self.change_state()

        elif self.state == 4:
            #Sends the drone to the side estipulated by self.side and then foward to 
            #pass by the pipe
            self.pass_by()
            self.count_pipe += 1
            self.change_state()

        elif self.state == 5:
            self.get_logger().info("Landing drone...")
            self.drone.land()

    def change_state(self):
        
        if self.state == 0:
            self.state = 1

        elif self.state == 1:
            self.state = 2

        elif self.state == 2:
            self.state = 3

        elif self.state == 3:
            self.state = 4
        
        elif self.state == 4:
            #If drone passed by all 4 pipes, land
            if self.count_pipe == 4:
                self.state = 5
            else:
                self.state = 1
                msg = Int8()
                #Sends msg to tell the DepthMeasurement to change the color being filtered
                self.depth_st_pub.publish(msg)
                self.get_logger().info("Repeating states for next pipe...")
                while self.changed_color_ok != 1:
                    rclpy.spin_once(self)
                self.changed_color_ok = 0


def main():
    rclpy.init()
    st = MovementStateMachine()
     
    while rclpy.ok():
        rclpy.spin_once(st)
        st.movement_st()

    rclpy.shutdown()

if __name__ == "__main__":
    main()