from mirela_sdk.control.mavros.mavros_api import MavDrone
from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from time import sleep
import time
from slalon.depth_st import DepthStateMachine

NOWHERE = 0
CENTER = 1
LEFT = 2
RIGHT = 3

class MovementStateMachine(Node):
    def __init__(self):
        self.drone = MavDrone(self)

        self.state = 0
        self.next_state = 1
        self.side = LEFT #controla de que lado o drone deve passar pela haste

        self.depth_st = DepthStateMachine()

        self.pub = self.create_subscription(Float32, "depth_topic", self.depth_callback, 10)

        self.find_sub = self.create_subscription(Int8, "where_is_it", self.find_object_callback, 10)

        self.object_location = 0

        self.distance_to_object = 0

        self.create_timer(0.1, self.check_state)

    def check_state(self):
        if self.state != self.next_state:
            self.state = self.next_state
            self.movement_st()

    def depth_callback(self, distance):
        self.distance_to_object = distance

    def find_object_callback(self, msg):
        self.object_location = msg.data

    def right_or_left(self):

        if self.object_location == NOWHERE:
            found = False
            start = time.now()
            now = start
            while now - start < 3:
                self.drone.offboard_velocity(0, 0, 0, 0.02, False) 
                rclpy.spin_once(self.drone.node) #Permite atualizar as variaveis
                                                 #Roda a fila de processos associados ao nó passado como parâmetro
                if self.object_location != NOWHERE:
                    found = True
                    self.drone.offboard_velocity_timer(0, 0, 0, -0.02, time=now - start)
                    return RIGHT
                
                now = time.now()
            
            

            if not found:
                start = time.now()
                now = start
                while now - start < 3: #3 segundos necessitam ser testados
                    self.drone.offboard_velocity(0, 0, 0, -0.02) #gira pra esquerda por 3 segundos
                    rclpy.spin_once(self.drone.node)

                    if self.object_location != NOWHERE:
                        found = True
                        self.drone.offboard_velocity_timer(0, 0, 0, 0.02, time=now - start)
                        return LEFT
                    
                    now = time.now()

        elif self.object_location == CENTER:
            return CENTER
        
        return NOWHERE

                
    
    def centralize(self, location):

        if location == NOWHERE:
            self.right_or_left()

        elif location == LEFT:
            while self.object_location != CENTER:
                self.drone.offboard_velocity(0, 0.02, 0, 0)
                rclpy.spin_once(self.drone.node)
        
        elif location == RIGHT:
            while self.object_location != CENTER:
                self.drone.offboard_velocity(0, -0.02, 0, 0)
                rclpy.spin_once(self.drone.node)            

    def move_foward(self):
        while self.distance_to_object > 200:
            self.drone.offboard_velocity(0.02, 0, 0, 0)
            rclpy.spin_once(self.drone.node)
        
        self.drone.offboard_velocity(0, 0, 0, 0, False)
    
    def pass_by(self):
        if self.side == LEFT:
            self.drone.offboard_velocity_timer(0, 0.02, 0, 0, time=5)
            self.side = RIGHT
        
        elif self.side == RIGHT:
            self.drone.offboard_velocity_timer(0, -0.02, 0, 0, time=5)
            self.side = LEFT
        
        self.drone.offboard_velocity_timer(0.02, 0, 0, 0, time=5) 


    def movement_st(self):

        count_pipe = 0
        if self.state == 0:
            self.drone.arm_takeoff(1.5)
            sleep(8)
            self.next_state = 1

        elif self.state == 1:
            location = self.right_or_left()
            self.next_state = 2

        elif self.state == 2:
            self.centralize(location)
            self.next_state = 3

        elif self.state == 3:
            self.move_foward()
            self.next_state = 4

        elif self.state == 4:
            self.pass_by()
            self.depth_st.switch_state() #Troca a cor buscada pela maquina de estados
            count_pipe += 1
            if count_pipe == 4:
                self.next_state = 5
            else:
                self.next_state = 1

        elif self.state == 5: #Estado final
            self.drone.land()


def main():
    rclpy.init()

    st = MovementStateMachine()

    rclpy.spin(st)

    rclpy.shutdown()