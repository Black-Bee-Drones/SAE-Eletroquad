import numpy as np
from rclpy.node import Node
import rclpy
from std_msgs.msg import Int8
import cv2
from std_msgs.msg import Float32
from slalon.depth_measurement import DepthMeasurement

#Movimentação do drone conforme as cores
#Altura max do drone é de 2.5 metros
#O lado que se deve percorrer a primeira trave é fornecido no dia da prova

#Cores traves: Preto fosco, Azul escuro, Rosa claro, Vermelho
        
START = 0
PINK = 1
RED = 2
BLUE = 3
BLACK = 4

class DepthStateMachine(DepthMeasurement):
    def __init__(self):
        super().__init__(0)


        self.state = START
        self.next_state = RED

        self.depth_st_sub = self.create_subscription(Int8, "switch_state", self.switch_state_callback, 10)

        self.changed_color_pub = self.create_publisher(Int8, "color_changed", 10)

        self.lower_pink = np.array([115, 62, 85]) 
        self.upper_pink = np.array([179, 255, 255])
        self.lower_range2 = None
        self.upper_range2 = None

        self.lower_blue = np.array([86, 162, 118])
        self.upper_blue = np.array([127, 255, 255])

        self.lower_black = np.array([100, 92, 0])
        self.upper_black = np.array([138, 166, 161])

        self.lower_red1 = np.array([0, 175, 117])
        self.upper_red1 = np.array([20, 255, 203])
        self.lower_red2 = np.array([151, 137, 100])
        self.upper_red2 = np.array([179, 252, 255])

        self.cont = 0
        
        self.run()

    def switch_state_callback(self, msg):
        self.switch_state()

    def switch_state(self):
        self.get_logger().info("Depth_StateMachine changing state...")
        if self.next_state == PINK:
            self.get_logger().info("Filtering PINK")
            self.state = PINK
            self.set_ranges(self.lower_pink, self.upper_pink)
            self.next_state = RED

        elif self.next_state == RED:
            self.get_logger().info("Filtering RED")
            self.state = RED
            self.set_ranges(
                self.lower_red1, self.upper_red1, self.lower_red2, self.upper_red2,
            )
            self.next_state = BLUE
        
        elif self.next_state == BLUE:
            self.get_logger().info("Filtering BLUE")
            self.state = BLUE
            self.set_ranges(self.lower_blue, self.upper_blue)
            self.next_state = BLACK

        elif self.next_state == BLACK:
            self.get_logger().info("Filtering BLACK")
            self.state = BLACK
            self.set_ranges(self.lower_black, self.upper_black)
            self.next_state = RED
        
        else:
            self.state = START
            self.next_state = PINK

        msg = Int8()
        msg.data = 1
        self.changed_color_pub.publish(msg)

    def teste(self):
        #so pra testar se a mudança de estados tava funcionando
        self.cont += 1
        print("AAAAAAAAA")

        if self.cont == 10:
            print("ALOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
            self.cont = 0
            self.switch_state()
            
    def run(self):
        self.switch_state()
        self.get_logger().info("rodei")
        self.create_timer(1/30, self.depth_callback)
        self.create_timer(1/30, self.find_object)

def main():
    rclpy.init()

    st = DepthStateMachine()
    rclpy.spin(st)


    rclpy.shutdown()

if __name__ == "__main__":
    main()
