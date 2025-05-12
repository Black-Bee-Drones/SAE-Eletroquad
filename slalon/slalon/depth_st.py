from slalon.depth_measurement import DepthMeasurement
import numpy as np
from rclpy.node import Node
import rclpy

START = 0
PINK = 1
RED = 2
BLUE = 3
BLACK = 4

class DepthStateMachine(DepthMeasurement):
    def __init__(self):
        super().__init__(2)


        self.state = START
        self.next_state = PINK

        #self.depth = DepthMeasurement(2) 

        #criar subscriber pra mudar de estado quando necessario

        self.lower_pink = np.array([115, 62, 85]) 
        self.upper_pink = np.array([179, 255, 255])
        self.lower_range2 = None
        self.upper_range2 = None

        self.lower_blue = np.array([75, 136, 90])
        self.upper_blue = np.array([129, 255, 199])

        self.lower_black = np.array([73, 0, 11])
        self.upper_black = np.array([103, 46, 122])

        self.lower_red1 = np.array([0, 175, 117])
        self.upper_red1 = np.array([20, 255, 203])
        self.lower_red2 = np.array([169, 128, 140])
        self.upper_red2 = np.array([179, 255, 223])

        self.cont = 0
        
        self.run()

    def switch_state(self):
        
        if self.next_state == PINK:
            self.state = PINK
            self.set_ranges(self.lower_pink, self.upper_pink)
            self.next_state = RED

        elif self.next_state == RED:
            self.state = RED
            self.set_ranges(
                self.lower_red1, self.upper_red1, self.lower_red2, self.upper_red2,
            )
            self.next_state = BLUE
        
        elif self.next_state == BLUE:
            self.state = BLUE
            self.set_ranges(self.lower_blue, self.upper_blue)
            self.next_state = BLACK

        elif self.next_state == BLACK:
            self.state = BLACK
            self.set_ranges(self.lower_black, self.upper_black)
            self.next_state = PINK
        
        else:
            self.state = START
            self.next_state = PINK

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
        self.create_timer(0.001, self.depth_callback)

def main():
    rclpy.init()

    st = DepthStateMachine()
    rclpy.spin(st)


    rclpy.shutdown()
