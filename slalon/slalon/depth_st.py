from depth_measurement import DepthMeasurement
import numpy as np
from rclpy.node import Node
import rclpy
ROSA = 0
VERMELHO = 1
AZUL = 2
PRETO = 3

class DepthStateMachine(Node):
    def __init__(self):
        super().__init__("DepthNode")
        self.state = ROSA
        self.depth = DepthMeasurement(2) 
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

        self.run()

        self.cont = 0

    def switch_state(self):
        
        if self.state == ROSA:
            self.depth.set_ranges(
                self.lower_red1, self.upper_red1, self.lower_red2, self.upper_red2,
            )
            self.state = VERMELHO

        elif self.state == VERMELHO:
            self.depth.set_ranges(self.lower_blue, self.upper_blue)
            self.state = AZUL
        
        elif self.state == AZUL:
            self.depth.set_ranges(self.lower_black, self.upper_black)
            self.state = PRETO

        elif self.state == PRETO:
            self.depth.set_ranges(self.lower_pink, self.upper_pink)
            self.state = ROSA
        
        else:
            self.state = ROSA

    def teste(self):
        self.cont += 1
        print("EBAAAAAAA")

        if self.cont == 10:
            print("ALOOOOOOOOO")

            self.switch_state()
            
    def run(self):
        self.get_logger().info("rodei")
        self.create_timer(0.001, self.depth.depth_callback)
        self.create_timer(1, self.teste)


rclpy.init()

st = DepthStateMachine()
rclpy.spin(st)


rclpy.shutdown()
