import numpy as np
from rclpy.node import Node
import rclpy
from std_msgs.msg import Int8
import cv2
from std_msgs.msg import Float32
from slalon.depth_measurement import DepthMeasurement
from mirela_sdk.image_processing.color import ColorDetector

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
    def __init__(self, cap = 0):
        super().__init__(cap)

        self.state = START
        self.next_state = BLACK

        self.depth_st_sub = self.create_subscription(Int8, "switch_state", self.switch_state_callback, 10)

        self.changed_color_pub = self.create_publisher(Int8, "color_changed", 10)

        self.blue_detector = ColorDetector("preset", "blue_sl")
        self.red_detector = ColorDetector("preset", "red_sl")
        self.black_detector = ColorDetector("preset", "black_sl")
        self.pink_detector = ColorDetector("preset", "pink_sl")


        self.cont = 0
        
        self.run()

    def switch_state_callback(self, msg):
        self.switch_state()

    def switch_state(self):
        self.get_logger().info("Depth_StateMachine changing state...")
        if self.next_state == PINK:
            self.get_logger().info("Filtering PINK")
            self.state = PINK
            self.detector = self.pink_detector
            self.next_state = RED

        elif self.next_state == RED:
            self.get_logger().info("Filtering RED")
            self.state = RED
            self.detector = self.red_detector
            self.next_state = BLUE
        
        elif self.next_state == BLUE:
            self.get_logger().info("Filtering BLUE")
            self.state = BLUE
            self.detector = self.blue_detector
            self.next_state = BLACK

        elif self.next_state == BLACK:
            self.get_logger().info("Filtering BLACK")
            self.state = BLACK
            self.detector = self.black_detector
            self.next_state = PINK
        

        msg = Int8()
        msg.data = 1
        self.changed_color_pub.publish(msg)

    def teste(self):
        #so pra testar se a mudança de estados tava funcionando
        self.cont += 1
        self.get_logger().info("opa")

        if self.cont == 10:
            self.get_logger().info("alooooooooooooooooooooooooooooo")
            self.cont = 0
            self.switch_state()
            
    def run(self):
        self.switch_state()
        self.get_logger().info("rodei")
        self.create_timer(1/30, self.depth_callback)
        self.create_timer(1/30, self.find_object)
        #self.create_timer(1, self.teste)

def main(args=None):
    rclpy.init()

    import argparse

    parser = argparse.ArgumentParser(description="Depth Measurement")

    parser.add_argument(
        "--cap", type=int, default=None, help="Camera index"
    )
    parsed_args, remaining_args = parser.parse_known_args(args=args)

    st = DepthStateMachine(cap=parsed_args.cap)
    rclpy.spin(st)


    rclpy.shutdown()

if __name__ == "__main__":
    main()
