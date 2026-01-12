import rclpy
from std_msgs.msg import Int8
from slalon.depth_measurement import DepthMeasurement
from mirela_sdk.image_processing.color import ColorDetector

#States
START = 0
PINK = 1
RED = 2
BLUE = 3
BLACK = 4
END = 5

class DepthStateMachine(DepthMeasurement):
    def __init__(self, cap = 0):
        super().__init__(cap)

        self.state = START


        #Color of the first pipe
        self.next_state = BLACK 

        #State machine changes color when drone passes by a pipe
        self.depth_st_sub = self.create_subscription(Int8, "switch_state", self.switch_state_callback, 10)

        #Signals the movement node that the color was changed
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
        #Changes the color being filtered
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
            self.next_state = END

        elif self.next_state == BLACK:
            self.get_logger().info("Filtering BLACK")
            self.state = BLACK
            self.detector = self.black_detector
            self.next_state = PINK
        

        msg = Int8()
        msg.data = 1
        #Signals movement node
        self.changed_color_pub.publish(msg)

            
    def run(self):
        self.switch_state()
        self.get_logger().info("Starting State Machine...")

        #Always calling the function to calculate the distance from the pipe
        #and its position in the screen
        self.create_timer(1/30, self.depth_callback)

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