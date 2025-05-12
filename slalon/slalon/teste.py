from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32

class Teste(Node):
    def __init__(self):
        super().__init__("teste")


        self.sub = self.create_subscription(Float32, "depth_topic", self.sub_callback, 10)

    def sub_callback(self, msg):
        self.get_logger().info("To rodando!!!")
        if msg.data < 30:
            print("ta perto")

def main():
    rclpy.init()
    t = Teste()
    rclpy.spin(t)
    rclpy.shutdown()

