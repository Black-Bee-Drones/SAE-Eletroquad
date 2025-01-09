import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from mirela_sdk.image_processing.camera.oakd_cam import OakdCam
from mirela_sdk.image_processing.camera.image_handler import ImageHandler
import cv2
from cv_bridge import CvBridge
import numpy as np

class DepthDetection(Node):
    
    def __init__(self) -> None:
        super().__init__("depth_detection")
        
        # OakdCam init
        self.oakd = OakdCam()
        self.oakd.init_cam()

        # Fiquei um pouco confuso acerca de como acessar essa imagem, por hora foi o que eu montei.
        self.depth_output = ImageHandler(self, "oakd", show_result="Depth Image")
        self.depth_output.run()

        # Depth queue
        self.depth_queue = self.oakd.getQueue('Depth Image', maxSize=1, blocking=False)
        # Lib that convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()
        
        # Create a publisher for the depth image
        self.depth_publisher = self.create_publisher(Image, 'depth_image', 10)
        
        self.create_timer(0.1, self.depth_callback)

    def depth_callback(self):
        depth_frame = self.oakd.getFrame(self.depth_queue)
        if depth_frame is not None:
            # Convert the depth frame to a ROS Image message
            depth_image = self.bridge.cv2_to_imgmsg(depth_frame, encoding="passthrough")
            depth_array = np.array(depth_frame, dtype=np.float32)

            # Find the minimum distance value in the depth frame
            min_distance = np.min(depth_array)
            self.get_logger().info(f"The drone is {min_distance:.2f} meters away from an obstacle.")
            
            # Publish the depth image
            self.depth_publisher.publish(depth_image)

def main(args=None):
    rclpy.init(args=args)
    node = DepthDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()