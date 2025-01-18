import math
import numpy as np
import depthai as dai
from mirela_sdk.image_processing.camera.oakd_cam import OakdCam
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from .spatial_calc import HostSpatialsCalc
import cv2

class ObstacleDetectionNode(Node):
    """
    ROS 2 Node for detecting obstacles using an OAK-D camera and publishing their spatial coordinates.

    Main Features:
    - Reads depth data from an OAK-D camera.
    - Dynamically computes spatial coordinates of obstacles in a region of interest (ROI).
    - Publishes the obstacle's spatial data (x, y, z) as a ROS message.
    - Optionally visualizes the depth frame with the ROI overlaid.
    """

    def __init__(self, enable_visualization=True):
        """
        Initialize the node, OAK-D pipeline, and set up publishers.

        :param enable_visualization: If True, shows a live visualization of the depth frame with the ROI.
        """
        super().__init__('obstacle_detection')  # Initialize ROS node with a name

        # Enable or disable visualization
        self.enable_visualization = enable_visualization

        # Publisher for obstacle spatial data
        self.publisher = self.create_publisher(Point, 'obstacle_distance', 10)

        # Initialize OAK-D camera
        self.oakd = OakdCam()
        self.oakd.get_stereo_depth()  # Set up stereo depth pipeline
        self.oakd.configure_stereo_node_output(["depth"])  # Configure which outputs to enable
        self.device = self.oakd.init_cam()  # Initialize the OAK-D device
        self.depth_queue = self.oakd.getQueue("depth", maxSize=1, blocking=False)  # Queue for depth data

        # Spatial calculator to compute (x, y, z) based on depth and ROI
        self.spatials_calc = HostSpatialsCalc(self.device)

        # Timer to periodically read data and publish it (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Default ROI dimensions
        self.roi_width = 80  # ROI width (in pixels)
        self.roi_height = 300  # ROI height (in pixels)
        self.roi = None  # ROI coordinates, to be calculated dynamically

    def centralize_roi(self, frame_width, frame_height, roi_width, roi_height):
        """
        Calculate the coordinates for a centralized ROI within the frame.

        :param frame_width: Width of the depth frame.
        :param frame_height: Height of the depth frame.
        :param roi_width: Desired width of the ROI.
        :param roi_height: Desired height of the ROI.
        :return: Centralized ROI as (xmin, ymin, xmax, ymax).
        """
        center_x = frame_width // 2
        center_y = frame_height // 2

        xmin = max(center_x - roi_width // 2, 0)
        ymin = max(center_y - roi_height // 2, 0)
        xmax = min(center_x + roi_width // 2, frame_width - 1)
        ymax = min(center_y + roi_height // 2, frame_height - 1)

        return xmin, ymin, xmax, ymax

    def timer_callback(self):
        """
        Periodically called to process depth data, calculate spatial coordinates, and publish them.
        If visualization is enabled, displays the depth frame with ROI.
        """
        # Get the depth frame
        depth_data = self.depth_queue.get()
        depth_frame = depth_data.getFrame()

        # Get frame dimensions
        frame_width = depth_frame.shape[1]
        frame_height = depth_frame.shape[0]

        # Centralize ROI based on frame dimensions
        self.roi = self.centralize_roi(frame_width, frame_height, self.roi_width, self.roi_height)

        # Calculate spatial data within the ROI
        spatials, centroid = self.spatials_calc.calc_spatials(depth_data, self.roi)

        # Publish the spatial data as a ROS Point message
        point_msg = Point()
        point_msg.x = spatials['x']
        point_msg.y = spatials['y']
        point_msg.z = spatials['z']
        self.publisher.publish(point_msg)

        # Log the spatial data for debugging
        self.get_logger().info(f"Published spatials: {spatials}")

        # Visualization (if enabled)
        if self.enable_visualization:
            # Normalize the depth frame for color mapping
            normalized_depth = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

            # Apply a color map to the normalized depth frame
            color_mapped_frame = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)

            # Draw the ROI on the color-mapped frame
            xmin, ymin, xmax, ymax = self.roi
            cv2.rectangle(color_mapped_frame, (xmin, ymin), (xmax, ymax), (255, 255, 255), 2)

            # Display the frame
            cv2.imshow("Depth Frame with Centralized ROI", color_mapped_frame)

            # Handle key inputs for quitting visualization
            key = cv2.waitKey(1)
            if key == ord('q'):  # Quit visualization and shutdown node
                rclpy.shutdown()

def main(args=None):
    """
    Main entry point for the ROS node. Initializes and runs the node.
    """
    rclpy.init(args=args)

    # Enable visualization (set to False to disable it)
    enable_visualization = False
    node = ObstacleDetectionNode(enable_visualization)

    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass  # Allow graceful shutdown on Ctrl+C
    finally:
        node.destroy_node()  # Destroy the node resources
        rclpy.shutdown()  # Shutdown ROS 2

if __name__ == '__main__':
    main()
