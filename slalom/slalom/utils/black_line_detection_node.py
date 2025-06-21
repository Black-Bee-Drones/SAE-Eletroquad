#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from mirela_interfaces.msg import LineInfo
import cv2
import numpy as np
from mirela_sdk.image_processing.camera.image_handler import ImageHandler


class BlackLineDetectionNode(Node):
    def __init__(self):
        super().__init__("black_line_detection_node")

        # Declare parameters
        self.declare_parameter("cap", 2)
        self.declare_parameter("show_visualization", True)
        self.declare_parameter("image_source", "webcam")

        # Get parameters
        self.cap = self.get_parameter("cap").get_parameter_value().integer_value
        self.show_visualization = (
            self.get_parameter("show_visualization").get_parameter_value().bool_value
        )
        self.image_source = (
            self.get_parameter("image_source").get_parameter_value().string_value
        )

        # Create publishers
        self.publisher_ = self.create_publisher(LineInfo, "line_state/black_sl", 10)
        self.detected_pub = self.create_publisher(Bool, "line_detect/black_sl", 10)

        # Initialize image handler with processing callback
        self.visualization_title = (
            "Black Line Detection" if self.show_visualization else None
        )
        self.image_handler = ImageHandler(
            node=self,
            image_source=self.image_source,
            image_processing_callback=self.process_image,
            show_result=self.visualization_title,
            cap=self.cap,
        )

        # Log the configuration
        self.get_logger().info(
            f"Black Line Detection Node initialized with: \
            \n - Image source: {self.image_source} \
            \n - Camera index: {self.cap} \
            \n - Show visualization: {self.show_visualization}"
        )

        # Start the image handler
        self.image_handler.run()

    def process_image(self, frame):
        if frame is None:
            self.get_logger().warn("Received empty frame")
            return

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Threshold to get binary image (inverse for black detection)
        _, mask = cv2.threshold(
            gray, 80, 255, cv2.THRESH_BINARY_INV
        )  # Ajuste o threshold conforme necessário

        # Remove noise
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=1)
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)

        # Filter columns with less than 200 white pixels
        col_sums = np.sum(mask == 255, axis=0)
        for col in range(mask.shape[1]):
            if col_sums[col] < 200:
                mask[:, col] = 0  # Zero the entire column

        # Show the mask if visualization is enabled
        if self.show_visualization:
            cv2.imshow("Black Line - Mask", mask)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected = False
        msg = LineInfo()

        if contours:
            # Find the largest contour
            largest = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(largest)
            (cx, cy), (w, h), angle = rect

            # Fill LineInfo message
            msg.center_x = float(cx)
            msg.center_y = float(cy)
            msg.angle = float(angle)
            msg.width = float(w)
            msg.height = float(h)
            detected = True

            # Draw the rotated rectangle (estimated line)
            if self.show_visualization:
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

                # Add text with measurements
                cv2.putText(
                    frame,
                    f"Center: ({cx:.1f}, {cy:.1f})",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                )
                cv2.putText(
                    frame,
                    f"Size: {w:.1f} x {h:.1f}",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                )
                cv2.putText(
                    frame,
                    f"Angle: {angle:.1f}",
                    (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                )

        # Publish the line info and detection status
        self.publisher_.publish(msg)
        self.detected_pub.publish(Bool(data=detected))

    def destroy_node(self):
        # Clean up the image handler
        if hasattr(self, "image_handler"):
            self.image_handler.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BlackLineDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
