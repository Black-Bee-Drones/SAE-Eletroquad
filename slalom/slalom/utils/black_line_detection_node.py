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

    def calc_width_height(self, mask):
        """
        Calculate the width and height of the detected line.

        Args:
            mask: Binary image containing the detected line

        Returns:
            Tuple containing width, height, x, y, w, h, and rotated box points
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0, 0.0, 0, 0, 0, 0, None  # No line detected

        largest_contour = max(contours, key=cv2.contourArea)

        rect = cv2.minAreaRect(largest_contour)
        (x_center, y_center), (w_rect, h_rect), angle_rect = rect

        box = cv2.boxPoints(rect)
        box = np.intp(box)

        x, y, w, h = cv2.boundingRect(box)

        # Ensure x, y, w, h are within image bounds
        y = max(0, y)
        x = max(0, x)
        h = min(h, mask.shape[0] - y)
        w = min(w, mask.shape[1] - x)

        if w > 0 and h > 0:
            roi = mask[y : y + h, x : x + w]

            threshold = 128  # Consider pixels with values > 128 as white
            white_pixels = roi > threshold

            if np.any(white_pixels):
                # Calculate non-zero columns (for height) using the threshold
                col_sums = np.sum(white_pixels, axis=0)
                non_zero_cols = col_sums[col_sums > 0]
                height = float(np.mean(non_zero_cols)) if len(non_zero_cols) > 0 else 0.0

                # Calculate non-zero rows (for width) using the threshold
                row_sums = np.sum(white_pixels, axis=1)
                non_zero_rows = row_sums[row_sums > 0]
                width = float(np.mean(non_zero_rows)) if len(non_zero_rows) > 0 else 0.0
            else:
                # If no white pixels found with threshold, try using the contour directly
                contour_area = cv2.contourArea(largest_contour)
                if contour_area > 0:
                    # Estimate width and height from contour
                    width = h_rect if h_rect > w_rect else w_rect
                    height = w_rect if h_rect > w_rect else h_rect
                else:
                    width = height = 0.0
        else:
            # Invalid ROI dimensions
            width = height = 0.0

        return width, height, x, y, w, h, box

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
            (cx, cy), (_, _), angle = rect

            # Calculate width and height using the improved method
            width, height, x, y, w, h, box = self.calc_width_height(mask)

            # Fill LineInfo message
            msg.center_x = float(cx)
            msg.center_y = float(cy)
            msg.angle = float(angle)
            msg.width = float(width)
            msg.height = float(height)
            detected = True

            # Draw the rotated rectangle (estimated line)
            if self.show_visualization:
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
                    f"Size: {width:.1f} x {height:.1f}",
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
