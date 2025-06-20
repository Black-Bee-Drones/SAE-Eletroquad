import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from mirela_sdk.image_processing.camera.image_handler import ImageHandler
import numpy as np
from PIL import Image
import cv2
import os
import sys

from .inference_onnx import YOLOv8, YoloDetection


class YOLOv8Node(Node):
    def __init__(self):
        super().__init__("yolov8_inference_node")

        self.declare_parameter(
            "model_path",
            os.path.abspath(
                os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), "models", "best.onnx"
                )
            ),
        )
        self.declare_parameter("conf_thres", 0.5)
        self.declare_parameter("iou_thres", 0.5)
        self.declare_parameter("image_source", "webcam")
        self.declare_parameter("c920_config", 2)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        conf_thres = self.get_parameter("conf_thres").get_parameter_value().double_value
        iou_thres = self.get_parameter("iou_thres").get_parameter_value().double_value
        image_source = (
            self.get_parameter("image_source").get_parameter_value().string_value
        )
        c920_config = (
            self.get_parameter("c920_config").get_parameter_value().integer_value
        )

        # Load YOLOv8 model
        self.yolo = YOLOv8(model_path)
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres

        # Publisher for detections
        # Structure of the Float32MultiArray:
        # [num_detections, det1_class_id, det1_center_x, det1_center_y, det1_width, det1_height, 
        #  det2_class_id, det2_center_x, det2_center_y, det2_width, det2_height, ...]
        self.detections_pub = self.create_publisher(
            Float32MultiArray, "/yolo_detections", 10
        )

        # ImageHandler
        self.image_handler = ImageHandler(
            node=self,
            image_source=image_source,
            image_processing_callback=self.process_image,
            show_result=None,
            c920_config=c920_config,
        )
        self.image_handler.run()

    def process_image(self, img: np.ndarray):
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(img_rgb)

        _, detections, inference_time_ms = self.yolo.detect(
            pil_img, conf_thres=self.conf_thres, iou_thres=self.iou_thres
        )

        print(f"Inference time: {inference_time_ms:.2f} ms")

        # Pack all detection information into a single message
        # Format: [num_detections, det1_class_id, det1_center_x, det1_center_y, det1_width, det1_height, ...]
        detection_data = [float(len(detections))]
        
        for det in detections:
            x1, y1, w, h = det.bbox
            cx = x1 + w / 2.0
            cy = y1 + h / 2.0
            
            # Add class_id, center_x, center_y, width, height for each detection
            detection_data.extend([
                float(det.class_id),
                float(cx),
                float(cy),
                float(w),
                float(h)
            ])
        
        # Publish detections
        detections_msg = Float32MultiArray()
        detections_msg.data = detection_data
        self.detections_pub.publish(detections_msg)

    def cleanup(self):
        self.image_handler.cleanup()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
