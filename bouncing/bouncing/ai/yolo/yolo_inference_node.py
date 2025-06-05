import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
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
                os.path.join(os.path.dirname(os.path.abspath(__file__)), "best.onnx")
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

        # Publishers
        self.center_pub = self.create_publisher(
            Float32MultiArray, "/yolo_detections/centers", 10
        )
        self.ids_pub = self.create_publisher(
            Int32MultiArray, "/yolo_detections/ids", 10
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

        _, detections = self.yolo.detect(
            pil_img, conf_thres=self.conf_thres, iou_thres=self.iou_thres
        )

        centers = []
        ids = []
        for det in detections:
            x1, y1, w, h = det.bbox
            cx = x1 + w / 2.0
            cy = y1 + h / 2.0
            centers.extend([float(cx), float(cy)])
            ids.append(int(det.class_id))
        # Publish
        center_msg = Float32MultiArray()
        center_msg.data = centers
        ids_msg = Int32MultiArray()
        ids_msg.data = ids
        self.center_pub.publish(center_msg)
        self.ids_pub.publish(ids_msg)

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
