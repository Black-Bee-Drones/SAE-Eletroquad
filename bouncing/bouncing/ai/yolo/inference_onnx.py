import argparse
import os

import cv2
import numpy as np
from PIL import Image
import onnxruntime as ort
from ultralytics.utils.checks import check_requirements
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class YoloDetection:
    bbox: Tuple[int, int, int, int]  # (x1, y1, w, h)
    score: float
    class_id: int


class YOLOv8:
    """YOLOv8 object detection model class for handling inference and visualization."""

    def __init__(self, onnx_model):
        """
        Initializes an instance of the YOLOv8 class.

        Args:
            onnx_model: Path to the ONNX model.
            input_image: Path to the input image.
            confidence_thres: Confidence threshold for filtering detections.
            iou_thres: IoU (Intersection over Union) threshold for non-maximum suppression.
        """
        self.onnx_model = onnx_model

        self.classes = ["0", "1", "2", "3", "4", "5", "6", "7"]
        self.color_palette = np.random.uniform(0, 255, size=(len(self.classes), 3))

        options = ort.SessionOptions()
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL
        self.session = ort.InferenceSession(self.onnx_model, options)
        self.session.set_providers(
            ["OpenVINOExecutionProvider"], [{"device_type": "CPU"}]
        )

        self.input_width = 320
        self.input_height = 320

    def draw_detections(self, img, box, score, class_id):
        """
        Draws bounding boxes and labels on the input image based on the detected objects.

        Args:
            img: The input image to draw detections on.
            box: Detected bounding box.
            score: Corresponding detection score.
            class_id: Class ID for the detected object.

        Returns:
            None
        """
        x1, y1, w, h = box

        color = self.color_palette[class_id]

        cv2.rectangle(img, (int(x1), int(y1)), (int(x1 + w), int(y1 + h)), color, 2)

        label = f"{self.classes[class_id]}: {score:.2f}"
        (label_width, label_height), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
        )
        label_x = x1
        label_y = y1 - 10 if y1 - 10 > label_height else y1 + 10

        cv2.rectangle(
            img,
            (label_x, label_y - label_height),
            (label_x + label_width, label_y + label_height),
            color,
            cv2.FILLED,
        )
        cv2.putText(
            img,
            label,
            (label_x, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )

    def preprocess(self, img: Image.Image) -> np.ndarray:
        """
        Preprocesses the image for inference.

        Args:
            img: The image to process.

        Returns:
            tuple: A tuple containing the processed image data and the original image.
        """
        # Convert PIL Image to cv2 format
        img_cv2 = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

        self.img_height, self.img_width = img_cv2.shape[:2]

        # Convert back to RGB for processing
        img_rgb = cv2.cvtColor(img_cv2, cv2.COLOR_BGR2RGB)

        # Resize
        img_resized = cv2.resize(img_rgb, (self.input_width, self.input_height))

        # Normalize and transpose
        image_data = np.array(img_resized) / 255.0
        image_data = np.transpose(image_data, (2, 0, 1))
        image_data = np.expand_dims(image_data, axis=0).astype(np.float32)

        return image_data, img_cv2

    def postprocess(
        self,
        input_image: np.ndarray,
        output: np.ndarray,
        conf_thres: float,
        iou_thres: float,
    ) -> Tuple[np.ndarray, List[YoloDetection]]:
        """
        Postprocesses the output from inference.
        Returns the output image with detections drawn and a list of detection results.
        """
        outputs = np.transpose(np.squeeze(output[0]))
        rows = outputs.shape[0]
        boxes = []
        scores = []
        class_ids = []
        detections: List[YoloDetection] = []
        x_factor = self.img_width / self.input_width
        y_factor = self.img_height / self.input_height
        for i in range(rows):
            classes_scores = outputs[i][4:]
            max_score = np.amax(classes_scores)
            if max_score >= conf_thres:
                class_id = int(np.argmax(classes_scores))
                x, y, w, h = outputs[i][0], outputs[i][1], outputs[i][2], outputs[i][3]
                left = int((x - w / 2) * x_factor)
                top = int((y - h / 2) * y_factor)
                width = int(w * x_factor)
                height = int(h * y_factor)
                class_ids.append(class_id)
                scores.append(max_score)
                boxes.append([left, top, width, height])
        indices = cv2.dnn.NMSBoxes(boxes, scores, conf_thres, iou_thres)
        for i in indices:
            box = boxes[i]
            score = scores[i]
            class_id = class_ids[i]
            self.draw_detections(input_image, box, score, class_id)
            detections.append(
                YoloDetection(bbox=tuple(box), score=score, class_id=class_id)
            )
        return cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB), detections

    def detect(
        self, image: Image.Image, conf_thres: float = 0.25, iou_thres: float = 0.5
    ) -> Tuple[Image.Image, List[YoloDetection]]:
        """
        Detects signatures in the given image.
        Returns the output image and a list of detection results.
        """
        img_data, original_image = self.preprocess(image)
        outputs = self.session.run(None, {self.session.get_inputs()[0].name: img_data})
        output_image, detections = self.postprocess(
            original_image, outputs, conf_thres, iou_thres
        )
        return output_image, detections


if __name__ == "__main__":
    default_model_path = os.path.abspath(
        os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "best.onnx",
        )
    )

    if not os.path.exists(default_model_path):
        print(f"Model not found at: {default_model_path}")
        print(
            "Check the utils folder and scripts for download the model. Or specify the path to the model using the --model argument."
        )
        exit(1)

    default_image_path = os.path.abspath(
        os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "2149_jpg.rf.28d3240f41bf9c1ced239168877ef667.jpg",
        )
    )

    if not os.path.exists(default_image_path):
        print(f"Image not found at: {default_image_path}")
        print(
            "Check the data folder and scripts for download the image. Or specify the path to the image using the --img argument."
        )
        exit(1)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", type=str, default=default_model_path, help="Input your ONNX model."
    )
    parser.add_argument(
        "--img", type=str, default=default_image_path, help="Path to input image."
    )
    parser.add_argument(
        "--conf-thres", type=float, default=0.5, help="Confidence threshold"
    )
    parser.add_argument(
        "--iou-thres", type=float, default=0.5, help="NMS IoU threshold"
    )
    args = parser.parse_args()

    # Check the requirements and select the appropriate backend (CPU or GPU)
    # check_requirements(
    #     "onnxruntime-gpu" if torch.cuda.is_available() else "onnxruntime"
    # )

    detection = YOLOv8(args.model)

    # Load the image using PIL
    image = Image.open(args.img)

    # Run detection
    output_image, detections = detection.detect(
        image, conf_thres=args.conf_thres, iou_thres=args.iou_thres
    )

    # Convert output_image (PIL Image) to OpenCV format for display
    output_image_cv2 = cv2.cvtColor(np.array(output_image), cv2.COLOR_RGB2BGR)
    cv2.imshow("Output", output_image_cv2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
