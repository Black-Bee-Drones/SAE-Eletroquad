import cv2
import numpy as np
import json
import os
import rclpy
from rclpy.node import Node


TRACKBAR_WINDOW = "Trackbars"
JSON_FILE = os.path.expanduser("~/ros2_ws/src/mirela-sdk/mirela_sdk/mirela_sdk/image_processing/color/color_calibration.json")
COLOR_SEQUENCE = ["red_sl", "blue_sl", "black_sl", "pink_sl"]

def empty(arg):
    pass


class ColorCalibrationNode(Node):
    def __init__(self, cam_index=2):
        super().__init__("auto_color_calibrator")

        self.cam = cv2.VideoCapture(cam_index)
        self.current_index = 0
        self.color_data = self.load_existing_calibration()
        self.current_color = COLOR_SEQUENCE[self.current_index]

        self.init_trackbars()
        self.get_logger().info(f"Calibrando: {self.current_color}")

    def load_existing_calibration(self):
        if os.path.exists(JSON_FILE):
            with open(JSON_FILE, "r") as f:
                return json.load(f)
        return {}

    def init_trackbars(self):
        cv2.namedWindow(TRACKBAR_WINDOW)

        vals = self.color_data.get(self.current_color, {}).get("HSV", None)

        if vals:
            lower, upper = vals
        else:
            lower = [0, 0, 0]
            upper = [179, 255, 255]

        cv2.createTrackbar("HMax", TRACKBAR_WINDOW, upper[0], 179, empty)
        cv2.createTrackbar("HMin", TRACKBAR_WINDOW, lower[0], 179, empty)
        cv2.createTrackbar("SMax", TRACKBAR_WINDOW, upper[1], 255, empty)
        cv2.createTrackbar("SMin", TRACKBAR_WINDOW, lower[1], 255, empty)
        cv2.createTrackbar("VMax", TRACKBAR_WINDOW, upper[2], 255, empty)
        cv2.createTrackbar("VMin", TRACKBAR_WINDOW, lower[2], 255, empty)

    def get_trackbar_values(self):
        lower = [
            cv2.getTrackbarPos("HMin", TRACKBAR_WINDOW),
            cv2.getTrackbarPos("SMin", TRACKBAR_WINDOW),
            cv2.getTrackbarPos("VMin", TRACKBAR_WINDOW),
        ]
        upper = [
            cv2.getTrackbarPos("HMax", TRACKBAR_WINDOW),
            cv2.getTrackbarPos("SMax", TRACKBAR_WINDOW),
            cv2.getTrackbarPos("VMax", TRACKBAR_WINDOW),
        ]
        return lower, upper


    def save_current_color(self):
        lower, upper = self.get_trackbar_values()
        self.color_data[self.current_color] = {
            "HSV": [lower, upper]
        }
        with open(JSON_FILE, "w") as f:
            json.dump(self.color_data, f, indent=4)
        self.get_logger().info(f"'{self.current_color}' salva com sucesso.")

    def next_color(self):
        cv2.destroyAllWindows() 
        self.current_index += 1
        if self.current_index >= len(COLOR_SEQUENCE):
            self.get_logger().info("Todas as cores calibradas.")
            self.cam.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return
        self.current_color = COLOR_SEQUENCE[self.current_index]
        self.get_logger().info(f"Próxima cor: {self.current_color}")
        self.init_trackbars()

    def run(self):
        while rclpy.ok():
            ret, frame = self.cam.read()
            if not ret:
                self.get_logger().error("Não foi possível capturar a imagem.")
                break

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower, upper = self.get_trackbar_values()
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(frame, frame, mask=mask)

            stacked = np.hstack((frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), result))
            cv2.imshow(f"Calibrando: {self.current_color}", stacked)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                self.save_current_color()
            elif key == ord("n"):
                self.next_color()

        self.cam.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ColorCalibrationNode()
    node.run()
    node.destroy_node()
