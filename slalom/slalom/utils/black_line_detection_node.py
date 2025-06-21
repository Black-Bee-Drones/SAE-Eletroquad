#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from mirela_interfaces.msg import LineInfo
import cv2
import numpy as np


class BlackLineDetectionNode(Node):
    def __init__(self):
        super().__init__("black_line_detection_node")
        self.publisher_ = self.create_publisher(LineInfo, "line_state/black_sl", 10)
        self.detected_pub = self.create_publisher(Bool, "line_detect/black_sl", 10)
        self.cap = cv2.VideoCapture(2)  # Ajuste o índice se necessário
        self.timer = self.create_timer(0.1, self.process_image)

    def process_image(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(
            gray, 80, 255, cv2.THRESH_BINARY_INV
        )  # Ajuste o threshold conforme necessário

        # Remover ruído
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=1)
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)

        # Filtrar colunas com menos de 200 pixels brancos
        col_sums = np.sum(
            mask == 255, axis=0
        )  # Conta quantos pixels brancos há em cada coluna
        for col in range(mask.shape[1]):
            if col_sums[col] < 200:
                mask[:, col] = 0  # Zera a coluna inteira

        # Visualização
        cv2.imshow("Black Line - Original", frame)
        cv2.imshow("Black Line - Mask", mask)
        cv2.waitKey(1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected = False
        msg = LineInfo()
        if contours:

            largest = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(largest)
            (cx, cy), (w, h), angle = rect
            msg.center_x = float(cx)
            msg.center_y = float(cy)
            msg.angle = float(angle)
            msg.width = float(w)
            msg.height = float(h)
            detected = True

            # Desenhar o retângulo rotacionado (linha estimada)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            frame = cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

        self.publisher_.publish(msg)
        self.detected_pub.publish(Bool(data=detected))

        # Atualizar visualização com linha desenhada
        cv2.imshow("Black Line - Original", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BlackLineDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
