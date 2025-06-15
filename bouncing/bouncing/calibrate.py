import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ThresholdCalibrator(Node):
    def __init__(self):
        super().__init__('threshold_calibrator')

        self.bridge = CvBridge()
        self.image = None
        self.json_path = os.path.join(os.path.dirname(__file__), "threshold_config.json")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            qos_profile
        )

        cv2.namedWindow("Threshold Control")
        cv2.createTrackbar("Threshold", "Threshold Control", 127, 255, self.nothing)

        self.get_logger().info("Calibrador de threshold LAB iniciado. Pressione 's' para salvar ou 'q' para sair.")

    def nothing(self, x):
        pass

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erro ao converter imagem: {e}")
            return

        frame = cv2.resize(frame, (640, 480))
        self.image = frame

        # Converte para LAB e extrai o canal L (luminosidade)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)

        thresh_val = cv2.getTrackbarPos("Threshold", "Threshold Control")
        _, binarized = cv2.threshold(l_channel, thresh_val, 255, cv2.THRESH_BINARY_INV)

        cv2.imshow("Imagem da Camera", frame)
        cv2.imshow("Imagem com Threshold", binarized)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            config = {"threshold": thresh_val}
            with open(self.json_path, "w") as f:
                json.dump(config, f)
            self.get_logger().info(f"Threshold {thresh_val} salvo em {self.json_path}")
        elif key == ord('q'):
            self.get_logger().info("Encerrando o nó.")
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ThresholdCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
