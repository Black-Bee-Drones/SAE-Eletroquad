import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import subprocess
import re
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Configura QoS com apenas 1 imagem no buffer
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher para imagem
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', qos_profile)

        # OpenCV bridge
        self.bridge = CvBridge()

        C920_DEVICES = [
            'HD Pro Webcam C920',
            'Logi Webcam C920e',
        ]
        
        result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True)
        lines = result.stdout.splitlines()
        self.device = None

        for i, line in enumerate(lines):
            for model_name in C920_DEVICES:
                if model_name in line:
                    j = i + 1
                    while j < len(lines) and lines[j].startswith('\t'):
                        match = re.search(r'(/dev/video\d+)', lines[j])
                        if match:
                            self.device = match.group(1)
                            break
                        j += 1
                    break
            if self.device:
                break

        if self.device is None:
            raise RuntimeError("C920 camera not detected. Please ensure the device is connected and that 'v4l2-ctl' is installed.")
        

        # Inicializa câmera
        self.cap = cv2.VideoCapture(self.device)
        if not self.cap.isOpened():
            self.get_logger().error("Não foi possível abrir a câmera.")
            exit(1)

        # Configurações da C920
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        # Desativa foco automático e fixa o foco
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_FOCUS, 0)  # pode variar entre 0–255

        # Desativa exposição automática e fixa a exposição
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)  # 1 = manual, 3 = auto

        time.sleep(1)

        subprocess.run([
            '/usr/bin/v4l2-ctl',
            '-d', self.device,
            '-c', 'exposure_absolute=3',
        ], check=True)

        # Timer para capturar imagens a 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Falha ao capturar imagem.")
            return
        
        side = 480
        center_x, center_y = 640 // 2, 480 // 2
        half_side = side // 2
        crop = frame[center_y - half_side:center_y + half_side, center_x - half_side:center_x + half_side]

        # Redimensionar para 320x320
        frame = cv2.resize(crop, (320, 320), interpolation=cv2.INTER_AREA)


        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
