import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Int8

#Movimentação do drone conforme as cores
#Altura max do drone é de 2.5 metros
#O lado que se deve percorrer a primeira trave é fornecido no dia da prova
#Transformar em classe

#Cores traves: Preto fosco, Azul escuro, Rosa claro, 

class DepthMeasurement(Node):
    def __init__(self, cap=0):
        super().__init__("DepthMeasurement")

        self.pub = self.create_publisher(Float32, "depth_topic", 10)

        self.left_right = self.create_publisher(Int8, "Left_Right", 10)

        self.cap = cv2.VideoCapture(cap)

        #Numero de pixels vezes a distancia da camera à esse numero de pixels
        self.const: float = 69*97 
        self.lower_range = np.array([115, 62, 85]) 
        self.upper_range = np.array([179, 255, 255])
        self.lower_range2 = None
        self.upper_range2 = None


    def set_ranges(self, lower_range, upper_range, lower_range2=None, upper_range2=None):
        self.lower_range = lower_range
        self.upper_range = upper_range
        self.lower_range2 = lower_range2
        self.upper_range2 = upper_range2

        
    def depth_callback(self):
        ret, frame = self.cap.read()
        frame = cv2.flip(frame, 1)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower_range, self.upper_range)
        if self.lower_range2 is not None and self.upper_range2 is not None:
            mask2 = cv2.inRange(hsv, self.lower_range2, self.upper_range2)
            mask = cv2.bitwise_or(mask, mask2)

        mask = cv2.dilate(mask, np.ones((11, 11), np.uint8), iterations=1)
        mask = cv2.erode(mask, np.ones((7, 7), np.uint8), iterations=1)

        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((8, 8), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((8, 8), np.uint8))


        roi = np.zeros_like(frame)

        roi[240:241,:] = 255


        result = cv2.bitwise_and(frame, roi, mask=mask)
        #Conversão para grayscale para que a imagem possua apenas 1 canal
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        pixels_nonzero = np.count_nonzero(gray)


        distance = self.const / pixels_nonzero if pixels_nonzero != 0 else 0.0

        msg = Float32()
        msg.data = float(distance)
        self.pub.publish(msg)
        
        print(f'{distance:.2f}')
        print(pixels_nonzero)

        cv2.imshow("preview", frame)
        cv2.imshow("result", result)

        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            self.cap.release()

    def right_or_left(self):
        pass