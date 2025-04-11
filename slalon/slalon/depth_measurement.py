import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from mirela_sdk.image_processing.camera import ImageHandler
from std_msgs.msg import Int8

#Incorporar ImageHandler no codigo
#Analisar o ColorDetector do Samuel
#Fazer filtro de cores para todos os postes
#Movimentação do drone conforme as cores
#Altura max do drone é de 2.5 metros
#O lado que se deve percorrer a primeira trave é fornecido no dia da prova
#Transformar em classe

#Cores traves: Preto fosco, Azul escuro, Rosa claro, 
ROSA = 0
VERMELHO = 1
AZUL = 2
PRETO = 3

class DepthMeasurement:
    def __init__(self, cap=0):

        self.cap = cv2.VideoCapture(cap)

        self.p_oneMeter_blue = 69
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

        kernel = np.ones((8,8), np.int8)

        const: float = 69*97

        result = cv2.bitwise_and(frame, roi, mask=mask)
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        pixels_nonzero = np.count_nonzero(gray)


        distance = const / pixels_nonzero if pixels_nonzero != 0 else 0

        print(distance)
        print(pixels_nonzero)

        cv2.imshow("preview", frame)
        cv2.imshow("result", result)

        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            self.cap.release()