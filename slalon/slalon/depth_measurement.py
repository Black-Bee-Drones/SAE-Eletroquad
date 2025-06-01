import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from itertools import groupby
from mirela_sdk.image_processing.color import ColorDetector

#Movimentação do drone conforme as cores
#Altura max do drone é de 2.5 metros
#O lado que se deve percorrer a primeira trave é fornecido no dia da prova

#Cores traves: Preto fosco, Azul escuro, Rosa claro, Vermelho

#filtro preto ta uma bosta

class DepthMeasurement(Node):
    def __init__(self, cap=0):
        super().__init__("depth_st")

        self.pub = self.create_publisher(Float32, "depth_topic", 10)

        self.find_pub = self.create_publisher(Int8, "where_is_it", 10)

        self.detector = ColorDetector("preset", "blue_sl")

        self.declare_parameter("cap", 0)
        cap_param = self.get_parameter("cap").value

        if cap is None:
            cap = cap_param

        self.cap = cv2.VideoCapture(cap)

        self.width = 0

        #Numero de pixels filtrado vezes a distancia da camera à esse numero de pixels
        self.const: float = 69*100 

        
    def depth_callback(self):
        ret, frame = self.cap.read()

        self.detector.filterColor(frame)
        

        # Calcular quantos pixels brancos existem em cada coluna
        col_sums = np.count_nonzero(self.detector.mask, axis=0)  # shape: (640,)

        # Criar máscara booleana das colunas que passam do limite (60% da altura = 288)
        valid_cols = col_sums > 200


        indices = np.where(valid_cols)[0]
        groups = [list(g) for k, g in groupby(enumerate(indices), lambda x: x[0] - x[1])]

        # Encontrar o maior grupo contínuo (maior faixa de colunas válidas)
        longest_group = max(groups, key=len, default=[])

        pipe_area = np.zeros_like(self.detector.mask)

        if 69 > len(longest_group) > 15:
            begin = longest_group[0][1]
            end = longest_group[-1][1]
            self.width = longest_group[-1][0] + 1
            pipe_area[:, begin:end] = 255
            #print(f'width: {width}')


        #Matriz do tamanho da mascara preenchida com zeros
        roi = np.zeros_like(self.detector.mask)

        #Define uma linha no centro dessa matriz com valor 255, que será a área de detecção
        roi[240:241,:] = 255

        self.pipe_in_roi = cv2.bitwise_and(pipe_area, roi)

        #Regra de 3 para calcular a distancia baseado na variação do número de pixels
        distance = self.const / self.width if self.width != 0 else 0.0
        #print(f'distance: {distance}')

        msg = Float32()
        msg.data = float(distance)
        self.pub.publish(msg)
        
        #print(f'{distance:.2f}')
        #print(pixels_nonzero)

        cv2.imshow("preview", frame)
        #cv2.imshow("pipe_area", pipe_area)
        #cv2.imshow("result", self.pipe_in_roi)
        #cv2.imshow("mask", self.detector.mask)

        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            self.cap.release()

    def find_object(self):


        """Identifies whether the object is located at the right side or the left side of the image or at its center"""
        left = np.count_nonzero(self.pipe_in_roi[:, 0:320])
        right= np.count_nonzero(self.pipe_in_roi[:, 320:640])

        msg = Int8()

        if not left and not right:
            msg.data = 0
            self.find_pub.publish(msg) #nenhum objeto encontrado
        elif abs(left - right) <= 10:
            msg.data = 1
            self.find_pub.publish(msg) #ta no centro
        elif left > right:
            msg.data = 2
            self.find_pub.publish(msg) #ta mais pra esquerda
        else:
            msg.data = 3
            self.find_pub.publish(msg) #ta mais pra direita