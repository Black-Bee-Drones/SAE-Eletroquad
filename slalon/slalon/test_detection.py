import rclpy
import cv2
from rclpy.node import Node
import numpy as np
from itertools import groupby


#Movimentação do drone conforme as cores
#Altura max do drone é de 2.5 metros
#O lado que se deve percorrer a primeira trave é fornecido no dia da prova

#Cores traves: Preto fosco, Azul escuro, Rosa claro, Vermelho

#filtro preto ta uma bosta

blue = np.array([[86, 162, 118], [127, 255, 255]]) 
black = np.array([[100, 92, 0], [138, 166, 161]])
red1 = np.array([[0, 175, 117], [20, 255, 203]])
red2 = np.array([[151, 137, 100], [179, 252, 255]])

class TestDetection(Node):
    def __init__(self, cap=0):

        super().__init__("teste_detection")

        self.get_logger().info("Detection initiated...")
        self.cap = cv2.VideoCapture(cap)
        self.width = 0
        #Numero de pixels filtrado vezes a distancia da camera à esse numero de pixels
        self.const: float = 69*100 
        self.lower_range = blue[0]
        self.upper_range = blue[1]
        self.lower_range2 = None
        self.upper_range2 = None


    def set_ranges(self, lower_range, upper_range, lower_range2=None, upper_range2=None):
        self.lower_range = lower_range
        self.upper_range = upper_range
        self.lower_range2 = lower_range2
        self.upper_range2 = upper_range2

        
    def detect(self):
        ret, frame = self.cap.read()
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


        mask = cv2.inRange(hsv, self.lower_range, self.upper_range)
        if self.lower_range2 is not None and self.upper_range2 is not None:
            mask2 = cv2.inRange(hsv, self.lower_range2, self.upper_range2)
            mask = cv2.bitwise_or(mask, mask2)

        mask = cv2.dilate(mask, np.ones((11, 11), np.uint8), iterations=1)
        mask = cv2.erode(mask, np.ones((7, 7), np.uint8), iterations=1)


        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((8, 8), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((8, 8), np.uint8))
                
        #cv2.imshow("mask", mask)

        # count = 0
        # begin = -1
        # for i in range(0, 640):
        #     if np.count_nonzero(mask[:, i]) > 288:
        #         if count == 0:
        #             begin = i
        #         count += 1
        #     else:
        #         if count > 32:
        #             break
        #         count = 0
        #         begin = -1
        # pipe_area = np.zeros_like(mask)
        # pipe_area[:, begin:begin+count] = 255

        # Calcular quantos pixels brancos existem em cada coluna
        col_sums = np.count_nonzero(mask, axis=0)  # shape: (640,)

        # Criar máscara booleana das colunas que passam do limite (60% da altura = 288)
        valid_cols = col_sums > 200


        indices = np.where(valid_cols)[0]
        groups = [list(g) for k, g in groupby(enumerate(indices), lambda x: x[0] - x[1])]

        # Encontrar o maior grupo contínuo (maior faixa de colunas válidas)
        longest_group = max(groups, key=len, default=[])

        pipe_area = np.zeros_like(mask)

        if len(longest_group) > 10:
            begin = longest_group[0][1]
            end = longest_group[-1][1]
            self.width = longest_group[-1][0] + 1
            pipe_area[:, begin:end] = 255
            #print(f'width: {width}')


        cv2.imshow("pipe_area", pipe_area)

        #Matriz do tamanho da mascara preenchida com zeros
        roi = np.zeros_like(mask)

        #Define uma linha no centro dessa matriz com valor 255, que será a área de detecção
        roi[240:241,:] = 255

        #result = cv2.bitwise_and(frame, roi, mask=mask)
        self.pipe_in_roi = cv2.bitwise_and(pipe_area, roi)
        cv2.imshow("result", self.pipe_in_roi)

        #Conversão para grayscale para que a imagem possua apenas 1 canal
        #self.pipe_in_roi = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

        #Conta o numero de pixels detectados pelo filtro de cor dentro da area roi
        #pixels_nonzero = np.count_nonzero(self.pipe_in_roi)
        #print(f'pixels_nonzero: {pixels_nonzero}')

        #Regra de 3 para calcular a distancia baseado na variação do número de pixels
        distance = self.const / self.width if self.width != 0 else 0.0
        #print(f'distance: {distance}')

        #print(f'{distance:.2f}')
        #print(pixels_nonzero)

        #cv2.imshow("preview", frame)
        #cv2.imshow("result", result)

        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            self.cap.release()
        
        if cv2.waitKey(1) == ord('p'):
            self.set_ranges(black[0], black[1])

        if cv2.waitKey(1) == ord('v'):
            self.set_ranges(red1[0], red2[1], red2[0], red2[1])
        
        if cv2.waitKey(1) == ord('a'):
            self.set_ranges(blue[0], blue[1])

    def find_object(self):

        #self.get_logger().info("find obj")

        """Identifies whether the object is located at the right side or the left side of the image or at its center"""
        left = np.count_nonzero(self.pipe_in_roi[:, 0:320]) #[:, 0:960]
        right= np.count_nonzero(self.pipe_in_roi[:, 320:640]) #[:, 320:640][:, 960:1920]

        if not left and not right:
            self.get_logger().info("NOWHERE")  #nenhum objeto encontrado
        elif abs(left - right) <= 10:
            self.get_logger().info("CENTER") #ta no centro
        elif left > right:
            self.get_logger().info("LEFT") #ta mais pra esquerda
        else:
            self.get_logger().info("RIGHT") #ta mais pra direita
        

def main():
    rclpy.init()
    
    test = TestDetection(0)

    while True:
        test.detect()
        
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            break
    rclpy.shutdown()


if __name__ == "__main__":
    main()
