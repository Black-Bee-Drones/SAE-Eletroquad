import rclpy
import cv2
from rclpy.node import Node
import numpy as np
from itertools import groupby
from mirela_sdk.image_processing.color import ColorDetector
from mirela_sdk.image_processing.color.color_calibration_node import ColorCalibrationNode
import cvzone 

#Movimentação do drone conforme as cores
#Altura max do drone é de 2.5 metros
#O lado que se deve percorrer a primeira trave é fornecido no dia da prova

#Cores traves: Preto fosco, Azul escuro, Rosa claro, Vermelho

#filtro preto ta uma bosta

class TestDetection(Node):
    def __init__(self, cap=2):

        super().__init__("teste_detection")

        #SETAR SATURACAO PARA 200 E TESTES

        self.get_logger().info("Detection initiated...")

        self.declare_parameter("cap", 2)
        cap_param = self.get_parameter("cap").value

        if cap is None:
            cap = cap_param


        self.cap = cv2.VideoCapture(cap)
        self.width = 0
        #Numero de pixels filtrado vezes a distancia da camera à esse numero de pixels
        self.const: float = 69*100 

        self.red_detector = ColorDetector("preset", "red_sl")
        self.blue_detector = ColorDetector("preset", "blue_sl")
        self.pink_detector = ColorDetector("preset", "pink_sl")
        self.black_detector = ColorDetector("preset", "black_sl")

        self.detector = self.black_detector

        self.get_logger().info(f"webcam index: {cap}")

        self.create_timer(1/30, self.detect)   
        
    def detect(self):

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

        #Exige teste. Pode ser um numero maior pois se o drone nn encontrar nada ele chega pra frente
        #o numero de pixels na lagura do drone a 5 metros é 23
        #Esse 69 tbm necessita teste
        #Caso o maior grupo não seja o cano temos que pensar numa maneira de procurar outro ignorando este
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

        pixels_in_roi = cv2.bitwise_and(self.detector.mask, roi)

        self.pipe_in_roi = cv2.bitwise_and(pipe_area, roi)

        #Conversão para grayscale para que a imagem possua apenas 1 canal
        #pixels_in_roi = cv2.cvtColor(pixels_in_roi, cv2.COLOR_BGR2GRAY)

        #Conta o numero de pixels detectados pelo filtro de cor dentro da area roi
        pixels_nonzero = np.count_nonzero(pixels_in_roi)
        #print(f'pixels_nonzero: {pixels_nonzero}')

        #Regra de 3 para calcular a distancia baseado na variação do número de pixels
        distance = self.const / self.width if self.width != 0 else 0.0
        #print(f'distance: {distance}')

        #print(f'{distance:.2f}')
        #print(pixels_nonzero)

        #cv2.imshow("mask", self.detector.mask)
        stack = cvzone.stackImages([self.detector.mask, pixels_in_roi, pipe_area], 3, 0.7)
        cv2.imshow("mask  pixels_in_roi  pipe_area", stack)
        #cv2.imshow("preview", frame)

        key = cv2.waitKey(1)

        if key == ord('q'):
            cv2.destroyAllWindows()
            self.cap.release()
        
        elif key == ord('p'):
            self.detector = self.black_detector

        elif key == ord('v'):
            self.detector = self.red_detector
        
        elif key == ord('a'):
            self.detector = self.blue_detector
        
        elif key == ord('r'):
            self.detector = self.pink_detector

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
        

def main(args=None):
    import argparse

    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description="Test Detection")
    
    parser.add_argument(
        "--cap", type=int, default=None, help="Webcam index"
    )

    parsed_args, remaing_args = parser.parse_known_args(args=args)

    test = TestDetection(cap=parsed_args.cap)

    rclpy.spin(test)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
