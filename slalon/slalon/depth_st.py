import numpy as np
from rclpy.node import Node
import rclpy
from std_msgs.msg import Int8
import cv2
from std_msgs.msg import Float32

#Movimentação do drone conforme as cores
#Altura max do drone é de 2.5 metros
#O lado que se deve percorrer a primeira trave é fornecido no dia da prova

#Cores traves: Preto fosco, Azul escuro, Rosa claro, Vermelho

class DepthMeasurement(Node):
    def __init__(self, cap=0):
        super().__init__("depth_st")

        self.pub = self.create_publisher(Float32, "depth_topic", 10)

        self.find_pub = self.create_publisher(Int8, "where_is_it", 10)

        self.cap = cv2.VideoCapture(cap)

        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        #Numero de pixels filtrado vezes a distancia da camera à esse numero de pixels
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
        #self.get_logger().info("depth cb")
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

        #Matriz do tamanho da imagem preenchida com zeros
        roi = np.zeros_like(frame)

        #Define uma linha no centro dessa matriz com valor 255, que será a área de detecção
        roi[240:241,:] = 255

        result = cv2.bitwise_and(frame, roi, mask=mask)

        #Conversão para grayscale para que a imagem possua apenas 1 canal
        self.gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

        #Conta o numero de pixels detectados pelo filtro de cor dentro da area roi
        pixels_nonzero = np.count_nonzero(self.gray)

        #Regra de 3 para calcular a distancia baseado na variação do número de pixels
        distance = self.const / pixels_nonzero if pixels_nonzero != 0 else 0.0

        msg = Float32()
        msg.data = float(distance)
        self.pub.publish(msg)
        
        #print(f'{distance:.2f}')
        #print(pixels_nonzero)

        cv2.imshow("preview", frame)
        cv2.imshow("result", result)

        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            self.cap.release()

    def find_object(self):

        #self.get_logger().info("find obj")

        """Identifies whether the object is located at the right side or the left side of the image or at its center"""
        left = np.count_nonzero(self.gray[:, 0:320]) #[:, 0:960]
        right= np.count_nonzero(self.gray[:, 320:640]) #[:, 320:640][:, 960:1920]

        msg = Int8()

        if not left and not right:
            msg.data = 0
            self.find_pub.publish(msg) #nenhum objeto encontrado
        elif abs(left - right) <= 50:
            msg.data = 1
            self.find_pub.publish(msg) #ta no centro
        elif left > right:
            msg.data = 2
            self.find_pub.publish(msg) #ta mais pra esquerda
        else:
            msg.data = 3
            self.find_pub.publish(msg) #ta mais pra direita
        

START = 0
PINK = 1
RED = 2
BLUE = 3
BLACK = 4

class DepthStateMachine(DepthMeasurement):
    def __init__(self):
        super().__init__(2)


        self.state = START
        self.next_state = RED

        self.depth_st_sub = self.create_subscription(Int8, "switch_state", self.switch_state_callback, 10)


        self.lower_pink = np.array([115, 62, 85]) 
        self.upper_pink = np.array([179, 255, 255])
        self.lower_range2 = None
        self.upper_range2 = None

        self.lower_blue = np.array([75, 136, 90])
        self.upper_blue = np.array([129, 255, 199])

        self.lower_black = np.array([73, 0, 11])
        self.upper_black = np.array([103, 46, 122])

        self.lower_red1 = np.array([0, 175, 117])
        self.upper_red1 = np.array([20, 255, 203])
        self.lower_red2 = np.array([169, 128, 140])
        self.upper_red2 = np.array([179, 255, 223])

        self.cont = 0
        
        self.run()

    def switch_state_callback(self, msg):
        self.switch_state()

    def switch_state(self):
        self.get_logger().info("Depth_StateMachine changing state...")
        if self.next_state == PINK:
            self.get_logger().info("Filtering PINK")
            self.state = PINK
            self.set_ranges(self.lower_pink, self.upper_pink)
            self.next_state = RED

        elif self.next_state == RED:
            self.get_logger().info("Filtering RED")
            self.state = RED
            self.set_ranges(
                self.lower_red1, self.upper_red1, self.lower_red2, self.upper_red2,
            )
            self.next_state = BLUE
        
        elif self.next_state == BLUE:
            self.get_logger().info("Filtering BLUE")
            self.state = BLUE
            self.set_ranges(self.lower_blue, self.upper_blue)
            self.next_state = BLACK

        elif self.next_state == BLACK:
            self.get_logger().info("Filtering BLACK")
            self.state = BLACK
            self.set_ranges(self.lower_black, self.upper_black)
            self.next_state = RED
        
        else:
            self.state = START
            self.next_state = PINK

    def teste(self):
        #so pra testar se a mudança de estados tava funcionando
        self.cont += 1
        print("AAAAAAAAA")

        if self.cont == 10:
            print("ALOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
            self.cont = 0
            self.switch_state()
            
    def run(self):
        self.switch_state()
        self.get_logger().info("rodei")
        self.create_timer(1/30, self.depth_callback)
        self.create_timer(1/30, self.find_object)

def main():
    rclpy.init()

    st = DepthStateMachine()
    rclpy.spin(st)


    rclpy.shutdown()

if __name__ == "__main__":
    main()