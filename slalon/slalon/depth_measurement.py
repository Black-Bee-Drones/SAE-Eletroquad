import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from mirela_sdk.image_processing.camera import ImageHandler

#Incorporar ImageHandler no codigo
#Analisar o ColorDetector do Samuel
#Fazer filtro de cores para todos os postes
#Movimentação do drone conforme as cores
#Altura max do drone é de 2.5 metros
#O lado que se deve percorrer a primeira trave é fornecido no dia da prova
#Transformar em classe

#Cores traves: Preto fosco, Azul escuro, Rosa claro, Vermelho

cap = cv2.VideoCapture(2)

p_oneMeter_blue = 69

cont =  0
distance = 1
while True:
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #Esses valores ainda não são da trave rosa
    lower_pink = np.array([115, 62, 85]) 
    upper_pink = np.array([179, 255, 255])

    lower_blue = np.array([75, 136, 90])
    upper_blue = np.array([129, 255, 199])

    lower_black = np.array([73, 0, 11])
    upper_black = np.array([103, 46, 122])

    lower_red1 = np.array([0, 175, 117])
    upper_red1 = np.array([20, 255, 203])
    lower_red2 = np.array([169, 128, 140])
    upper_red2 = np.array([179, 255, 223])

    mask = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
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

    # result = cv2.morphologyEx(result, cv2.MORPH_DILATE, kernel)
    # result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel)

    #while cont <= 100:
    #     p_oneMeter = np.where(result > 0, 1, 0)
    #     p_oneMeter = p_oneMeter.sum()
    #     cont+=1
    # else:
    #     p_oneMeter = np.where(result > 0, 1, 0).sum()
    #     p_distance = result.sum()

        #Seria necessario fazer um teorema de pitagoras?
    distance = const / pixels_nonzero if pixels_nonzero != 0 else 0


    print(distance)
    print(pixels_nonzero)

    cv2.imshow("preview", frame)
    cv2.imshow("result", result)

    if cv2.waitKey(1) == ord('q'):
        break
        
cv2.destroyAllWindows()
cap.release()
