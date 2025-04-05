import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from mirela_sdk.image_processing.camera import ImageHandler

cap = cv2.VideoCapture(2)

cont =  0
distance = 1
while True:
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_pink = np.array([115, 62, 85]) 
    upper_pink = np.array([179, 255, 255])

    mask = cv2.inRange(hsv, lower_pink, upper_pink)
    roi = np.zeros_like(frame)


    roi[240:241,:] = 255

    kernel = np.ones((8,8), np.int8)

    result = cv2.bitwise_and(frame, roi, mask=mask)

    result = cv2.morphologyEx(result, cv2.MORPH_DILATE, kernel)
    result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel)

    if cont <= 100:
        p_oneMeter = result.sum()
        cont+=1
    else:
        p_distance = result.sum()
        
        distance = p_oneMeter/p_distance


    print(distance)

    cv2.imshow("preview", frame)
    cv2.imshow("result", result)

    if cv2.waitKey(1) == ord('q'):
        break
        
cv2.destroyAllWindows()
cap.release()
