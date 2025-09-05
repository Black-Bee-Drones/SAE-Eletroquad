from rclpy.node import Node
import cv2
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from itertools import groupby
from mirela_sdk.image_processing.color import ColorDetector


class DepthMeasurement(Node):
    def __init__(self, cap=0):
        super().__init__("depth_st")

        self.pub = self.create_publisher(Float32, "depth_topic", 10)

        self.find_pub = self.create_publisher(Int8, "where_is_it", 10)

        #Defines which color is being filtered
        self.detector = ColorDetector("preset", "blue_sl")

        self.declare_parameter("cap", 0)
        cap_param = self.get_parameter("cap").value

        if cap is None:
            cap = cap_param

        self.cap = cv2.VideoCapture(cap)

        self.width = 0

        #Number of filtered pixels times the distance to the camera at that number of pixels
        self.const: float = 69*100 

        
    def depth_callback(self):
        ret, frame = self.cap.read()

        self.detector.filterColor(frame)
        
        #Calculates how many white pixels are in each column of the mask filtered
        col_sums = np.count_nonzero(self.detector.mask, axis=0) 

        #Validates only the columns that have at least 42% of their pixels white
        valid_cols = col_sums > 200


        indices = np.where(valid_cols)[0]
        #Groups consecutive valid columns
        groups = [list(g) for k, g in groupby(enumerate(indices), lambda x: x[0] - x[1])]

        #Finds the biggest group (with more valid consecutive columns)
        longest_group = max(groups, key=len, default=[])

        pipe_area = np.zeros_like(self.detector.mask)

        #Limits the size of a valid group to bigger than 15 columns and smaller than 69
        if 69 > len(longest_group) > 15:
            begin = longest_group[0][1]
            end = longest_group[-1][1]
            self.width = longest_group[-1][0] + 1
            pipe_area[:, begin:end] = 255
            #print(f'width: {width}')


        #Matriz do tamanho da mascara preenchida com zeros
        roi = np.zeros_like(self.detector.mask)

        #Defines a line at the center of the matrix that filters out anything outside its region
        roi[240:241,:] = 255

        #Pixels filtered that are in the line region
        pixels_in_roi = cv2.bitwise_and(self.detector.mask, roi)

        self.pipe_in_roi = cv2.bitwise_and(pipe_area, roi)

        pixels_nonzero = np.count_nonzero(pixels_in_roi)

        #Calculates distance based on the number of pixels
        distance = self.const / pixels_nonzero if pixels_nonzero != 0 else 0.0

        left = np.count_nonzero(self.pipe_in_roi[:, 0:320])
        right= np.count_nonzero(self.pipe_in_roi[:, 320:640])
        center = np.count_nonzero(self.pipe_in_roi[:, 230:410])

        msg = Int8()

        if not left and not right:
            msg.data = 0
            self.find_pub.publish(msg) #No object found
        elif center >= 384: #80% of 480
            msg.data = 1
            self.find_pub.publish(msg) #Found in center
        elif left > right:
            msg.data = 2
            self.find_pub.publish(msg) #Left side
        else:
            msg.data = 3
            self.find_pub.publish(msg) #Right side

        #print(f'distance: {distance}')

        msg = Float32()
        msg.data = float(distance)
        #Publishes distance to pipe
        self.pub.publish(msg)
        
        #print(f'{distance:.2f}')
        #print(pixels_nonzero)

        #cv2.imshow("preview", frame)
        #cv2.imshow("pipe_area", pipe_area)
        #cv2.imshow("result", self.pipe_in_roi)
        #cv2.imshow("mask", self.detector.mask)

        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            self.cap.release()