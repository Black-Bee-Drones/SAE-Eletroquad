import math
import numpy as np
import depthai as dai
from mirela_sdk.image_processing.camera.oakd_cam import OakdCam
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class HostSpatialsCalc:
    def __init__(self, device):
        self.calibData = device.readCalibration()
        self.DELTA = 5
        self.THRESH_LOW = 200  # 20cm
        self.THRESH_HIGH = 30000  # 30m

    def setLowerThreshold(self, threshold_low):
        self.THRESH_LOW = threshold_low

    def setUpperThreshold(self, threshold_high):
        self.THRESH_HIGH = threshold_high

    def setDeltaRoi(self, delta):
        self.DELTA = delta

    def _check_input(self, roi, frame):
        if len(roi) == 4:
            return roi
        if len(roi) != 2:
            raise ValueError("You have to pass either ROI (4 values) or point (2 values)!")
        x = min(max(roi[0], self.DELTA), frame.shape[1] - self.DELTA)
        y = min(max(roi[1], self.DELTA), frame.shape[0] - self.DELTA)
        return (x - self.DELTA, y - self.DELTA, x + self.DELTA, y + self.DELTA)

    def _calc_angle(self, frame, offset, HFOV):
        return math.atan(math.tan(HFOV / 2.0) * offset / (frame.shape[1] / 2.0))

    def calc_spatials(self, depthData, roi, averaging_method=np.mean):
        depthFrame = depthData.getFrame()
        roi = self._check_input(roi, depthFrame)
        xmin, ymin, xmax, ymax = roi

        depthROI = depthFrame[ymin:ymax, xmin:xmax]
        inRange = (self.THRESH_LOW <= depthROI) & (depthROI <= self.THRESH_HIGH)
        HFOV = np.deg2rad(self.calibData.getFov(dai.CameraBoardSocket(depthData.getInstanceNum()), useSpec=False))

        averageDepth = averaging_method(depthROI[inRange])

        centroid = {
            'x': int((xmax + xmin) / 2),
            'y': int((ymax + ymin) / 2)
        }

        midW = int(depthFrame.shape[1] / 2)
        midH = int(depthFrame.shape[0] / 2)
        bb_x_pos = centroid['x'] - midW
        bb_y_pos = centroid['y'] - midH

        angle_x = self._calc_angle(depthFrame, bb_x_pos, HFOV)
        angle_y = self._calc_angle(depthFrame, bb_y_pos, HFOV)

        spatials = {
            'z': averageDepth / 1000,
            'x': averageDepth * math.tan(angle_x) / 1000,
            'y': -averageDepth * math.tan(angle_y) / 1000
        }
        return spatials, centroid