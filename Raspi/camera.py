import cv2
import cv2 as cv
import numpy as np

class VideoCamera(object):
    def __init__(self, capture_source=0, width=640, height=480):
        self.capture_source = capture_source
        self.width = width
        self.height = height
        self.capture = cv.VideoCapture(self.capture_source)
        self.capture.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)

    def get_frame(self):
        ret, frame = self.capture.read()
        if not ret:
            return None
        # 应用变换
        ret, jpeg = cv.imencode('.jpg', frame)
        if not ret:
            return None
        # 返回编码后的字节数据
        return jpeg.tobytes()
        
    def change_resolution(self, width, height):
        if width == self.width or height == self.height:
            return

        self.capture.release()
        self.capture = cv2.VideoCapture(self.capture_source)
        self.capture.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, height)
        self.width = width
        self.height = height
    
    def close(self):
        self.capture.release()

    def __del__(self):
        self.capture.release()

