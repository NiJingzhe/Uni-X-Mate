import cv2
import cv2 as cv
import numpy as np

class VideoCamera(object):
    
    def __init__(self, capture_source=0, width=1024, height=768):
        self.capture_source = capture_source
        self.width = width
        self.height = height
        self.capture = cv.VideoCapture(self.capture_source, cv.CAP_V4L2)
        #self.capture = cv.VideoCapture(self.capture_source)
        if not self.capture.isOpened():
            raise Exception("不能打开摄像头")
        self.capture.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv.CAP_PROP_FPS, 20)

    
    def get_frame(self):
        ret, frame = self.capture.read()
        #print(frame)
        if not ret:
            raise Exception("不能读取视频帧")
        # 应用变换
        ret, jpeg = cv.imencode('.jpg', frame)
        if not ret:
            raise Exception("不能进行转码")
        # 返回编码后的字节数据
        return jpeg.tobytes()
 
    def change_resolution(self, width, height):
        if width == self.width or height == self.height:
            return

        self.capture.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, height)
        self.width = width
        self.height = height
    
    def close(self):
        self.capture.release()

    def __del__(self):
        self.capture.release()

