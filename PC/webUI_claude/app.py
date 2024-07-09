from flask import Flask, render_template, Response
import cv2
import numpy as np
from queue import Queue
import threading

app = Flask(__name__)

# 创建一个队列来存储视频帧
frame_queue = Queue(maxsize=10)


def tele_camera():
    # 使用本地摄像头
    cap = cv2.VideoCapture(0)  # 0 表示默认摄像头，如果有多个摄像头，可能需要尝试不同的数字

    while True:
        ret, frame = cap.read()
        if ret:
            if not frame_queue.full():
                frame_queue.put(frame)
        else:
            print("Failed to capture frame")
            break

    cap.release()


def generate_frames():
    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/get_speed')
def get_speed():
    # 这里应该实现获取速度的逻辑
    # 现在我们只是返回一个随机数作为示例
    import random
    return {'speed': random.randint(0, 200)}


@app.route('/get_result')
def get_result():
    # 这里应该实现获取处理结果的逻辑
    # 现在我们只是返回一个固定的结果作为示例
    return {
        'image': '',  # 这里应该是base64编码的图像数据
        'name': 'Example Cat'
    }


if __name__ == '__main__':
    # 启动视频获取线程
    video_thread = threading.Thread(target=tele_camera)
    video_thread.daemon = True
    video_thread.start()

    app.run(host='0.0.0.0', debug=True, threaded=True)