import multiprocessing
from multiprocessing import Process, Array, Manager
from flask import Flask, Response, send_file, send_from_directory, jsonify
import numpy as np
import io
from PIL import Image
import time
import base64
import requests
import logging
import random
import cv2

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = './static/images'

# 使用共享数组和锁
monitor_shape = (480, 640, 3)
monitor_size = int(np.prod(monitor_shape))  # 将 monitor_size 转换为整数

def create_shared_arrays(manager):
    monitor = Array('B', monitor_size)  # 'B' 表示 unsigned char
    result_image = manager.list(np.random.randint(0, 255, monitor_shape, dtype=np.uint8).flatten())  # 共享 result_image
    result_name = manager.Value('c', b'Persian')  # 共享 result_name
    speed = manager.Value('d', 20.0)  # 共享 speed
    return monitor, result_image, result_name, speed

def array_to_image(array):
    img = Image.fromarray(array)
    return img

def array_from_shared(shared_array):
    return np.frombuffer(shared_array, dtype=np.uint8).reshape(monitor_shape)

@app.route('/')
def home():
    return app.send_static_file('index.html')

@app.route('/get_image')
def get_image():
    img_array = array_from_shared(monitor)
    img = array_to_image(img_array)
    img_io = io.BytesIO()
    img.save(img_io, 'JPEG')
    img_io.seek(0)
    return send_file(img_io, mimetype='image/jpeg')

@app.route('/get_local_image/<filename>')
def get_local_image(filename):
    return send_from_directory(app.config['UPLOAD_FOLDER'], filename)

@app.route('/get_speed')
def get_speed():
    speed_data = {
        'speed': random.uniform(0, 200)  # 生成0到200之间的随机速度
    }
    return jsonify(speed_data)

@app.route('/get_result')
def get_result():
    result_data = {
        'image': get_image_base64(np.array(result_image).reshape(monitor_shape)),
        'name': result_name.value.decode()  # 解码为字符串
    }
    return jsonify(result_data)

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def gen_frames():
    while True:
        frame = array_from_shared(monitor)
        frame = frame.reshape(monitor_shape)
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def generate_noise_image(shared_monitor):
    while True:
        noise_image = np.random.randint(0, 255, monitor_shape, dtype=np.uint8)
        shared_monitor[:] = noise_image.flatten()
        time.sleep(0.02)  # 每0.02秒生成一次噪声图片

def get_image_base64(image_array):
    # 将NumPy数组转换为PIL图像
    image = Image.fromarray(image_array.astype('uint8'))
    buffered = io.BytesIO()
    image.save(buffered, format="JPEG")
    return base64.b64encode(buffered.getvalue()).decode('utf-8')

if __name__ == '__main__':
    multiprocessing.freeze_support()  # 添加这行代码
    manager = Manager()
    monitor, result_image, result_name, speed = create_shared_arrays(manager)
    noise_process = Process(target=generate_noise_image, args=(monitor,))
    noise_process.daemon = True
    noise_process.start()
    app.run(host="0.0.0.0", debug=True)
    noise_process.join()