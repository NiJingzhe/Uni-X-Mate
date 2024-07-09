from multiprocessing import Process, Queue

from flask import Flask, render_template, jsonify, send_file, send_from_directory
import numpy as np
import io
from PIL import Image
import time
import base64
import requests
import logging
import random

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = 'static/images'

# 注意不能使用queue，当多人访问时，会造成数据消耗过快。

# 模拟图片队列
monitor = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
result_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
result_name = 'Persian'
speed = 20


def array_to_image(array):
    img = Image.fromarray(array)
    return img


@app.route('/')
def home():
    return app.send_static_file('index.html')


@app.route('/get_image')
def get_image():
    img = array_to_image(monitor)
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
        'image': get_image_base64(result_image),
        'name': result_name
    }
    return jsonify(result_data)


def generate_noise_image():
    global monitor
    while True:
        monitor = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        time.sleep(0.02)  # 每0.02秒生成一次噪声图片


def get_image_base64(image_array):
    # 将NumPy数组转换为PIL图像
    image = Image.fromarray(image_array.astype('uint8'))
    buffered = io.BytesIO()
    image.save(buffered, format="JPEG")
    return base64.b64encode(buffered.getvalue()).decode('utf-8')


if __name__ == '__main__':
    noise_process = Process(target=generate_noise_image)
    noise_process.daemon = True
    noise_process.start()
    app.run(host="0.0.0.0", debug=True)
    noise_process.join()
