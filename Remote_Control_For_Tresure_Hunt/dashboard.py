import multiprocessing
from multiprocessing import Process, Manager

from flask import Flask, render_template, jsonify, send_file, send_from_directory, request
import numpy as np
import io
from PIL import Image
import time
import base64
import requests
import logging
import random
import keyboard
import cv2

#log = logging.getLogger('werkzeug')
#log.setLevel(logging.ERROR)

app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = 'static/images'

# 注意不能使用queue，当多人访问时，会造成数据消耗过快。

# 模拟图片队
monitor = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
result_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
result_name = 'Non Detected'
#speed = 20
info = None

def array_to_image(array):
    img = Image.fromarray(array)
    return img


@app.route('/')
def home():
    return app.send_static_file('index.html')


@app.route('/get_image')
def get_image():
    img = array_to_image(info['monitor'])
    img_io = io.BytesIO()
    img.save(img_io, 'JPEG')
    img_io.seek(0)
    return send_file(img_io, mimetype='image/jpeg')


@app.route('/get_local_image/<filename>')
def get_local_image(filename):
    return send_from_directory(app.config['UPLOAD_FOLDER'], filename)


@app.route('/get_speed')
def get_speed():
    pressed_key = ''
    speed = 0
    if keyboard.is_pressed('w'):
        pressed_key += 'w'
    elif keyboard.is_pressed('s'):
        pressed_key += 's'
    else:
        pressed_key += ' '

    if keyboard.is_pressed('a'):
        pressed_key += 'a'
    elif keyboard.is_pressed('d'):
        pressed_key += 'd'
    else:
        pressed_key += ' '

    if pressed_key == ' a' or pressed_key == ' d' or pressed_key == '  ':
        speed = 0
    elif 'w' in pressed_key or 's' in pressed_key:
        speed += 240
        if 'a' in pressed_key or 'd' in pressed_key:
            speed -= 164

    speed += random.randint(-5, 5)
    if speed < 0:
        speed = 0

    info['speed'] = speed
    speed_data = {
        'speed': info['speed']  # 生成0到200之间的随机速度
    }
    return jsonify(speed_data)

@app.route('/set_speed', methods=['POST'])
def set_speed():
    try:
        data = request.get_json()
        info['speed'] = data['speed']
        return jsonify(success=True)
    except Exception as e:
        return jsonify(success=False, message=str(e)), 400

@app.route('/get_result')
def get_result():
    result_data = {
        'image': get_image_base64(info['result_image']),
        'name': info['result_name']
    }
    return jsonify(result_data)


def generate_noise_image(info):
    while True:
        info['monitor'] = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        time.sleep(0.02)  # 每0.02秒生成一次噪声图片


def get_image_base64(image_array):
    # 将NumPy数组转换为PIL图像
    image = Image.fromarray(image_array.astype('uint8'))
    buffered = io.BytesIO()
    image.save(buffered, format="JPEG")
    return base64.b64encode(buffered.getvalue()).decode('utf-8')


# 设置monitor
@app.route('/set_monitor', methods=['POST'])
def set_value():
    try:
        data = request.get_json()
        info['monitor'] = np.asarray(data['image'], dtype=np.uint8)
        return jsonify(success=True)
    except Exception as e:
        return jsonify(success=False, message=str(e)), 400

@app.route('/set_result', methods=['POST'])
def set_result():
    try:
        data = request.get_json()
        info['result_image'] = np.asarray(data['image'])
        info['result_name'] = data['name']
        return jsonify(success=True)
    except Exception as e:
        return jsonify(success=False, message=str(e)), 400

def run_dashborad():
    with multiprocessing.Manager():
        global info
        manager = Manager()
        info = manager.dict()
        info['monitor'] = monitor
        info['result_image'] = result_image
        info['result_name'] = result_name
        info['speed'] = 0
        #noise_process = Process(target=generate_noise_image, args=(info,))
        #noise_process.daemon = True
        #noise_process.start()
        app.run(host="0.0.0.0", port=5000)
        #noise_process.join()
