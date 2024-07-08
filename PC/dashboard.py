import multiprocessing
from multiprocessing import Process, Queue

from flask import Flask, render_template, jsonify, send_file
import numpy as np
import io
from PIL import Image
import time

app = Flask(__name__)

result_queue = multiprocessing.Queue()

def array_to_image(array):
    img = Image.fromarray(array)
    return img

@app.route('/')
def home():
    return app.send_static_file('index.html')

@app.route('/get_image')
def get_image():
    if not result_queue.empty():
        result = result_queue.get()
        if len(result.keys()) == 0:
            return
        image_array = result["image"]
        img = array_to_image(image_array)
        img_io = io.BytesIO()
        img.save(img_io, 'JPEG')
        img_io.seek(0)
        return send_file(img_io, mimetype='image/jpeg')
    else:
        print('ooooooops')
        return jsonify({"image_path": None})

def decided_name():
    counter = 0
    name_list = []
    while not result_queue.empty() and counter <= 20:
        result = result_queue.get()
        if len(result.keys()) == 0:
            continue
        counter += 1
        name_list.append(result["class_name"])


@app.route('/get_name')
def get_name():


def generate_noise_image(queue):
    while True:
        # 生成随机噪声图片
        noise_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        if queue.qsize() < 100:
            queue.put(noise_image)
        time.sleep(0.02)  # 每0.02秒生成一次噪声图片


if __name__ == '__main__':
    app.run()
