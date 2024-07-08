from flask import Flask, render_template, Response, request, send_from_directory
from camera import VideoCamera
from piserial import piSerial
import os
from enum import Enum
import json
import struct
from flask import Flask, request, jsonify
import signal
import serial

CAMERA_ENABLE = True
#SERIAL_ENABLE = True

try:
    pi_camera = VideoCamera(capture_source=0)
    print("====="*5)
    print("相机成功打开")
    print("====="*5)
    CAMERA_ENABLE = True
except Exception as e:
    print(f"Error initializing camera: {e}")
    CAMERA_ENABLE = False
    
# App Globals (do not edit)
app = Flask(__name__)
def generate(camera, width, height):
    while True:
        camera.change_resolution(width, height)
        frame = camera.get_frame()
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/video_feed', methods=['POST', 'GET'])
def video_feed():
    if not CAMERA_ENABLE:
        return Response(status=500)
    
    width = request.args.get('width', default=640, type=int)
    height = request.args.get('height', default=480, type=int)
    
    print(f"width: {width}, height: {height}")
    
    return Response(generate(pi_camera, width, height),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
    
@app.route("/video", methods=['POST'])
def video():
    if not CAMERA_ENABLE:
        return Response(status=500)
    
    data = request.get_json()   
    width = data.get('width', 640)
    height = data.get('height', 480)
    
    try:
        pi_camera.change_resolution(width, height)
        frame = pi_camera.get_frame()
        return Response(frame, mimetype='image/jpeg')
    except Exception as e:
        print(f"/Video Route Error happend : {e}")
        return Response(status=204) 

def video_server_main():
    try:
        app.run('0.0.0.0', port='5000', threaded=True)
    except KeyboardInterrupt:
        print("Server stopped by user")
    except Exception as e:
        print(f"Server stopped due to an error: {e}")
    finally:
        # 清理工作
        # if SERIAL_ENABLE:
        #     pi_serial.close()
        if CAMERA_ENABLE:
            pi_camera.close()
        #print("Serial port closed")
