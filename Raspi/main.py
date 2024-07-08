#modified by smartbuilds.io
#Date: 27.09.20
#Desc: This web application serves a motion JPEG stream
# main.py
# import the necessary packages
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
SERIAL_ENABLE = True

try:
    pi_camera = VideoCamera(capture_source=0)
    print("====="*5)
    print("相机成功打开")
    print("====="*5)
    CAMERA_ENABLE = True
except Exception as e:
    print(f"Error initializing camera: {e}")
    CAMERA_ENABLE = False
    
try:
    pi_serial = piSerial('/dev/ttyACM0', 9600)
    print("=====" * 5)
    print("串口通讯成功打开")
    print("=====" * 5)
    SERIAL_ENABLE = True
except Exception as e:
    print(f"Error initializing serial port: {e}")
    SERIAL_ENABLE = False

class COMMANDS(Enum):
    MOVE = 0
    IMM_STOP = 1
    
    
# App Globals (do not edit)
app = Flask(__name__)

@app.route('/video', methods=['POST'])
def video():
    if not CAMERA_ENABLE:
        return Response(status=500)
    
    data = request.get_json()   
    width = data.get('width', 640)
    height = data.get('height', 480)
    pi_camera.change_resolution(width, height)
    
    frame = pi_camera.get_frame()
    if frame is None:
        return Response(status=204)  # No Content
    return Response(frame, mimetype='image/jpeg')


def int_to_signed_byte(value):
    """Convert an integer to a signed byte."""
    #if value < 0:
    #    value += 256
    #return value.to_bytes(1, byteorder='big')
    return struct.pack('b', value)

# 定义超时处理函数
def timeout_handler(signum, frame):
    raise TimeoutError("Request timed out")

# 注册信号处理器
if SERIAL_ENABLE:
    signal.signal(signal.SIGALRM, timeout_handler)
#signal.signal(signal.SIGALRM, timeout_handler)

@app.route('/move', methods=['POST'])
def move_control():
    
    if not SERIAL_ENABLE:
        return Response(status=500)
    
    data = request.get_json()
    forward_back = int(data.get('forward_back', 0))
    left_right = int(data.get('left_right', 0))

    # 构建MOVE命令的字节流
    command_byte = COMMANDS.MOVE.value.to_bytes(1, 'big')
    forward_back_byte = int_to_signed_byte(forward_back)
    left_right_byte = int_to_signed_byte(left_right)
    command_stream_bytes = command_byte + forward_back_byte + left_right_byte

    try:
        # 启动超时警报
        signal.alarm(10)
        
        pi_serial = piSerial('/dev/ttyACM0', 9600)

        # 发送命令字节流到Arduino
        pi_serial.write(command_stream_bytes)
        print("to serial : ", command_stream_bytes)

        # 读取反馈
        feed_back = pi_serial.readline().decode().strip()
        print("from serial: ", feed_back)

        # 取消超时警报
        signal.alarm(0)

        pi_serial.close()

        return jsonify({'feedback': feed_back})

    except TimeoutError:
        print("Request timed out")
        return jsonify({'error': 'Request timed out'}), 500

    except serial.SerialTimeoutException:
        print("Serial timeout occurred while writting to the serial port")
        return jsonify({'error': 'Serial timeout occurred while writing to the serial port'}), 500

    except serial.SerialException as e:
        print("Serial communication error")
        return jsonify({'error': f'Serial communication error: {e}'}), 500

    except Exception as e:
        print("Unexpected error")
        return jsonify({'error': f'An unexpected error occurred: {e}'}), 500

    finally:
        # 确保超时警报被取消
        signal.alarm(0)

if __name__ == '__main__':
    try:
        app.run('0.0.0.0', port='5000')
    except KeyboardInterrupt:
        print("Server stopped by user")
    except Exception as e:
        print(f"Server stopped due to an error: {e}")
    finally:
        # 清理工作
        if SERIAL_ENABLE:
            pi_serial.close()
        if CAMERA_ENABLE:
            pi_camera.close()
        #print("Serial port closed")

