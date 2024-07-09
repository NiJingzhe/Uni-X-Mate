from flask import Flask, render_template, Response, request, send_from_directory
from camera import VideoCamera
from piserial import piSerial
from enum import Enum
import struct
from flask import Flask, request, jsonify
import serial
import time

#CAMERA_ENABLE = True
SERIAL_ENABLE = True
    
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

def int_to_signed_byte(value):
    """Convert an integer to a signed byte."""
    return struct.pack('b', value)

@app.route('/move', methods=['POST'])
def move_control():
    
    if not SERIAL_ENABLE:
        return Response(status=500)
    
    data = request.get_json()
    forward_back = int(data.get('forward_back', 0))
    left_right = int(data.get('left_right', 0))

    # 构建MOVE命令的字节流
    command_byte = COMMANDS.MOVE.value.to_bytes(1, 'big')
    forward_back_byte = struct.pack('b', forward_back)
    left_right_byte = struct.pack('b', left_right)
    command_stream_bytes = command_byte + forward_back_byte + left_right_byte

    try:
        # 发送命令字节流到Arduino
        start_time = time.time()
        pi_serial.write(command_stream_bytes)
        end_time = time.time()
        print("Serial send time : ", end_time - start_time)
        #print("to serial : ", command_stream_bytes)

        # 读取反馈
        pi_serial.reset_input_buffer()
        pi_serial.ser.reset_output_buffer()
        #feed_back = pi_serial.readline().decode().strip()
        #end_time = time.time()
        #print("Serila read time : ", end_time - start_time)
        #print("from serial: ", feed_back)

        return jsonify({'feedback': '111'})

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
        pass

def control_server_main():
    try:
        app.run('0.0.0.0', port='5001', threaded=True)
    except KeyboardInterrupt:
        print("Server stopped by user")
    except Exception as e:
        print(f"Server stopped due to an error: {e}")
    finally:
        # 清理工作
        if SERIAL_ENABLE:
            pi_serial.close()
        # if CAMERA_ENABLE:
        #     pi_camera.close()
        #print("Serial port closed")

