import random
import socket
import keyboard
import time
from utils import *
import queue

def get_key_state():
    if keyboard.is_pressed('w'):
        return "w"
    elif keyboard.is_pressed('s'):
        return "s"
        
    elif keyboard.is_pressed('a'):
        return "a"
    elif keyboard.is_pressed('d'):
        return "d"
    else:
        return ""

def remote_control_socket(target_ip, target_port, robot_state_info_queue):
    host = target_ip
    port = target_port
    print("遥控器已启动")

    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((host, port))

        while not keyboard.is_pressed("p"):

        #     pressed_key = ''
        #     speed = 0
        #     if keyboard.is_pressed('w'):
        #         pressed_key += 'w'
        #     elif keyboard.is_pressed('s'):
        #         pressed_key += 's'
        #     else:
        #         pressed_key += ' '

        #     if keyboard.is_pressed('a'):
        #         pressed_key += 'a'
        #     elif keyboard.is_pressed('d'):
        #         pressed_key += 'd'
        #     else:
        #         pressed_key += ' '

        #     client_socket.send(pressed_key.encode('utf-8'))

        #     #print("send time is : ", end_time - start_time)
            
            try:
                feedback = client_socket.recv(1024).decode('utf-8')
                print(feedback)
                robot_state_info_queue.put(feedback)

                if robot_state_info_queue.qsize() > 1:
                    robot_state_info_queue.get()
            except Exception as e:
                print(f"遥控器错误: {e}")

            time.sleep(0.3)
    except KeyboardInterrupt:
        print("遥控器已停止。")
    except Exception as e:
        print(f"遥控器错误: {e}")
    finally:
        client_socket.close()
        print("遥控器线程终止。")