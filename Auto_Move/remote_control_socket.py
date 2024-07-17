import socket
import keyboard
import time
from utils import *
import json

def remote_control_socket(target_ip, target_port, robot_state_info_queue, command_queue):
    host = target_ip
    port = target_port
    print("遥控器已启动")

    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((host, port))
        client_socket.settimeout(0.5)

        while not keyboard.is_pressed("p"):
            #time.sleep(0.1)
            command = None
            while not command_queue.empty():    
                command = command_queue.get()
                command_str = json.dumps(command) + '\n'
                print("遥控器发送： ", command_str)
                client_socket.send(command_str.encode('utf-8'))
                
                try:
                    count = 0
                    feedback = client_socket.recv(512).decode('utf-8')
                    while not feedback and count < 3:
                        feedback = client_socket.recv(512).decode('utf-8')
                        count += 1
                    feedback = json.loads(feedback)
                except Exception as e:
                    feedback = {"robot_state" : 0}
                #print("遥控器收到feed_back: ", feedback)
                robot_state_info_queue.put(feedback)

                if robot_state_info_queue.qsize() > 5:
                    robot_state_info_queue.get()
                
        print("遥控器被用户停止")

    except KeyboardInterrupt:
        print("遥控器已停止。")
    except Exception as e:
        print(f"遥控器错误: {e}")
    finally:
        client_socket.close()
        print("遥控器线程终止。")
        