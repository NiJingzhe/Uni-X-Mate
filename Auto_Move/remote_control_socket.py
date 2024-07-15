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
        client_socket.settimeout(3.0)

        while not keyboard.is_pressed("p"):
            
            time.sleep(0.5)
            
            command = None
            while not command_queue.empty():    
                command = command_queue.get()
                if command['command'] == COMMAND.SELF_CHECK.value:
                    print("遥控器发送: ", int_to_one_bytes(COMMAND.SELF_CHECK.value))
                    client_socket.send(int_to_one_bytes(COMMAND.SELF_CHECK.value))
                elif command['command'] == COMMAND.FIND_TAG.value:
                    print("遥控器发送: ", int_to_one_bytes(COMMAND.FIND_TAG.value))
                    client_socket.send(int_to_one_bytes(COMMAND.FIND_TAG.value))
                elif command['command'] == COMMAND.GRAB.value:
                    print("遥控器发送: ", int_to_one_bytes(COMMAND.GRAB.value))
                    client_socket.send(int_to_one_bytes(COMMAND.GRAB.value))
                elif command['command'] == COMMAND.GOTO_TARGET.value:
                    print("遥控器发送: ", int_to_one_bytes(COMMAND.GOTO_TARGET.value), float_to_bytes(command['distance']), float_to_bytes(command['angle']))
                    client_socket.send(
                        int_to_one_bytes(COMMAND.GOTO_TARGET.value) +
                        float_to_bytes(command['distance']) +
                        float_to_bytes(command['angle'])
                    )
                else:
                    pass
            
           
                feedback = json.loads(client_socket.recv(512).decode('utf-8'))
                #print("遥控器收到feed_back: ", feedback)
                robot_state_info_queue.put(feedback)

                if robot_state_info_queue.qsize() > 50:
                    robot_state_info_queue.get()
                
        print("遥控器被用户停止")

    except KeyboardInterrupt:
        print("遥控器已停止。")
    except Exception as e:
        print(f"遥控器错误: {e}")
    finally:
        client_socket.close()
        print("遥控器线程终止。")
        