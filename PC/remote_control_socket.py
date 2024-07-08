import socket
import keyboard
import time
import queue

def get_key_state():
    forward_back = 0
    left_right = 0
    
    if keyboard.is_pressed('w'):
        forward_back = 1
    elif keyboard.is_pressed('s'):
        forward_back = -1
        
    if keyboard.is_pressed('a'):
        left_right = 1
    elif keyboard.is_pressed('d'):
        left_right = -1
        
    return forward_back, left_right

def remote_control(target_ip, target_port, movement_info_queue):
    host = target_ip
    port = target_port
    print("遥控器已启动")

    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((host, port))
        
        counter = 0
        while not keyboard.is_pressed("p"):
            forward_back, left_right = get_key_state()
            command = f"{forward_back},{left_right}\n"
            start_time = time.time()
            client_socket.sendall(command.encode('utf-8'))
            end_time = time.time()
            print("send time is : ", end_time - start_time)
            
            counter += 1
            if counter < 20:
                continue
            counter = 20
            
            try:
                feedback = client_socket.recv(1024).decode('utf-8')
                movement_info_queue.put(feedback)
                
                if movement_info_queue.qsize() > 1:
                    movement_info_queue.get()
            except Exception as e:
                print(f"遥控器错误: {e}")
    except KeyboardInterrupt:
        print("遥控器已停止。")
    except Exception as e:
        print(f"遥控器错误: {e}")
    finally:
        client_socket.close()
        print("遥控器线程终止。")