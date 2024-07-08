# remote_control.py
import keyboard
from utils import *

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

def remote_control(target, movement_info_queue):

    host = target
    print("遥控器已启动")
    try:
        while not keyboard.is_pressed("p"):
            forward_back, left_right = get_key_state()
            # 发送运动控制指令
            response = send_post_request("http://" + host + "/move", {"forward_back": forward_back, "left_right": left_right})
            movement_state = response.json()["movement_state"]
            movement_info_queue.put(movement_state)
            if movement_info_queue.qsize() > 100:
                movement_info_queue.get()
    except KeyboardInterrupt:
        print("遥控器已停止。")
        exit(0)
    except Exception as e:
        print(f"遥控器错误: {e}")
        exit(0)
    finally:
        print("遥控器线程终止。")
        exit(0)
