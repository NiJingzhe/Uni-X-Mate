import keyboard
import requests
import time

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

def send_post_request(target_ip, forward_back, left_right):
    data = {
        'forward_back': forward_back,
        'left_right': left_right
    }
    try:
        response = requests.post(target_ip, json=data)
        print(f"Response: {response.status_code}, {response.text}")
    except requests.exceptions.RequestException as e:
        print(f"Error sending request: {e}")

def main():
    ip = input("请输入目标IP地址：")
    if ip == "":
        ip = "192.168.137.3"
    print("遥控器已启动，按下 'Ctrl + C' 退出。")
    try:
        while True:
            forward_back, left_right = get_key_state()
            print(f"forward is : {forward_back}, left is : {left_right}")
            send_post_request("http://"+ip+":5000/move", forward_back, left_right)
            time.sleep(0.01)  # 调整请求发送频率
    except KeyboardInterrupt:
        print(f"遥控器已停止。")

if __name__ == "__main__":
    main()
