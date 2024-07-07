# tele_camera.py
from utils import create_mqtt_client, send_post_request
import time
import json
import numpy as np
import cv2
import keyboard
import os
from queue import Queue

SUB_TOPIC_MOVE = 'robot/movement_state'
PUB_TOPIC = 'robot/camera_frame'
SUB_TOPIC_AI = 'ai/yolo_result'

result_queue = Queue()
   
def on_message(client, userdata, msg):
    result_dict = json.loads(msg.payload)
    

frame_sender = create_mqtt_client("图传MQTT", on_message=on_message, sub_topic=[SUB_TOPIC_MOVE, SUB_TOPIC_AI])

def tele_camera(target, width, height):
    global frame_sender
    
    host = target
    try:
        while not keyboard.is_pressed("p"):  
            response = send_post_request("http://" + host + "/video", {'width': width, 'height': height})
            if response is not None and response.headers['Content-Type'] == 'image/jpeg':
                image_data = np.frombuffer(response.content, np.uint8)
                frame_sender.publish(PUB_TOPIC, response.content)
                frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                if frame is not None:
                    cv2.imshow("Tele Camera", frame)
                    if cv2.waitKey(1) & 0xFF == ord('p'):
                        cv2.destroyAllWindows()
                        break
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("图传线程终止。")
        exit(0)
    finally:
        cv2.destroyAllWindows()
        print("图传线程终止。")
        exit(0)