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

result_list = []
   
def on_message(client, userdata, msg):
    
    global result_list
    if msg.topic == SUB_TOPIC_AI:
        result_list = json.loads(msg.payload)["result"]
        
        #print("===="*20)
        #print(result_list)
        #print("\n\n\nlen: ", len(result_list))
        #print("===="*20)
        
        

frame_sender = create_mqtt_client("图传MQTT", on_message=on_message, sub_topic=[SUB_TOPIC_MOVE, SUB_TOPIC_AI])

def tele_camera(target, width, height):
    global frame_sender
    global result_list
    
    host = target
    try:
        while not keyboard.is_pressed("p"):  
            response = send_post_request("http://" + host + "/video", {'width': width, 'height': height})
            if response is not None and response.headers['Content-Type'] == 'image/jpeg':
                image_data = np.frombuffer(response.content, np.uint8)
                frame_sender.publish(PUB_TOPIC, response.content)
                frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                if frame is not None:
                    for result in result_list:
                        x1, y1, x2, y2 = result["xyxy"]
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        class_name = result["class_name"]
                        confidence = result["confidence"]

                        print("xyxy: ", x1, y1, x2, y2, "class_name: ", class_name, "confidence: ", confidence)
                        
                        if confidence > 0.5:      
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(frame, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                            cv2.putText(frame, str(confidence), (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                            
                    cv2.imshow("Tele Camera", frame)
                    if cv2.waitKey(1) & 0xFF == ord('p'):
                        cv2.destroyAllWindows()
                        break
    except KeyboardInterrupt:
        
        cv2.destroyAllWindows()
        print("图传线程终止。")
        exit(0)
        
    except Exception as e:
        print(f"图传线程异常终止 : {e}")
    finally:
        cv2.destroyAllWindows()
        print("图传线程终止。")
        exit(0)