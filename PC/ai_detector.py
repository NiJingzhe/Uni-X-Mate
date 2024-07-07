import cv2
import torch
from queue import Queue
from ultralytics import YOLOv10
from utils import *
import json
import keyboard
import numpy as np

SUB_TOPIC = "robot/camera_frame"
PUB_TOPIC = "ai/yolo_result"

frame_queue = Queue()

def on_message(client, userdata, msg):
    #print(msg.topic + " " + str(msg.payload))
    img = msg.payload
    img = cv2.imdecode(np.frombuffer(img, np.uint8), cv2.IMREAD_COLOR)
    # turn img to numpy array
    frame_queue.put(img)
    if frame_queue.qsize() > 50:
        frame_queue.get()
    
    #print("img received")

frame_reveiver_sender = create_mqtt_client(name="AI MQTT", on_message=on_message, sub_topic=SUB_TOPIC)

def ai_detect(model_path):
    
    model = YOLOv10(model_path)
    # 检查是否有可用的GPU
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model.to(device)
    
    try:
        while not keyboard.is_pressed("p"):
            while not frame_queue.empty():
                frame = frame_queue.get()
                results = model(frame, device = device)
                results_list = []
                for result in results:
                    result_dict = {}
                    for box in result.boxes:
                        x1, y1, x2, y2 = box.xyxy[0]
                        x1, y1, x2, y2 = int(x1.item()), int(y1.item()), int(x2.item()), int(y2.item())
                        result_dict["xyxy"] = (x1, y1, x2, y2)
                        result_dict["confidence"] = float(box.conf[0].item())
                        result_dict["class_name"] = model.names[int(box.cls[0].item())]
                    results_list.append(result_dict)
                        
                frame_reveiver_sender.publish(PUB_TOPIC, json.dumps({"result":results_list}))
                
    except KeyboardInterrupt:
        print("AI检测线程终止。")
        exit(0)
    except Exception as e:
        print("AI检测线程异常终止。", e)
    finally:
        print("AI检测线程终止。")
        exit(0)