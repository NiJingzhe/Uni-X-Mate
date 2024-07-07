import cv2
import torch
from queue import Queue
from ultralytics import YOLOv10
from utils import *
import json
import keyboard

SUB_TOPIC = "robot/camera_frame"
PUB_TOPIC = "ai/yolo_result"

frame_queue = Queue()

def on_message(client, userdata, msg):
    #print(msg.topic + " " + str(msg.payload))
    img = msg.payload
    frame_queue.put(img)
    

frame_reveiver_sender = create_mqtt_client(name="AI MQTT", on_message=on_message, sub_topic=SUB_TOPIC)

def ai_detect(model_path):
    
    model = YOLOv10(model_path)
    # 检查是否有可用的GPU
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model.to_device(device)
    
    try:
        while not keyboard.is_pressed("p"):
            while not frame_queue.empty():
                frame = frame_queue.get()
                results = model(frame, device = device)
                results_list = []
                for result in results:
                    result_dict = {}
                    for box in result.boxes:
                        result_dict["xmin"] = box.xyxy[0][0].item()
                        result_dict["ymin"] = box.xyxy[0][1].item()
                        result_dict["xmax"] = box.xyxy[0][2].item()
                        result_dict["ymax"] = box.xyxy[0][3].item()
                        result_dict["confidence"] = box.conf[0].item()
                        result_dict["class"] = box.cls[0].item()
                    results_list.append(result_dict)
                        
                frame_reveiver_sender.publish(PUB_TOPIC, json.dumps({"result":results_list}))
                
    except KeyboardInterrupt:
        print("AI检测线程终止。")
        exit(0)
    finally:
        print("AI检测线程终止。")
        exit(0)