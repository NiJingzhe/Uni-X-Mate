# tele_camera.py
from utils import *
import numpy as np
import cv2
import keyboard
from queue import Queue
import time

def tele_camera(target, width, height, frame_queue : Queue, result_queue : Queue):
    
    host = target
    try:
        while not keyboard.is_pressed("p"):
            #start_time = time.time()
            response = send_post_request("http://" + host + "/video", {'width': width, 'height': height})
            #end_time = time.time()
            #print("=======" * 5, "\n图传时间: ", end_time - start_time, "\n====================")
            if response is not None and response.headers['Content-Type'] == 'image/jpeg':
                image_data = np.frombuffer(response.content, np.uint8)
                frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                #start_time = time.time()
                frame_queue.put(frame)
                if frame_queue.qsize() > 5:
                    frame_queue.get()
                #end_time = time.time()
                #print("=======" * 5, "\n图传->AI时间: ", end_time - start_time, "\n====================")
                #start_time = time.time()
                if frame is not None:
                    while not result_queue.empty():
                        result = result_queue.get()
                        if len(result.keys()) == 0:
                            continue
                        x1, y1, x2, y2 = result["xyxy"]
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        class_name = result["class_name"]
                        confidence = result["confidence"]

                        #print("xyxy: ", x1, y1, x2, y2, "class_name: ", class_name, "confidence: ", confidence)

                        if confidence > 0.5:
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(frame, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                            cv2.putText(frame, str(confidence), (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                    cv2.imshow("Tele Camera", frame)

                    if cv2.waitKey(1) & 0xFF == ord('p'):
                        cv2.destroyAllWindows()
                        break
                #end_time = time.time()
                #print("=======" * 5, "\n拿结果+绘制时间: ", end_time - start_time, "\n====================\n\n\n")
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