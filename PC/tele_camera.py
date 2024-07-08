from utils import *
import numpy as np
import cv2
import keyboard
from queue import Queue

def tele_camera(target, width, height, frame_queue: Queue, result_queue: Queue):
    host = target
    try:
        while not keyboard.is_pressed("p"):
            # 获取视频流
            # response = requests.get(f"http://{host}/video_feed", params = {"width" : width, "height": height}, stream=True)
            
            # if response.status_code == 200 and response.headers['Content-Type'] == 'multipart/x-mixed-replace; boundary=frame':
            #     bytes = b''
            #     for chunk in response.iter_content(chunk_size=1024):
            #         bytes += chunk
            #         a = bytes.find(b'\xff\xd8')
            #         b = bytes.find(b'\xff\xd9')
            #         if a != -1 and b != -1:
            #             jpg = bytes[a:b+2]
            #             bytes = bytes[b+2:]
            #             image_data = np.frombuffer(jpg, np.uint8)
            #             frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            #             #frame = image_data
            #             frame_queue.put(frame)
            #             if frame_queue.qsize() > 5:
            #                 frame_queue.get()
            #             if frame is not None:
            #                 while not result_queue.empty():
            #                     result = result_queue.get()
            #                     if len(result.keys()) == 0:
            #                         continue
            #                     x1, y1, x2, y2 = result["xyxy"]
            #                     x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            #                     class_name = result["class_name"]
            #                     confidence = result["confidence"]

            #                     if confidence > 0.5:
            #                         cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            #                         cv2.putText(frame, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            #                         cv2.putText(frame, str(confidence), (x1, y1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            #                 cv2.imshow("Tele Camera", frame)

            #                 if cv2.waitKey(1) & 0xFF == ord('p'):
            #                     cv2.destroyAllWindows()
            #                     break
            # else:
            #     print("Failed to connect to video feed")
            
            response = send_post_request(f"http://{host}/video", params = {"width" : width, "height": height})
            if response.status_code == 200:
                image_data = np.frombuffer(response.content, np.uint8)
                frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                frame_queue.put(frame)
                if frame_queue.qsize() > 5:
                    frame_queue.get()
                if frame is not None:
                    while not result_queue.empty():
                        result = result_queue.get()
                        if len(result.keys()) == 0:
                            continue
                        x1, y1, x2, y2 = result["xyxy"]
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        class_name = result["class_name"]
                        confidence = result["confidence"]

                        if confidence > 0.5:
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(frame, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                            cv2.putText(frame, str(confidence), (x1, y1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

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
