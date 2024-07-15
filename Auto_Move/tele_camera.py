from utils import *
import numpy as np
import cv2
import keyboard
from queue import Queue

def tele_camera(target_ip, target_port, width, height, frame_queue: Queue):
    host = target_ip + ":" + str(target_port)
    try:
        while not keyboard.is_pressed("p"):
            #获取视频流
            response = requests.get(f"http://{host}/video_feed", params = {"width" : width, "height": height}, stream=True)
            
            if response.status_code == 200 and response.headers['Content-Type'] == 'multipart/x-mixed-replace; boundary=frame':
                bytes = b''
                for chunk in response.iter_content(chunk_size=1024):
                    bytes += chunk
                    a = bytes.find(b'\xff\xd8')
                    b = bytes.find(b'\xff\xd9')
                    if a != -1 and b != -1:
                        jpg = bytes[a:b+2]
                        bytes = bytes[b+2:]
                        image_data = np.frombuffer(jpg, np.uint8)
                        frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                        frame_queue.put(frame)
                        if frame_queue.qsize() > 5:
                            frame_queue.get()
                        if frame is not None:
                            cv2.imshow("Tele Camera", frame)

                            if cv2.waitKey(1) & 0xFF == ord('p'):
                                cv2.destroyAllWindows()
                                break
            else:
                print("Failed to connect to video feed")
                
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
