import torch
from ultralytics import YOLOv10
import keyboard
def ai_detect(model_path, frame_queue, result_queue):
    
    model = YOLOv10(model_path)
    # 检查是否有可用的GPU
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model.to(device)
    
    try:
        while not keyboard.is_pressed("p"):
            while not frame_queue.empty():
                frame = frame_queue.get()
                results = model(frame, device = device)
                for result in results:
                    result_dict = {}
                    for box in result.boxes:
                        x1, y1, x2, y2 = box.xyxy[0]
                        x1, y1, x2, y2 = int(x1.item()), int(y1.item()), int(x2.item()), int(y2.item())
                        result_dict["xyxy"] = (x1, y1, x2, y2)
                        result_dict["confidence"] = float(box.conf[0].item())
                        result_dict["class_name"] = model.names[int(box.cls[0].item())]
                    result_queue.put(result_dict)
                    if result_queue.qsize() > 10:
                        result_queue.get()
                
    except KeyboardInterrupt:
        print("AI检测线程终止。")
        exit(0)
    except Exception as e:
        print("AI检测线程异常终止。", e)
    finally:
        print("AI检测线程终止。")
        exit(0)