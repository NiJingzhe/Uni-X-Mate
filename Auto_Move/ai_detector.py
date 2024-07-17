import torch
from ultralytics import YOLOv10
import keyboard
from auto_label_locate import calc_pos_and_trans
import cv2

model = None
device = None
label_info = None


def ai_detector_init(model_path, label_info_: dict):
    global model
    global device
    global label_info

    print("==="*4)
    print(f"Loading Model from {model_path}")
    print("==="*4)
    model = YOLOv10(model_path)

    # 检查是否有可用的GPU
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model.to(device)
    label_info = label_info_

    print("==="*4)
    print(f"Finished loading Model from {model_path}")
    print("==="*4)


def ai_detect(frame, debug=True) -> list:

    global model
    global device
    global label_info

    try:
        if model is not None:
            results = model(frame, device=device, verbose=False)
            result_list = []
            for result in results:
                for box in result.boxes:
                    result_dict = {}  # 每次都创建一个新的字典
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 =    \
                        int(x1.item()), int(y1.item()), \
                        int(x2.item()), int(y2.item())
                    result_dict["xyxy"] = (x1, y1, x2, y2)
                    result_dict["confidence"] = float(box.conf[0].item())
                    result_dict["class_name"] = model.names[int(box.cls[0].item())]

                    if result_dict["confidence"] >= 0.5:
                        # 放大bounding box到原来的1.1倍
                        x1 = max(0, x1 - int((x2 - x1) * 0.15))
                        y1 = max(0, y1 - int((y2 - y1) * 0.15))
                        x2 = min(frame.shape[1], x2 + int((x2 - x1) * 0.15))
                        y2 = min(frame.shape[0], y2 + int((y2 - y1) * 0.15))
                        result_dict["xyxy"] = (x1, y1, x2, y2)
                        _, _, position, _ = calc_pos_and_trans(
                            frame, label_info, result_dict["xyxy"], debug)
                        # 绘制bounding box
                        x1, y1, x2, y2 = result_dict["xyxy"]

                        cv2.rectangle(frame, (x1, y1), (x2, y2),
                                    (255, 105, 180), 1)
                        # cv2.rectangle(frame, )
                        cv2.putText(
                            frame, f"{result_dict['class_name']} {result_dict['confidence']:.2f}",
                            (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 105, 180), 2
                        )
                        if position is not None:
                            result_dict["position"] = (position[0], position[2])
                            result_list.append(result_dict)

        cv2.imshow('ai_detect', frame)
        cv2.waitKey(1)

        return result_list

    except KeyboardInterrupt:
        print("AI检测线程终止。")
    except Exception as e:
        print("AI检测线程异常终止。", e)
