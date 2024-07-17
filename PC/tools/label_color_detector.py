import cv2
import numpy as np
import json

# 打开摄像头
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# 用于保存标签信息的列表
labels = []

def save_labels_to_file(labels, filename="labels.json"):
    with open(filename, 'w') as file:
        json.dump(labels, file, indent=4)

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w, _ = hsv.shape
        x_start, x_end = max(0, x - 20), min(w, x + 20)
        y_start, y_end = max(0, y - 20), min(h, y + 20)
        region = hsv[y_start:y_end, x_start:x_end]
        avg_hsv = np.mean(region, axis=(0, 1))
        label_id = f"label{len(labels)}"
        label_info = {
            "id": label_id,
            "hsv_range": {
                "lower": (max(0, avg_hsv[0] - 20), max(0, avg_hsv[1] - 50), max(0, avg_hsv[2] - 50)),
                "upper": (min(179, avg_hsv[0] + 20), min(255, avg_hsv[1] + 50), min(255, avg_hsv[2] + 50))
            }
        }
        labels.append(label_info)
        print(f"标签信息已保存：{label_info}")
        save_labels_to_file(labels)

# 设置鼠标回调函数
cv2.namedWindow('Frame')
cv2.setMouseCallback('Frame', mouse_callback)

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法从摄像头读取视频帧")
        break

    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
