from remote_controller import remote_control
from tele_camera import tele_camera
from ai_detector import ai_detect
import multiprocessing
import os
import json
from queue import Queue

# 使用 multiprocessing 的 Queue
g_frame_queue = multiprocessing.Queue()
g_result_queue = multiprocessing.Queue()
g_movement_info_queue = multiprocessing.Queue()


def main():
    config = json.load(open('config.json'))
    target = config["ip"] + ":" + str(config["port"])  # Ensure correct formatting

    # 进程取代线程
    remote_control_process = multiprocessing.Process(target=remote_control, args=(target, g_movement_info_queue,))
    tele_camera_process = multiprocessing.Process(target=tele_camera, args=(target, config["width"],
                                                                            config["height"],
                                                                            g_frame_queue, g_result_queue,))
    ai_detect_process = multiprocessing.Process(target=ai_detect, args=(config["model_path"],
                                                                        g_frame_queue, g_result_queue,))

    remote_control_process.start()
    tele_camera_process.start()
    ai_detect_process.start()

    remote_control_process.join()
    tele_camera_process.join()
    ai_detect_process.join()


if __name__ == "__main__":
    main()
