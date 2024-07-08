# main.py
from remote_controller import remote_control
from tele_camera import tele_camera
from ai_detector import ai_detect
import threading
import os
import json
import keyboard
from queue import Queue

g_frame_queue = Queue()
g_result_queue = Queue()
g_movement_info_queue = Queue

def main():
    config = json.load(open('config.json'))
    target = config["ip"] + ":" + str(config["port"])  # Ensure correct formatting
    
    #stop_event = threading.Event()  # Event to signal threads to stop
    
    # Correct argument passing as a tuple
    remote_control_thread = threading.Thread(target=remote_control, args=(target,g_movement_info_queue,))
    tele_camera_thread = threading.Thread(target=tele_camera, args=(target, config["width"],
                                                                    config["height"],
                                                                    g_frame_queue,g_result_queue,))
    ai_detect_thread = threading.Thread(target=ai_detect, args=(config["model_path"],
                                                                g_frame_queue,g_result_queue,))
    
    remote_control_thread.start()
    tele_camera_thread.start()
    ai_detect_thread.start()
    
    remote_control_thread.join()
    tele_camera_thread.join()
    ai_detect_thread.join()
        
    
if __name__ == "__main__":
    main()
