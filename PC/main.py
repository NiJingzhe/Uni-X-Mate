# main.py
from remote_controller import remote_control
from tele_camera import tele_camera
from ai_detector import ai_detect
import threading
import os
import json
import keyboard

def main():
    config = json.load(open('config.json'))
    target = config["ip"] + ":" + str(config["port"])  # Ensure correct formatting
    
    #stop_event = threading.Event()  # Event to signal threads to stop
    
    # Correct argument passing as a tuple
    remote_control_thread = threading.Thread(target=remote_control, args=(target,))
    tele_camera_thread = threading.Thread(target=tele_camera, args=(target, config["width"], config["height"],))
    ai_detect_thread = threading.Thread(target=ai_detect, args=(config["model_path"],))
    
    remote_control_thread.start()
    tele_camera_thread.start()
    ai_detect_thread.start()
    
    remote_control_thread.join()
    tele_camera_thread.join()
    ai_detect_thread.join()
        
    
if __name__ == "__main__":
    main()
