# main.py
from remote_controller import remote_control
from tele_camera import tele_camera
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
    
    remote_control_thread.start()
    tele_camera_thread.start()
    
    remote_control_thread.join()
    tele_camera_thread.join()
    
        
    
if __name__ == "__main__":
    main()
