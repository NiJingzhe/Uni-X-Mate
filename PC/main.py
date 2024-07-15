from tele_camera import tele_camera
from remote_control_socket import remote_control_socket
from dashboard import run_dashborad
from ai_detector import ai_detect
from auto_label_locate import mono_cam_orb
import multiprocessing
import json
import keyboard

# 使用 multiprocessing 的 Queue
g_frame_queue = multiprocessing.Queue()
g_result_queue = multiprocessing.Queue()
g_movement_info_queue = multiprocessing.Queue()
g_trajectory_list_queue = multiprocessing.Queue()

def main():
    config = json.load(open('config.json'))
    video_target = config["ip"] + ":" + str(config["video_port"])  # Ensure correct formatting
    label_list = json.load(open('labels.json'))

    # 进程取代线程
    #remote_control_process = multiprocessing.Process(target=remote_control, args=(control_target, g_movement_info_queue,))
    remote_control_process = multiprocessing.Process(target=remote_control_socket, args=(config["ip"],
                                                                                         config["control_port"],g_movement_info_queue,))
    tele_camera_process = multiprocessing.Process(target=tele_camera, args=(video_target, config["width"],
                                                                            config["height"],
                                                                            g_frame_queue, g_result_queue,))
    ai_detect_process = multiprocessing.Process(target=ai_detect, args=(config["model_path"],
                                                                        g_frame_queue, g_result_queue,))
    
    mono_cam_orb_process = multiprocessing.Process(target=mono_cam_orb, args=(g_frame_queue, label_list, 
                                                                              g_trajectory_list_queue, True,))
    #dashboard_process = multiprocessing.Process(target=run_dashborad)

    # 将进程设置为守护进程
    remote_control_process.daemon = True
    tele_camera_process.daemon = True
    ai_detect_process.daemon = True
    mono_cam_orb_process.daemon = True

    #dashboard_process.start()
    ai_detect_process.start()
    tele_camera_process.start()
    remote_control_process.start()
    mono_cam_orb_process.start()


    print("Press 'p' to terminate the program.")

    # 监听按键事件
    keyboard.wait('p')

    # 程序结束时终止所有进程
    remote_control_process.terminate()
    tele_camera_process.terminate()
    ai_detect_process.terminate()
    mono_cam_orb_process.terminate()
    #dashboard_process.terminate()

    # 等待进程终止
    remote_control_process.join()
    tele_camera_process.join()
    ai_detect_process.join()
    mono_cam_orb_process.join()
    #dashboard_process.join()

    print("Program terminated.")


if __name__ == "__main__":
    main()
