from multiprocessing import Process
import os

def run_video_server():
    os.system('python3 video_server.py')

def run_control_server():
    os.system('python3 control_server.py')

if __name__ == '__main__':
    video_server_process = Process(target=run_video_server)
    control_server_process = Process(target=run_control_server)

    video_server_process.start()
    control_server_process.start()

    video_server_process.join()
    control_server_process.join()
