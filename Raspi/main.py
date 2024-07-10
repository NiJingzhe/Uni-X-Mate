from multiprocessing import Process
from video_server import video_server_main
from control_server_socket import control_socket_start

if __name__ == '__main__':
    video_server_process = Process(target=video_server_main)
    control_server_process = Process(target=control_socket_start)

    try:
        video_server_process.start()
        control_server_process.start()

        video_server_process.join()
        control_server_process.join()
    
    except KeyboardInterrupt:
        print("Server Interrupted by user")
        
    except Exception as e:
        print(f"Error : {e}")
        
    finally:
        video_server_process.terminate()
        control_server_process.terminate()  