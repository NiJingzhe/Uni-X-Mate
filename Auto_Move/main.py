from FSM import State, Event, FSM
from keyboard import is_pressed
from enum import Enum
from utils import *
from auto_label_locate import calc_pos_and_trans, rotation_matrix_to_euler_angles
import math
import time
import multiprocessing as mtp
import json
import keyboard
from V2T import get_voice_result
from tele_camera import tele_camera
from remote_control_socket import remote_control_socket
from ai_detector import ai_detect

# 实例化状态
init_state = State("init")
idle_state = State("idle")
listen_target_state = State("listen_target")
process_target_state = State("process_target")

# 实例化转移事件
init_finished_event = Event("init_finished")
target_listen_start_event = Event("target_listen_start")
target_listen_finished_event = Event("target_listen_finished")
target_process_finished_event = Event("target_process_finished")

# 添加转移关系
init_state.add_transition(init_finished_event, idle_state)
idle_state.add_transition(target_listen_start_event, listen_target_state)
listen_target_state.add_transition(target_listen_finished_event, process_target_state)
process_target_state.add_transition(target_process_finished_event, idle_state)

# 设置部分状态的进入和退出动作
init_state.set_enter_action(lambda s: print("->>开始开机自检..."))
init_state.set_exit_action(lambda s: print("开机自检完成->>"))
idle_state.set_process_action(lambda self: self.fsm.trigger_event("target_listen_start"))
listen_target_state.set_enter_action(lambda s: print("->>开始监听指令..."))
listen_target_state.set_exit_action(lambda s: print("监听指令完成->>"))
process_target_state.set_enter_action(lambda s: print("->>开始处理指令..."))
process_target_state.set_exit_action(lambda s: print("处理指令完成->>"))

# 实例化FSM
RoboFSM = FSM()
RoboFSM.add_state([init_state, idle_state, listen_target_state, process_target_state])
#RoboFSM.add_event([init_finished_event, target_listen_start_event, target_listen_finished_event, target_process_finished_event])
RoboFSM.set_start_state("init")


def FSM_decider_process(frame_queue, robo_state_queue, voice_command_queue, yolo_result_queue, command_queue):
    
    RoboFSM.add_blackboard_data("frame_queue", frame_queue)
    RoboFSM.add_blackboard_data("robo_state_queue", robo_state_queue)
    RoboFSM.add_blackboard_data("voice_command_queue", voice_command_queue)
    RoboFSM.add_blackboard_data("command_queue", command_queue)
    RoboFSM.add_blackboard_data("yolo_result_queue", yolo_result_queue)
    
    while not keyboard.is_pressed("p"):
        RoboFSM.process()
    
    
def init_process_action(self : State):
    
    command_queue = RoboFSM.get_blackboard_data("command_queue")
    robo_state_queue = RoboFSM.get_blackboard_data("robo_state_queue")  
    
    command = {}
    command["command"] = COMMAND.SELF_CHECK.value

    if robo_state_queue.empty():
        command_queue.put(command)
        if command_queue.qsize() > 5:
            command_queue.get()
        #command_queue.put(command)
        print("***开机自检中***")
        self.fsm.trigger_event(init_finished_event) 
    
    state = robo_state_queue.get()
    if state["self_check_result"] == 1:
        print("***自检成功***")
        self.fsm.trigger_event(init_finished_event)
    else:
        print("***自检失败，程序退出***")
        exit(0)
        

def idle_process_action(self : State):
    
    self.fsm.trigger_event(target_listen_start_event)   
    
    
def listen_target_process_action(self : State):
    
    voice_command_queue = self.fsm.get_blackboard_data("voice_command_queue")
    
    voice_command = get_voice_result()
    while not voice_command:
        voice_command = get_voice_result()
        
    #print("Voice command is : ", voice_command)
    
    voice_command_queue.put(voice_command)  
    
    self.fsm.trigger_event(target_listen_finished_event)
    
def process_target_process_action(self : State):
    
    voice_command_queue = self.fsm.get_blackboard_data("voice_command_queue")
    yolo_result_queue = self.fsm.get_blackboard_data("yolo_result_queue")   
    command_queue = self.fsm.get_blackboard_data("command_queue")

    if not voice_command_queue.empty():
        voice_command = voice_command_queue.get()
        
    print("voice command is : ", voice_command)
        
    command_result = {}
    delta_x = 99999
    delta_y = 99999
    while delta_x > 0.03 or delta_y > 0.35:
        time.sleep(0.2)
        if not yolo_result_queue.empty():  # yolo是否是多线程的？问题1，当前画面中没有目标，yolo会一直检测，导致队列不为空，导致程序卡死
            yolo_result_list = yolo_result_queue.get()
            for result in yolo_result_list:
                if result["class_name"] == voice_command["object"]:
                    print("***视野中有该物体***")
                    command_result['command'] = COMMAND.GOTO_TARGET.value
                    command_result['distance'] = math.sqrt(
                        result["position"][0] ** 2 + result["position"][1] ** 2
                    )
                    command_result['angle'] = math.atan2(result["position"][1], result["position"][0]) - 1.57
                    delta_x, delta_y = result["position"]
                    print("command is : ", command_result, "delta is : ", delta_x, delta_y)
                    command_queue.put(command_result)
                    break
        else:
            command_result['command'] = COMMAND.FIND_TAG.value
            command_queue.put(command_result)

            
        if command_queue.qsize() > 3:
            command_queue.get()

    command_result['command'] = COMMAND.GRAB.value
    command_queue.put(command_result)
    
    if command_queue.qsize() > 3:
        command_queue.get()
    
    if not voice_command["in_memory"]:
        pass 
        # 之后要实现未知物体入库
    
    print("***到达目标并发送抓取指令***")
    while not yolo_result_queue.empty():
        yolo_result_queue.get()
    self.fsm.trigger_event(target_process_finished_event)
    
    
init_state.set_process_action(init_process_action)
idle_state.set_process_action(idle_process_action)
listen_target_state.set_process_action(listen_target_process_action)
process_target_state.set_process_action(process_target_process_action)

g_frame_queue = mtp.Queue() 
g_robo_state_queue = mtp.Queue()
g_voice_command_queue = mtp.Queue()
g_yolo_result_queue = mtp.Queue()
g_command_queue = mtp.Queue()

if __name__ == "__main__":
    
    config = json.load(open("config.json", "r"))    
    obj_label_info = config["object_label"]
    ip = config["ip"]
    video_port = config["video_port"]
    remote_control_port = config["control_port"]

    cam_width = config["width"]
    cam_height = config["height"]
    
    camera_process = mtp.Process(
        target=tele_camera, 
        args=(
            ip, video_port, cam_width, cam_height, g_frame_queue,
        ),
        daemon=True
    )
    
    ai_detector_process = mtp.Process(
        target=ai_detect, 
        args=(
            config["model_path"], g_frame_queue, g_yolo_result_queue, obj_label_info, True,
        ),
        daemon=True
    )
    
    auto_decider_process = mtp.Process(
        target=FSM_decider_process, 
        args=(
            g_frame_queue, g_robo_state_queue, g_voice_command_queue, g_yolo_result_queue, g_command_queue,
        ),
        daemon=True
    )
    
    remote_control_process = mtp.Process(
        target=remote_control_socket, 
        args=(
            ip, remote_control_port, g_robo_state_queue, g_command_queue,
        ),
        daemon=True
    )
    
    
    camera_process.start()
    auto_decider_process.start()
    remote_control_process.start()  
    ai_detector_process.start()
    
    keyboard.wait("p")
    
    camera_process.terminate()
    auto_decider_process.terminate()
    remote_control_process.terminate()
    ai_detector_process.terminate()
    
    camera_process.join()
    auto_decider_process.join()
    remote_control_process.join()
    ai_detector_process.join()
    
    
    
    