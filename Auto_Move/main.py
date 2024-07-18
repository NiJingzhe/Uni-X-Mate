from FSM import State, Event, FSM
from keyboard import is_pressed
from enum import Enum
from utils import *
from auto_label_locate import *
import math
import time
import multiprocessing as mtp
import json
import keyboard
from V2T import get_voice_result
from tele_camera import tele_camera
from remote_control_socket import remote_control_socket
from ai_detector import ai_detect, ai_detector_init
from LLM import RobotGPT
from TTS import GPTSoVits

# 实例化状态
init_state = State("init")
idle_state = State("idle")
listen_target_state = State("listen_target")
process_target_state = State("process_target")
auto_walk_state = State("auto_walk")

# 实例化转移事件
init_finished_event = Event("init_finished")
target_listen_start_event = Event("target_listen_start")
target_listen_finished_event = Event("target_listen_finished")
target_listen_nothing_event = Event("target_listen_nothing")
target_process_finished_event = Event("target_process_finished")
auto_walk_finished_event = Event("auto_walk_finished")


# 添加转移关系
init_state.add_transition(init_finished_event, idle_state)
idle_state.add_transition(target_listen_start_event, listen_target_state)
listen_target_state.add_transition(
    target_listen_finished_event, process_target_state)
listen_target_state.add_transition(
    target_listen_nothing_event, auto_walk_state)
auto_walk_state.add_transition(auto_walk_finished_event, process_target_state)
process_target_state.add_transition(target_process_finished_event, idle_state)

# 设置部分状态的进入和退出动作
init_state.set_enter_action(lambda s: print("->>开始开机自检..."))
init_state.set_exit_action(lambda s: print("开机自检完成->>"))
idle_state.set_process_action(
    lambda self: self.fsm.trigger_event("target_listen_start"))
listen_target_state.set_enter_action(lambda s: print("->>开始监听指令..."))
listen_target_state.set_exit_action(lambda s: print("监听指令完成->>"))
process_target_state.set_enter_action(lambda s: print("->>开始处理指令..."))
process_target_state.set_exit_action(lambda s: print("处理指令完成->>"))
auto_walk_state.set_enter_action(lambda s: print("->>开始自动游走..."))
auto_walk_state.set_exit_action(lambda s: print("自动游走结束->>"))


# 实例化FSM
RoboFSM = FSM()
RoboFSM.add_state([init_state, idle_state, listen_target_state,
                  process_target_state, auto_walk_state])
# RoboFSM.add_event([init_finished_event, target_listen_start_event, target_listen_finished_event, target_process_finished_event])
RoboFSM.set_start_state("init")


def FSM_decider_process(
    frame_queue, robo_state_queue,
    voice_command_queue, command_queue,
    model_path, obj_label_info, map_label_info
):

    RoboFSM.add_blackboard_data("frame_queue", frame_queue)
    RoboFSM.add_blackboard_data("robo_state_queue", robo_state_queue)
    RoboFSM.add_blackboard_data("voice_command_queue", voice_command_queue)
    RoboFSM.add_blackboard_data("command_queue", command_queue)
    RoboFSM.add_blackboard_data("model_path", model_path)
    RoboFSM.add_blackboard_data("obj_label_info", obj_label_info)
    RoboFSM.add_blackboard_data("map_label_info", map_label_info)

    while not keyboard.is_pressed("p"):
        RoboFSM.process()


def init_process_action(self: State):

    command_queue = RoboFSM.get_blackboard_data("command_queue")
    robo_state_queue = RoboFSM.get_blackboard_data("robo_state_queue")

    command = {}
    command["command"] = COMMAND.SELF_CHECK.value

    if robo_state_queue.empty():
        command_queue.put(command)
        if command_queue.qsize() > 5:
            command_queue.get()
        # command_queue.put(command)
        print("***开机自检中***")
        # self.fsm.trigger_event(init_finished_event)
    
    state = robo_state_queue.get()
    if state["robot_state"] == "正常":
        print("***自检成功***")
        model_path = self.fsm.get_blackboard_data("model_path")
        obj_label_info = self.fsm.get_blackboard_data("obj_label_info")
        ai_detector_init(model_path=model_path, label_info_=obj_label_info)
        self.fsm.trigger_event(init_finished_event)
    else:
        print("***自检失败，程序退出***")
        exit(0)


def idle_process_action(self: State):

    self.fsm.trigger_event(target_listen_start_event)


def listen_target_process_action(self: State):

    voice_command_queue = self.fsm.get_blackboard_data("voice_command_queue")
    robot_gpt = RobotGPT()
    tts_model = GPTSoVits("http://172.25.96.166:9880/tts/")

    voice_command = get_voice_result()
    # 尝试监听三次语音指令，如果三次都没有获取到，则进入自动游走状态
    count = 0
    while not voice_command and count < 2:
        voice_command = get_voice_result()
        count += 1

    if not voice_command:
        self.fsm.trigger_event(target_listen_nothing_event)
        return 
    
    # print("Voice command is : ", voice_command)
    user_said_content = voice_command["content"]
    if user_said_content == "No object found":
        tts_text = "Sorry but I can't find that object for you."
        user_said_content_type = 1
    else:
        text_type_result = robot_gpt.get_command(user_said_content)
        tts_text = text_type_result["content"]
        user_said_content_type = text_type_result["command"]
        
    
    # TTS 过程
    tts_model.run(tts_text)
    
    if user_said_content_type == 0:
        voice_command_queue.put(voice_command)
        self.fsm.trigger_event(target_listen_finished_event)
        return


def process_target_process_action(self: State):

    voice_command_queue = self.fsm.get_blackboard_data("voice_command_queue")
    frame_queue = self.fsm.get_blackboard_data("frame_queue")
    command_queue = self.fsm.get_blackboard_data("command_queue")

    if not voice_command_queue.empty():
        voice_command = voice_command_queue.get()

    print("voice command is : ", voice_command)

    delta_x = 99999
    delta_y = 99999

    while delta_x > 0.03 or delta_y > 0.20:
        # time.sleep(0.2)
        command_result = {}
        if not frame_queue.empty():  # yolo是否是多线程的？问题1，当前画面中没有目标，yolo会一直检测，导致队列不为空，导致程序卡死
            frame = frame_queue.get()
            yolo_result_list = ai_detect(frame, debug=True)
            detected_flag = False
            for result in yolo_result_list:
                if result["class_name"] == voice_command["object"]:
                    detected_flag = True
                    # print("***视野中有该物体***")
                    command_result['command'] = COMMAND.GOTO_TARGET.value
                    command_result['distance'] = float(
                        math.sqrt(
                            result["position"][0] ** 2 + result["position"][1] ** 2
                        )
                    )
                    command_result['angle'] = math.atan2(
                        result["position"][1], result["position"][0]) - 1.57
                    command_result['angle'] = float(
                        command_result['angle'] / 
                        math.pi * 180
                    )
                    delta_x, delta_y = result["position"]
                    # print("Command 0 被放入")
                    command_queue.put(command_result)
                    while command_queue.qsize() > 1:
                        command_queue.get()
                    # 绝对不应期
                    time.sleep(0.2)
                    break

            if not detected_flag:
                command_result['command'] = COMMAND.FIND_TAG.value
                # print("Command 1 被放入")
                command_queue.put(command_result)
                while command_queue.qsize() > 3:
                    command_queue.get()
                    # time.sleep(0.1)
                # 绝对不应期
                #time.sleep(0.2)

    command_result = {}
    command_result['command'] = COMMAND.GRAB.value
    command_queue.put(command_result)


    while command_queue.qsize() > 1:
        command_queue.get()

    time.sleep(0.5)

    command_result = {}
    command_result['command'] = COMMAND.FIND_TAG.value
    command_queue.put(command_result)

    while command_queue.qsize() > 1:
        command_queue.get()
    # 绝对不应期
    time.sleep(0.2)
    
    find_person = False
    while not find_person and not frame_queue.empty():
        frame = frame_queue.get()
        yolo_result_list = ai_detect(frame, debug=True)
        for result in yolo_result_list:
            if result["class_name"] == "person":
                x1, y1, x2, y2 = result["xyxy"]
                x_center = (x1 + x2) / 2
                y_center = (y1 + y2) / 2
                if x_center > 0.4 and x_center < 0.6 and y_center > 0.4 and y_center < 0.6:
                    find_person = True
                    break
                else:
                    command_result['command'] = COMMAND.FIND_TAG.value
                    command_queue.put(command_result)
                    while command_queue.qsize() > 1:
                        command_queue.get()

    command_result['command'] = COMMAND.GOTO_TARGET.value
    command_result['distance'] = 0.5
    command_result['angle'] = 0
    command_queue.put(command_result)
    while command_queue.qsize() > 1:
        command_queue.get()

    if not voice_command["in_memory"]:
        pass
        # 之后要实现未知物体入库

    print("***到达目标并发送抓取指令***")
    while not voice_command_queue.empty():
        voice_command_queue.get()
    while not command_queue.empty():
        command_queue.get()

    self.fsm.trigger_event(target_process_finished_event)


def listen_for_voice_commands(voice_command_queue, stop_event : mtp.Event):
    while not stop_event.is_set():
        print("Enter listen_for_voice_commands while loop")
        voice_command = get_voice_result()
        if voice_command:
            voice_command_queue.put(voice_command)
            stop_event.set()

def auto_walk_process_action(self: State):
    frame_queue = self.fsm.get_blackboard_data("frame_queue")
    map_label_info = self.fsm.get_blackboard_data("map_label_info")
    voice_command_queue = self.fsm.get_blackboard_data("voice_command_queue")
    command_queue = self.fsm.get_blackboard_data("command_queue")
    
    #while not voice_command_queue.empty():
    #    voice_command_queue.get()   

    stop_event = mtp.Event()
    listener_process = mtp.Process(
        target=listen_for_voice_commands, args=(voice_command_queue, stop_event, ))
    listener_process.start()

    while not stop_event.is_set():
        commamd = {"command": COMMAND.GOTO_TARGET.value,
                   "distance": 9999, "angle": 0}
        command_queue.put(commamd)
        while command_queue.qsize() > 1:
            command_queue.get()
            
        #time.sleep(0.2)
        # if not frame_queue.empty():
        #     frame = frame_queue.get()
        #     # 循环检查每个世界标签，并求解相机位姿
        #     for map_label in map_label_info:
        #         # 标签在世界坐标系下的位姿
        #         label_world_pos = np.array(map_label["position"])
        #         label_world_rotation_mat = \
        #             euler_angles_to_rotation_matrix(np.array(map_label["rotation"]))

        #         # 求解相机在世界坐标系下的位姿

        #         cam_pos_to_label, _, _, _ = calc_pos_and_trans(frame, map_label, bounding_box)

        #     time.sleep(0.1)  # 控制游走频率

    listener_process.terminate()
    listener_process.join()
    self.fsm.trigger_event(auto_walk_finished_event)


init_state.set_process_action(init_process_action)
idle_state.set_process_action(idle_process_action)
listen_target_state.set_process_action(listen_target_process_action)
process_target_state.set_process_action(process_target_process_action)
auto_walk_state.set_process_action(auto_walk_process_action)

g_frame_queue = mtp.Queue()
g_robo_state_queue = mtp.Queue()
g_voice_command_queue = mtp.Queue()
# g_yolo_result_queue = mtp.Queue()
g_command_queue = mtp.Queue()

if __name__ == "__main__":

    config = json.load(open("config.json", "r"))
    obj_label_info = config["object_label"]
    map_label_info = config["map_label"]
    ip = config["ip"]
    video_port = config["video_port"]
    remote_control_port = config["control_port"]
    model_path = config["model_path"]

    cam_width = config["width"]
    cam_height = config["height"]

    camera_process = mtp.Process(
        target=tele_camera,
        args=(
            ip, video_port, cam_width, cam_height, g_frame_queue,
        ),
        daemon=True
    )

    auto_decider_process = mtp.Process(
        target=FSM_decider_process,
        args=(
            g_frame_queue, g_robo_state_queue, g_voice_command_queue, g_command_queue, model_path, obj_label_info, map_label_info
        ),
        daemon=False
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
    # ai_detector_process.start()

    keyboard.wait("p")

    camera_process.terminate()
    auto_decider_process.terminate()
    remote_control_process.terminate()
    # ai_detector_process.terminate()

    camera_process.join()
    auto_decider_process.join()
    remote_control_process.join()
    # ai_detector_process.join()
