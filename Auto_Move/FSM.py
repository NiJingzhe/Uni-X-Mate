class State:
    def __init__(self, name):
        self.name = name
        self.transitions = {}
        self.enter_action = None
        self.process_action = None
        self.exit_action = None
        self.fsm = None

    def add_transition(self, input_event, target_state):
        self.transitions[input_event] = target_state

    def set_enter_action(self, action):
        self.enter_action = action

    def set_process_action(self, action):
        self.process_action = action

    def set_exit_action(self, action):
        self.exit_action = action

    def on_enter(self):
        if self.enter_action:
            self.enter_action(self)

    def on_process(self):
        if self.process_action:
            self.process_action(self)

    def on_exit(self):
        if self.exit_action:
            self.exit_action(self)
            
class Event:
    
    def __init__(self, name):
        self.name = name
        
        
class FSM:
    def __init__(self):
        self.states = {}
        self.current_state = None
        self.events = {}
        # 添加Blackboard用于状态之间的数据共享
        self.blackboard = {}

    def add_state(self, state):
        if isinstance(state, list):
            for s in state:
                self.states[s.name] = s
                s.fsm = self  # 设置State的FSM引用
        else:
            self.states[state.name] = state
            state.fsm = self  # 设置State的FSM引用

    def add_event(self, event):
        if isinstance(event, list):
            for e in event:
                self.events[e.name] = e
        else:
            self.events[event.name] = event

    def set_start_state(self, state_name):
        self.current_state = self.states.get(state_name)
        if self.current_state:
            self.current_state.on_enter()
            #self.current_state.on_process()

    def trigger_event(self, event_name):
        if self.current_state and (event_name in self.current_state.transitions):
            next_state = self.current_state.transitions[event_name]
            self.current_state.on_exit()
            self.current_state = next_state
            self.current_state.on_enter()
            #self.current_state.on_process()
            
    def add_blackboard_data(self, key, value):
        self.blackboard[key] = value
    
    def get_blackboard_data(self, key): 
        return self.blackboard.get(key)        
    
    def process(self):
        if self.current_state:
            self.current_state.on_process()


    