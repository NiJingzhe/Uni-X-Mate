#include <AFMotor.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6050.h>
#include <String.h>
#include <Kalman.h>

#define USE_ULTRASONIC 1
#define USE_CHASSIS 1
#define USE_IMU 1

// 定义电机
AF_DCMotor Motor_LB(1); // 电机1
AF_DCMotor Motor_LF(2); // 电机2
AF_DCMotor Motor_RF(3); // 电机3
AF_DCMotor Motor_RB(4); // 电机4

// 常量区
static unsigned long last_trigger_time_front = 0;
static unsigned long last_trigger_time_back = 0;
static unsigned long last_trigger_time_left = 0;
static unsigned long last_trigger_time_right = 0;

const int TRIG_PIN_FRONT = 40; // 前方超声波传感器的TRIG引脚
const int TRIG_PIN_BACK = 41;  // 后方超声波传感器的TRIG引脚
const int TRIG_PIN_LEFT = 42;  // 左侧超声波传感器的TRIG引脚
const int TRIG_PIN_RIGHT = 43; // 右侧超声波传感器的TRIG引脚
const int ECHO_PIN_FRONT = 18; // 超声波传感器的ECHO引脚
const int ECHO_PIN_BACK = 19;  // 超声波传感器的ECHO引脚
const int ECHO_PIN_LEFT = 20;  // 超声波传感器的ECHO引脚
const int ECHO_PIN_RIGHT = 21; // 超声波传感器的ECHO引脚
const int threshold_distance = 20; // 超过20cm阈值停车

const int WHEEL_MAX_SPEED = 112.5;  // 轮子最大速度

volatile long start_time_front;       // 记录前方脉冲开始时间
volatile long start_time_back;        // 记录后方脉冲开始时间
volatile long start_time_left;        // 记录左侧脉冲开始时间
volatile long start_time_right;       // 记录右侧脉冲开始时间
volatile long end_time_front;         // 记录前方脉冲结束时间
volatile long end_time_back;          // 记录后方脉冲结束时间
volatile long end_time_left;          // 记录左侧脉冲结束时间
volatile long end_time_right;         // 记录右侧脉冲结束时间
volatile bool g_is_measurement_done_front = false;  // 测距完成标志
volatile bool g_is_measurement_done_back = false;   // 测距完成标志
volatile bool g_is_measurement_done_left = false;   // 测距完成标志
volatile bool g_is_measurement_done_right = false;  // 测距完成标志

bool g_is_imm_stop_active_front = false;
bool g_is_imm_stop_active_back = false;
bool g_is_imm_stop_active_left = false;
bool g_is_imm_stop_active_right = false;
bool g_obstacle_avoidance_mode_is_on = false;
bool g_is_ready = false; // 自检完成标志
String g_order = ""; // 指令
bool g_is_order_correct = true; // 命令正确标志

float theta = 0; // 在小车惯性坐标系下，小车与目标物的偏差角度，精度默认为小数点后两位
float threshold_theta_mistake = 1.00; // 偏差角最大接受度

// 全局单例以及全局变量

// Command info 命令信息
struct CommandData {
  int8_t forward_back, left_right;  // 前后和左右移动值，注意右转是-1
  float distance; // 距离
  float angle; // 角度
} command_data, next_step_command_data;

// Sensor data 传感器数据
struct UltrasonicData {
  long distance_right = 9999;  // 右侧距离
  long distance_left = 9999;   // 左侧距离
  long distance_front = 9999;  // 前方距离
  long distance_back = 9999;   // 后方距离
} ultrasonic_data;

// 四面障碍物信息
struct Obstacle {
  bool there_is_obstacle_ahead = false;
  bool there_is_obstacle_behind = false;
  bool there_is_obstacle_on_the_left = false;
  bool there_is_obstacle_on_the_right = false;
} obstacle;

int last_update_time;

// 移动状态单例
struct MovementState {
  int l_motor_speed = 0;  // 左电机速度
  int r_motor_speed = 0;  // 右电机速度
  double x = 0.0;   // 机器人的x坐标
  double y = 0.0;   // 机器人的y坐标
  double theta = 0.0; // 机器人的角度（弧度）
} movement_state;

// 枚举类
enum COMMAND {
  GOTO_TARGET = 0, // 前往目标
  FIND_TAG = 1,    // 寻找标签
  GRAB = 2,        // 抓取物体
  SELF_CHECK = 3   // 自检
};

// 函数声明
// setup & loop
void setup();
void loop();

// Step1: Process_Command
void Process_Command();

// Step2: Get_Sensor_Data
void Get_Sensor_Data();
void Get_Ultrasonic_Sensor_Data();
long Get_Ultrasonic_Sensor_Data_Front();
long Get_Ultrasonic_Sensor_Data_Back();
long Get_Ultrasonic_Sensor_Data_Left();
long Get_Ultrasonic_Sensor_Data_Right();
void Echo_ISR_Front();
void Echo_ISR_Back();
void Echo_ISR_Left();
void Echo_ISR_Right();
void Trigger_Ultrasonic(int trigPin, unsigned long &last_trigger_time);

// Step3: Making_Decision
void Making_Decision();
bool whether_direction_is_correct();
void correct_the_direction();
void escape_corner();

// Step4: Execute_Command
void Execute_Command();
void Control_Motors();
void Search_Tag();
void Grab_Object();

// Step5: Feed_Back
void Feed_Back();

// 自检函数
void Self_Check();

void setup() {
  Serial.begin(115200);
  //Serial.println("Motor Shield initialized");

  pinMode(TRIG_PIN_FRONT, OUTPUT); // 设置TRIG引脚为输出模式
  pinMode(TRIG_PIN_BACK, OUTPUT);  // 设置TRIG引脚为输出模式
  pinMode(TRIG_PIN_LEFT, OUTPUT);  // 设置TRIG引脚为输出模式
  pinMode(TRIG_PIN_RIGHT, OUTPUT); // 设置TRIG引脚为输出模式

  pinMode(ECHO_PIN_FRONT, INPUT); // 设置ECHO引脚为输入模式
  pinMode(ECHO_PIN_BACK, INPUT);  // 设置ECHO引脚为输入模式
  pinMode(ECHO_PIN_LEFT, INPUT);  // 设置ECHO引脚为输入模式
  pinMode(ECHO_PIN_RIGHT, INPUT); // 设置ECHO引脚为输入模式

  // 配置外部中断，Echo引脚的上升沿和下降沿触发中断
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_FRONT), Echo_ISR_Front, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_BACK), Echo_ISR_Back, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_LEFT), Echo_ISR_Left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_RIGHT), Echo_ISR_Right, CHANGE);

  Self_Check();
}

void loop() {

  Process_Command(); // 处理命令
  Get_Sensor_Data();     // 获取传感器数据
  Making_Decision();     // 根据数据做出决策
  Execute_Command();     // 执行命令
  //Feed_Back();           // 反馈
}

// 自检函数
void Self_Check() {
  // 检查四个超声波传感器是否正常工作
  bool all_sensors_working = true;
  long front_distance, back_distance, left_distance, right_distance;

  // 测试前方超声波传感器
  Trigger_Ultrasonic(TRIG_PIN_FRONT, last_trigger_time_front);
  delay(60);
  front_distance = Get_Ultrasonic_Sensor_Data_Front();
  if (front_distance >= 9999) {
    all_sensors_working = false;
    //Serial.println("Front ultrasonic sensor failed");
  }

  // 测试后方超声波传感器
  Trigger_Ultrasonic(TRIG_PIN_BACK, last_trigger_time_back);
  delay(60);
  back_distance = Get_Ultrasonic_Sensor_Data_Back();
  if (back_distance >= 9999) {
    all_sensors_working = false;
    //Serial.println("Back ultrasonic sensor failed");
  }

  // 测试左侧超声波传感器
  Trigger_Ultrasonic(TRIG_PIN_LEFT, last_trigger_time_left);
  delay(60);
  left_distance = Get_Ultrasonic_Sensor_Data_Left();
  if (left_distance >= 9999) {
    all_sensors_working = false;
    //Serial.println("Left ultrasonic sensor failed");
  }

  // 测试右侧超声波传感器
  Trigger_Ultrasonic(TRIG_PIN_RIGHT, last_trigger_time_right);
  delay(60);
  right_distance = Get_Ultrasonic_Sensor_Data_Right();
  if (right_distance >= 9999) {
    all_sensors_working = false;
    //Serial.println("Right ultrasonic sensor failed");
  }

  // 返回传感器状态信息给上位机树莓派
  // StaticJsonDocument<256> doc;
  // doc["front_sensor"] = front_distance < 9999 ? "working" : "failed";
  // doc["back_sensor"] = back_distance < 9999 ? "working" : "failed";
  // doc["left_sensor"] = left_distance < 9999 ? "working" : "failed";
  // doc["right_sensor"] = right_distance < 9999 ? "working" : "failed";
  // doc["overall_status"] = all_sensors_working ? "all sensors working" : "one or more sensors failed";

  // char json_output[256];
  // serializeJson(doc, json_output);
  // Serial.println(json_output);

  if (all_sensors_working) {
    g_is_ready = 1;
  } else {
    g_is_ready = 0;
  }
}

// 处理串口接收到的命令
void Process_Command() {
  if (Serial.available()) {
    String json_command = Serial.readStringUntil('\n'); // 从串口读取JSON格式的字符串
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, json_command);

    if (error) {
      g_order = "INVALID JSON!";
      g_is_order_correct = false;
      return;
    }

    int cmd = doc["command"];

    switch (cmd) {
      case GOTO_TARGET:
        command_data.distance = doc["distance"];
        command_data.angle = doc["angle"];
        next_step_command_data = command_data;
        g_order = cmd;
        g_is_order_correct = true;
        Feed_Back();
        break;
      case FIND_TAG:
        g_order = cmd;
        g_is_order_correct = true;
        Feed_Back();
        Search_Tag();
        break;
      case GRAB:
        g_order = cmd;
        g_is_order_correct = true;
        Feed_Back();
        Grab_Object();
        break;
      case SELF_CHECK:
        Self_Check();
        g_order = cmd;
        g_is_order_correct = true;
        Feed_Back();
        break;
      default:
        g_order = "UNKNOWN COMMAND!";
        g_is_order_correct = false;
    }
  }
}

// 获取传感器数据
void Get_Sensor_Data() {
  #if USE_ULTRASONIC
  Get_Ultrasonic_Sensor_Data();
  #endif
}

void Making_Decision() {
  // 更新障碍物信息
  obstacle.there_is_obstacle_ahead = ultrasonic_data.distance_front < threshold_distance;
  obstacle.there_is_obstacle_behind = ultrasonic_data.distance_back < threshold_distance;
  obstacle.there_is_obstacle_on_the_left = ultrasonic_data.distance_left < threshold_distance;
  obstacle.there_is_obstacle_on_the_right = ultrasonic_data.distance_right < threshold_distance;

  // 判断是否需要避障
  g_obstacle_avoidance_mode_is_on = false;
  if ((next_step_command_data.forward_back == 1 && obstacle.there_is_obstacle_ahead) ||
      (next_step_command_data.forward_back == -1 && obstacle.there_is_obstacle_behind) ||
      (next_step_command_data.left_right == -1 && obstacle.there_is_obstacle_on_the_right) ||
      (next_step_command_data.left_right == 1 && obstacle.there_is_obstacle_on_the_left)) {
    g_obstacle_avoidance_mode_is_on = true;
  }

  // 非避障模式，正常运动
  if (!g_obstacle_avoidance_mode_is_on) {
    if (whether_direction_is_correct()) {
      command_data.forward_back = 0;
      command_data.left_right = 0;
    } else {
      correct_the_direction();
    }
  } else {
    // 避障模式
    if (next_step_command_data.forward_back == 1 && obstacle.there_is_obstacle_ahead) {
      if (!obstacle.there_is_obstacle_on_the_left) {
        command_data.forward_back = 0;
        command_data.left_right = 1; // 左转
      } else if (!obstacle.there_is_obstacle_on_the_right) {
        command_data.forward_back = 0;
        command_data.left_right = -1; // 右转
      } else {
        command_data.forward_back = -1; // 后退
        command_data.left_right = 0;
      }
    } else if (next_step_command_data.forward_back == -1 && obstacle.there_is_obstacle_behind) {
      if (!obstacle.there_is_obstacle_on_the_left) {
        command_data.forward_back = 0;
        command_data.left_right = 1; // 左转
      } else if (!obstacle.there_is_obstacle_on_the_right) {
        command_data.forward_back = 0;
        command_data.left_right = -1; // 右转
      } else {
        command_data.forward_back = 1; // 前进
        command_data.left_right = 0;
      }
    } else if (next_step_command_data.left_right == -1 && obstacle.there_is_obstacle_on_the_right) {
      if (!obstacle.there_is_obstacle_ahead) {
        command_data.forward_back = 1;
        command_data.left_right = 0;
      } else if (!obstacle.there_is_obstacle_on_the_left) {
        command_data.forward_back = 0;
        command_data.left_right = 1;
      } else {
        command_data.forward_back = -1;
        command_data.left_right = 0;
      }
    } else if (next_step_command_data.left_right == 1 && obstacle.there_is_obstacle_on_the_left) {
      if (!obstacle.there_is_obstacle_ahead) {
        command_data.forward_back = 1;
        command_data.left_right = 0;
      } else if (!obstacle.there_is_obstacle_on_the_right) {
        command_data.forward_back = 0;
        command_data.left_right = -1;
      } else {
        command_data.forward_back = -1;
        command_data.left_right = 0;
      }
    } else {
      escape_corner(); // 尝试脱困
    }
  }

  // 更新下一步命令数据
  if (!g_obstacle_avoidance_mode_is_on) {
    next_step_command_data.forward_back = command_data.forward_back;
    next_step_command_data.left_right = command_data.left_right;
  }
}

// 执行命令，控制电机
void Execute_Command() {
  Control_Motors();  // 控制电机
}

// 创建反馈信息
void Feed_Back() {
  // 创建JSON对象
  StaticJsonDocument<512> doc;

  // 添加命令数据
  if (g_is_order_correct) {
    doc["command"] = g_order;
    doc["distance"] = command_data.distance;
    doc["angle"] = command_data.angle;
  } else {
    doc["command"] = "UNKNOWN OR INVALID COMMAND";
    doc["distance"] = 0;
    doc["angle"] = 0;
  }
    // 添加传感器数据
  doc["ultrasonic_data"]["distance_front"] = ultrasonic_data.distance_front;
  doc["ultrasonic_data"]["distance_right"] = ultrasonic_data.distance_right;
  doc["ultrasonic_data"]["distance_left"] = ultrasonic_data.distance_left;
  doc["ultrasonic_data"]["distance_back"] = ultrasonic_data.distance_back;

  // 添加机器人状态信息
  doc["robot_state"] = g_is_ready ? 1 : 0;

  // 序列化JSON对象到字符串
  char json_output[512];
  serializeJson(doc, json_output);

  // 打印JSON字符串到串口，并添加换行符
  Serial.print(json_output);
  Serial.print("\r\n");
}

// 判断前进方向是否正确
bool whether_direction_is_correct() {
  return abs(theta) <= threshold_theta_mistake;
}

// 自动纠偏
void correct_the_direction() {
  if (theta > threshold_theta_mistake) {
    if (theta > 0) {
      command_data.forward_back = 0;
      command_data.left_right = 1;
    } else {
      command_data.forward_back = 0;
      command_data.left_right = -1;
    }
  }
}

// 控制电机速度和方向
void Control_Motors() {
  if (command_data.left_right == 0) {
    movement_state.l_motor_speed = WHEEL_MAX_SPEED * (command_data.forward_back * 1);
    movement_state.r_motor_speed = WHEEL_MAX_SPEED * (command_data.forward_back * 1);
  } else if (command_data.forward_back == 0) {
    movement_state.l_motor_speed = WHEEL_MAX_SPEED * (command_data.left_right * 1.6);
    movement_state.r_motor_speed = WHEEL_MAX_SPEED * (-command_data.left_right * 1.6);
  } else {
    movement_state.l_motor_speed = WHEEL_MAX_SPEED * (command_data.forward_back + command_data.left_right);
    movement_state.r_motor_speed = WHEEL_MAX_SPEED * (command_data.forward_back - command_data.left_right);
  }

  Motor_LB.setSpeed(abs(movement_state.l_motor_speed));
  Motor_LF.setSpeed(abs(movement_state.l_motor_speed));
  Motor_RB.setSpeed(abs(movement_state.r_motor_speed));
  Motor_RF.setSpeed(abs(movement_state.r_motor_speed));

  if (movement_state.l_motor_speed < 0) {
    Motor_LB.run(BACKWARD);
    Motor_LF.run(BACKWARD);
  } else {
    Motor_LB.run(FORWARD);
    Motor_LF.run(FORWARD);
  }

  if (movement_state.r_motor_speed > 0) {
    Motor_RB.run(BACKWARD);
    Motor_RF.run(BACKWARD);
  } else {
    Motor_RB.run(FORWARD);
    Motor_RF.run(FORWARD);
  }
}

// 获取超声波传感器数据
void Get_Ultrasonic_Sensor_Data() {
  Trigger_Ultrasonic(TRIG_PIN_FRONT, last_trigger_time_front);
  delay(50);
  ultrasonic_data.distance_front = Get_Ultrasonic_Sensor_Data_Front();

  Trigger_Ultrasonic(TRIG_PIN_BACK, last_trigger_time_back);
  delay(50);
  ultrasonic_data.distance_back = Get_Ultrasonic_Sensor_Data_Back();

  Trigger_Ultrasonic(TRIG_PIN_LEFT, last_trigger_time_left);
  delay(50);
  ultrasonic_data.distance_left = Get_Ultrasonic_Sensor_Data_Left();

  Trigger_Ultrasonic(TRIG_PIN_RIGHT, last_trigger_time_right);
  delay(50);
  ultrasonic_data.distance_right = Get_Ultrasonic_Sensor_Data_Right();
}

// 获取前置超声波传感器距离数据
long Get_Ultrasonic_Sensor_Data_Front() {
  if (!g_is_measurement_done_front) {
    g_is_measurement_done_front = true;
    unsigned long duration = end_time_front - start_time_front;
    return (duration * 0.0343) / 2;
  }
  return 9999;  // 未测量到距离时返回最大值
}

// 获取后置超声波传感器距离数据
long Get_Ultrasonic_Sensor_Data_Back() {
  if (!g_is_measurement_done_back) {
    g_is_measurement_done_back = true;
    unsigned long duration = end_time_back - start_time_back;
    return (duration * 0.0343) / 2;
  }
  return 9999;  // 未测量到距离时返回最大值
}

// 获取左侧超声波传感器距离数据
long Get_Ultrasonic_Sensor_Data_Left() {
  if (!g_is_measurement_done_left) {
    g_is_measurement_done_left = true;
    unsigned long duration = end_time_left - start_time_left;
    return (duration * 0.0343) / 2;
  }
  return 9999;  // 未测量到距离时返回最大值
}

// 获取右侧超声波传感器距离数据
long Get_Ultrasonic_Sensor_Data_Right() {
  if (!g_is_measurement_done_right) {
    g_is_measurement_done_right = true;
    unsigned long duration = end_time_right - start_time_right;
    return (duration * 0.0343) / 2;
  }
  return 9999;  // 未测量到距离时返回最大值
}

// 中断服务程序，处理Echo引脚的上升沿和下降沿
void Echo_ISR_Front() {
  if (digitalRead(ECHO_PIN_FRONT) == HIGH) {
    start_time_front = micros();  // 记录脉冲开始时间
  } else {
    end_time_front = micros();  // 记录脉冲结束时间
    g_is_measurement_done_front = false;  // 设置测距完成标志
  }
}

// 中断服务程序，处理Echo引脚的上升沿和下降沿
void Echo_ISR_Back() {
  if (digitalRead(ECHO_PIN_BACK) == HIGH) {
    start_time_back = micros();  // 记录脉冲开始时间
  } else {
    end_time_back = micros();  // 记录脉冲结束时间
    g_is_measurement_done_back = false;  // 设置测距完成标志
  }
}

// 中断服务程序，处理Echo引脚的上升沿和下降沿
void Echo_ISR_Left() {
  if (digitalRead(ECHO_PIN_LEFT) == HIGH) {
    start_time_left = micros();  // 记录脉冲开始时间
  } else {
    end_time_left = micros();  // 记录脉冲结束时间
    g_is_measurement_done_left = false;  // 设置测距完成标志
  }
}

// 中断服务程序，处理Echo引脚的上升沿和下降沿
void Echo_ISR_Right() {
  if (digitalRead(ECHO_PIN_RIGHT) == HIGH) {
    start_time_right = micros();  // 记录脉冲开始时间
  } else {
    end_time_right = micros();  // 记录脉冲结束时间
    g_is_measurement_done_right = false;  // 设置测距完成标志
  }
}

// 发送超声波脉冲
void Trigger_Ultrasonic(int trigPin, unsigned long &last_trigger_time) {
  if (millis() - last_trigger_time >= 50) {
    digitalWrite(trigPin, LOW);  // 确保Trig引脚为低电平
    delayMicroseconds(2);         // 等待2微秒
    digitalWrite(trigPin, HIGH); // 发送高电平脉冲，持续10微秒
    delayMicroseconds(10);        // 等待10微秒
    digitalWrite(trigPin, LOW);  // 结束脉冲，恢复低电平
    last_trigger_time = millis();
  }
}

// 搜索标签
void Search_Tag() {
  // 添加慢速旋转的代码
  Motor_LB.setSpeed(100);
  Motor_LF.setSpeed(100);
  Motor_RB.setSpeed(100);
  Motor_RF.setSpeed(100);

  Motor_LB.run(FORWARD);
  Motor_LF.run(FORWARD);
  Motor_RB.run(BACKWARD);
  Motor_RF.run(BACKWARD);

  delay(1000); // 停止旋转1秒
  Motor_LB.run(RELEASE);
  Motor_LF.run(RELEASE);
  Motor_RB.run(RELEASE);
  Motor_RF.run(RELEASE);
}

// 抓取物体
void Grab_Object() {
  // 添加抓取物体的代码
  //Serial.println("Grabbing object...");
}

// 脱困逻辑
void escape_corner() {
  // 如果前方和两侧都有障碍物，则倒车
  if (obstacle.there_is_obstacle_ahead &&
      obstacle.there_is_obstacle_on_the_left &&
      obstacle.there_is_obstacle_on_the_right) {
    command_data.forward_back = -1;
    command_data.left_right = 0;
    delay(500); // 倒车0.5秒
  }

  // 如果后方和两侧都有障碍物，则前进
  if (obstacle.there_is_obstacle_behind &&
      obstacle.there_is_obstacle_on_the_left &&
      obstacle.there_is_obstacle_on_the_right) {
    command_data.forward_back = 1;
    command_data.left_right = 0;
    delay(500); // 前进0.5秒
  }

  // 更新下一步命令数据
  next_step_command_data.forward_back = command_data.forward_back;
  next_step_command_data.left_right = command_data.left_right;
}
