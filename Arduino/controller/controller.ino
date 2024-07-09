#include <AFMotor.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Kalman.h>

#define USE_ULTRASONIC 0
#define USE_CHASSIS 1
#define USE_IMU 0

// 定义电机
AF_DCMotor Motor_LB(1); // 电机1
AF_DCMotor Motor_LF(2); // 电机2
AF_DCMotor Motor_RF(3); // 电机3
AF_DCMotor Motor_RB(4); // 电机4
MPU6050 mpu;    // MPU6050
Kalman kalmanX; // X轴的卡尔曼滤波器
Kalman kalmanY; // Y轴的卡尔曼滤波器

// 常量区
const int LED_PIN = 24;  // 红色LED引脚
const int TRIG_PIN = 40; // 超声波传感器的TRIG引脚
const int ECHO_PIN_FRONT = 18; // 超声波传感器的ECHO引脚
const int ECHO_PIN_BACK = 19; // 超声波传感器的ECHO引脚
const int ECHO_PIN_LEFT = 20; // 超声波传感器的ECHO引脚
const int ECHO_PIN_RIGHT = 21; // 超声波传感器的ECHO引脚

const long DISTANCE_THRESHOLD = 10;  // 距离阈值（厘米）
const int WHEEL_MAX_SPEED = 112.5;  //轮子最大速度
const uint8_t MPU6050_ADDR = 0x68; // MPU6050的总线ID
volatile long start_time;       // 记录脉冲开始时间
volatile long end_time_front;         // 记录脉冲结束时间
volatile long end_time_back;         // 记录脉冲结束时间
volatile long end_time_left;         // 记录脉冲结束时间
volatile long end_time_right;         // 记录脉冲结束时间
volatile bool g_is_measurement_done_front = false;  // 测距完成标志
volatile bool g_is_measurement_done_back = false;  // 测距完成标志
volatile bool g_is_measurement_done_left = false;  // 测距完成标志
volatile bool g_is_measurement_done_right = false;  // 测距完成标志
unsigned long g_imm_stop_start_time = 0;
bool g_is_imm_stop_active = false;
// 全局单例以及全局变量

// Command info 命令信息
struct CommandData {
  int8_t forward_back, left_right;  // 前后和左右移动值
  bool imm_stop_flag = false;  // 紧急停止标志
} command_data;

// Sensor data 传感器数据
struct UltrasonicData {
  long distance_right = 9999;  // 右侧距离
  long distance_left = 9999;   // 左侧距离
  long distance_front = 9999;  // 前方距离
  long distance_back = 9999;   // 后方距离
} ultrasonic_data;

// IMU数据单例
struct IMUData {
  int16_t accX, accY, accZ;
  int16_t gyroX, gyroY, gyroZ;
  double kalAngleX, kalAngleY;
} imu_data;

int last_update_time;

//移动状态单例
struct MovementState {
  int l_motor_speed = 0;  // 左电机速度
  int r_motor_speed = 0;  // 右电机速度
  double x = 0.0;   // 机器人的x坐标
  double y = 0.0;   // 机器人的y坐标
  double theta = 0.0; // 机器人的角度（弧度）
} movement_state;

// 枚举类
enum COMMAND {
  MOVE,      // 移动命令
  IMM_STOP   // 紧急停止命令
};

// 函数声明
void setup();
void loop();
void Process_Command();
void Get_Sensor_Data();
void Making_Decision();
void Execute_Command();
void Feed_Back();
void Control_Motors();
long Measure_Distance_Front();
long Measure_Distance_Back();
long Measure_Distance_Left();
long Measure_Distance_Right();
void Echo_ISR_Front();
void Echo_ISR_Back();
void Echo_ISR_Left();
void Echo_ISR_Right();
void Trigger_Ultrasonic();
void Get_IMU_Data();

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Shield initialized");

  pinMode(TRIG_PIN, OUTPUT);      // 设置TRIG引脚为输出模式
  pinMode(ECHO_PIN_FRONT, INPUT);       // 设置ECHO引脚为输入模式
  pinMode(ECHO_PIN_BACK, INPUT);       // 设置ECHO引脚为输入模式
  pinMode(ECHO_PIN_LEFT, INPUT);       // 设置ECHO引脚为输入模式
  pinMode(ECHO_PIN_RIGHT, INPUT);       // 设置ECHO引脚为输入模式
  pinMode(LED_PIN, OUTPUT);       // 设置LED引脚为输出模式



  // 初始化LED灯状态，确保开始时LED灯关闭
  digitalWrite(LED_PIN, LOW);

  Wire.begin(); // 初始化I2C
  mpu.initialize(); // 初始化MPU6050

  int try_times = 0;
  while (!mpu.testConnection() && try_times <= 15) {
    Serial.println("MPU6050 connecting ...");
    try_times++;
  }

  Serial.println("MPU6050 connection successful");

  // 初始化MPU6050
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  // MPU6050启动后需要等待一段时间
  delay(600);
  
  // 读取初始的加速度和陀螺仪数据
  mpu.getMotion6(&imu_data.accX, &imu_data.accY, &imu_data.accZ, &imu_data.gyroX, &imu_data.gyroY, &imu_data.gyroZ);

  // 初始化卡尔曼滤波器的角度
  double accAngleX = atan2((double)imu_data.accY, (double)imu_data.accZ) * RAD_TO_DEG;
  double accAngleY = atan2((double)imu_data.accX, (double)imu_data.accZ) * RAD_TO_DEG;
  kalmanX.setAngle(accAngleX);
  kalmanY.setAngle(accAngleY);

  Serial.println("Kalman filter initialized");

  last_update_time = millis(); // 初始化上一次更新的时间
  // 配置外部中断，Echo引脚的上升沿和下降沿触发中断
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_FRONT), Echo_ISR_Front, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_BACK), Echo_ISR_Back, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_LEFT), Echo_ISR_Left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_RIGHT), Echo_ISR_Right, CHANGE);
  Serial.println("Setup finished!");

}

void loop() {
  Process_Command();     // 处理命令
  Get_Sensor_Data();     // 获取传感器数据
  Making_Decision();     // 根据数据做出决策
  Execute_Command();     // 执行命令
  Feed_Back();
  
}
/*==================================== Loop Function =======================================*/
// 处理串口接收到的命令
void Process_Command() {
  //Serial.println("Process_Command start!");
  if (Serial.available() < 1) {
    command_data.forward_back = 0;
    command_data.left_right = 0;
    command_data.imm_stop_flag = false;
    //Serial.println("good");

    return;
  }
  //Serial.println("good");

  int8_t command = Serial.read();
  Serial.println("good");

  switch (command) {
    case MOVE:
      if (Serial.available() >= 2) {
        command_data.forward_back = Serial.read();
        command_data.left_right = Serial.read();
        command_data.imm_stop_flag = false;
      }
      break;
    case IMM_STOP:
      command_data.forward_back = 0;
      command_data.left_right = 0;
      command_data.imm_stop_flag = true;
      break;
    default:
      Serial.println("Unknown command received");
      break;
  }

  
  // command_data.imm_stop_flag = false;
  // command_data.forward_back = 0;
  // command_data.left_right = 0;
  //Serial.println("Process_Command start!");

}

// 获取传感器数据
void Get_Sensor_Data() {
  //Serial.println("Measuring...");
  #if USE_ULTRASONIC
  ultrasonic_data.distance_front = Measure_Distance_Front();  // 测量前方距离
  ultrasonic_data.distance_back = Measure_Distance_Back();  // 测量前方距离
  ultrasonic_data.distance_left = Measure_Distance_Left();  // 测量前方距离
  ultrasonic_data.distance_right = Measure_Distance_Right();  // 测量前方距离
  #endif
  #if USE_IMU
  Get_IMU_Data();
  #endif
}

// 根据传感器数据做出决策
void Making_Decision() {
  if (ultrasonic_data.distance_front < DISTANCE_THRESHOLD) {
    command_data.imm_stop_flag = true;  // 距离小于阈值时设置紧急停止标志
  }
}

// 执行命令，控制电机
void Execute_Command() {
  Control_Motors();  // 控制电机
}

// 创建反馈信息
void Feed_Back() {
  //Serial.println("Feed_Back start!");
  // 创建JSON对象
  StaticJsonDocument<512> doc;

  // 添加运动状态数据
  doc["movement_state"]["left_wheel_speed"] = movement_state.l_motor_speed;
  doc["movement_state"]["right_wheel_speed"] = movement_state.r_motor_speed;
  doc["movement_state"]["x"] = movement_state.x;
  doc["movement_state"]["y"] = movement_state.y;
  doc["movement_state"]["theta"] = movement_state.theta;

  // 添加命令数据
  doc["command_data"]["forward_back"] = command_data.forward_back;
  doc["command_data"]["left_right"] = command_data.left_right;
  doc["command_data"]["imm_stop_flag"] = command_data.imm_stop_flag;

  // 添加传感器数据
  doc["ultrasonic_data"]["distance_front"] = ultrasonic_data.distance_front;
  doc["ultrasonic_data"]["distance_right"] = ultrasonic_data.distance_right;
  doc["ultrasonic_data"]["distance_left"] = ultrasonic_data.distance_left;
  doc["ultrasonic_data"]["distance_back"] = ultrasonic_data.distance_back;

  // 序列化JSON对象到字符串
  char json_output[512];
  serializeJson(doc, json_output);

  // 打印JSON字符串到串口
  Serial.println(json_output);
}

/*==================================== Motor Control Part =======================================*/
// 控制电机速度和方向
void Control_Motors() {
  if (command_data.imm_stop_flag && !g_is_imm_stop_active) {
    // 如果紧急停止标志为真并且没有激活紧急停止，立即反转马达并记录开始时间
    Motor_LB.setSpeed(abs(movement_state.l_motor_speed));
    Motor_LF.setSpeed(abs(movement_state.l_motor_speed));
    Motor_RB.setSpeed(abs(movement_state.r_motor_speed));
    Motor_RF.setSpeed(abs(movement_state.r_motor_speed));

    // 反转当前马达的方向
    if (movement_state.l_motor_speed > 0) {
      Motor_LB.run(BACKWARD);
      Motor_LF.run(BACKWARD);
    } else {
      Motor_LB.run(FORWARD);
      Motor_LF.run(FORWARD);
    }

    if (movement_state.r_motor_speed < 0) {
      Motor_RB.run(BACKWARD);
      Motor_RF.run(BACKWARD);
    } else {
      Motor_RB.run(FORWARD);
      Motor_RF.run(FORWARD);
    }

    // 设置反转后的速度
    movement_state.l_motor_speed = -movement_state.l_motor_speed;
    movement_state.r_motor_speed = -movement_state.r_motor_speed;

    g_imm_stop_start_time = millis();
    g_is_imm_stop_active = true;

    return;
  }

  if (g_is_imm_stop_active) {
    // 检查是否已经后退了半秒
    if (millis() - g_imm_stop_start_time >= 500) {
      // 停止所有马达
      Motor_LB.setSpeed(0);
      Motor_LF.setSpeed(0);
      Motor_RB.setSpeed(0);
      Motor_RF.setSpeed(0);

      Motor_LB.run(RELEASE);
      Motor_LF.run(RELEASE);
      Motor_RB.run(RELEASE);
      Motor_RF.run(RELEASE);

      // 清空紧急停止标志
      command_data.imm_stop_flag = false;
      g_is_imm_stop_active = false;

      movement_state.l_motor_speed = 0;
      movement_state.r_motor_speed = 0;
    }
    return;
  }

  if (command_data.left_right == 0) {
    movement_state.l_motor_speed = WHEEL_MAX_SPEED * (command_data.forward_back * 1.2);
    movement_state.r_motor_speed = WHEEL_MAX_SPEED * (command_data.forward_back * 1.2);
  }
  else if (command_data.forward_back == 0) {
    movement_state.l_motor_speed = WHEEL_MAX_SPEED * (-command_data.left_right*1.1);
    movement_state.r_motor_speed = WHEEL_MAX_SPEED * (command_data.left_right*1.1);
  }
  else {
    movement_state.l_motor_speed = WHEEL_MAX_SPEED * (command_data.forward_back - command_data.left_right);
    movement_state.r_motor_speed = WHEEL_MAX_SPEED * (command_data.forward_back + command_data.left_right);
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

/*==================================== Ultrasonic Part =======================================*/


// 测距，若距离小于10cm则立刻停车
long Measure_Distance_Front() {
  //Serial.println("Measure_Distance_Front");
  // 如果测距完成
  if (g_is_measurement_done_front) {
    // 计算脉冲持续时间
    long duration = end_time_front - start_time;
    // 计算距离，单位为厘米
    long distance = duration * 0.034 / 2;

    // 打印距离到串口
    //Serial.print("Distance: ");
    //Serial.print(distance);
    //Serial.println(" cm");
  
    // 重置测距完成标志
    g_is_measurement_done_front = false;
    return distance;
  }
  // 触发一次超声波测距
  Trigger_Ultrasonic();
  delay(100); // 每秒测量10次
  return 9999; // 如果测距未完成，返回一个大值
}

// 测距，若距离小于10cm则立刻停车
long Measure_Distance_Back() {
  // 如果测距完成
  if (g_is_measurement_done_back) {
    // 计算脉冲持续时间
    long duration = end_time_back - start_time;
    // 计算距离，单位为厘米
    long distance = duration * 0.034 / 2;

    // 打印距离到串口
    //Serial.print("Distance: ");
    //Serial.print(distance);
    //Serial.println(" cm");
  
    // 重置测距完成标志
    g_is_measurement_done_back = false;
    return distance;
  }
  // 触发一次超声波测距
  Trigger_Ultrasonic();
  delay(100); // 每秒测量10次
  return 9999; // 如果测距未完成，返回一个大值
}

// 测距，若距离小于10cm则立刻停车
long Measure_Distance_Left() {
  // 如果测距完成
  if (g_is_measurement_done_left) {
    // 计算脉冲持续时间
    long duration = end_time_left - start_time;
    // 计算距离，单位为厘米
    long distance = duration * 0.034 / 2;

    // 打印距离到串口
    //Serial.print("Distance: ");
    //Serial.print(distance);
    //Serial.println(" cm");
  
    // 重置测距完成标志
    g_is_measurement_done_left = false;
    return distance;
  }
  // 触发一次超声波测距
  Trigger_Ultrasonic();
  delay(100); // 每秒测量10次
  return 9999; // 如果测距未完成，返回一个大值
}

// 测距，若距离小于10cm则立刻停车
long Measure_Distance_Right() {
  
  // 如果测距完成
  if (g_is_measurement_done_right) {
    // 计算脉冲持续时间
    long duration = end_time_right - start_time;
    // 计算距离，单位为厘米
    long distance = duration * 0.034 / 2;

    // 打印距离到串口
    //Serial.print("Distance: ");
    //Serial.print(distance);
    //Serial.println(" cm");
  
    // 重置测距完成标志
    g_is_measurement_done_right = false;
    return distance;
  }
  // 触发一次超声波测距
  Trigger_Ultrasonic();
  delay(100); // 每秒测量10次
  return 9999; // 如果测距未完成，返回一个大值
}

// 中断服务程序，处理Echo引脚的上升沿和下降沿
void Echo_ISR_Front() {
  if (digitalRead(ECHO_PIN_FRONT) == HIGH) {
    // Echo引脚上升沿，中断开始计时
    start_time = micros();       // 记录脉冲开始时间
  } else {
    // Echo引脚下降沿，中断结束计时
    end_time_front = micros();         // 记录脉冲结束时间
    g_is_measurement_done_front = true;   // 设置测距完成标志
  }
}

// 中断服务程序，处理Echo引脚的上升沿和下降沿
void Echo_ISR_Back() {
  if (digitalRead(ECHO_PIN_BACK) == HIGH) {
    // Echo引脚上升沿，中断开始计时
    start_time = micros();       // 记录脉冲开始时间
  } else {
    // Echo引脚下降沿，中断结束计时
    end_time_back = micros();         // 记录脉冲结束时间
    g_is_measurement_done_back = true;   // 设置测距完成标志
  }
}

// 中断服务程序，处理Echo引脚的上升沿和下降沿
void Echo_ISR_Left() {
  if (digitalRead(ECHO_PIN_LEFT) == HIGH) {
    // Echo引脚上升沿，中断开始计时
    start_time = micros();       // 记录脉冲开始时间
  } else {
    // Echo引脚下降沿，中断结束计时
    end_time_left = micros();         // 记录脉冲结束时间
    g_is_measurement_done_left = true;   // 设置测距完成标志
  }
}

// 中断服务程序，处理Echo引脚的上升沿和下降沿
void Echo_ISR_Right() {
  if (digitalRead(ECHO_PIN_RIGHT) == HIGH) {
    // Echo引脚上升沿，中断开始计时
    start_time = micros();       // 记录脉冲开始时间
  } else {
    // Echo引脚下降沿，中断结束计时
    end_time_right = micros();         // 记录脉冲结束时间
    g_is_measurement_done_right = true;   // 设置测距完成标志
  }
}

// 发送超声波脉冲
void Trigger_Ultrasonic() {
  digitalWrite(TRIG_PIN, LOW);    // 确保Trig引脚为低电平
  delayMicroseconds(2);          // 等待2微秒
  digitalWrite(TRIG_PIN, HIGH);   // 发送高电平脉冲，持续10微秒
  delayMicroseconds(10);         // 等待10微秒
  digitalWrite(TRIG_PIN, LOW);    // 结束脉冲，恢复低电平
  g_is_measurement_done_front = false;
  g_is_measurement_done_back = false;
  g_is_measurement_done_left = false;
  g_is_measurement_done_right = false;
}

/*==================================== IMU Part =======================================*/
void Get_IMU_Data() {
  // 获取当前时间
  unsigned long current_time = millis();
  double dt = (current_time - last_update_time) / 1000.0; // 时间间隔（秒）

  // 更新上一次更新时间
  last_update_time = current_time;

  // 获取MPU6050的加速度和陀螺仪数据
  mpu.getMotion6(&imu_data.accX, &imu_data.accY, &imu_data.accZ, &imu_data.gyroX, &imu_data.gyroY, &imu_data.gyroZ);

  // 计算加速度的角度
  double accAngleX = atan2((double)imu_data.accY, (double)imu_data.accZ) * RAD_TO_DEG;
  double accAngleY = atan2((double)imu_data.accX, (double)imu_data.accZ) * RAD_TO_DEG;

  // 陀螺仪数据需要转换为角速度，单位从 LSB/s 转换为 °/s
  double gyroXrate = (double)imu_data.gyroX / 131.0;
  double gyroYrate = (double)imu_data.gyroY / 131.0;

  // 卡尔曼滤波
  imu_data.kalAngleX = kalmanX.getAngle(accAngleX, gyroXrate, dt);
  imu_data.kalAngleY = kalmanY.getAngle(accAngleY, gyroYrate, dt);

  // 使用积分法计算位姿
  double velocity = (movement_state.l_motor_speed + movement_state.r_motor_speed) / 2.0; // 平均速度
  movement_state.theta += imu_data.kalAngleY * dt; // 更新角度
  movement_state.x += velocity * cos(movement_state.theta) * dt; // 更新x坐标
  movement_state.y += velocity * sin(movement_state.theta) * dt; // 更新y坐标
}
