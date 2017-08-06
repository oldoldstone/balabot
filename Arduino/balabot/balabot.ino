/*****************************************************************************
   两轮平衡车arduino源程序
   作者：oldoldstone
   版本：1.0
   日期: 2017.7.25
   修改记录：
*****************************************************************************/
#include <MsTimer2.h>       //定时中断
#include "MPU6050.h"
#define STOP    0
#define FORWARD 1
#define BACK    2
#define RIGHT   3
#define LEFT    4
//电机控制线以及PWM接口
const int kMotorPinLA = 8,  kMotorPinLB = 7;
const int kMotorPinRA = 11, kMotorPinRB = 12;
const int kPwmPinL = 9, kPwmPinR = 10;
//左右编码器接口
const int kEncPinLA = 2, kEncPinLB = 4;
const int kEncPinRA = 3, kEncPinRB = 5;
//全局变量
MPU6050 accelgyro;
int16_t g_ax, g_ay, g_az;  //陀螺仪测量的加速度
int16_t g_gx, g_gy, g_gz;  //角速度
volatile long g_enc_left, g_enc_right = 0;   //左右轮编码器数据
int g_speed_left, g_speed_right = 0;     //左右轮速度
int g_upright_pwm, g_velocity_pwm, g_turn_pwm;
unsigned char g_cmd;
//小车状态
float g_battery_voltage;   //电池电压
float g_banlance_angle = 0; //小车平衡时的倾角
unsigned char g_stop_flag = 1;
float g_angle = 0.0f; //滤波器根据陀螺仪得到的小车倾角
//PID参数
float g_upright_kp = 15, g_upright_kd = 0.8; //直立PD参数
float g_velocity_kp = 1, g_velocity_ki = 1.0 / 200.0;   //速度PI参数
float g_turn_kp = 0.5, g_turn_kd = 0.001;  //转向PD参数

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);       //开启串口，设置波特率为 9600
  Wire.begin();
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" :
                 "MPU6050 connection failed");
  delay(100);              //延时等待初始化完成
  g_battery_voltage = 12.0;
  // text display tests
  MsTimer2::set(5, timingControl);  //使用Timer2设置5ms定时中断
  MsTimer2::start();          //使用中断使能
  //开启编码器接口外部中断
  attachInterrupt(digitalPinToInterrupt(kEncPinLA), readEncLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kEncPinRA), readEncRight, CHANGE);
}



void loop() {
  // 串口蓝牙发送指令
  Serial.print("BAT");
  Serial.println(g_battery_voltage);
  delay(500);
  Serial.print("ANG");
  Serial.println(g_angle);
  delay(500);
}
/*************************************************
  函数功能：串口响应函数，当串口接收到数据时自动调用。
           当蓝牙连接至串口时，接受蓝牙指令。
  修改记录：
*************************************************/
void serialEvent() {
  static unsigned char Receive_Data;
  while (Serial.available()) {
    Receive_Data = Serial.read();
    switch (Receive_Data)   {
      //蓝牙指令，字符0~8,对应的ascii码为0x30～0x38
      case 0x30:
        g_cmd = STOP;
        break;            //前进
      case 0x31:
        g_cmd = RIGHT;
        break;           //右转
      case 0x32:
        g_cmd = RIGHT;
        break;           //右转
      case 0x33:
        g_cmd = FORWARD;
        break;            //右转
      case 0x34:
        g_cmd = LEFT;
        break;           //后退
      case 0x35:
        g_cmd = LEFT;
        break;             //左转
      case 0x36:
        g_cmd = LEFT;
        break;             //左转
      case 0x37:
        g_cmd = BACK;
        break;           //左转
      case 0x38:
        g_cmd = RIGHT;
        break;           //左转
      default:
        g_cmd = STOP;
        break;              //停止
    }
  }
}
/*************************************************
  函数功能：定时控制函数，处理小车的平衡，前进和转向控制。
  修改记录：
*************************************************/
void timingControl() {
  //注意静态变量static的用法
  static int s_velocity_count = 0, s_turn_count = 0, s_voltage_count = 0;
  static float s_voltage = 0;
  int motor1, motor2;
  sei();//全局中断开启
  getAngle(); //设置全局g_angle
  float gyro_x = g_gx / 131.0 + 2.8;
  g_upright_pwm = uprightPid(g_angle, gyro_x);
  //直立PD控制 控制周期5ms
  if (++s_velocity_count >= 8) { //速度控制，控制周期40ms
    g_speed_left = g_enc_left;
    g_speed_right = g_enc_right;  //积分速度
    g_velocity_pwm = velocityPid(g_speed_left, g_speed_right);
    g_enc_left = g_enc_right = 0;   //编码器清零，
    s_velocity_count = 0;
  }
  if (++s_turn_count >= 4) { //转向控制，控制周期20ms
    g_turn_pwm = turnPid(g_gz);
    s_turn_count = 0;
  }
 // g_velocity_pwm=g_turn_pwm=0;
  motor1 = -g_upright_pwm + g_velocity_pwm - g_turn_pwm;
  motor2 = -g_upright_pwm + g_velocity_pwm + g_turn_pwm;
  if (isPickup())   g_stop_flag = 1;  //小车被拿起关闭电机
  if (isPutdown())  g_stop_flag = 0;  //小车被放下开启电机
  if (checkCarStatus())  setPwm(motor1, motor2);
  s_voltage += analogRead(0);
  if (++s_voltage_count == 200) {
    //Voltage*5.0/1023.0/200.0*3
    g_battery_voltage = s_voltage * 0.000073314;
    s_voltage = 0;
    s_voltage_count = 0;
  }
}

/**************************************************************************
  函数功能：直立PD控制
  入口参数：角度、角速度
  返回值：直立控制PWM
**************************************************************************/
int uprightPid(float angle, float gyro) {
  float bias;
  int balance;
  bias = angle - g_banlance_angle; //修正,正负号需要调试才知道
  balance = g_upright_kp * bias + gyro * g_upright_kd;
  return balance;
}

/**************************************************************************
  函数功能：速度PI控制
  入口参数：左轮编码器、右轮编码器
  返回值：速度控制PWM
**************************************************************************/
float velocityPid(int encoder_left, int encoder_right) {
  static float Velocity, Encoder_Least, Encoder, Movement;
  static float Encoder_Integral;
  switch (g_cmd) { //判定按下的是哪个按键，串口显示按钮名称
    case STOP:  //'1 '
      g_cmd = STOP;
      Movement = 0;
      if (Encoder_Integral > 300)   Encoder_Integral -= 200;
      if (Encoder_Integral < -300)  Encoder_Integral += 200;
      break;
    case FORWARD:  //'2'
      Movement = 300;
      break;
    case BACK:  //'3'
      Movement = -300;
      break;
    default:      
      Movement = 0;
      break;
  }
  //这一段要好好研究一下，我基本是参考的
  //u=Kp*e(t)+Ki*Int(e(t)); 这里的e(t)应该指的是目标位移
  //为什么这里不是减movement，而是Encoder_Integral减movement
  Encoder_Least = (encoder_left + encoder_right) - 0; 
  Encoder *= 0.7;
  Encoder += Encoder_Least * 0.3; //一阶低通滤波器
  Encoder_Integral += Encoder;     //===积分出位移 积分时间：40ms
  Encoder_Integral = Encoder_Integral - Movement;     //控制前进后退
  if (Encoder_Integral > 21000)  Encoder_Integral = 21000;
  if (Encoder_Integral < -21000) Encoder_Integral = -21000;
  Velocity = Encoder * g_velocity_kp + Encoder_Integral * g_velocity_ki;
  if (checkCarStatus() == false || g_stop_flag == 1)
     Encoder_Integral = 0;//小车停止的时候积分清零
  return Velocity;
}
 
/**************************************************************************
  函数功能：转向控制
  入口参数：Z轴陀螺仪
  返回  值：转向控制PWM
**************************************************************************/
int turnPid(float gyro) { //转向控制
  static float Turn_Target, Turn, Turn_Convert = 2;
  float Turn_Amplitude = 60;
  if (g_cmd == LEFT)         Turn_Target += Turn_Convert;  //根据遥控指令改变转向偏差
  else if (g_cmd == RIGHT)   Turn_Target -= Turn_Convert;//根据遥控指令改变转向偏差
  else Turn_Target = 0;
  if (Turn_Target > Turn_Amplitude)  Turn_Target =
      Turn_Amplitude; //===转向速度限幅
  if (Turn_Target < -Turn_Amplitude) Turn_Target = -Turn_Amplitude;
  Turn = -Turn_Target * g_turn_kp + gyro * g_turn_kd;
  // Serial.println(Turn);
  return Turn;
}
/**************************************************************************
  函数功能：根据陀螺仪测量结果，计算小车倾角
  返回值：设置全局g_angle
**************************************************************************/
void getAngle() {
  float dt = 0.005; //dt的取值为滤波器采样时间 5ms
  accelgyro.getMotion6(&g_ax, &g_ay, &g_az, &g_gx, &g_gy, &g_gz);
  float angle_measure = atan2(g_ay, g_az) * 180.0 / PI; //计算倾角
  float gyro_measure = (g_gx + 400) / 131.0; //角速度 ,400是漂移修正值
  ComplementaryFilter2(angle_measure, gyro_measure,dt);
  //KalmanFilter(angle_measure, gyro_measure, dt);
}
/**************************************************************************
  函数功能：二阶互补滤波
  参数：陀螺仪角度angle_m，角速度gyro_m
  最后修改：
**************************************************************************/
void ComplementaryFilter2(float angle_m, float gyro_m, float dt) {
  static float x1, x2, y1; //静态中间变量
  float K2 = 0.2; // 对加速度计取值的权重
  x1 = (angle_m - g_angle) * (1 - K2) * (1 - K2);
  y1 = y1 + x1 * dt;
  x2 = y1 + 2 * (1 - K2) * (angle_m - g_angle) + gyro_m;
  g_angle = g_angle + x2 * dt;
}

/**************************************************************************
  函数功能：卡尔曼滤波
  参数：陀螺仪角度angle_m，角速度gyro_m,积分时间
  https://github.com/TKJElectronics/KalmanFilter/blob/master/Kalman.cpp
**************************************************************************/
void KalmanFilter(float newAngle, float newRate, float dt) {
  //静态变量
  static float P[2][2] = {{ 1, 0 }, { 0, 1 }};
  static float Q_angle = 0.001f, Q_bias = 0.005f, R_measure = 0.03f;
  static float kalman_bias = 0.0f, kalman_rate = 0.0f;
  /* Step 1 */
  kalman_rate = newRate - kalman_bias;
  g_angle += dt * kalman_rate;
  // Update estimation error covariance - Project the error covariance ahead
  /* Step 2 */
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
  // Discrete Kalman filter measurement update equations
  // Calculate Kalman gain - Compute the Kalman gain
  /* Step 4 */
  float S = P[0][0] + R_measure; // Estimate error
  /* Step 5 */
  float K[2]; // Kalman gain - This is a 2x1 vector
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
  // Calculate angle and bias - Update estimate with measurement zk (newAngle)
  /* Step 3 */
  float y = newAngle - g_angle; // Angle difference
  /* Step 6 */
  g_angle += K[0] * y;
  kalman_bias += K[1] * y;
  // Calculate estimation error covariance - Update the error covariance
  /* Step 7 */
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
}
/**************************************************************************
  函数功能：设置左右电机的pwm值
  参数：左右电机的pwm值，正负号代表转动方向
  最后修改：
**************************************************************************/
void setPwm(int motor_left, int motor_right) {
  int max_pwm = 210, min_pwm = 45;
  //PWM满幅是0~255, 电机大概从60左右开始转动
  if (motor_left < -max_pwm) motor_left = -max_pwm;
  if (motor_left > max_pwm)  motor_left = max_pwm;
  if (motor_right < -max_pwm) motor_right = -max_pwm;
  if (motor_right > max_pwm)  motor_right = max_pwm;
  if (motor_left > 0) {
    digitalWrite(kMotorPinLA, 1);
    digitalWrite(kMotorPinLB, 0);
  } else {
    digitalWrite(kMotorPinLA, 0);
    digitalWrite(kMotorPinLB, 1);
  }
  if (motor_right > 0) {
    digitalWrite(kMotorPinRA, 1);
    digitalWrite(kMotorPinRB, 0);
  } else {
    digitalWrite(kMotorPinRA, 0);
    digitalWrite(kMotorPinRB, 1);
  }
  analogWrite(kPwmPinL, abs(motor_left)  + min_pwm);
  analogWrite(kPwmPinR, abs(motor_right) + min_pwm);
}
/**************************************************************************
  函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发
  入口参数：无
  返回  值：无
**************************************************************************/

void readEncLeft() {
  if (digitalRead(kEncPinLA) == LOW) {     //下降沿触发的中断
    digitalRead(kEncPinLB) == LOW ? g_enc_left++ : g_enc_left--;
  } else {
    digitalRead(kEncPinLB) == LOW ? g_enc_left-- : g_enc_left++;
  }
}
/**************************************************************************
  函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发
  入口参数：无
  返回  值：无
**************************************************************************/
void readEncRight() {
  if (digitalRead(kEncPinRA) == LOW) {
    digitalRead(kEncPinRB) == LOW ? g_enc_right++ : g_enc_right--;
  } else {
    digitalRead(kEncPinRB) == LOW ? g_enc_right-- : g_enc_right++;
  }
}

/**************************************************************************
  函数功能：检测小车是否被拿起
  全局参数：Z轴加速度 平衡倾角 左轮编码器 右轮编码器
  返回  值：0：无事件 1：小车被拿起
**************************************************************************/
bool isPickup() {
  static unsigned int flag, count0, count1;
  if (flag == 0) { //第一步
    if (abs(g_speed_left) + abs(g_speed_right) < 15) {
      count0++;  //条件1，小车接近静止
    } else {
      count0 = 0;
    }
    if (count0 > 10) {
      flag = 1;
      count0 = 0;
    }
  }
  if (flag == 1) { //进入第二步
    if (++count1 > 400) {
      count1 = 0;
      flag = 0;                         //超时不再等待2000ms
    }
    if (g_az > 20000 && (g_angle > -15 ) && (g_angle < 15 )) {
      flag = 2; //条件2，小车是在0度附近被拿起
      return true;
    }
  }
  return false;
}
/**************************************************************************
  函数功能：检测小车是否被放下
  全局参数：全局平衡倾角 左右轮速度
  返回  值：0：无事件 1：小车放置并启动
**************************************************************************/
bool isPutdown() {
  static u16 flag, count;
  if (g_stop_flag == 0)    return 0;
  if (flag == 0) {
    if ( abs(g_angle) < 10 && g_speed_left == 0 && g_speed_right == 0)
      flag = 1; //条件1，小车倾角在0度附近的
  }
  if (flag == 1) {
    if (++count > 100) {
      count = 0;
      flag = 0;  // 500ms超时,退出状态1
    }
    //条件2，小车的轮胎被人为转动
    if (g_speed_right > 8   && g_speed_right < 80 ) {
      flag = 0;
      g_cmd = STOP;
      return true;
    }
  }
  return false;
}

/**************************************************************************
  函数功能：检查小车状态
  入口参数：倾角和电池电压
  返回  值：false：异常，需要关闭  true：正常
**************************************************************************/
bool checkCarStatus() {
  if (g_angle < -50 || g_angle > 50 || 1 == g_stop_flag )  {
    g_stop_flag = 1;
    analogWrite(kPwmPinL, 0);  //PWM输出为0
    analogWrite(kPwmPinR, 0); //PWM输出为0
    return false;
  } else   {
    return true;
  }
}

