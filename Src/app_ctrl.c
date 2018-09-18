#include "app_ctrl.h"
#include "motor_pwm.h"
#include "rc_pwm.h"

//油门、横滚、俯仰、航向输入量
int throttle_input,roll_input,pitch_input,yaw_input;
//遥控器输入
int rc_roll,rc_pitch,rc_throttle,rc_yaw,rc_e,rc_g,rc_d;
int rc_roll_max, rc_roll_min;
int rc_pitch_max, rc_pitch_min;
int rc_yaw_max,rc_yaw_min;
int rc_throttle_max, rc_throttle_balance, rc_throttle_min;
float rc_roll_threshold,rc_pitch_threshold,rc_throttle_threshold,rc_yaw_threshold;//注：0.7+0.1+0.1+0.1=1.0，系数满足这样的条件时，不会超量程
//电机输出
//int motor1_output,motor2_output,motor3_output,motor4_output;

void app_ctrl_thread(float dT)
{
  remote_control_data_obtain();
  control();
}

void control(void)
{
  //姿态控制
  attitude_ctrl();
  //设定电机转速
  motor_pwm_set_value(motor1_output, motor2_output, motor3_output, motor4_output);
}

void attitude_ctrl(void)
{
  throttle_input = rc_throttle;
  roll_input = rc_roll;
  pitch_input = rc_pitch;
  yaw_input = rc_yaw;
  write_motor((int)throttle_input, (int)roll_input, (int)pitch_input, (int)yaw_input);
  // write_motor((int)throttle_input, (int)0, (int)0, (int)0);
}

void write_motor(int throttle, int roll, int pitch, int yaw)
{
  motor1_output = throttle - roll - pitch - yaw;
  motor2_output = throttle + roll - pitch + yaw;
  motor3_output = throttle + roll + pitch - yaw;
  motor4_output = throttle - roll + pitch + yaw;
  
  // // 电机PWM设定1000~2000
  // if(motor1_output<0) motor1_output=0;
  // if(motor1_output>1000)  motor1_output=1000;

  // if(motor2_output<0) motor2_output=0;
  // if(motor2_output>1000)  motor2_output=1000;

  // if(motor3_output<0) motor3_output=0;
  // if(motor3_output>1000)  motor3_output=1000;

  // if(motor4_output<0) motor4_output=0;
  // if(motor4_output>1000)  motor4_output=1000;

  motor1_output += 1000;
  motor2_output += 1000;
  motor3_output += 1000;
  motor4_output += 1000;
}

void remote_control_data_obtain(void)
{
  rc_roll = RC_PWM[0];      //roll stick left:1110 mid:1515 right:1930
  rc_pitch = RC_PWM[1];      //pitch stick down:1930 mid:1515 up:1110
  rc_throttle = RC_PWM[2];      //throttle stick down:1110 mid:- up:1930
  rc_yaw = RC_PWM[3];      //yaw stick left:1110 mid:1522 right:1930
  rc_e = RC_PWM[4];      //E stick down:970 mid:1520 up:2070
  rc_g = RC_PWM[5];      //G stick down:970 mid:- up:2070
  //RC_PWM[6];      //roll stick left:1109 mid:1516 right:1930
  rc_d = RC_PWM[7];      //D stick down:2070 mid:- right:970
  
  //范围限定  （注：此处int之间的除法产生浮点数，不进行数据类型转换的话会产生致命误差）
  //rc_throttle : [0,1000]
  rc_throttle_max = 1935; rc_throttle_min = 1122;
  rc_throttle_threshold = 0.7f;
  rc_throttle = (int)(((float)((float)(rc_throttle-rc_throttle_min))/((float)(rc_throttle_max-rc_throttle_min)))*1000.0f); 
  rc_throttle = (int)(rc_throttle * rc_throttle_threshold);
  //rc_roll : [-1000,1000]
  rc_roll_max = 1930; rc_roll_min = 1110;
  rc_roll_threshold = 0.1f;
  rc_roll = (int)(((float)((float)(rc_roll-rc_roll_min))/((float)(rc_roll_max-rc_roll_min)))*2000.0f-1000.0f);
  rc_roll = (int)(rc_roll * rc_roll_threshold);
  //rc_pitch : [-1000,1000]
  rc_pitch_max = 1930; rc_pitch_min = 1110;
  rc_pitch_threshold = 0.1f;
  rc_pitch = (int)(((float)((float)(rc_pitch-rc_pitch_min))/((float)(rc_pitch_max-rc_pitch_min)))*2000.0f-1000.0f);
  rc_pitch = (int)(rc_pitch * rc_pitch_threshold);
  //rc_yaw : [-1000,1000]
  rc_yaw_max = 1935; rc_yaw_min = 1110;
  rc_yaw_threshold = 0.1f;
  rc_yaw = (int)(((float)((float)(rc_yaw-rc_yaw_min))/((float)(rc_yaw_max-rc_yaw_min)))*2000.0f-1000.0f);
  rc_yaw = (int)(rc_yaw * rc_yaw_threshold);

  //操控方向
  rc_throttle = +rc_throttle;
  rc_pitch = -rc_pitch;
  rc_roll = -rc_roll;
  rc_yaw = +rc_yaw;
}
