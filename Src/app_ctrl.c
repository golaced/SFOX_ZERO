#include "app_ctrl.h"
#include "motor_pwm.h"
#include "rc_pwm.h"

//电机的直接输入量
uint16_t throttle_for_motor;
int roll_for_motor;
int pitch_for_motor;
int yaw_for_motor;
//遥控器数据
int rc_roll,rc_pitch,rc_throttle,rc_yaw,rc_e,rc_g,rc_d;

void app_ctrl_thread(float dT)
{
  remote_control_data_obtain();
  attitude_ctrl();
  write_motor(throttle_for_motor, roll_for_motor, pitch_for_motor, yaw_for_motor);
}

void attitude_ctrl(void)
{
  
}

void write_motor(uint16_t throttle, int roll, int pitch, int yaw)
{
  int16_t motor1_output = throttle - roll + pitch + yaw;
  int16_t motor2_output = throttle + roll - pitch + yaw;
  int16_t motor3_output = throttle + roll + pitch - yaw;
  int16_t motor4_output = throttle - roll - pitch - yaw;
  
  //防止PWM设定值出现负值,电机输出最小为0
  if(motor1_output<0)     motor1_output = 0;
  if(motor2_output<0)     motor2_output = 0;
  if(motor3_output<0)     motor3_output = 0;
  if(motor4_output<0)     motor4_output = 0;
  
  //写入PWM
  motor_pwm_set_value(motor1_output, motor2_output, motor3_output, motor4_output);
}

void remote_control_data_obtain(void)
{
  rc_roll = RC_PWM[0];      //roll stick left:1110 mid:1515 right:1930
  rc_pitch = RC_PWM[1];      //pitch stick down:1930 mid:1515 up:1110
  rc_throttle = RC_PWM[2];      //throttle stick down:1110 mid:- up:1930
  rc_yaw = RC_PWM[3];      //yaw stick left:1110 mid:1522 right:1930
  rc_e = RC_PWM[4];      //E stick down:970 mid:1520 up:2070
  rc_g = RC_PWM[5];      //G stick down:970 mid:- up:2070
  //RC_PWM[6];未定义      //roll stick left:1109 mid:1516 right:1930
  rc_d = RC_PWM[7];      //D stick down:2070 mid:- right:970
}