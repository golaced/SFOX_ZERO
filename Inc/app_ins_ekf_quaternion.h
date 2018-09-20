#ifndef __APP_INS_EKF_QUATERNION_H_
#define __APP_INS_EKF_QUATERNION_H_

//全局变量
extern float roll_deg,pitch_deg,yaw_deg;
extern float quat0,quat1,quat2,quat3;
extern float pos_north,pos_east,pos_alt;
extern float vel_north,pos_east,vel_alt;
extern float bias_gyrox,bias_gyroy,bias_gyroz;
extern float bias_accx,bias_accy,bias_accz;

//全局函数
extern int app_ins_ekf_quaternion_thread(float dT_s);

//私有函数
int obtain_sensors_data(void);
int ekf_basedon_quaternion(float dT_s,
                           float wx, float wy, float wz, float fx, float fy, float fz, float mx, float my, float mz,
                           float pos_north, float pos_east, float pos_alt, float vel_north, float vel_east, float vel_down);

#endif
