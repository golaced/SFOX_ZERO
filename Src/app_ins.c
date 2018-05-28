#include "app_ins.h"
#include "mpu9250.h"
#include <math.h>

#define DEG_TO_RAD 0.01745329251994329576f
#define RAD_TO_DEG 57.29577951308232087679f

struct T_axis t_pitch, t_roll, t_yaw;

#define GYRO_LEN	12
static float gyro_c[GYRO_LEN]={0.171875,0.171875,0.200195313,0.211914063,0.201171875,0.163085938,0.10546875,0.044921875,-0.0078125,-0.048828125,-0.075195313,-0.137695313};

void app_ins_thread(float dT)
{
  obtain_imu_data();
  filter();
  ins_update(dT, t_pitch.gyro_fir[0], t_roll.gyro_fir[0], t_yaw.gyro_fir[0], t_pitch.acc_fir[0], t_roll.acc_fir[0], t_yaw.acc_fir[0],  t_pitch.mag_fir[0], t_roll.mag_fir[0], t_yaw.mag_fir[0]);
  data_shift();
}

void obtain_imu_data(void)
{
  t_pitch.gyro_raw[0] = gyrox_raw_dps * DEG_TO_RAD;
  t_roll.gyro_raw[0] = gyroy_raw_dps * DEG_TO_RAD;
  t_yaw.gyro_raw[0] = gyroz_raw_dps * DEG_TO_RAD;
  
  t_pitch.acc_raw[0] = accx_raw_mps;
  t_roll.acc_raw[0] = accy_raw_mps;
  t_yaw.acc_raw[0] = accz_raw_mps;

  t_pitch.mag_raw[0] = magx_raw_uT;
  t_roll.mag_raw[0] = magy_raw_uT;
  t_yaw.mag_raw[0] = magz_raw_uT;
}

void data_shift_1(float* ddd,int len)
{
  for(int i=len;i>=2;i--)		ddd[i-1] = ddd[i-2];
}

void filter_fir(float* in, float* out,float* c,int len)
{
  out[0]=0;
  for(int i=0;i<len;i++)
          out[0] += (float)in[i] * c[i];
}

void data_shift(void)
{
  data_shift_1(t_pitch.gyro_raw,DATA_LEN);
  data_shift_1(t_roll.gyro_raw,DATA_LEN);
  data_shift_1(t_yaw.gyro_raw,DATA_LEN);

  data_shift_1(t_pitch.acc_raw,DATA_LEN);
  data_shift_1(t_roll.acc_raw,DATA_LEN);
  data_shift_1(t_yaw.acc_raw,DATA_LEN);

  data_shift_1(t_pitch.mag_raw,DATA_LEN);
  data_shift_1(t_roll.mag_raw,DATA_LEN);
  data_shift_1(t_yaw.mag_raw,DATA_LEN);
}

void filter(void)
{		
  filter_fir(t_pitch.gyro_raw, t_pitch.gyro_fir,gyro_c,GYRO_LEN);
  filter_fir(t_roll.gyro_raw, t_roll.gyro_fir,gyro_c,GYRO_LEN);
  filter_fir(t_yaw.gyro_raw, t_yaw.gyro_fir,gyro_c,GYRO_LEN);
          
  filter_fir(t_pitch.acc_raw,t_pitch.acc_fir,gyro_c,GYRO_LEN);
  filter_fir(t_roll.acc_raw,t_roll.acc_fir,gyro_c,GYRO_LEN);
  filter_fir(t_yaw.acc_raw,t_yaw.acc_fir,gyro_c,GYRO_LEN);

  filter_fir(t_pitch.mag_raw,t_pitch.mag_fir,gyro_c,GYRO_LEN);
  filter_fir(t_roll.mag_raw,t_roll.mag_fir,gyro_c,GYRO_LEN);
  filter_fir(t_yaw.mag_raw,t_yaw.mag_fir,gyro_c,GYRO_LEN);
}

void ins_update(float dT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
  float Kp=15.0f;
  float Ki=0.008f;
  static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;              
  float q0_old,q1_old,q2_old,q3_old;    

  float acc_norm; 
  float quaternion_norm;
  float vx, vy, vz;                                     
  float ex, ey, ez;     
  static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f; 

  // float mag_xy_norm; 
  // float mx_h, my_h, mz_h;
  // float m_vx, m_vy, m_vz;  
  // float m_ex, m_ey, m_ez;    
  // static float m_exInt = 0, m_eyInt = 0, m_ezInt = 0;                              

  // float q0q0 = q0*q0;
  // float q0q1 = q0*q1;
  // float q0q2 = q0*q2;
  // float q0q3 = q0*q3;
  // float q1q1 = q1*q1;
  // float q1q2 = q1*q2;
  // float q1q3 = q1*q3;
  // float q2q2 = q2*q2;
  // float q2q3 = q2*q3;
  // float q3q3 = q3*q3;
  
  ////////////////////////////////////////////////////////////
  //加速度计测量的重力
  acc_norm = sqrt(ax*ax + ay*ay + az*az);
  ax = ax / acc_norm;
  ay = ay / acc_norm;
  az = az / acc_norm;

  //四元数矩阵得到的重力
  vx = 2*(q1*q3 - q0*q2);
  vy = 2*(q0*q1 + q2*q3);
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3 ;

  //两个向量做叉乘
  ex = (ay*vz - az*vy) ;
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  //积分
  exInt = exInt + ex * Ki;                                           
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  //比例+积分
  gx = gx + Kp*ex + exInt;  
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;

  // //////////////////////////////////////////////////////////////
  // //磁强计测量数据
  // mx_h = mx*cos(t_roll.euler_rad) - mz*cos(t_roll.euler_rad);
  // my_h = mx*sin(t_roll.euler_rad)*sin(t_pitch.euler_rad) + my*cos(t_pitch.euler_rad) + mz*cos(t_roll.euler_rad)*sin(t_pitch.euler_rad);
  // mz_h = mx*sin(t_roll.euler_rad)*cos(t_pitch.euler_rad) - my*sin(t_pitch.euler_rad) + mz*cos(t_roll.euler_rad)*cos(t_pitch.euler_rad);
  // mag_xy_norm = sqrt(mx_h*mx_h + my_h*my_h);
  // if(mag_xy_norm < 0.0001)
  // {
  //   mx_h = 0.0f;
  //   my_h = 0.0f;
  //   mz_h = 0.0f;
  // }
  // else
  // {
  //   mx_h = mx_h/mag_xy_norm;
  //   my_h = my_h/mag_xy_norm;
  //   mz_h = 0.0f;
  // }

  // //四元数矩阵得到的地磁场
  // m_vx = 2*(q1*q2 + q0*q3);
  // m_vy = 1 - 2*(q1*q1+q3*q3);
  // m_vz = 2*(q2*q3 - q0*q1);

  // //两个向量做叉乘
  // m_ex = (my_h*m_vz - mz_h*m_vy) ;
  // m_ey = (mz_h*m_vx - mx_h*m_vz) ;
  // m_ez = (mx_h*m_vy - my_h*m_vx) ;

  // //积分
  // m_exInt = m_exInt + m_ex * Ki;                                           
  // m_eyInt = m_eyInt + m_ey * Ki;
  // m_ezInt = m_ezInt + m_ez * Ki;

  // //比例+积分
  // gx = gx + Kp*m_ex + m_exInt;  
  // gy = gy + Kp*m_ey + m_eyInt;
  // gz = gz + Kp*m_ez + m_ezInt;

  // //////////////////////////////////////////////////////////////

  //四元数迭代
  q0_old=q0;
  q1_old=q1;
  q2_old=q2;
  q3_old=q3;

  //四元数更新（纯陀螺仪）
  q0 = q0_old + (-q1_old*gx - q2_old*gy - q3_old*gz) * 0.5f * dT;
  q1 = q1_old + (q0_old*gx + q2_old*gz -q3_old*gy) * 0.5f * dT;
  q2 = q2_old + (q0_old*gy - q1_old*gz +q3_old*gx) * 0.5f * dT;
  q3 = q3_old + (q0_old*gz + q1_old*gy -q2_old*gx) * 0.5f * dT;

  //四元数归一化
  quaternion_norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / quaternion_norm;
  q1 = q1 / quaternion_norm;
  q2 = q2 / quaternion_norm;
  q3 = q3 / quaternion_norm;
  if(isnan(q0)||isnan(q1)||isnan(q2)||isnan(q3))
  {
    q0=1;q1=0;q2=0;q3=0;
  }

  //��Ԫ����ŷ���ǵ�ת��
  t_pitch.euler_rad = atan2(2*(q2*q3-q0*q1),1-2*(q1*q1+q2*q2));
  t_roll.euler_rad = -asin(2*(q1*q3+q0*q2));
  //t_yaw.euler_rad = atan2(2*(q1*q2-q0*q3),1-2*(q2*q2+q3*q3));
  //t_yaw.euler_rad = atan2(my,mx);

  
  t_pitch.euler_deg = t_pitch.euler_rad * RAD_TO_DEG;
  t_roll.euler_deg = t_roll.euler_rad * RAD_TO_DEG;
  t_yaw.euler_deg = t_yaw.euler_rad * RAD_TO_DEG; 
}
