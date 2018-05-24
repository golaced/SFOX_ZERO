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
  ins_update(dT, t_pitch.gyro_fir[0], t_roll.gyro_fir[0], t_yaw.gyro_fir[0], t_pitch.acc_fir[0], t_roll.acc_fir[0], t_yaw.acc_fir[0]);
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
}

void filter(void)
{		
  filter_fir(t_pitch.gyro_raw, t_pitch.gyro_fir,gyro_c,GYRO_LEN);
  filter_fir(t_roll.gyro_raw, t_roll.gyro_fir,gyro_c,GYRO_LEN);
  filter_fir(t_yaw.gyro_raw, t_yaw.gyro_fir,gyro_c,GYRO_LEN);
          
  filter_fir(t_pitch.acc_raw,t_pitch.acc_fir,gyro_c,GYRO_LEN);
  filter_fir(t_roll.acc_raw,t_roll.acc_fir,gyro_c,GYRO_LEN);
  filter_fir(t_yaw.acc_raw,t_yaw.acc_fir,gyro_c,GYRO_LEN);
}

void ins_update(float dT, float gx, float gy, float gz, float ax, float ay, float az)
{
  float Kp=15.0f;                       
  float Ki=0.008f;
  static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // ��ʼ��̬��Ԫ��������ƪ�����ᵽ�ı任��Ԫ����ʽ����
  static float exInt = 0, eyInt = 0, ezInt = 0;         //��ǰ�ӼƲ�õ��������ٶ��������ϵķ������õ�ǰ��̬��������������������ϵķ��������Ļ���
  float q0_old,q1_old,q2_old,q3_old;                    //��Ԫ���ݴ���������΢�ַ���ʱҪ��
  float acc_norm; 
  float quaternion_norm;
  float vx, vy, vz;                                     //��ǰ��̬��������������������ϵķ���
  float ex, ey, ez;                                     //��ǰ�ӼƲ�õ��������ٶ��������ϵķ������õ�ǰ��̬��������������������ϵķ��������

  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q1q1 = q1*q1;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
  
  if(ax*ay*az==0)//�Ӽƴ�����������״̬ʱ��������̬���㣬��Ϊ�������ĸ���������
        return;
  //��λ�����ٶȼƣ�
  acc_norm = sqrt(ax*ax + ay*ay + az*az);
  ax = ax / acc_norm;
  ay = ay / acc_norm;
  az = az / acc_norm;

  //�õ�ǰ��̬������������������ϵķ�����
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  //�����õ������������������������������Ա�ʾ��һ���
  ex = (ay*vz - az*vy) ;
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;                                           //�������л���
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  gx = gx + Kp*ex + exInt;  //�����PI�󲹳��������ǣ����������Ư��
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;

  //�ݴ浱ǰֵ���ڼ���
  q0_old=q0;
  q1_old=q1;
  q2_old=q2;
  q3_old=q3;

  //һ�ױϿ��ⷨ,��Ԫ������
  q0 = q0_old + (-q1_old*gx - q2_old*gy - q3_old*gz) * 0.5f * dT;
  q1 = q1_old + (q0_old*gx + q2_old*gz -q3_old*gy) * 0.5f * dT;
  q2 = q2_old + (q0_old*gy - q1_old*gz +q3_old*gx) * 0.5f * dT;
  q3 = q3_old + (q0_old*gz + q1_old*gy -q2_old*gx) * 0.5f * dT;

  //��Ԫ����һ��
  quaternion_norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / quaternion_norm;
  q1 = q1 / quaternion_norm;
  q2 = q2 / quaternion_norm;
  q3 = q3 / quaternion_norm;

  //��Ԫ����ŷ���ǵ�ת��
  t_pitch.euler_rad = atan2(2*(q2*q3-q0*q1),1-2*(q1*q1+q2*q2));
  t_roll.euler_rad = -asin(2*(q1*q3+q0*q2));
  t_yaw.euler_rad = atan2(2*(q1*q2-q0*q3),1-2*(q2*q2+q3*q3));
  
  t_pitch.euler_deg = t_pitch.euler_rad * RAD_TO_DEG;
  t_roll.euler_deg = t_roll.euler_rad * RAD_TO_DEG;
  t_yaw.euler_deg = t_yaw.euler_rad * RAD_TO_DEG; 
}