#include "app_ins.h"
#include "mpu9250.h"
#include <math.h>

#define DEG_TO_RAD 0.01745329251994329576f
#define RAD_TO_DEG 57.29577951308232087679f

struct T_axis t_pitch, t_roll, t_yaw;

#define GYRO_LEN	12
static float gyro_c[GYRO_LEN]={0.171875,0.171875,0.200195313,0.211914063,0.201171875,0.163085938,0.10546875,0.044921875,-0.0078125,-0.048828125,-0.075195313,-0.137695313};

#define MAG_LEN 101
static float mag_c[MAG_LEN]={
    0.04754435644, 0.004697572906, 0.004919932224, 0.005150574725, 0.005376234651,
   0.005611456465,  0.00583946798, 0.006078177132, 0.006309270393, 0.006546489894,
   0.006777013186,  0.00701350579, 0.007245077752, 0.007482835092,  0.00771403173,
    0.00794617366, 0.008175152354, 0.008409786969, 0.008627979085, 0.008859299123,
   0.009071470238, 0.009296617471, 0.009500969201, 0.009705152363, 0.009914943948,
    0.01010550652,  0.01029173378,  0.01047855895,  0.01066237688,  0.01084415615,
    0.01101681869,  0.01118485257,  0.01134161744,  0.01149071939,  0.01162635349,
    0.01175176259,  0.01187754236,   0.0120053459,  0.01215935219,  0.01221348532,
    0.01231596619,  0.01241615135,  0.01247958653,  0.01255952101,  0.01260889601,
    0.01267067157,  0.01270479523,  0.01274325605,   0.0127606364,  0.01278058346,
    0.01277900953,  0.01278058346,   0.0127606364,  0.01274325605,  0.01270479523,
    0.01267067157,  0.01260889601,  0.01255952101,  0.01247958653,  0.01241615135,
    0.01231596619,  0.01221348532,  0.01215935219,   0.0120053459,  0.01187754236,
    0.01175176259,  0.01162635349,  0.01149071939,  0.01134161744,  0.01118485257,
    0.01101681869,  0.01084415615,  0.01066237688,  0.01047855895,  0.01029173378,
    0.01010550652, 0.009914943948, 0.009705152363, 0.009500969201, 0.009296617471,
   0.009071470238, 0.008859299123, 0.008627979085, 0.008409786969, 0.008175152354,
    0.00794617366,  0.00771403173, 0.007482835092, 0.007245077752,  0.00701350579,
   0.006777013186, 0.006546489894, 0.006309270393, 0.006078177132,  0.00583946798,
   0.005611456465, 0.005376234651, 0.005150574725, 0.004919932224, 0.004697572906,
    0.04754435644
};

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

  filter_fir(t_pitch.mag_raw,t_pitch.mag_fir,mag_c,MAG_LEN);
  filter_fir(t_roll.mag_raw,t_roll.mag_fir,mag_c,MAG_LEN);
  filter_fir(t_yaw.mag_raw,t_yaw.mag_fir,mag_c,MAG_LEN);
}

float Kp_acc=15.0f;
float Ki_acc=0.008f;

float Kp_mag=5.0f;
float Ki_mag=0.003f;

void ins_update(float dT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
  static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
  float q0_old,q1_old,q2_old,q3_old;
  float quaternion_norm;
  //重力校正姿态
  static float acc_exInt = 0, acc_eyInt = 0, acc_ezInt = 0;
  float acc_norm;
  float acc_vx, acc_vy, acc_vz;
  float acc_ex, acc_ey, acc_ez;

  if(ax*ay*az==0) return;

  acc_norm = sqrt(ax*ax + ay*ay + az*az);
  ax = ax / acc_norm;
  ay = ay / acc_norm;
  az = az / acc_norm;

  acc_vx = 2*(q1*q3 - q0*q2);
  acc_vy = 2*(q0*q1 + q2*q3);
  acc_vz = q0*q0 - q1*q1 - q2*q2 + q3*q3 ;

  acc_ex = (ay*acc_vz - az*acc_vy) ;
  acc_ey = (az*acc_vx - ax*acc_vz) ;
  acc_ez = (ax*acc_vy - ay*acc_vx) ;

  acc_exInt = acc_exInt + acc_ex * Ki_acc;
  acc_eyInt = acc_eyInt + acc_ey * Ki_acc;
  acc_ezInt = acc_ezInt + acc_ez * Ki_acc;

  gx = gx + Kp_acc*acc_ex + acc_exInt;
  gy = gy + Kp_acc*acc_ey + acc_eyInt;
  gz = gz + Kp_acc*acc_ez + acc_ezInt;

  //地磁场校正姿态
  static float mag_exInt = 0, mag_eyInt = 0, mag_ezInt = 0;
  float mag_norm;
  float mhx,mhy,mhz;
  float mag_vx, mag_vy, mag_vz;
  float mag_ex, mag_ey, mag_ez;

  mhx = mx*cos(t_roll.euler_rad) + my*sin(t_roll.euler_rad)*sin(t_pitch.euler_rad) + mz*sin(t_roll.euler_rad)*cos(t_pitch.euler_rad);
  mhy = my*cos(t_pitch.euler_rad) - mz*sin(t_pitch.euler_rad);
  mhz = -mx*sin(t_roll.euler_rad) + my*cos(t_roll.euler_rad)*sin(t_pitch.euler_rad) + mz*cos(t_roll.euler_rad)*cos(t_pitch.euler_rad);

  mag_norm = sqrt(mhx*mhx + mhy*mhy);
  mhx = mhx / mag_norm;
  mhy = mhy / mag_norm;

  mag_vx = 2*(q1*q2 + q0*q3);
  mag_vy = 1-2*(q1*q1+q3*q3);
  mag_vz = 2*(q2*q3 - q0*q1);

  mag_ex = (mhy*mag_vz - mhz*mag_vy) ;
  mag_ey = (mhz*mag_vx - mhx*mag_vz) ;
  mag_ez = (mhx*mag_vy - mhy*mag_vx) ;

  mag_exInt = mag_exInt + mag_ex * Ki_mag;
  mag_eyInt = mag_eyInt + mag_ey * Ki_mag;
  mag_ezInt = mag_ezInt + mag_ez * Ki_mag;

  // gx = gx + Kp_mag*mag_ex + mag_exInt;
  // gy = gy + Kp_mag*mag_ey + mag_eyInt;
  gz = gz + Kp_mag*mag_ez + mag_ezInt;
  // mhz = 0;
  t_pitch.mag_h[0] = mhx;
  t_roll.mag_h[0] = mhy;

  //四元数迭代
  q0_old=q0;
  q1_old=q1;
  q2_old=q2;
  q3_old=q3;

  //四元数更新
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
  //输出结果
  t_pitch.euler_rad = atan2(2*(q2*q3-q0*q1),1-2*(q1*q1+q2*q2));
  t_roll.euler_rad = -asin(2*(q1*q3+q0*q2));
  t_yaw.euler_rad = atan2(2*(q1*q2-q0*q3),1-2*(q2*q2+q3*q3));
  // t_yaw.euler_rad = atan2(t_pitch.mag_h[0],t_roll.mag_h[0]);

  t_pitch.euler_deg = t_pitch.euler_rad * RAD_TO_DEG;
  t_roll.euler_deg = t_roll.euler_rad * RAD_TO_DEG;
  t_yaw.euler_deg = t_yaw.euler_rad * RAD_TO_DEG;
}
