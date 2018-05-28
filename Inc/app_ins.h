#ifndef APP_INS_H
#define APP_INS_H

#define DATA_LEN 64

struct T_axis{
  float gyro_raw[DATA_LEN];
  float gyro_fir[1];
  float acc_raw[DATA_LEN];
  float acc_fir[1];
  float mag_raw[DATA_LEN];
  float mag_fir[1];
  float mag_h[1];
  float euler_deg;
  float euler_rad;
};

struct T_quaternion{
    float q0;
    float q1;
    float q2;
    float q3;
};

struct Vector3f{
  float x;
  float y;
  float z;
};

extern struct T_axis t_pitch, t_roll, t_yaw;

extern void app_ins_thread(float dT);
void obtain_imu_data(void);
void ins_update(float dT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

void data_shift_1(float* ddd,int len);
void filter_fir(float* in, float* out,float* c,int len);
void data_shift(void);
void filter(void);

#endif
