// #ifndef APP_INS_H
// #define APP_INS_H

// #define DATA_LEN 100

// struct T_axis{
//   float gyro_raw_rad_per_s[DATA_LEN];
//   float gyro_fir_rad_per_s[1];
//   float acc_raw_meter_per_s[DATA_LEN];
//   float acc_fir_meter_per_s[1];
//   // float mag_raw_uT[DATA_LEN];
//   // float mag_fir_uT[1];
//   // float mag_h[1];
// };

// struct T_euler{
//   float roll; //rad
//   float roll_rate;  //rad/s
//   float pitch;  //rad
//   float pitch_rate; //rad/s
//   float yaw;  //rad
//   float yaw_rate; //rad/s
// };

// struct T_quaternion{
//     float q0;
//     float q1;
//     float q2;
//     float q3;
// };

// struct Vector3f{
//   float x;
//   float y;
//   float z;
// };

// extern struct T_axis t_body_x, t_body_y, t_body_z;
// extern struct T_euler t_attitude;

// extern float temp0,temp1,temp2,temp3,temp4,temp5,temp6;

// extern void app_ins_thread(float dT);
// void obtain_imu_data(void);
// void ins_complementary_update(float dT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);


// void data_shift_1(float* ddd,int len);
// void filter_fir(float* in, float* out,float* c,int len);
// void data_shift(void);
// void filter(void);

// #endif
