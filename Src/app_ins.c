// #include "app_ins.h"
// #include "mpu9250.h"
// #include <math.h>

// #define DEG_TO_RAD 0.01745329251994329576f
// #define RAD_TO_DEG 57.29577951308232087679f

// struct T_axis t_body_x, t_body_y, t_body_z;
// struct T_euler t_attitude;

// #define LPF_20HZ_20ORDER_LEN	21
// static float LPF_20HZ_20ORDER[21] = {
//     0.01115075871,  0.07922872156,  0.02872786298,  0.04355410859,  0.04863154516,
//     0.05459419265,  0.05965759233,  0.06390339881,  0.06703530997,  0.06894449145,
//     0.06959385425,  0.06894449145,  0.06703530997,  0.06390339881,  0.05965759233,
//     0.05459419265,  0.04863154516,  0.04355410859,  0.02872786298,  0.07922872156,
//     0.01115075871
// };

// #define LPF_5HZ_40ORDER_LEN	41
// static float LPF_5HZ_40ORDER[41] = {
//     0.04748015106,  0.01170152053,  0.01301597059,   0.0144028645,  0.01571816579,
//     0.01708364673,  0.01845890842,   0.0197705999,  0.02104663849,   0.0222909078,
//     0.02346809208,  0.02457327023,  0.02559579723,  0.02650931664,  0.02731505223,
//     0.02801789343,  0.02860432677,  0.02907093801,  0.02941625006,  0.02962467633,
//      0.0296923127,  0.02962467633,  0.02941625006,  0.02907093801,  0.02860432677,
//     0.02801789343,  0.02731505223,  0.02650931664,  0.02559579723,  0.02457327023,
//     0.02346809208,   0.0222909078,  0.02104663849,   0.0197705999,  0.01845890842,
//     0.01708364673,  0.01571816579,   0.0144028645,  0.01301597059,  0.01170152053,
//     0.04748015106
// };

// #define LPF_3HZ_60ORDER_LEN	61
// static float LPF_3HZ_60ORDER[61] = {
//     0.02573291957, 0.006101033185, 0.006788037717, 0.007498088293, 0.008228044957,
//     0.00897594355, 0.009739640169,  0.01051809452,  0.01130779833,  0.01209843066,
//     0.01289304253,  0.01368193422,  0.01446652971,  0.01523294859,  0.01598395035,
//     0.01671725139,  0.01742048562,  0.01809951849,  0.01874696091,  0.01935563982,
//     0.01993170567,  0.02045349218,  0.02094293013,  0.02137640491,  0.02173508704,
//      0.0220839195,  0.02234760113,  0.02256037109,  0.02270742506,  0.02280978486,
//     0.02283978648,  0.02280978486,  0.02270742506,  0.02256037109,  0.02234760113,
//      0.0220839195,  0.02173508704,  0.02137640491,  0.02094293013,  0.02045349218,
//     0.01993170567,  0.01935563982,  0.01874696091,  0.01809951849,  0.01742048562,
//     0.01671725139,  0.01598395035,  0.01523294859,  0.01446652971,  0.01368193422,
//     0.01289304253,  0.01209843066,  0.01130779833,  0.01051809452, 0.009739640169,
//     0.00897594355, 0.008228044957, 0.007498088293, 0.006788037717, 0.006101033185,
//     0.02573291957
// };


// void app_ins_thread(float dT)
// {
//   obtain_imu_data();
//   filter();
//   ins_complementary_update(dT, \
//   t_body_x.gyro_fir_rad_per_s[0], t_body_y.gyro_fir_rad_per_s[0], t_body_z.gyro_fir_rad_per_s[0],\
//   t_body_x.acc_fir_meter_per_s[0], t_body_y.acc_fir_meter_per_s[0], t_body_z.acc_fir_meter_per_s[0],\
//   0, 0, 0);
//   data_shift();
// }

// void obtain_imu_data(void)
// {
//   t_body_x.gyro_raw_rad_per_s[0] = gyrox_raw_dps * DEG_TO_RAD;
//   t_body_y.gyro_raw_rad_per_s[0] = gyroy_raw_dps * DEG_TO_RAD;
//   t_body_z.gyro_raw_rad_per_s[0] = gyroz_raw_dps * DEG_TO_RAD;
  
//   t_body_x.acc_raw_meter_per_s[0] = accx_raw_mps;
//   t_body_y.acc_raw_meter_per_s[0] = accy_raw_mps;
//   t_body_z.acc_raw_meter_per_s[0] = accz_raw_mps;

//   // t_body_x.mag_raw_uT[0] = magx_raw_uT;
//   // t_body_y.mag_raw_uT[0] = magy_raw_uT;
//   // t_body_z.mag_raw_uT[0] = magz_raw_uT;
// }

// static void LPF_1(float hz,float time,float in,float *out)  
// {
// 	*out += ( 1 / ( 1 + 1 / ( hz *6.28f *time ) ) ) *( in - *out );
// }

// void data_shift_1(float* ddd,int len)
// {
//   for(int i=len;i>=2;i--)		ddd[i-1] = ddd[i-2];
// }

// void filter_fir(float* in, float* out,float* c,int len)
// {
//   out[0]=0;
//   for(int i=0;i<len;i++)
//           out[0] += (float)in[i] * c[i];
// }

// void data_shift(void)
// {
//   data_shift_1(t_body_x.gyro_raw_rad_per_s,LPF_20HZ_20ORDER_LEN);
//   data_shift_1(t_body_y.gyro_raw_rad_per_s,LPF_20HZ_20ORDER_LEN);
//   data_shift_1(t_body_z.gyro_raw_rad_per_s,LPF_20HZ_20ORDER_LEN);

//   data_shift_1(t_body_x.acc_raw_meter_per_s,LPF_20HZ_20ORDER_LEN);
//   data_shift_1(t_body_y.acc_raw_meter_per_s,LPF_20HZ_20ORDER_LEN);
//   data_shift_1(t_body_z.acc_raw_meter_per_s,LPF_20HZ_20ORDER_LEN);

//   // data_shift_1(t_body_x.mag_raw_uT,LPF_3HZ_60ORDER_LEN);
//   // data_shift_1(t_body_y.mag_raw_uT,LPF_3HZ_60ORDER_LEN);
//   // data_shift_1(t_body_z.mag_raw_uT,LPF_3HZ_60ORDER_LEN);
// }

// void filter(void)
// {		
//   filter_fir(t_body_x.gyro_raw_rad_per_s, t_body_x.gyro_fir_rad_per_s,LPF_5HZ_40ORDER,LPF_5HZ_40ORDER_LEN);
//   filter_fir(t_body_y.gyro_raw_rad_per_s, t_body_y.gyro_fir_rad_per_s,LPF_5HZ_40ORDER,LPF_5HZ_40ORDER_LEN);
//   filter_fir(t_body_z.gyro_raw_rad_per_s, t_body_z.gyro_fir_rad_per_s,LPF_5HZ_40ORDER,LPF_5HZ_40ORDER_LEN);
          
//   filter_fir(t_body_x.acc_raw_meter_per_s,t_body_x.acc_fir_meter_per_s,LPF_5HZ_40ORDER,LPF_5HZ_40ORDER_LEN);
//   filter_fir(t_body_y.acc_raw_meter_per_s,t_body_y.acc_fir_meter_per_s,LPF_5HZ_40ORDER,LPF_5HZ_40ORDER_LEN);
//   filter_fir(t_body_z.acc_raw_meter_per_s,t_body_z.acc_fir_meter_per_s,LPF_5HZ_40ORDER,LPF_5HZ_40ORDER_LEN);

//   // filter_fir(t_body_x.mag_raw_uT,t_body_x.mag_fir_uT,LPF_3HZ_60ORDER,LPF_3HZ_60ORDER_LEN);
//   // filter_fir(t_body_y.mag_raw_uT,t_body_y.mag_fir_uT,LPF_3HZ_60ORDER,LPF_3HZ_60ORDER_LEN);
//   // filter_fir(t_body_z.mag_raw_uT,t_body_z.mag_fir_uT,LPF_3HZ_60ORDER,LPF_3HZ_60ORDER_LEN);
// }

// float Kp_acc=15.0f;
// float Ki_acc=0.1f;

// float Kp_mag=7.0f;
// float Ki_mag=0.07f;

// float yaw_threshold = 10.0f;

// float temp0,temp1,temp2,temp3,temp4,temp5,temp6;
// void ins_complementary_update(float dT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
// {
//   float quaternion_norm;
//   static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
//   static float q0_old,q1_old,q2_old,q3_old;
//   /***************************************重力&磁力计校正姿态*************************************************/
//   float acc_norm;
//   float ax_g, ay_g, az_g;
//   float acc_vx, acc_vy, acc_vz;
//   float acc_ex, acc_ey, acc_ez;
//   static float acc_exInt = 0, acc_eyInt = 0, acc_ezInt = 0;

//   //加速度数据单位化处理
//   acc_norm = sqrt(ax*ax + ay*ay + az*az);
//   if((int)(acc_norm) == 0) return;               //传感器未读取数据时，acc_norm为0,此时会出现NAN错误
//   ax_g = ax / acc_norm;
//   ay_g = ay / acc_norm;
//   az_g = az / acc_norm;
//   //重力向量提取
//   acc_vx = 2*(q1*q3 - q0*q2);
//   acc_vy = 2*(q0*q1 + q2*q3);
//   acc_vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
//   //重量向量与实测重力值叉乘得到误差
//   acc_ex = (ay_g*acc_vz - az_g*acc_vy);
//   acc_ey = (az_g*acc_vx - ax_g*acc_vz);
//   acc_ez = (ax_g*acc_vy - ay_g*acc_vx);
//   //误差积分+比例，校正x和y角速度值
//   acc_exInt = acc_exInt + acc_ex * Ki_acc;
//   acc_eyInt = acc_eyInt + acc_ey * Ki_acc;
//   acc_ezInt = acc_ezInt + acc_ez * Ki_acc;
//   gx = gx + Kp_acc*acc_ex + acc_exInt;
//   gy = gy + Kp_acc*acc_ey + acc_eyInt;
//   gz = gz + Kp_acc*acc_ez + acc_ezInt;
//   //四元数迭代
//   q0_old=q0;
//   q1_old=q1;
//   q2_old=q2;
//   q3_old=q3;
//   //四元数更新
//   q0 = q0_old + (-q1_old*gx - q2_old*gy - q3_old*gz) * 0.5f * dT;
//   q1 = q1_old + (q0_old*gx + q2_old*gz -q3_old*gy) * 0.5f * dT;
//   q2 = q2_old + (q0_old*gy - q1_old*gz +q3_old*gx) * 0.5f * dT;
//   q3 = q3_old + (q0_old*gz + q1_old*gy -q2_old*gx) * 0.5f * dT;
//   //四元数归一化
//   quaternion_norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//   q0 = q0 / quaternion_norm;
//   q1 = q1 / quaternion_norm;
//   q2 = q2 / quaternion_norm;
//   q3 = q3 / quaternion_norm;
//   //输出结果
//   t_attitude.roll = -atan2(2*(q2*q3+q0*q1),1-2*(q1*q1+q2*q2));
//   t_attitude.pitch = asin(2*(q1*q3-q0*q2));
//   // t_attitude.roll = atan2(2*(q2*q3-q0*q1),1-2*(q1*q1+q2*q2));
//   // t_attitude.pitch = -asin(2*(q1*q3+q0*q2));

//   /***************************************地磁场校正姿态*************************************************/
//   float mag_norm;
//   float mnx,mny,mnz;
//   float mhx,mhy,mhz;
//   float mwx, mwy, mwz;
//   float mag_ex, mag_ey, mag_ez;
//   static float mag_exInt = 0, mag_eyInt = 0, mag_ezInt = 0;
//   float det_mag;
//   float roll = t_attitude.roll;
//   float pitch = -t_attitude.pitch;
//   float yaw = 0.0f;
//   //将磁力计数据投影至导航系
//   mnx = mx*cos(pitch) - mz*sin(pitch);
//   mny = mx*sin(pitch)*sin(roll) + my*cos(roll) + mz*cos(pitch)*sin(roll);
//   mnz = mx*sin(pitch)*cos(roll) - my*sin(roll) + mz*cos(pitch)*cos(roll);
//   //构造伪磁力计数据
//   float pseudo_yaw = atan2(mny,mnx);
//   float mpx = cos(pitch)*cos(pseudo_yaw);
//   float mpy = cos(pitch)*sin(pseudo_yaw);
//   float mpz = -sin(pitch);
//   temp1 = mpx;  temp2 = mpy;  temp3 = mpz;


  
//   // // //磁力计在XOY平面上(n系)的向量大小必定相同
//   // // mhx = sqrt(mnx*mnx + mny*mny);
//   // // mhy = 0.0f;
//   // // mhz = mnz;
//   // // //将其再投影至b系(n系到b系)
//   // // mwx = mhx*cos(pitch)*cos(yaw) + mhy*(cos(yaw)*sin(pitch)*sin(roll)-cos(roll)*sin(yaw)) + mhz*(sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch));
//   // // mwy = mhx*cos(pitch)*sin(yaw) + mhy*(cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw)) + mhz*(cos(roll)*sin(pitch)*sin(yaw)-cos(yaw)*sin(roll));
//   // // mwz = mhx*(-sin(pitch)) + mhy*cos(pitch)*sin(roll) + mhz*cos(pitch)*cos(roll);
//   // // mwx = mhx*(1-2*(q2*q2+q3*q3)) + mhy*2*(q1*q2+q0*q3) + mhz*2*(q1*q3-q0*q2);
//   // // mwy = mhx*2*(q1*q2-q0*q3) + mhy*(1-2*(q1*q1+q3*q3)) + mhz*2*(q2*q3+q0*q1);
//   // // mwz = mhx*2*(q1*q3+q0*q2) + mhy*2*(q2*q3-q0*q1) + mhz*(1-2*(q1*q1+q2*q2));
//   // //构造伪磁场测量量，磁强计的测量值不影响横滚和俯仰值


//   // //地磁向量提取
//   // mwx = 1-2*(q2*q2+q3*q3);
//   // mwy = 2*(q1*q2-q0*q3);
//   // mwz = 2*(q1*q3+q0*q2);
//   // // //磁场向量与实测磁场叉乘得到误差
//   // mag_ex = (mpy*mwz - mpz*mwy);
//   // mag_ey = (mpz*mwx - mpx*mwz);
//   // mag_ez = (mpx*mwy - mpy*mwx);
//   // temp1 = mag_ex;
//   // temp2 = mag_ey;
//   // temp3 = mag_ez;
//   // // //误差积分+比例，校正z轴角速度值
//   // mag_exInt = mag_exInt + mag_ex * Ki_mag;
//   // mag_eyInt = mag_eyInt + mag_ey * Ki_mag;
//   // mag_ezInt = mag_ezInt + mag_ez * Ki_mag;
//   // gx = gx + Kp_mag*mag_ex + mag_exInt;
//   // gy = gy + Kp_mag*mag_ey + mag_eyInt;
//   // gz = gz + Kp_mag*mag_ez + mag_ezInt;
//   // // //四元数迭代
//   // q0_old=q0;
//   // q1_old=q1;
//   // q2_old=q2;
//   // q3_old=q3;
//   // // //四元数更新
//   // q0 = q0_old + (-q1_old*gx - q2_old*gy - q3_old*gz) * 0.5f * dT;
//   // q1 = q1_old + (q0_old*gx + q2_old*gz -q3_old*gy) * 0.5f * dT;
//   // q2 = q2_old + (q0_old*gy - q1_old*gz +q3_old*gx) * 0.5f * dT;
//   // q3 = q3_old + (q0_old*gz + q1_old*gy -q2_old*gx) * 0.5f * dT;
//   // // //四元数归一化
//   // quaternion_norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//   // q0 = q0 / quaternion_norm;
//   // q1 = q1 / quaternion_norm;
//   // q2 = q2 / quaternion_norm;
//   // q3 = q3 / quaternion_norm;
//   // // 输出结果
//   // t_attitude.yaw = atan2(2*(q1*q2+q0*q3),1-2*(q2*q2+q3*q3));
//   // t_attitude.yaw = atan2(mpy,mpx);
//   t_attitude.yaw = pseudo_yaw;

//   // //磁强计数据单位化
//   // mag_norm = sqrt(mx*mx + my*my + mz*mz);
//   // mx = mx / mag_norm;
//   // my = my / mag_norm;
//   // mz = mz / mag_norm;
//   // //将三轴磁强计测量值投影至水平面(b系到n系)(四元数已经被加速度计修正过)
//   // mhx_n = mx*(1-2*(q2*q2+q3*q3)) + my*2*(q1*q2-q0*q3) + mz*2*(q1*q3+q0*q2);
//   // mhy_n = mx*2*(q1*q2+q0*q3) + my*(1-2*(q1*q1+q3*q3)) + mz*2*(q2*q3-q0*q1);
//   // mhz_n = mx*2*(q1*q3-q0*q2) + my*2*(q2*q3+q0*q1) + mz*(1-2*(q1*q1+q2*q2));
//   // //磁力计在XOY平面上(n系)的向量大小必定相同
//   // // mhx = sqrt(mhx*mhx + mhy*mhy);
//   // // mhy = 0.0f;
//   // // mhz = 0.0f;
//   // //将其再投影至b系(n系到b系)
//   // // mwx = mhx*(1-2*(q2*q2+q3*q3)) + mhy*2*(q1*q2+q0*q3) + mhz*2*(q1*q3-q0*q2);
//   // // mwy = mhx*2*(q1*q2-q0*q3) + mhy*(1-2*(q1*q1+q3*q3)) + mhz*2*(q2*q3+q0*q1);
//   // // mwz = mhx*2*(q1*q3+q0*q2) + mhy*2*(q2*q3-q0*q1) + mhz*(1-2*(q1*q1+q2*q2));

//   // mwx_n = (1-2*(q2*q2+q3*q3));
//   // mwy_n = 2*(q1*q2+q0*q3);
//   // mwz_n = 2*(q1*q3-q0*q2);
//   // //磁场向量与实测磁场叉乘得到误差
//   // mag_ex = (mhy*mwz - mhz*mwy) ;
//   // mag_ey = (mhz*mwx - mhx*mwz) ;
//   // mag_ez = (mhx*mwy - mhy*mwx) ;
//   // // mag_ex = (my*mwz - mz*mwy) ;
//   // // mag_ey = (mz*mwx - mx*mwz) ;
//   // // mag_ez = (mx*mwy - my*mwx) ;
//   // //误差积分+比例，校正z轴角速度值
//   // mag_exInt = mag_exInt + mag_ex * Ki_mag;
//   // mag_eyInt = mag_eyInt + mag_ey * Ki_mag;
//   // mag_ezInt = mag_ezInt + mag_ez * Ki_mag;
//   // gx = gx + Kp_mag*mag_ex + mag_exInt;
//   // gy = gy + Kp_mag*mag_ey + mag_eyInt;
//   // gz = gz + Kp_mag*mag_ez + mag_ezInt;
//   // //四元数迭代
//   // q0_old=q0;
//   // q1_old=q1;
//   // q2_old=q2;
//   // q3_old=q3;
//   // //四元数更新
//   // q0 = q0_old + (-q1_old*gx - q2_old*gy - q3_old*gz) * 0.5f * dT;
//   // q1 = q1_old + (q0_old*gx + q2_old*gz -q3_old*gy) * 0.5f * dT;
//   // q2 = q2_old + (q0_old*gy - q1_old*gz +q3_old*gx) * 0.5f * dT;
//   // q3 = q3_old + (q0_old*gz + q1_old*gy -q2_old*gx) * 0.5f * dT;
//   // //四元数归一化
//   // quaternion_norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//   // q0 = q0 / quaternion_norm;
//   // q1 = q1 / quaternion_norm;
//   // q2 = q2 / quaternion_norm;
//   // q3 = q3 / quaternion_norm;
//   // //输出结果
//   // t_attitude.roll = atan2(2*(q2*q3-q0*q1),1-2*(q1*q1+q2*q2));
//   // t_attitude.pitch = -asin(2*(q1*q3+q0*q2));
//   // t_attitude.yaw = atan2(2*(q1*q2-q0*q3),1-2*(q2*q2+q3*q3));

//   // t_pitch.acc_navigation = ax*(1-2*(q2*q2+q3*q3)) + 2*ay*(q1*q2-q0*q3) + 2*az*(q1*q3+q0*q2);
//   // t_roll.acc_navigation = 2*ax*(q1*q2+q0*q3) + ay*(1-2*(q1*q1+q3*q3)) + 2*az*(q2*q3-q0*q1);
//   // t_yaw.acc_navigation = 2*ax*(q1*q3-q0*q2) + 2*ay*(q2*q3+q0*q1) + az*(1-2*(q1*q1+q2*q2));
//   // //角度单位弧度转度
//   // t_pitch.euler_deg = t_pitch.euler_rad * RAD_TO_DEG;
//   // t_roll.euler_deg = t_roll.euler_rad * RAD_TO_DEG;
//   // t_yaw.euler_deg = t_yaw.euler_rad * RAD_TO_DEG;
// }


// // void ins_complementary_update(float dT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
// // {
// //   float Kp_acc=15.0f;
// //   float Ki_acc=0.008f;

// //   float Kp_mag=5.0f;
// //   float Ki_mag=0.003f;

// //   float yaw_threshold = 10.0f;

// //   static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
// //   float q0_old,q1_old,q2_old,q3_old;
// //   float quaternion_norm;
// //   /***************************************重力校正姿态*************************************************/
// //   static float acc_exInt = 0, acc_eyInt = 0, acc_ezInt = 0;
// //   float acc_norm;
// //   float ax_g, ay_g, az_g;
// //   float acc_vx, acc_vy, acc_vz;
// //   float acc_ex, acc_ey, acc_ez;

// //   acc_norm = sqrt(ax*ax + ay*ay + az*az);
// //   if(acc_norm==0) return;               //传感器未读取数据时，acc_norm为0,此时会出现NAN错误
// //   ax_g = ax / acc_norm;
// //   ay_g = ay / acc_norm;
// //   az_g = az / acc_norm;
// //   //重力向量提取
// //   acc_vx = 2*(q1*q3 - q0*q2);
// //   acc_vy = 2*(q0*q1 + q2*q3);
// //   acc_vz = q0*q0 - q1*q1 - q2*q2 + q3*q3 ;
// //   //重量向量与实测重力值叉乘得到误差
// //   acc_ex = (ay_g*acc_vz - az_g*acc_vy) ;
// //   acc_ey = (az_g*acc_vx - ax_g*acc_vz) ;
// //   acc_ez = (ax_g*acc_vy - ay_g*acc_vx) ;
// //   //误差积分+比例，校正x和y角速度值
// //   acc_exInt = acc_exInt + acc_ex * Ki_acc;
// //   acc_eyInt = acc_eyInt + acc_ey * Ki_acc;
// //   acc_ezInt = acc_ezInt + acc_ez * Ki_acc;
// //   gx = gx + Kp_acc*acc_ex + acc_exInt;
// //   gy = gy + Kp_acc*acc_ey + acc_eyInt;
// //   // if(fabs(t_pitch.euler_deg)>=yaw_threshold || fabs(t_roll.euler_deg)>=yaw_threshold)
// //   // {
// //   //   gz = gz + Kp_acc*acc_ez + acc_ezInt; //重力向量只能校正x，y轴的角速度值
// //   // }
  

// //   /***************************************地磁场校正姿态*************************************************/
// //   static float mag_exInt = 0, mag_eyInt = 0, mag_ezInt = 0;
// //   float mag_norm;
// //   float mhx,mhy,mhz;
// //   float mag_vx, mag_vy, mag_vz;
// //   float mag_ex, mag_ey, mag_ez;
// //   //将三轴磁强计测量值投影至水平面
// //   mhx = mx*cos(t_roll.euler_rad) + my*sin(t_roll.euler_rad)*sin(t_pitch.euler_rad) + mz*sin(t_roll.euler_rad)*cos(t_pitch.euler_rad);
// //   mhy = my*cos(t_pitch.euler_rad) - mz*sin(t_pitch.euler_rad);
// //   // mhz = -mx*sin(t_roll.euler_rad) + my*cos(t_roll.euler_rad)*sin(t_pitch.euler_rad) + mz*cos(t_roll.euler_rad)*cos(t_pitch.euler_rad);

// //   mag_norm = sqrt(mhx*mhx + mhy*mhy);
// //   mhx = mhx / mag_norm;
// //   mhy = mhy / mag_norm;
// //   mhz = 0;
// //   //磁场向量提取
// //   mag_vx = 2*(q1*q2 + q0*q3);
// //   mag_vy = 1-2*(q1*q1+q3*q3);
// //   mag_vz = 2*(q2*q3 - q0*q1);
// //   //磁场向量与实测磁场叉乘得到误差
// //   mag_ex = (mhy*mag_vz - mhz*mag_vy) ;
// //   mag_ey = (mhz*mag_vx - mhx*mag_vz) ;
// //   mag_ez = (mhx*mag_vy - mhy*mag_vx) ;
// //   //误差积分+比例，校正z轴角速度值
// //   mag_exInt = mag_exInt + mag_ex * Ki_mag;
// //   mag_eyInt = mag_eyInt + mag_ey * Ki_mag;
// //   mag_ezInt = mag_ezInt + mag_ez * Ki_mag;

// //   // gx = gx + Kp_mag*mag_ex + mag_exInt;
// //   // gy = gy + Kp_mag*mag_ey + mag_eyInt;
// //   if(fabs(t_pitch.euler_deg)<yaw_threshold || fabs(t_roll.euler_deg)<yaw_threshold)
// //   {
// //     gz = gz + Kp_mag*mag_ez + mag_ezInt;
// //   }
// //   //四元数迭代
// //   q0_old=q0;
// //   q1_old=q1;
// //   q2_old=q2;
// //   q3_old=q3;
// //   //四元数更新
// //   q0 = q0_old + (-q1_old*gx - q2_old*gy - q3_old*gz) * 0.5f * dT;
// //   q1 = q1_old + (q0_old*gx + q2_old*gz -q3_old*gy) * 0.5f * dT;
// //   q2 = q2_old + (q0_old*gy - q1_old*gz +q3_old*gx) * 0.5f * dT;
// //   q3 = q3_old + (q0_old*gz + q1_old*gy -q2_old*gx) * 0.5f * dT;
// //   //四元数归一化
// //   quaternion_norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
// //   q0 = q0 / quaternion_norm;
// //   q1 = q1 / quaternion_norm;
// //   q2 = q2 / quaternion_norm;
// //   q3 = q3 / quaternion_norm;
// //   //输出结果
// //   t_pitch.euler_rad = atan2(2*(q2*q3-q0*q1),1-2*(q1*q1+q2*q2));
// //   t_roll.euler_rad = -asin(2*(q1*q3+q0*q2));
// //   t_yaw.euler_rad = atan2(2*(q1*q2-q0*q3),1-2*(q2*q2+q3*q3));

// //   t_pitch.acc_navigation = ax*(1-2*(q2*q2+q3*q3)) + 2*ay*(q1*q2-q0*q3) + 2*az*(q1*q3+q0*q2);
// //   t_roll.acc_navigation = 2*ax*(q1*q2+q0*q3) + ay*(1-2*(q1*q1+q3*q3)) + 2*az*(q2*q3-q0*q1);
// //   t_yaw.acc_navigation = 2*ax*(q1*q3-q0*q2) + 2*ay*(q2*q3+q0*q1) + az*(1-2*(q1*q1+q2*q2));
// //   //角度单位弧度转度
// //   t_pitch.euler_deg = t_pitch.euler_rad * RAD_TO_DEG;
// //   t_roll.euler_deg = t_roll.euler_rad * RAD_TO_DEG;
// //   t_yaw.euler_deg = t_yaw.euler_rad * RAD_TO_DEG;
// // }

// // mnx = mx*cos(pitch)*cos(yaw) + my*cos(pitch)*sin(yaw) - mz*sin(pitch);
// // mny = mx*(cos(yaw)*sin(pitch)*sin(roll)-cos(roll)*sin(yaw)) + my*(cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw)) + mz*cos(pitch)*sin(roll);
// // mnz = mx*(sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch)) + my*(cos(roll)*sin(pitch)*sin(yaw)-cos(yaw)*sin(roll)) + mz*cos(pitch)*cos(roll);
