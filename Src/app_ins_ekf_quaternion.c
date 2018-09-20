#include "app_ins_ekf_quaternion.h"
#include "mpu9250.h"
#include "CMatrix.h"
#include <math.h>
#include <stdlib.h>

//全局变量
float roll_deg,pitch_deg,yaw_deg;
float quat0,quat1,quat2,quat3;
float pos_north,pos_east,pos_alt;
float vel_north,pos_east,vel_alt;
float bias_gyrox,bias_gyroy,bias_gyroz;
float bias_accx,bias_accy,bias_accz;
//私有变量
MatDataType gravity_mps2 = 9.81;	//重力加速度
MatDataType mag_declination_deg = 0;	//磁偏角

MatDataType gyrox_rps,gyroy_rps,gyroz_rps;  //rad per second
MatDataType accx_mps,accy_mps,accz_mps; //meter per second
MatDataType magx_ut,magy_ut,magz_ut;    //uT

MatDataType pos_north = 0.0f;
MatDataType pos_east = 0.0f;
MatDataType pos_alt = 0.0f;

MatDataType vel_north = 0.0f;
MatDataType vel_east = 0.0f;
MatDataType vel_down = 0.0f;
//EKF相关
MatDataType large_quad_uncertainty = 0.1;
MatDataType large_pos_uncertainty_m = 100.0f;
MatDataType large_vel_uncertainty_mps = 10.0f;
MatDataType ekf_sigmas_gyro_bias_rps = 0.01;
MatDataType ekf_sigmas_accel_bias_mps2 = 0.1;

MatDataType ekf_sigmas_quad_process_noise = 0.001;
MatDataType ekf_sigmas_pos_process_noise_m = 0.001;
MatDataType ekf_sigmas_vel_process_noise_mps = 2;
MatDataType ekf_sigmas_gyroBias_process_noise_rps = 1e-6;
MatDataType ekf_sigmas_accelBias_process_noise_mps2 = 1e-6;

MatDataType ekf_sigmas_mag3D_unitVector_meas = 1.0f;
MatDataType ekf_sigmas_pos_meas_m = 1.0f;
MatDataType ekf_sigmas_vel_meas_m = 1.0f;

Mat ekf_xhat;
Mat ekf_P;


int app_ins_ekf_quaternion_thread(float dt_s)
{
    obtain_sensors_data();
    ekf_basedon_quaternion(dt_s,gyrox_rps,gyroy_rps,gyroz_rps,accx_mps,accy_mps,accz_mps,magx_ut,magy_ut,magz_ut,\
    pos_north,pos_east,pos_alt,vel_north,vel_east,vel_down);
    return 0;
}

int obtain_sensors_data(void)
{
    accx_mps = accx_raw_mps;    
    accy_mps = accy_raw_mps;    
    accz_mps = accz_raw_mps;

    gyrox_rps = gyrox_raw_dps * DEG_TO_RAD; 
    gyroy_rps = gyroy_raw_dps * DEG_TO_RAD; 
    gyroz_rps = gyroz_raw_dps * DEG_TO_RAD; 

    magx_ut = magx_raw_uT;
    magy_ut = magy_raw_uT;
    magz_ut = magz_raw_uT;

    pos_north = 0.0f;
    pos_east = 0.0f;
    pos_alt = 0.0f;

    vel_north = 0.0f;
    vel_east = 0.0f;
    vel_down = 0.0f;

    return 0;
}

int ekf_basedon_quaternion(float dt_s,
                           float wx, float wy, float wz, float fx, float fy, float fz, float mx, float my, float mz,
                           float pos_north, float pos_east, float pos_alt, float vel_north, float vel_east, float vel_down)
{
    static int RunOnce = 0;
    if(RunOnce==0)
    {
        //状态初始化
        MatCreate(&ekf_xhat, 16, 1);
        MatDataType ekf_xhat_init[16] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        MatInit(&ekf_xhat, ekf_xhat_init);
        //P矩阵初始化
        MatCreate(&ekf_P, 16, 16);
        MatDataType ekf_P_init[16] = {large_quad_uncertainty, large_quad_uncertainty, large_quad_uncertainty, large_quad_uncertainty,
                                      large_pos_uncertainty_m, large_pos_uncertainty_m, large_pos_uncertainty_m,
                                      large_vel_uncertainty_mps, large_vel_uncertainty_mps, large_vel_uncertainty_mps,
                                      ekf_sigmas_gyro_bias_rps, ekf_sigmas_gyro_bias_rps, ekf_sigmas_gyro_bias_rps,
                                      ekf_sigmas_accel_bias_mps2, ekf_sigmas_accel_bias_mps2, ekf_sigmas_accel_bias_mps2};
        MatDiag(&ekf_P, ekf_P_init, 16);

        RunOnce = 1;
    }
    Mat ekf_Q;
    Mat ekf_R;
    //Q矩阵初始化
    MatCreate(&ekf_Q, 16, 16);
    MatDataType ekf_Q_init[16] = {ekf_sigmas_quad_process_noise, ekf_sigmas_quad_process_noise, ekf_sigmas_quad_process_noise, ekf_sigmas_quad_process_noise,
                                  ekf_sigmas_pos_process_noise_m, ekf_sigmas_pos_process_noise_m, ekf_sigmas_pos_process_noise_m,
                                  ekf_sigmas_vel_process_noise_mps, ekf_sigmas_vel_process_noise_mps, ekf_sigmas_vel_process_noise_mps,
                                  ekf_sigmas_gyroBias_process_noise_rps, ekf_sigmas_gyroBias_process_noise_rps, ekf_sigmas_gyroBias_process_noise_rps,
                                  ekf_sigmas_accelBias_process_noise_mps2, ekf_sigmas_accelBias_process_noise_mps2, ekf_sigmas_accelBias_process_noise_mps2};
    MatDiag(&ekf_Q, ekf_Q_init, 16);
    //R矩阵初始化
    MatCreate(&ekf_R, 9, 9);
    MatDataType ekf_R_init[9] = {
        ekf_sigmas_pos_meas_m, ekf_sigmas_pos_meas_m, ekf_sigmas_pos_meas_m,\
        ekf_sigmas_pos_meas_m, ekf_sigmas_pos_meas_m, ekf_sigmas_pos_meas_m,\
        ekf_sigmas_vel_meas_m, ekf_sigmas_vel_meas_m, ekf_sigmas_vel_meas_m};
    MatDiag(&ekf_R, ekf_R_init, 9);
        
    MatDataType q0, q1, q2, q3;
    MatDataType Vn, Ve, Vd;
    MatDataType bwx, bwy, bwz;
    MatDataType bax, bay, baz;

    q0 = MatAt(&ekf_xhat, 0, 0), q1 = MatAt(&ekf_xhat, 1, 0), q2 = MatAt(&ekf_xhat, 2, 0), q3 = MatAt(&ekf_xhat, 3, 0);
    Vn = MatAt(&ekf_xhat, 7, 0), Ve = MatAt(&ekf_xhat, 8, 0), Vd = MatAt(&ekf_xhat, 9, 0);
    bwx = MatAt(&ekf_xhat, 10, 0), bwy = MatAt(&ekf_xhat, 11, 0), bwz = MatAt(&ekf_xhat, 12, 0);
    bax = MatAt(&ekf_xhat, 13, 0), bay = MatAt(&ekf_xhat, 14, 0), baz = MatAt(&ekf_xhat, 15, 0);
    int nStates = 16;

    //角速度测量值
    Mat Wxyz;	MatCreate(&Wxyz, 3, 1);
    MatDataType Wxyz_data[3] = { wx,wy,wz };
    MatInit(&Wxyz, Wxyz_data);
    //加表测量值
    Mat Fxyz;	MatCreate(&Fxyz, 3, 1);
    MatDataType Fxyz_data[3] = { fx,fy,fz };
    MatInit(&Fxyz, Fxyz_data);
    //陀螺仪三轴偏置
    Mat Bw;	MatCreate(&Bw, 3, 1);
    MatDataType Bw_data[3] = { bwx,bwy,bwz };
    MatInit(&Bw, Bw_data);
    //加表三轴偏置
    Mat Bf;	MatCreate(&Bf, 3, 1);
    MatDataType Bf_data[3] = { bax,bay,baz };
    MatInit(&Bf, Bf_data);
    //重力向量
    Mat Gravity;	MatCreate(&Gravity, 3, 1);
    MatDataType Gravity_data[3] = { 0.0f,0.0f,gravity_mps2 };
    MatInit(&Gravity, Gravity_data);
    //四元数微分方程
    Mat C_bodyrate2qdot;
    MatCreate(&C_bodyrate2qdot, 4, 3);
    MatDataType C_bodyrate2qdot_data[12] = {-0.5 * q1, -0.5 * q2, -0.5 * q3,
                                            0.5 * q0, -0.5 * q3, 0.5 * q2,
                                            0.5 * q3, 0.5 * q0, -0.5 * q1,
                                            -0.5 * q2, 0.5 * q1, 0.5 * q0};
    MatInit(&C_bodyrate2qdot, C_bodyrate2qdot_data);
    //计算旋转矩阵C_ned2b
    Mat C_ned2b;
    MatCreate(&C_ned2b, 3, 3);
    MatDataType C_ned2b_data[9] = {1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 + q3 * q0), 2 * (q1 * q3 - q2 * q0),
                                   2 * (q1 * q2 - q3 * q0), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 + q1 * q0),
                                   2 * (q1 * q3 + q2 * q0), 2 * (q2 * q3 - q1 * q0), 1 - 2 * (q1 * q1 + q2 * q2)};
    MatInit(&C_ned2b, C_ned2b_data);
    //计算旋转矩阵的逆C_b2ned
    Mat C_b2ned;
    MatCreate(&C_b2ned, 3, 3);
    MatZeros(&C_b2ned);
    MatTrans(&C_b2ned, &C_ned2b);

    //计算Xdot
    Mat Xdot;
    MatCreate(&Xdot, 16, 1);

    Mat Xdot_part1;
    MatCreate(&Xdot_part1, 4, 1);
    Mat Xdot_part1_aux1;
    MatCreate(&Xdot_part1_aux1, 3, 1);
    MatSub(&Xdot_part1_aux1, &Wxyz, &Bw);
    MatMultiply(&Xdot_part1, &C_bodyrate2qdot, &Xdot_part1_aux1);

    Mat Xdot_part2;
    MatCreate(&Xdot_part2, 3, 1);
    MatDataType Xdot_part2_data[3] = {Vn, Ve, -Vd};
    MatInit(&Xdot_part2, Xdot_part2_data);

    Mat Xdot_part3;
    MatCreate(&Xdot_part3, 3, 1);
    Mat Xdot_part3_aux1;
    MatCreate(&Xdot_part3_aux1, 3, 1);
    MatSub(&Xdot_part3_aux1, &Fxyz, &Bf);
    Mat Xdot_part3_aux2;
    MatCreate(&Xdot_part3_aux2, 3, 1);
    MatMultiply(&Xdot_part3_aux2, &C_b2ned, &Xdot_part3_aux1);
    MatAdd(&Xdot_part3, &Xdot_part3_aux2, &Gravity);

    Mat Xdot_part4;
    MatCreate(&Xdot_part4, 3, 1);
    MatZeros(&Xdot_part4);

    Mat Xdot_part5;
    MatCreate(&Xdot_part5, 3, 1);
    MatZeros(&Xdot_part5);

    Mat Xdot_blocks[] = {Xdot_part1, Xdot_part2, Xdot_part3, Xdot_part4, Xdot_part5};
    MatBlockCompose(&Xdot, (Mat*)&Xdot_blocks, 5, 1);
    //F
    Mat F;
    MatCreate(&F, 16, 16);
    MatDataType F_data[256] = {
        0, bwx / 2 - wx / 2, bwy / 2 - wy / 2, bwz / 2 - wz / 2, 0, 0, 0, 0, 0, 0, q1 / 2, q2 / 2, q3 / 2, 0, 0, 0,
        wx / 2 - bwx / 2, 0, wz / 2 - bwz / 2, bwy / 2 - wy / 2, 0, 0, 0, 0, 0, 0, -q0 / 2, q3 / 2, -q2 / 2, 0, 0, 0,
        wy / 2 - bwy / 2, bwz / 2 - wz / 2, 0, wx / 2 - bwx / 2, 0, 0, 0, 0, 0, 0, -q3 / 2, -q0 / 2, q1 / 2, 0, 0, 0,
        wz / 2 - bwz / 2, wy / 2 - bwy / 2, bwx / 2 - wx / 2, 0, 0, 0, 0, 0, 0, 0, q2 / 2, -q1 / 2, -q0 / 2, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
        2 * q3 * (bay - fy) - 2 * q0 * (bax - fx) - 2 * q2 * (baz - fz), -2 * q1 * (bax - fx) - 2 * q2 * (bay - fy) - 2 * q3 * (baz - fz), 2 * q2 * (bax - fx) - 2 * q1 * (bay - fy) - 2 * q0 * (baz - fz), 2 * q3 * (bax - fx) + 2 * q0 * (bay - fy) - 2 * q1 * (baz - fz), 0, 0, 0, 0, 0, 0, 0, 0, 0, -q0 * q0 - q1 * q1 + q2 * q2 + q3 * q3, 2 * q0 * q3 - 2 * q1 * q2, -2 * q0 * q2 - 2 * q1 * q3,
        2 * q1 * (baz - fz) - 2 * q0 * (bay - fy) - 2 * q3 * (bax - fx), 2 * q1 * (bay - fy) - 2 * q2 * (bax - fx) + 2 * q0 * (baz - fz), -2 * q1 * (bax - fx) - 2 * q2 * (bay - fy) - 2 * q3 * (baz - fz), 2 * q3 * (bay - fy) - 2 * q0 * (bax - fx) - 2 * q2 * (baz - fz), 0, 0, 0, 0, 0, 0, 0, 0, 0, -2 * q0 * q3 - 2 * q1 * q2, -q0 * q0 + q1 * q1 - q2 * q2 + q3 * q3, 2 * q0 * q1 - 2 * q2 * q3,
        2 * q2 * (bax - fx) - 2 * q1 * (bay - fy) - 2 * q0 * (baz - fz), 2 * q1 * (baz - fz) - 2 * q0 * (bay - fy) - 2 * q3 * (bax - fx), 2 * q0 * (bax - fx) - 2 * q3 * (bay - fy) + 2 * q2 * (baz - fz), -2 * q1 * (bax - fx) - 2 * q2 * (bay - fy) - 2 * q3 * (baz - fz), 0, 0, 0, 0, 0, 0, 0, 0, 0, 2 * q0 * q2 - 2 * q1 * q3, -2 * q0 * q1 - 2 * q2 * q3, -q0 * q0 + q1 * q1 + q2 * q2 - q3 * q3,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    MatInit(&F, F_data);
    Mat Ft;
    MatCreate(&Ft, 16, 16);
    MatTrans(&Ft, &F);
    //PHI_K & Q_K
    Mat AA;
    MatCreate(&AA, 32, 32);
    Mat AA_Zeros16;
    MatCreate(&AA_Zeros16, 16, 16);
    MatZeros(&AA_Zeros16);
    Mat AA_a;
    MatCreate(&AA_a, 16, 16);
    MatCopy(&AA_a, MatScalarMultiply(&F, -1));
    Mat AA_b;
    MatCreate(&AA_b, 16, 16);
    MatCopy(&AA_b, &ekf_Q);
    Mat AA_c;
    MatCreate(&AA_c, 16, 16);
    MatCopy(&AA_c, &AA_Zeros16);
    Mat AA_d;
    MatCreate(&AA_d, 16, 16);
    MatCopy(&AA_d, &Ft);
    Mat AA_blocks[] = {AA_a, AA_b, AA_c, AA_d};
    MatBlockCompose(&AA, (Mat*)&AA_blocks, 2, 2);
    MatScalarMultiply(&AA, dt_s);

    Mat BB;
    MatCreate(&BB, 32, 32);
    MatZeros(&BB);
    Mat BB_EYE32;
    MatCreate(&BB_EYE32, 32, 32);
    MatEye(&BB_EYE32);
    MatAdd(&BB, &BB_EYE32, &AA);

    Mat PHI_K;
    MatCreate(&PHI_K, 16, 16);
    MatZeros(&PHI_K);
    Mat PHI_Kt;
    MatCreate(&PHI_Kt, 16, 16);
    MatZeros(&PHI_Kt);
    MatBlockDecompose(&PHI_Kt, &BB, nStates, nStates, nStates * 2 - 1, nStates * 2 - 1);
    MatTrans(&PHI_K, &PHI_Kt);

    Mat BB_rightup;
    MatCreate(&BB_rightup, 16, 16);
    MatBlockDecompose(&BB_rightup, &BB, 0, nStates, nStates - 1, nStates * 2 - 1);
    Mat Q_K;
    MatCreate(&Q_K, 16, 16);
    MatMultiply(&Q_K, &PHI_K, &BB_rightup);
    //一步预测
    MatAdd(&ekf_xhat, &ekf_xhat, MatScalarMultiply(&Xdot, dt_s));
    //一步预测均方误差
    Mat ekf_Pp_aux1;
    MatCreate(&ekf_Pp_aux1, 16, 16);
    Mat ekf_Pp_aux2;
    MatCreate(&ekf_Pp_aux2, 16, 16);
    MatMultiply(&ekf_Pp_aux1, &PHI_K, &ekf_P);        //PHI_K*ekf_P
    MatMultiply(&ekf_Pp_aux2, &ekf_Pp_aux1, &PHI_Kt); //PHI_K*ekf_P*PHI_K'
    MatAdd(&ekf_P, &ekf_Pp_aux2, &Q_K);
    //磁偏角矩阵
    Mat C_mag2ned;
    MatCreate(&C_mag2ned, 3, 3);
    //MatDataType C_mag2ned_data[9] = {
    //	cos(-mag_declination_deg * DEG_TO_RAD),sin(-mag_declination_deg * DEG_TO_RAD),0,\
		//	- sin(-mag_declination_deg * DEG_TO_RAD),cos(-mag_declination_deg * DEG_TO_RAD),0,\
		//	0,0,1
    //};
    //MatInit(&C_mag2ned, C_mag2ned_data);
    MatEye(&C_mag2ned);
    //H
    Mat ekf_H;
    MatCreate(&ekf_H, 9, 16);
    MatDataType ekf_H_data[144] = 
    {
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0
    };
    MatInit(&ekf_H, ekf_H_data);
    Mat ekf_Ht;
    MatCreate(&ekf_Ht, 16, 9);
    MatTrans(&ekf_Ht, &ekf_H);
    //K
    Mat ekf_K;
    MatCreate(&ekf_K, 16, 9);
    Mat ekf_K_aux1;
    MatCreate(&ekf_K_aux1, 9, 16);
    MatMultiply(&ekf_K_aux1, &ekf_H, &ekf_P); //aux1 = H*P
    Mat ekf_K_aux2;
    MatCreate(&ekf_K_aux2, 9, 9);
    MatMultiply(&ekf_K_aux2, &ekf_K_aux1, &ekf_Ht); //aux2 = H*P*Ht
    Mat ekf_K_aux3;
    MatCreate(&ekf_K_aux3, 9, 9);
    MatAdd(&ekf_K_aux3, &ekf_K_aux2, &ekf_R); //aux3 = H*P*Ht+R
    Mat ekf_K_aux4;
    MatCreate(&ekf_K_aux4, 9, 9);
    MatInverse(&ekf_K_aux4, &ekf_K_aux3); //aux4 = inv(H*P*Ht+R)
    Mat ekf_K_aux5;
    MatCreate(&ekf_K_aux5, 16, 9);
    MatMultiply(&ekf_K_aux5, &ekf_P, &ekf_Ht); //aux5 = P*Ht
    MatMultiply(&ekf_K, &ekf_K_aux5, &ekf_K_aux4);
    //Z
    Mat ekf_Z;
    MatCreate(&ekf_Z, 9, 1);
    MatDataType ekf_Z_data[9] = {
        pos_north, pos_east, pos_alt, pos_north, pos_east, pos_alt,vel_north,vel_east,vel_down};
    MatInit(&ekf_Z, ekf_Z_data);
    //状态估计
    Mat ekf_xhat_aux1;
    MatCreate(&ekf_xhat_aux1, 9, 1);
    MatMultiply(&ekf_xhat_aux1, &ekf_H, &ekf_xhat); //H*X
    Mat ekf_xhat_aux2;
    MatCreate(&ekf_xhat_aux2, 9, 1);
    MatSub(&ekf_xhat_aux2, &ekf_Z, &ekf_xhat_aux1); //Z-H*X
    Mat ekf_xhat_aux3;
    MatCreate(&ekf_xhat_aux3, 16, 1);
    MatMultiply(&ekf_xhat_aux3, &ekf_K, &ekf_xhat_aux2); //K*(Z-H*X)
    MatAdd(&ekf_xhat, &ekf_xhat, &ekf_xhat_aux3);        //X+K*(Z-H*X)
    //状态均方误差更新
    Mat P_EYE16;
    MatCreate(&P_EYE16, 16, 16);
    MatEye(&P_EYE16); //I
    Mat ekf_Pe_aux1;
    MatCreate(&ekf_Pe_aux1, 16, 16);
    MatMultiply(&ekf_Pe_aux1, &ekf_K, &ekf_H); //KH
    Mat ekf_Pe_aux2;
    MatCreate(&ekf_Pe_aux2, 16, 16);
    MatSub(&ekf_Pe_aux2, &P_EYE16, &ekf_Pe_aux1); //I-KH
    Mat ekf_Pe_aux3;
    MatCreate(&ekf_Pe_aux3, 16, 16);
    MatMultiply(&ekf_Pe_aux3, &ekf_Pe_aux2, &ekf_P); //(I-KH)*P
    MatCopy(&ekf_P, &ekf_Pe_aux3);
    //四元数归一化
    q0 = MatAt(&ekf_xhat, 0, 0);
    q1 = MatAt(&ekf_xhat, 1, 0);
    q2 = MatAt(&ekf_xhat, 2, 0);
    q3 = MatAt(&ekf_xhat, 3, 0);
    q0 = q0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q1 = q1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q2 = q2 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q3 = q3 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    MatSetValAt(&ekf_xhat, 0, 0, q0);
    MatSetValAt(&ekf_xhat, 1, 0, q1);
    MatSetValAt(&ekf_xhat, 2, 0, q2);
    MatSetValAt(&ekf_xhat, 3, 0, q3);

    //输出结果
    roll_deg = 180 / PI * atan2(2*(q2*q3 + q1*q0), 1-2*(q1*q1 + q2*q2));
    pitch_deg = 180 / PI * asin(-2*(q1*q3 - q2*q0));
    yaw_deg = 180 / PI * atan2(2*(q1*q2 + q3*q0), 1-2*(q2*q2 + q3*q3)); // -180 <= yaw <= 180

    //回收变量
    MatDelete(&ekf_Q);
    MatDelete(&ekf_R);
    MatDelete(&Wxyz);
    MatDelete(&Fxyz);
    MatDelete(&Xdot_part1_aux1);
    MatDelete(&Bf);
    MatDelete(&Bw);
    MatDelete(&Gravity);
    MatDelete(&C_bodyrate2qdot);
    MatDelete(&C_b2ned);
    MatDelete(&C_ned2b);
    MatDelete(&Xdot_part1);
    MatDelete(&Xdot_part2);
    MatDelete(&Xdot_part3);
    MatDelete(&Xdot_part3_aux1);
    MatDelete(&Xdot_part3_aux2);
    MatDelete(&Xdot_part4);
    MatDelete(&Xdot_part5);
    MatDelete(&Xdot);
    MatDelete(&F);
    MatDelete(&Ft);
    MatDelete(&AA_Zeros16);
    MatDelete(&AA_a);
    MatDelete(&AA_b);
    MatDelete(&AA_c);
    MatDelete(&AA_d);
    MatDelete(&AA);
    MatDelete(&BB);
    MatDelete(&BB_EYE32);
    MatDelete(&PHI_K);
    MatDelete(&PHI_Kt);
    MatDelete(&BB_rightup);
    MatDelete(&Q_K);
    MatDelete(&ekf_Pp_aux1);
    MatDelete(&ekf_Pp_aux2);
    MatDelete(&C_mag2ned);
    MatDelete(&ekf_H);
    MatDelete(&ekf_Ht);
    MatDelete(&ekf_K);
    MatDelete(&ekf_K_aux1);
    MatDelete(&ekf_K_aux2);
    MatDelete(&ekf_K_aux3);
    MatDelete(&ekf_K_aux4);
    MatDelete(&ekf_K_aux5);
    MatDelete(&ekf_Z);
    MatDelete(&ekf_xhat_aux1);
    MatDelete(&ekf_xhat_aux2);
    MatDelete(&ekf_xhat_aux3);
    MatDelete(&P_EYE16);
    MatDelete(&ekf_Pe_aux1);
    MatDelete(&ekf_Pe_aux2);
    MatDelete(&ekf_Pe_aux3);
    return 0;
}