#include "app_ins_ekf_quaternion.h"
#include "mpu9250.h"
#include "CMatrix.h"
#include <math.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "gps_m8n.h"
#include "mpu9250.h"

extern osMessageQId myQueue02GPSM8NToInsHandle;
extern osMessageQId myQueue03MPU9250ToInsHandle;
osEvent gps_m8n_evt;
osEvent mpu9250_evt;

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
    gps_m8n_evt = osMessageGet(myQueue02GPSM8NToInsHandle,0);
    if(gps_m8n_evt.status == osEventMessage)
    {
        pos_north = (*((_GPS_M8N*)gps_m8n_evt.value.v)).pos_north;
        pos_east = (*((_GPS_M8N*)gps_m8n_evt.value.v)).pos_east;
        pos_alt = (*((_GPS_M8N*)gps_m8n_evt.value.v)).alt;
    }
    
    mpu9250_evt = osMessageGet(myQueue03MPU9250ToInsHandle,0);
    if(mpu9250_evt.status == osEventMessage)
    {
        accx_mps = (*((_MPU9250*)mpu9250_evt.value.v)).accx_raw_mps;    
        accy_mps = (*((_MPU9250*)mpu9250_evt.value.v)).accy_raw_mps;    
        accz_mps = (*((_MPU9250*)mpu9250_evt.value.v)).accz_raw_mps;

        gyrox_rps = (*((_MPU9250*)mpu9250_evt.value.v)).gyrox_raw_dps * DEG_TO_RAD; 
        gyroy_rps = (*((_MPU9250*)mpu9250_evt.value.v)).gyroy_raw_dps * DEG_TO_RAD; 
        gyroz_rps = (*((_MPU9250*)mpu9250_evt.value.v)).gyroz_raw_dps * DEG_TO_RAD; 

        magx_ut = (*((_MPU9250*)mpu9250_evt.value.v)).magx_raw_uT;
        magy_ut = (*((_MPU9250*)mpu9250_evt.value.v)).magy_raw_uT;
        magz_ut = (*((_MPU9250*)mpu9250_evt.value.v)).magz_raw_uT;
    }
    return 0;
}


//状态维数
#define NSTATES 16
//量测维数
#define NMEASURES  9
int ekf_basedon_quaternion(float dt_s,
                           float wx, float wy, float wz, float fx, float fy, float fz, float mx, float my, float mz,
                           float pos_north, float pos_east, float pos_alt, float vel_north, float vel_east, float vel_down)
{
    static int RunOnce = 0;
    if(RunOnce==0)
    {
        //状态初始化
        MatCreate(&ekf_xhat, NSTATES, 1);
        MatDataType ekf_xhat_init[NSTATES] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        MatInit(&ekf_xhat, ekf_xhat_init);
        //P矩阵初始化
        MatCreate(&ekf_P, NSTATES, NSTATES);
        MatDataType *ekf_P_init;	ekf_P_init = (MatDataType*)malloc(sizeof(MatDataType) * 16);
        ekf_P_init[0] = large_quad_uncertainty; ekf_P_init[1] = large_quad_uncertainty; ekf_P_init[2] = large_quad_uncertainty;ekf_P_init[3] = large_quad_uncertainty;
        ekf_P_init[4] = large_pos_uncertainty_m;    ekf_P_init[5] = large_pos_uncertainty_m;    ekf_P_init[6] = large_pos_uncertainty_m;
        ekf_P_init[7] = large_vel_uncertainty_mps;  ekf_P_init[8] = large_vel_uncertainty_mps;  ekf_P_init[9] = large_vel_uncertainty_mps;
        ekf_P_init[10] = ekf_sigmas_gyro_bias_rps;   ekf_P_init[11] = ekf_sigmas_gyro_bias_rps;   ekf_P_init[12] = ekf_sigmas_gyro_bias_rps;
        ekf_P_init[13] = ekf_sigmas_accel_bias_mps2; ekf_P_init[14] = ekf_sigmas_accel_bias_mps2; ekf_P_init[15] = ekf_sigmas_accel_bias_mps2;
        MatDiag(&ekf_P, ekf_P_init, NSTATES);
        free(ekf_P_init);
        //
        RunOnce = 1;
    }
    //Q矩阵初始化
    Mat ekf_Q;
    MatCreate(&ekf_Q, NSTATES, NSTATES);
    MatDataType *ekf_Q_init;    ekf_Q_init = (MatDataType*)malloc(sizeof(MatDataType) * NSTATES);
    ekf_Q_init[0] = ekf_sigmas_quad_process_noise;   ekf_Q_init[1] = ekf_sigmas_quad_process_noise;   ekf_Q_init[2] = ekf_sigmas_quad_process_noise;   ekf_Q_init[3] = ekf_sigmas_quad_process_noise;
    ekf_Q_init[4] = ekf_sigmas_pos_process_noise_m;   ekf_Q_init[5] = ekf_sigmas_pos_process_noise_m;   ekf_Q_init[6] = ekf_sigmas_pos_process_noise_m;
    ekf_Q_init[7] = ekf_sigmas_vel_process_noise_mps;   ekf_Q_init[8] = ekf_sigmas_vel_process_noise_mps;   ekf_Q_init[9] = ekf_sigmas_vel_process_noise_mps;
    ekf_Q_init[10] = ekf_sigmas_gyroBias_process_noise_rps;   ekf_Q_init[11] = ekf_sigmas_gyroBias_process_noise_rps;   ekf_Q_init[12] = ekf_sigmas_gyroBias_process_noise_rps;
    ekf_Q_init[13] = ekf_sigmas_accelBias_process_noise_mps2;   ekf_Q_init[14] = ekf_sigmas_accelBias_process_noise_mps2;   ekf_Q_init[15] = ekf_sigmas_accelBias_process_noise_mps2;
    MatDiag(&ekf_Q, ekf_Q_init, NSTATES);
    free(ekf_Q_init);
    //R矩阵初始化
    Mat ekf_R;
    MatCreate(&ekf_R, NMEASURES, NMEASURES);
    MatDataType *ekf_R_init;    ekf_R_init = (MatDataType*)malloc(sizeof(MatDataType)*9);
    ekf_R_init[0] = ekf_sigmas_mag3D_unitVector_meas; ekf_R_init[1] = ekf_sigmas_mag3D_unitVector_meas; ekf_R_init[2] = ekf_sigmas_mag3D_unitVector_meas;
    ekf_R_init[3] = ekf_sigmas_pos_meas_m; ekf_R_init[4] = ekf_sigmas_pos_meas_m; ekf_R_init[5] = ekf_sigmas_pos_meas_m;
    ekf_R_init[6] = ekf_sigmas_vel_meas_m; ekf_R_init[7] = ekf_sigmas_vel_meas_m; ekf_R_init[8] = ekf_sigmas_vel_meas_m;
    MatDiag(&ekf_R, ekf_R_init, NMEASURES);
    free(ekf_R_init);
    //计算xdot需要的量
    MatDataType q0, q1, q2, q3;
    MatDataType Vn, Ve, Vd;
    MatDataType bwx, bwy, bwz;
    MatDataType bax, bay, baz;
    q0 = MatAt(&ekf_xhat, 0, 0), q1 = MatAt(&ekf_xhat, 1, 0), q2 = MatAt(&ekf_xhat, 2, 0), q3 = MatAt(&ekf_xhat, 3, 0);
    Vn = MatAt(&ekf_xhat, 7, 0), Ve = MatAt(&ekf_xhat, 8, 0), Vd = MatAt(&ekf_xhat, 9, 0);
    bwx = MatAt(&ekf_xhat, 10, 0), bwy = MatAt(&ekf_xhat, 11, 0), bwz = MatAt(&ekf_xhat, 12, 0);
    bax = MatAt(&ekf_xhat, 13, 0), bay = MatAt(&ekf_xhat, 14, 0), baz = MatAt(&ekf_xhat, 15, 0);
    //角速度测量值
    Mat Wxyz;	MatCreate(&Wxyz, 3, 1);
    MatDataType *Wxyz_data = (MatDataType*)malloc(sizeof(MatDataType)*3);
    Wxyz_data[0] = wx;Wxyz_data[1] = wy;Wxyz_data[2] = wz;
    MatInit(&Wxyz, Wxyz_data);
    free(Wxyz_data);
    //加表测量值
    Mat Fxyz;	MatCreate(&Fxyz, 3, 1);
    MatDataType *Fxyz_data = (MatDataType*)malloc(sizeof(MatDataType)*3);
    Fxyz_data[0] = fx;Fxyz_data[1] = fy;Fxyz_data[2] = fz;
    MatInit(&Fxyz, Fxyz_data);
    free(Fxyz_data);
    //陀螺仪三轴偏置
    Mat Bw;	MatCreate(&Bw, 3, 1);
    MatDataType *Bw_data = (MatDataType*)malloc(sizeof(MatDataType)*3);
    Bw_data[0] = bwx;Bw_data[1] = bwy;Bw_data[2] = bwz;
    MatInit(&Bw, Bw_data);
    free(Bw_data);
    //加表三轴偏置
    Mat Bf;	MatCreate(&Bf, 3, 1);
    MatDataType *Bf_data = (MatDataType*)malloc(sizeof(MatDataType)*3);
    Bf_data[0] = bax;Bf_data[1] = bay;Bf_data[2] = baz;
    MatInit(&Bf, Bf_data);
    free(Bf_data);
    //重力向量
    Mat Gravity;	MatCreate(&Gravity, 3, 1);
    MatDataType *Gravity_data = (MatDataType*)malloc(sizeof(MatDataType)*3);
    Gravity_data[0] = 0.0f;Gravity_data[1] = 0.0f;Gravity_data[2] = gravity_mps2;
    MatInit(&Gravity, Gravity_data);
    free(Gravity_data);
    //四元数微分方程
    Mat C_bodyrate2qdot;
    MatCreate(&C_bodyrate2qdot, 4, 3);
    MatDataType *C_bodyrate2qdot_data;  C_bodyrate2qdot_data = (MatDataType*)malloc(sizeof(MatDataType)*12);
    C_bodyrate2qdot_data[0] = -0.5 * q1; C_bodyrate2qdot_data[1] = -0.5 * q2; C_bodyrate2qdot_data[2] = -0.5 * q3;
    C_bodyrate2qdot_data[3] = 0.5 * q0; C_bodyrate2qdot_data[4] = -0.5 * q3; C_bodyrate2qdot_data[5] = 0.5 * q2;
    C_bodyrate2qdot_data[6] = 0.5 * q3; C_bodyrate2qdot_data[7] = 0.5 * q0; C_bodyrate2qdot_data[8] = -0.5 * q1;
    C_bodyrate2qdot_data[9] = -0.5 * q2 ;  C_bodyrate2qdot_data[10] = 0.5 * q1 ;  C_bodyrate2qdot_data[11] = 0.5 * q0 ;
    MatInit(&C_bodyrate2qdot, C_bodyrate2qdot_data);
    free(C_bodyrate2qdot_data);
    //计算旋转矩阵C_ned2b
    Mat C_ned2b;
    MatCreate(&C_ned2b, 3, 3);
    MatDataType *C_ned2b_data;  C_ned2b_data = (MatDataType*)malloc(sizeof(MatDataType)*9);
    C_ned2b_data[0] = 1 - 2 * (q2 * q2 + q3 * q3); C_ned2b_data[1] = 2 * (q1 * q2 + q3 * q0); C_ned2b_data[2] = 2 * (q1 * q3 - q2 * q0);
    C_ned2b_data[3] = 2 * (q1 * q2 - q3 * q0); C_ned2b_data[4] = 1 - 2 * (q1 * q1 + q3 * q3); C_ned2b_data[5] = 2 * (q2 * q3 + q1 * q0);
    C_ned2b_data[6] = 2 * (q1 * q3 + q2 * q0); C_ned2b_data[7] = 2 * (q2 * q3 - q1 * q0); C_ned2b_data[8] = 1 - 2 * (q1 * q1 + q2 * q2);
    MatInit(&C_ned2b, C_ned2b_data);
    free(C_ned2b_data);
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
    MatZeros(&Xdot_part2);
    Xdot_part2.elements[0][0] = Vn;
    Xdot_part2.elements[1][0] = Ve;
    Xdot_part2.elements[2][0] = -Vd;

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

    Mat Xdot_blocks[5];
    Xdot_blocks[0] = Xdot_part1;
    Xdot_blocks[1] = Xdot_part2;
    Xdot_blocks[2] = Xdot_part3;
    Xdot_blocks[3] = Xdot_part4;
    Xdot_blocks[4] = Xdot_part5;
    MatBlockCompose(&Xdot, (Mat*)&Xdot_blocks, 5, 1);
    //F
    // Mat F;
    // MatCreate(&F, 16, 16);
    // MatDataType F_data[256] = {
    //     0, bwx / 2 - wx / 2, bwy / 2 - wy / 2, bwz / 2 - wz / 2, 0, 0, 0, 0, 0, 0, q1 / 2, q2 / 2, q3 / 2, 0, 0, 0,
    //     wx / 2 - bwx / 2, 0, wz / 2 - bwz / 2, bwy / 2 - wy / 2, 0, 0, 0, 0, 0, 0, -q0 / 2, q3 / 2, -q2 / 2, 0, 0, 0,
    //     wy / 2 - bwy / 2, bwz / 2 - wz / 2, 0, wx / 2 - bwx / 2, 0, 0, 0, 0, 0, 0, -q3 / 2, -q0 / 2, q1 / 2, 0, 0, 0,
    //     wz / 2 - bwz / 2, wy / 2 - bwy / 2, bwx / 2 - wx / 2, 0, 0, 0, 0, 0, 0, 0, q2 / 2, -q1 / 2, -q0 / 2, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    //     2 * q3 * (bay - fy) - 2 * q0 * (bax - fx) - 2 * q2 * (baz - fz), -2 * q1 * (bax - fx) - 2 * q2 * (bay - fy) - 2 * q3 * (baz - fz), 2 * q2 * (bax - fx) - 2 * q1 * (bay - fy) - 2 * q0 * (baz - fz), 2 * q3 * (bax - fx) + 2 * q0 * (bay - fy) - 2 * q1 * (baz - fz), 0, 0, 0, 0, 0, 0, 0, 0, 0, -q0 * q0 - q1 * q1 + q2 * q2 + q3 * q3, 2 * q0 * q3 - 2 * q1 * q2, -2 * q0 * q2 - 2 * q1 * q3,
    //     2 * q1 * (baz - fz) - 2 * q0 * (bay - fy) - 2 * q3 * (bax - fx), 2 * q1 * (bay - fy) - 2 * q2 * (bax - fx) + 2 * q0 * (baz - fz), -2 * q1 * (bax - fx) - 2 * q2 * (bay - fy) - 2 * q3 * (baz - fz), 2 * q3 * (bay - fy) - 2 * q0 * (bax - fx) - 2 * q2 * (baz - fz), 0, 0, 0, 0, 0, 0, 0, 0, 0, -2 * q0 * q3 - 2 * q1 * q2, -q0 * q0 + q1 * q1 - q2 * q2 + q3 * q3, 2 * q0 * q1 - 2 * q2 * q3,
    //     2 * q2 * (bax - fx) - 2 * q1 * (bay - fy) - 2 * q0 * (baz - fz), 2 * q1 * (baz - fz) - 2 * q0 * (bay - fy) - 2 * q3 * (bax - fx), 2 * q0 * (bax - fx) - 2 * q3 * (bay - fy) + 2 * q2 * (baz - fz), -2 * q1 * (bax - fx) - 2 * q2 * (bay - fy) - 2 * q3 * (baz - fz), 0, 0, 0, 0, 0, 0, 0, 0, 0, 2 * q0 * q2 - 2 * q1 * q3, -2 * q0 * q1 - 2 * q2 * q3, -q0 * q0 + q1 * q1 + q2 * q2 - q3 * q3,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // MatInit(&F, F_data);
    Mat F;
    MatCreate(&F, NSTATES, NSTATES);
    MatDataType *F_data;    F_data = (MatDataType*)malloc(sizeof(MatDataType)*NSTATES*NSTATES);
    F_data[0]=0,F_data[1]=bwx / 2 - wx / 2,F_data[2]=bwy / 2 - wy / 2,F_data[3]=bwz / 2 - wz / 2,F_data[4]=0,F_data[5]=0,F_data[6]=0,F_data[7]=0,F_data[8]=0,F_data[9]=0,F_data[10]=q1 / 2,F_data[11]=q2 / 2,F_data[12]=q3 / 2,F_data[13]=0,F_data[14]=0,F_data[15]=0,
    F_data[16]=wx / 2 - bwx / 2,F_data[17]=0,F_data[18]=wz / 2 - bwz / 2,F_data[19]=bwy / 2 - wy / 2,F_data[20]=0,F_data[21]=0,F_data[22]=0,F_data[23]=0,F_data[24]=0,F_data[25]=0,F_data[26]=-q0 / 2,F_data[27]=q3 / 2,F_data[28]=-q2 / 2,F_data[29]=0,F_data[30]=0,F_data[31]=0,
    F_data[32]=wy / 2 - bwy / 2,F_data[33]=bwz / 2 - wz / 2,F_data[34]=0,F_data[35]=wx / 2 - bwx / 2,F_data[36]=0,F_data[37]=0,F_data[38]=0,F_data[39]=0,F_data[40]=0,F_data[41]=0,F_data[42]=-q3 / 2,F_data[43]=-q0 / 2,F_data[44]=q1 / 2,F_data[45]=0,F_data[46]=0,F_data[47]=0,
    F_data[48]=wz / 2 - bwz / 2,F_data[49]=wy / 2 - bwy / 2,F_data[50]=bwx / 2 - wx / 2,F_data[51]=0,F_data[52]=0,F_data[53]=0,F_data[54]=0,F_data[55]=0,F_data[56]=0,F_data[57]=0,F_data[58]=q2 / 2,F_data[59]=-q1 / 2,F_data[60]=-q0 / 2,F_data[61]=0,F_data[62]=0,F_data[63]=0,
    F_data[64]=0,F_data[65]=0,F_data[66]=0,F_data[67]=0,F_data[68]=0,F_data[69]=0,F_data[70]=0,F_data[71]=1,F_data[72]=0,F_data[73]=0,F_data[74]=0,F_data[75]=0,F_data[76]=0,F_data[77]=0,F_data[78]=0,F_data[79]=0,
    F_data[80]=0,F_data[81]=0,F_data[82]=0,F_data[83]=0,F_data[84]=0,F_data[85]=0,F_data[86]=0,F_data[87]=0,F_data[88]=1,F_data[89]=0,F_data[90]=0,F_data[91]=0,F_data[92]=0,F_data[93]=0,F_data[94]=0,F_data[95]=0,
    F_data[96]=0,F_data[97]=0,F_data[98]=0,F_data[99]=0,F_data[100]=0,F_data[101]=0,F_data[102]=0,F_data[103]=0,F_data[104]=0,F_data[105]=-1,F_data[106]=0,F_data[107]=0,F_data[108]=0,F_data[109]=0,F_data[110]=0,F_data[111]=0,
    F_data[112]=2 * q3 * (bay - fy) - 2 * q0 * (bax - fx) - 2 * q2 * (baz - fz),F_data[113]=-2 * q1 * (bax - fx) - 2 * q2 * (bay - fy) - 2 * q3 * (baz - fz),F_data[114]=2 * q2 * (bax - fx) - 2 * q1 * (bay - fy) - 2 * q0 * (baz - fz),F_data[115]=2 * q3 * (bax - fx) + 2 * q0 * (bay - fy) - 2 * q1 * (baz - fz),F_data[116]=0,F_data[117]=0,F_data[118]=0,F_data[119]=0,F_data[120]=0,F_data[121]=0,F_data[122]=0,F_data[123]=0,F_data[124]=0,F_data[125]=-q0 * q0 - q1 * q1 + q2 * q2 + q3 * q3,F_data[126]=2 * q0 * q3 - 2 * q1 * q2,F_data[127]=-2 * q0 * q2 - 2 * q1 * q3,
    F_data[128]=2 * q1 * (baz - fz) - 2 * q0 * (bay - fy) - 2 * q3 * (bax - fx),F_data[129]=2 * q1 * (bay - fy) - 2 * q2 * (bax - fx) + 2 * q0 * (baz - fz),F_data[130]=-2 * q1 * (bax - fx) - 2 * q2 * (bay - fy) - 2 * q3 * (baz - fz),F_data[131]=2 * q3 * (bay - fy) - 2 * q0 * (bax - fx) - 2 * q2 * (baz - fz),F_data[132]=0,F_data[133]=0,F_data[134]=0,F_data[135]=0,F_data[136]=0,F_data[137]=0,F_data[138]=0,F_data[139]=0,F_data[140]=0,F_data[141]=-2 * q0 * q3 - 2 * q1 * q2,F_data[142]=-q0 * q0 + q1 * q1 - q2 * q2 + q3 * q3,F_data[143]=2 * q0 * q1 - 2 * q2 * q3,
    F_data[144]=2 * q2 * (bax - fx) - 2 * q1 * (bay - fy) - 2 * q0 * (baz - fz),F_data[145]=2 * q1 * (baz - fz) - 2 * q0 * (bay - fy) - 2 * q3 * (bax - fx),F_data[146]=2 * q0 * (bax - fx) - 2 * q3 * (bay - fy) + 2 * q2 * (baz - fz),F_data[147]=-2 * q1 * (bax - fx) - 2 * q2 * (bay - fy) - 2 * q3 * (baz - fz),F_data[148]=0,F_data[149]=0,F_data[150]=0,F_data[151]=0,F_data[152]=0,F_data[153]=0,F_data[154]=0,F_data[155]=0,F_data[156]=0,F_data[157]=2 * q0 * q2 - 2 * q1 * q3,F_data[158]=-2 * q0 * q1 - 2 * q2 * q3,F_data[159]=-q0 * q0 + q1 * q1 + q2 * q2 - q3 * q3,
    F_data[160]=0,F_data[161]=0,F_data[162]=0,F_data[163]=0,F_data[164]=0,F_data[165]=0,F_data[166]=0,F_data[167]=0,F_data[168]=0,F_data[169]=0,F_data[170]=0,F_data[171]=0,F_data[172]=0,F_data[173]=0,F_data[174]=0,F_data[175]=0,
    F_data[176]=0,F_data[177]=0,F_data[178]=0,F_data[179]=0,F_data[180]=0,F_data[181]=0,F_data[182]=0,F_data[183]=0,F_data[184]=0,F_data[185]=0,F_data[186]=0,F_data[187]=0,F_data[188]=0,F_data[189]=0,F_data[190]=0,F_data[191]=0,
    F_data[192]=0,F_data[193]=0,F_data[194]=0,F_data[195]=0,F_data[196]=0,F_data[197]=0,F_data[198]=0,F_data[199]=0,F_data[200]=0,F_data[201]=0,F_data[202]=0,F_data[203]=0,F_data[204]=0,F_data[205]=0,F_data[206]=0,F_data[207]=0,
    F_data[208]=0,F_data[209]=0,F_data[210]=0,F_data[211]=0,F_data[212]=0,F_data[213]=0,F_data[214]=0,F_data[215]=0,F_data[216]=0,F_data[217]=0,F_data[218]=0,F_data[219]=0,F_data[220]=0,F_data[221]=0,F_data[222]=0,F_data[223]=0,
    F_data[224]=0,F_data[225]=0,F_data[226]=0,F_data[227]=0,F_data[228]=0,F_data[229]=0,F_data[230]=0,F_data[231]=0,F_data[232]=0,F_data[233]=0,F_data[234]=0,F_data[235]=0,F_data[236]=0,F_data[237]=0,F_data[238]=0,F_data[239]=0,
    F_data[240]=0,F_data[241]=0,F_data[242]=0,F_data[243]=0,F_data[244]=0,F_data[245]=0,F_data[246]=0,F_data[247]=0,F_data[248]=0,F_data[249]=0,F_data[250]=0,F_data[251]=0,F_data[252]=0,F_data[253]=0,F_data[254]=0,F_data[255]=0;
    MatInit(&F, F_data);
    free(F_data);

    Mat Ft;
    MatCreate(&Ft, NSTATES, NSTATES);
    MatTrans(&Ft, &F);
    //PHI_K & Q_K
    Mat AA;
    MatCreate(&AA, NSTATES*2, NSTATES*2);
    Mat AA_Zeros;
    MatCreate(&AA_Zeros, NSTATES, NSTATES);
    MatZeros(&AA_Zeros);
    Mat AA_a;
    MatCreate(&AA_a, NSTATES, NSTATES);
    MatCopy(&AA_a, MatScalarMultiply(&F, -1));
    Mat AA_b;
    MatCreate(&AA_b, NSTATES, NSTATES);
    MatCopy(&AA_b, &ekf_Q);
    Mat AA_c;
    MatCreate(&AA_c, NSTATES, NSTATES);
    MatCopy(&AA_c, &AA_Zeros);
    Mat AA_d;
    MatCreate(&AA_d, NSTATES, NSTATES);
    MatCopy(&AA_d, &Ft);

    Mat AA_blocks[4];
    AA_blocks[0] = AA_a;
    AA_blocks[1] = AA_b;
    AA_blocks[2] = AA_c;
    AA_blocks[3] = AA_d;
    MatBlockCompose(&AA, (Mat*)&AA_blocks, 2, 2);
    MatScalarMultiply(&AA, dt_s);

    Mat BB;
    MatCreate(&BB, NSTATES*2, NSTATES*2);
    MatZeros(&BB);
    Mat BB_EYE;
    MatCreate(&BB_EYE, NSTATES*2, NSTATES*2);
    MatEye(&BB_EYE);
    MatAdd(&BB, &BB_EYE, &AA);

    Mat PHI_K;
    MatCreate(&PHI_K, NSTATES, NSTATES);
    MatZeros(&PHI_K);
    Mat PHI_Kt;
    MatCreate(&PHI_Kt, NSTATES, NSTATES);
    MatZeros(&PHI_Kt);
    MatBlockDecompose(&PHI_Kt, &BB, NSTATES, NSTATES, NSTATES * 2 - 1, NSTATES * 2 - 1);
    MatTrans(&PHI_K, &PHI_Kt);

    Mat BB_rightup;
    MatCreate(&BB_rightup, NSTATES, NSTATES);
    MatBlockDecompose(&BB_rightup, &BB, 0, NSTATES, NSTATES - 1, NSTATES * 2 - 1);
    Mat Q_K;
    MatCreate(&Q_K, NSTATES, NSTATES);
    MatMultiply(&Q_K, &PHI_K, &BB_rightup);
    //一步预测
    MatAdd(&ekf_xhat, &ekf_xhat, MatScalarMultiply(&Xdot, dt_s));
    //一步预测均方误差
    Mat ekf_Pp_aux1;
    MatCreate(&ekf_Pp_aux1, NSTATES, NSTATES);
    Mat ekf_Pp_aux2;
    MatCreate(&ekf_Pp_aux2, NSTATES, NSTATES);
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
    // Mat ekf_H;
    // MatCreate(&ekf_H, 9, 16);
    // MatDataType ekf_H_data[144] = 
    // {
    //     0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0
    // };
    // MatInit(&ekf_H, ekf_H_data);
    Mat ekf_H;
    MatCreate(&ekf_H, NMEASURES, NSTATES);
    MatDataType *ekf_H_data;    ekf_H_data = (MatDataType*)malloc(sizeof(MatDataType)*144);
    ekf_H_data[0] = 0,ekf_H_data[1] =  0,ekf_H_data[2] =  0,ekf_H_data[3] =  0,ekf_H_data[4] =  1,ekf_H_data[5] =  0,ekf_H_data[6] =  0,ekf_H_data[7] =  0,ekf_H_data[8] =  0,ekf_H_data[9] =  0,ekf_H_data[10] =  0,ekf_H_data[11] =  0,ekf_H_data[12] =  0,ekf_H_data[13] =  0,ekf_H_data[14] =  0,ekf_H_data[15] =  0,
    ekf_H_data[16] = 0,ekf_H_data[17] =  0,ekf_H_data[18] =  0,ekf_H_data[19] =  0,ekf_H_data[20] =  0,ekf_H_data[21] =  1,ekf_H_data[22] =  0,ekf_H_data[23] =  0,ekf_H_data[24] =  0,ekf_H_data[25] =  0,ekf_H_data[26] =  0,ekf_H_data[27] =  0,ekf_H_data[28] =  0,ekf_H_data[29] =  0,ekf_H_data[30] =  0,ekf_H_data[31] =  0,
    ekf_H_data[32] = 0,ekf_H_data[33] =  0,ekf_H_data[34] =  0,ekf_H_data[35] =  0,ekf_H_data[36] =  0,ekf_H_data[37] =  0,ekf_H_data[38] =  1,ekf_H_data[39] =  0,ekf_H_data[40] =  0,ekf_H_data[41] =  0,ekf_H_data[42] =  0,ekf_H_data[43] =  0,ekf_H_data[44] =  0,ekf_H_data[45] =  0,ekf_H_data[46] =  0,ekf_H_data[47] =  0,
    ekf_H_data[48] = 0,ekf_H_data[49] =  0,ekf_H_data[50] =  0,ekf_H_data[51] =  0,ekf_H_data[52] =  1,ekf_H_data[53] =  0,ekf_H_data[54] =  0,ekf_H_data[55] =  0,ekf_H_data[56] =  0,ekf_H_data[57] =  0,ekf_H_data[58] =  0,ekf_H_data[59] =  0,ekf_H_data[60] =  0,ekf_H_data[61] =  0,ekf_H_data[62] =  0,ekf_H_data[63] =  0,
    ekf_H_data[64] = 0,ekf_H_data[65] =  0,ekf_H_data[66] =  0,ekf_H_data[67] =  0,ekf_H_data[68] =  0,ekf_H_data[69] =  1,ekf_H_data[70] =  0,ekf_H_data[71] =  0,ekf_H_data[72] =  0,ekf_H_data[73] =  0,ekf_H_data[74] =  0,ekf_H_data[75] =  0,ekf_H_data[76] =  0,ekf_H_data[77] =  0,ekf_H_data[78] =  0,ekf_H_data[79] =  0,
    ekf_H_data[80] = 0,ekf_H_data[81] =  0,ekf_H_data[82] =  0,ekf_H_data[83] =  0,ekf_H_data[84] =  0,ekf_H_data[85] =  0,ekf_H_data[86] =  1,ekf_H_data[87] =  0,ekf_H_data[88] =  0,ekf_H_data[89] =  0,ekf_H_data[90] =  0,ekf_H_data[91] =  0,ekf_H_data[92] =  0,ekf_H_data[93] =  0,ekf_H_data[94] =  0,ekf_H_data[95] =  0,
    ekf_H_data[96] = 0,ekf_H_data[97] =  0,ekf_H_data[98] =  0,ekf_H_data[99] =  0,ekf_H_data[100] =  0,ekf_H_data[101] =  0,ekf_H_data[102] =  0,ekf_H_data[103] =  1,ekf_H_data[104] =  0,ekf_H_data[105] =  0,ekf_H_data[106] =  0,ekf_H_data[107] =  0,ekf_H_data[108] =  0,ekf_H_data[109] =  0,ekf_H_data[110] =  0,ekf_H_data[111] =  0,
    ekf_H_data[112] = 0,ekf_H_data[113] =  0,ekf_H_data[114] =  0,ekf_H_data[115] =  0,ekf_H_data[116] =  0,ekf_H_data[117] =  0,ekf_H_data[118] =  0,ekf_H_data[119] =  0,ekf_H_data[120] =  1,ekf_H_data[121] =  0,ekf_H_data[122] =  0,ekf_H_data[123] =  0,ekf_H_data[124] =  0,ekf_H_data[125] =  0,ekf_H_data[126] =  0,ekf_H_data[127] =  0,
    ekf_H_data[128] = 0,ekf_H_data[129] =  0,ekf_H_data[130] =  0,ekf_H_data[131] =  0,ekf_H_data[132] =  0,ekf_H_data[133] =  0,ekf_H_data[134] =  0,ekf_H_data[135] =  0,ekf_H_data[136] =  0,ekf_H_data[137] =  1,ekf_H_data[138] =  0,ekf_H_data[139] =  0,ekf_H_data[140] =  0,ekf_H_data[141] =  0,ekf_H_data[142] =  0,ekf_H_data[143] =  0;
    MatInit(&ekf_H, ekf_H_data);
    free(ekf_H_data);


    Mat ekf_Ht;
    MatCreate(&ekf_Ht, NSTATES, NMEASURES);
    MatTrans(&ekf_Ht, &ekf_H);
    //K
    Mat ekf_K;
    MatCreate(&ekf_K, NSTATES, NMEASURES);
    Mat ekf_K_aux1;
    MatCreate(&ekf_K_aux1, NMEASURES, NSTATES);
    MatMultiply(&ekf_K_aux1, &ekf_H, &ekf_P); //aux1 = H*P
    Mat ekf_K_aux2;
    MatCreate(&ekf_K_aux2, NMEASURES, NMEASURES);
    MatMultiply(&ekf_K_aux2, &ekf_K_aux1, &ekf_Ht); //aux2 = H*P*Ht
    Mat ekf_K_aux3;
    MatCreate(&ekf_K_aux3, NMEASURES, NMEASURES);
    MatAdd(&ekf_K_aux3, &ekf_K_aux2, &ekf_R); //aux3 = H*P*Ht+R
    Mat ekf_K_aux4;
    MatCreate(&ekf_K_aux4, NMEASURES, NMEASURES);
    MatInverse(&ekf_K_aux4, &ekf_K_aux3); //aux4 = inv(H*P*Ht+R)
    Mat ekf_K_aux5;
    MatCreate(&ekf_K_aux5, NSTATES, NMEASURES);
    MatMultiply(&ekf_K_aux5, &ekf_P, &ekf_Ht); //aux5 = P*Ht
    MatMultiply(&ekf_K, &ekf_K_aux5, &ekf_K_aux4);
    //Z
    // Mat ekf_Z;
    // MatCreate(&ekf_Z, 9, 1);
    // MatDataType ekf_Z_data[9] = {
    //     pos_north, pos_east, pos_alt, pos_north, pos_east, pos_alt,vel_north,vel_east,vel_down};
    // MatInit(&ekf_Z, ekf_Z_data);
    Mat ekf_Z;
    MatCreate(&ekf_Z, NMEASURES, 1);
    MatDataType *ekf_Z_data;    ekf_Z_data = (MatDataType*)malloc(sizeof(MatDataType)*9);
    ekf_Z_data[0] = pos_north; ekf_Z_data[1] = pos_east; ekf_Z_data[2] = pos_alt;
    ekf_Z_data[3] = pos_north; ekf_Z_data[4] = pos_east; ekf_Z_data[5] = pos_alt;
    ekf_Z_data[6] = vel_north; ekf_Z_data[7] = vel_east; ekf_Z_data[8] = vel_down;
    MatInit(&ekf_Z, ekf_Z_data);
    free(ekf_Z_data);
    //状态估计
    Mat ekf_xhat_aux1;
    MatCreate(&ekf_xhat_aux1, NMEASURES, 1);
    MatMultiply(&ekf_xhat_aux1, &ekf_H, &ekf_xhat); //H*X
    Mat ekf_xhat_aux2;
    MatCreate(&ekf_xhat_aux2, NMEASURES, 1);
    MatSub(&ekf_xhat_aux2, &ekf_Z, &ekf_xhat_aux1); //Z-H*X
    Mat ekf_xhat_aux3;
    MatCreate(&ekf_xhat_aux3, NSTATES, 1);
    MatMultiply(&ekf_xhat_aux3, &ekf_K, &ekf_xhat_aux2); //K*(Z-H*X)
    MatAdd(&ekf_xhat, &ekf_xhat, &ekf_xhat_aux3);        //X+K*(Z-H*X)
    //状态均方误差更新
    Mat P_EYE;
    MatCreate(&P_EYE, NSTATES, NSTATES);
    MatEye(&P_EYE); //I
    Mat ekf_Pe_aux1;
    MatCreate(&ekf_Pe_aux1, NSTATES, NSTATES);
    MatMultiply(&ekf_Pe_aux1, &ekf_K, &ekf_H); //KH
    Mat ekf_Pe_aux2;
    MatCreate(&ekf_Pe_aux2, NSTATES, NSTATES);
    MatSub(&ekf_Pe_aux2, &P_EYE, &ekf_Pe_aux1); //I-KH
    Mat ekf_Pe_aux3;
    MatCreate(&ekf_Pe_aux3, NSTATES, NSTATES);
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
    //yaw_deg = 180 / PI * atan2(2*(q1*q2 + q3*q0), 1-2*(q2*q2 + q3*q3)); // -180 <= yaw <= 180
    yaw_deg = 180 / PI *atan2(my,mx);

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
    MatDelete(&AA_Zeros);
    MatDelete(&AA_a);
    MatDelete(&AA_b);
    MatDelete(&AA_c);
    MatDelete(&AA_d);
    MatDelete(&AA);
    MatDelete(&BB);
    MatDelete(&BB_EYE);
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
    MatDelete(&P_EYE);
    MatDelete(&ekf_Pe_aux1);
    MatDelete(&ekf_Pe_aux2);
    MatDelete(&ekf_Pe_aux3);
    return 0;
}