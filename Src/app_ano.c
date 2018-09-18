#include "app_ano.h"
#include "stm32f4xx_hal.h"
#include "flash_on_chip.h"
#include "usart_cfg.h"
#include "mpu9250.h"
#include "ms5803.h"
#include "rc_pwm.h"
#include "app_ins.h"
#include <stdlib.h>
#include "cmsis_os.h"
#include "uwb1000.h"
#include "app_ros.h"

#define BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 0) )
#define BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )

#define DEG_TO_RAD 0.01745329251994329576f
#define RAD_TO_DEG 57.29577951308232087679f

extern osSemaphoreId myBinarySem01MPU9250GyroAccCalibrateOffsetHandle;
extern osSemaphoreId myBinarySem04MPU9250MagCalibrateHandle;

static uint8_t ano_data_to_send[100];	//
static uint8_t RxBuffer[50];		//

static int state;
static int _data_len, _data_cnt;
static int get_data_from_ANO;
static int send_pid_para;
static int data_from_ANO_updata;
static int write_data_to_inner_flash;
static int read_data_from_inner_flash;

float rol_p, rol_i, rol_d;
float pit_p, pit_i, pit_d;
float yaw_p, yaw_i, yaw_d;

float alt_p, alt_i, alt_d;
float pos_p, pos_i, pos_d;
float pid6_p, pid6_i, pid6_d;

float pid7_p, pid7_i, pid7_d;
float pid8_p, pid8_i, pid8_d;
float pid9_p, pid9_i, pid9_d;

float pid10_p, pid10_i, pid10_d;
float pid11_p, pid11_i, pid11_d;
float pid12_p, pid12_i, pid12_d;

float pid13_p, pid13_i, pid13_d;
float pid14_p, pid14_i, pid14_d;
float pid15_p, pid15_i, pid15_d;

float pid16_p, pid16_i, pid16_d;
float pid17_p, pid17_i, pid17_d;
float pid18_p, pid18_i, pid18_d;

void app_ano_parameter_init(void)
{
	rol_p = 0; rol_i = 0; rol_d = 0;
	pit_p = 0; pit_i = 0; pit_d = 0;
	yaw_p = 0; yaw_i = 0; yaw_d = 0;

	alt_p = 0; alt_i = 0; alt_d = 0;
	pos_p = 0; pos_i = 0; pos_d = 0;
	pid6_p = 0; pid6_i = 0; pid6_d = 0;

	pid7_p = 0; pid7_i = 0; pid7_d = 0;
	pid8_p = 0; pid8_i = 0; pid8_d = 0;
	pid9_p = 0; pid9_i = 0; pid9_d = 0;

	pid10_p = 0; pid10_i = 0; pid10_d = 0;
	pid11_p = 0; pid11_i = 0; pid11_d = 0;
	pid12_p = 0; pid12_i = 0; pid12_d = 0;

	pid13_p = 0; pid13_i = 0; pid13_d = 0;
	pid14_p = 0; pid14_i = 0; pid14_d = 0;
	pid15_p = 0; pid15_i = 0; pid15_d = 0;

	pid16_p = 0; pid16_i = 0; pid16_d = 0;
	pid17_p = 0; pid17_i = 0; pid17_d = 0;
	pid18_p = 0; pid18_i = 0; pid18_d = 0;
}

int ANO_sending(int mode)
{
	if (mode == 0)
	{
		if ((get_data_from_ANO == 0) && (send_pid_para == 0))
		{
			ANO_send_15_data(
				(int16_t)(1000*t_body_x.acc_raw_meter_per_s[0]), (int16_t)(1000*t_body_y.acc_raw_meter_per_s[0]), (int16_t)(1000*t_body_z.acc_raw_meter_per_s[0]),
				(int16_t)(1000*t_body_x.gyro_raw_rad_per_s[0]), (int16_t)(1000*t_body_y.gyro_raw_rad_per_s[0]), (int16_t)(1000*t_body_z.gyro_raw_rad_per_s[0]),
				(int16_t)(10*magx_raw_uT[0]), (int16_t)(10*magy_raw_uT[0]), (int16_t)(10*magz_raw_uT[0]),
				// (int16_t)(temp1*100), (int16_t)(temp2*100), (int16_t)(temp3*100),
				// (int16_t)(temp4*100), (int16_t)(temp5*100), (int16_t)(temp6*100),
				// (int16_t)(magx_raw_uT), (int16_t)(magy_raw_uT), (int16_t)(temp0),
				(float)(1*t_attitude.roll*RAD_TO_DEG), (float)(1*t_attitude.pitch*RAD_TO_DEG), (float)(1*t_attitude.yaw*RAD_TO_DEG),
				(int32_t)(15), (uint8_t)(13), (uint8_t)(14));
		}
	}
	if (mode == 1)
	{

	}
	return 0;
}

int ANO_data_push(void)
{
	if (data_from_ANO_updata == 1)
	{
		//rol_p; rol_i; rol_d;
		//pit_p; pit_i; pit_d;
		//yaw_p; yaw_i; yaw_d;

		//alt_p; alt_i; alt_d;
		//pos_p; pos_i; pos_d;
		//pid6_p; pid6_i; pid6_d;

		//pid7_p; pid7_i; pid7_d;
		//pid8_p; pid8_i; pid8_d;
		//pid9_p; pid9_i; pid9_d;

		//pid10_p; pid10_i; pid10_d;
		//pid11_p; pid11_i; pid11_d;
		//pid12_p; pid12_i; pid12_d;

		//pid13_p; pid13_i; pid13_d;
		//pid14_p; pid14_i; pid14_d;
		//pid15_p; pid15_i; pid15_d;

		//pid16_p; pid16_i; pid16_d;
		//pid17_p; pid17_i; pid17_d;
		//pid18_p; pid18_i; pid18_d;

		data_from_ANO_updata = 0;
	}
	return 0;
}

int32_t ANO_usart_send_data(uint8_t *dataToSend, uint8_t length)
{
	USART1_DMA_send_data(dataToSend, length);
	return 0;
}

void ANO_send_15_data(
		int16_t a_x, int16_t a_y, int16_t a_z,
		int16_t g_x, int16_t g_y, int16_t g_z,
		int16_t m_x, int16_t m_y, int16_t m_z,
	float angle_rol, float angle_pit, float angle_yaw,
	int32_t alt, uint8_t fly_model, uint8_t armed)
{
	uint8_t _cnt = 0;
	__IO int16_t _temp;
	ano_data_to_send[_cnt++] = 0xAA;
	ano_data_to_send[_cnt++] = 0xAA;
	ano_data_to_send[_cnt++] = 0x02;
	ano_data_to_send[_cnt++] = 0;

	_temp = a_x;
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = a_y;
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = a_z;
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);

	_temp = g_x;
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = g_y;
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = g_z;
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);

	_temp = m_x;
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = m_y;
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = m_z;
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	/////////////////////////////////////////
	_temp = 0;
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);

	ano_data_to_send[3] = _cnt - 4;

	uint8_t sum = 0;
	for (uint8_t i = 0; i < _cnt; i++)
		sum += ano_data_to_send[i];
	ano_data_to_send[_cnt++] = sum;


	uint8_t _cnt2 = 0;
	uint8_t _cnt3 = 0;
	int32_t _temp2 = alt;
	_cnt3 = _cnt;
	ano_data_to_send[_cnt++] = 0xAA;
	ano_data_to_send[_cnt++] = 0xAA;
	ano_data_to_send[_cnt++] = 0x01;
	_cnt2 = _cnt;
	ano_data_to_send[_cnt++] = 0;

	_temp = (int)(angle_rol * 100);
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int)(angle_pit * 100);
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int)(angle_yaw * 100);
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);

	ano_data_to_send[_cnt++] = BYTE3(_temp2);
	ano_data_to_send[_cnt++] = BYTE2(_temp2);
	ano_data_to_send[_cnt++] = BYTE1(_temp2);
	ano_data_to_send[_cnt++] = BYTE0(_temp2);

	ano_data_to_send[_cnt++] = fly_model;

	ano_data_to_send[_cnt++] = armed;

	ano_data_to_send[_cnt2] = _cnt - _cnt2 - 1;

	sum = 0;
	for (uint8_t i = _cnt3; i < _cnt; i++)
		sum += ano_data_to_send[i];
	ano_data_to_send[_cnt++] = sum;

	ANO_usart_send_data(ano_data_to_send, _cnt);
}

void ANO_DT_Send_PID(uint8_t group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p, float p3_i, float p3_d)
{
	uint8_t _cnt = 0;
	__IO int16_t _temp;

	ano_data_to_send[_cnt++] = 0xAA;
	ano_data_to_send[_cnt++] = 0xAA;
	ano_data_to_send[_cnt++] = 0x10 + group - 1;
	ano_data_to_send[_cnt++] = 0;

	_temp = (short)(p1_p * 1000);
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p1_i * 1000);
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p1_d * 1000);
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p2_p * 1000);
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p2_i * 1000);
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p2_d * 1000);
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p3_p * 1000);
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p3_i * 1000);
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p3_d * 1000);
	ano_data_to_send[_cnt++] = BYTE1(_temp);
	ano_data_to_send[_cnt++] = BYTE0(_temp);

	ano_data_to_send[3] = _cnt - 4;

	uint8_t sum = 0;
	for (uint8_t i = 0; i<_cnt; i++)
		sum += ano_data_to_send[i];

	ano_data_to_send[_cnt++] = sum;

	ANO_usart_send_data(ano_data_to_send, _cnt);
}

void data_trans_with_ano()
{
	/*---------------obtain data from ANO_DT-----------*/
	if (get_data_from_ANO == 1)
	{
		ANO_DT_Data_Receive_Anl(RxBuffer, _data_cnt + 5);
		data_from_ANO_updata = 1;
		get_data_from_ANO = 0;
		return;
	}
	/*---------------send data to ANO_DT---------------*/
	if (send_pid_para)
	{
		if (send_pid_para == 1)
		{
			ANO_DT_Send_PID(1, rol_p, rol_i, rol_d , pit_p, pit_i, pit_d, yaw_p, yaw_i, yaw_d);
			send_pid_para++;
			return;
		}
		if (send_pid_para == 3)
		{
			ANO_DT_Send_PID(2, alt_p, alt_i, alt_d, pos_p, pos_i, pos_d, pid6_p, pid6_i, pid6_d);
			send_pid_para++;
			return;
		}
		if (send_pid_para == 5)
		{
			ANO_DT_Send_PID(3, pid7_p, pid7_i, pid7_d, pid8_p, pid8_i, pid8_d, pid9_p, pid9_i, pid9_d);
			send_pid_para++;
			return;
		}
		if (send_pid_para == 7)
		{
			ANO_DT_Send_PID(4, pid10_p, pid10_i, pid10_d, pid11_p, pid11_i, pid11_d, pid12_p, pid12_i, pid12_d);
			send_pid_para++;
			return;
		}
		if (send_pid_para == 9)
		{
			ANO_DT_Send_PID(5, pid13_p, pid13_i, pid13_d, pid14_p, pid14_i, pid14_d, pid15_p, pid15_i, pid15_d);
			send_pid_para++;
			return;
		}
		if (send_pid_para == 11)
		{
			ANO_DT_Send_PID(6, pid16_p, pid16_i, pid16_d, pid17_p, pid17_i, pid17_d, pid18_p, pid18_i, pid18_d);
			send_pid_para = 0;
			return;
		}
		send_pid_para++;
		return;
	}
}

void ANO_data_receive_prepara(uint8_t data)
{
        //get_data_from_ANO = -1;
	if (state == 0 && data == 0xAA)
	{
		state = 1;
		RxBuffer[0] = data;
	}
	else if (state == 1 && data == 0xAF)
	{
		state = 2;
		RxBuffer[1] = data;
	}
	else if (state == 2 && data<0XF1)
	{
		state = 3;
		RxBuffer[2] = data;
	}
	else if (state == 3 && data<50)
	{
		state = 4;
		RxBuffer[3] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if (state == 4 && _data_len>0)
	{
		_data_len--;
		RxBuffer[4 + _data_cnt++] = data;
		if (_data_len == 0)
			state = 5;
	}
	else if (state == 5)
	{
		state = 0;
		RxBuffer[4 + _data_cnt] = data;
		get_data_from_ANO = 1;
	}
	else
		state = 0;
}

void ANO_DT_Data_Receive_Anl(uint8_t *data_buf, uint8_t num)
{
	uint8_t sum = 0;
	for (uint8_t i = 0; i<(num - 1); i++)
		sum += *(data_buf + i);
	//�ж�sum
	if (!(sum == *(data_buf + num - 1)))
	{
		return;
	}
	//�ж�֡ͷ
	if (!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAF))
	{
		return;
	}
	/*------������У׼------*/
	if (*(data_buf + 2) == 0X01)
	{
		if (*(data_buf + 4) == 0X01)
		{
			//���ٶ�У׼
			asm("nop");
//			task_imu_calibration.acc_calibrate = 1;
		}
		if (*(data_buf + 4) == 0X02)
		{
			//������У׼
			asm("nop");
                        osSemaphoreRelease(myBinarySem01MPU9250GyroAccCalibrateOffsetHandle);// �ͷ��ź���
//			task_imu_calibration.gyro_calibrate = 1;
		}
		if (*(data_buf + 4) == 0X05)
		{
			//��ѹ��У׼
			asm("nop");
		}
		if (*(data_buf + 4) == 0X04)
		{
			//����У׼
			asm("nop");
                        osSemaphoreRelease(myBinarySem04MPU9250MagCalibrateHandle);// �ͷ��ź���
//			task_imu_calibration.mag_calibrate++;
		}
	}
	if (*(data_buf + 2) == 0X02)
	{
		//���ز���
		if (*(data_buf + 4) == 0X01)
		{
			//��ȡ�ɿ�
			read_data_from_inner_flash = 1;
			send_pid_para = 1;
		}
		if (*(data_buf + 4) == 0XA1)
		{
			//�ָ�Ĭ��ֵ
			write_data_to_inner_flash = 1;
			//asm("nop");
		}
	}
	/******************д��ɿ�**********************/
	//PID1
	if (*(data_buf + 2) == 0X10)
	{
		rol_p = 0.001*((int)(*(data_buf + 4) << 8) | *(data_buf + 5));
		rol_i = 0.001*((int)(*(data_buf + 6) << 8) | *(data_buf + 7));
		rol_d = 0.001*((int)(*(data_buf + 8) << 8) | *(data_buf + 9));

		pit_p = 0.001*((int)(*(data_buf + 10) << 8) | *(data_buf + 11));
		pit_i = 0.001*((int)(*(data_buf + 12) << 8) | *(data_buf + 13));
		pit_d = 0.001*((int)(*(data_buf + 14) << 8) | *(data_buf + 15));

		yaw_p = 0.001*((int)(*(data_buf + 16) << 8) | *(data_buf + 17));
		yaw_i = 0.001*((int)(*(data_buf + 18) << 8) | *(data_buf + 19));
		yaw_d = 0.001*((int)(*(data_buf + 20) << 8) | *(data_buf + 21));
		ANO_DT_Send_Check(*(data_buf + 2), sum);
	}
	//PID2
	if (*(data_buf + 2) == 0X11)
	{
		alt_p = 0.001*((int)(*(data_buf + 4) << 8) | *(data_buf + 5));
		alt_i = 0.001*((int)(*(data_buf + 6) << 8) | *(data_buf + 7));
		alt_d = 0.001*((int)(*(data_buf + 8) << 8) | *(data_buf + 9));

		pos_p = 0.001*((int)(*(data_buf + 10) << 8) | *(data_buf + 11));
		pos_i = 0.001*((int)(*(data_buf + 12) << 8) | *(data_buf + 13));
		pos_d = 0.001*((int)(*(data_buf + 14) << 8) | *(data_buf + 15));

		pid6_p = 0.001*((int)(*(data_buf + 16) << 8) | *(data_buf + 17));
		pid6_i = 0.001*((int)(*(data_buf + 18) << 8) | *(data_buf + 19));
		pid6_d = 0.001*((int)(*(data_buf + 20) << 8) | *(data_buf + 21));
		ANO_DT_Send_Check(*(data_buf + 2), sum);
	}
	//PID3
	if (*(data_buf + 2) == 0X12)
	{
		pid7_p = 0.001*((int)(*(data_buf + 4) << 8) | *(data_buf + 5));
		pid7_i = 0.001*((int)(*(data_buf + 6) << 8) | *(data_buf + 7));
		pid7_d = 0.001*((int)(*(data_buf + 8) << 8) | *(data_buf + 9));

		pid8_p = 0.001*((int)(*(data_buf + 10) << 8) | *(data_buf + 11));
		pid8_i = 0.001*((int)(*(data_buf + 12) << 8) | *(data_buf + 13));
		pid8_d = 0.001*((int)(*(data_buf + 14) << 8) | *(data_buf + 15));

		pid9_p = 0.001*((int)(*(data_buf + 16) << 8) | *(data_buf + 17));
		pid9_i = 0.001*((int)(*(data_buf + 18) << 8) | *(data_buf + 19));
		pid9_d = 0.001*((int)(*(data_buf + 20) << 8) | *(data_buf + 21));
		ANO_DT_Send_Check(*(data_buf + 2), sum);
	}
	//PID4
	if (*(data_buf + 2) == 0X13)
	{
		pid10_p = 0.001*((int)(*(data_buf + 4) << 8) | *(data_buf + 5));
		pid10_i = 0.001*((int)(*(data_buf + 6) << 8) | *(data_buf + 7));
		pid10_d = 0.001*((int)(*(data_buf + 8) << 8) | *(data_buf + 9));

		pid11_p = 0.001*((int)(*(data_buf + 10) << 8) | *(data_buf + 11));
		pid11_i = 0.001*((int)(*(data_buf + 12) << 8) | *(data_buf + 13));
		pid11_d = 0.001*((int)(*(data_buf + 14) << 8) | *(data_buf + 15));

		pid12_p = 0.001*((int)(*(data_buf + 16) << 8) | *(data_buf + 17));
		pid12_i = 0.001*((int)(*(data_buf + 18) << 8) | *(data_buf + 19));
		pid12_d = 0.001*((int)(*(data_buf + 20) << 8) | *(data_buf + 21));
		ANO_DT_Send_Check(*(data_buf + 2), sum);
	}
	//PID5
	if (*(data_buf + 2) == 0X14)
	{
		pid13_p = 0.001*((int)(*(data_buf + 4) << 8) | *(data_buf + 5));
		pid13_i = 0.001*((int)(*(data_buf + 6) << 8) | *(data_buf + 7));
		pid13_d = 0.001*((int)(*(data_buf + 8) << 8) | *(data_buf + 9));

		pid14_p = 0.001*((int)(*(data_buf + 10) << 8) | *(data_buf + 11));
		pid14_i = 0.001*((int)(*(data_buf + 12) << 8) | *(data_buf + 13));
		pid14_d = 0.001*((int)(*(data_buf + 14) << 8) | *(data_buf + 15));

		pid15_p = 0.001*((int)(*(data_buf + 16) << 8) | *(data_buf + 17));
		pid15_i = 0.001*((int)(*(data_buf + 18) << 8) | *(data_buf + 19));
		pid15_d = 0.001*((int)(*(data_buf + 20) << 8) | *(data_buf + 21));
		ANO_DT_Send_Check(*(data_buf + 2), sum);
	}
	//PID6
	if (*(data_buf + 2) == 0X15)
	{
		pid16_p = 0.001*((int)(*(data_buf + 4) << 8) | *(data_buf + 5));
		pid16_i = 0.001*((int)(*(data_buf + 6) << 8) | *(data_buf + 7));
		pid16_d = 0.001*((int)(*(data_buf + 8) << 8) | *(data_buf + 9));

		pid17_p = 0.001*((int)(*(data_buf + 10) << 8) | *(data_buf + 11));
		pid17_i = 0.001*((int)(*(data_buf + 12) << 8) | *(data_buf + 13));
		pid17_d = 0.001*((int)(*(data_buf + 14) << 8) | *(data_buf + 15));

		pid18_p = 0.001*((int)(*(data_buf + 16) << 8) | *(data_buf + 17));
		pid18_i = 0.001*((int)(*(data_buf + 18) << 8) | *(data_buf + 19));
		pid18_d = 0.001*((int)(*(data_buf + 20) << 8) | *(data_buf + 21));
		ANO_DT_Send_Check(*(data_buf + 2), sum);
	}
}

void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum)
{
	ano_data_to_send[0] = 0xAA;
	ano_data_to_send[1] = 0xAA;
	ano_data_to_send[2] = 0xEF;
	ano_data_to_send[3] = 2;
	ano_data_to_send[4] = head;
	ano_data_to_send[5] = check_sum;

	uint8_t sum = 0;
	for (uint8_t i = 0; i<6; i++)
		sum += ano_data_to_send[i];
	ano_data_to_send[6] = sum;

	ANO_usart_send_data(ano_data_to_send, 7);
}

int ano_process(int mode)
{
	int static delay_cnt;
	delay_cnt++;
	ANO_data_push();
	ANO_sending(mode);
	if (delay_cnt>4)
	{
		data_trans_with_ano();
		delay_cnt = 0;
	}
	if (write_data_to_inner_flash == 1)
	{
		flash_save_parameters();
		write_data_to_inner_flash = 0;
	}
        if (read_data_from_inner_flash == 1)
	{
		flash_read_parameters();
		read_data_from_inner_flash = 0;
	}
	return 0;
}
