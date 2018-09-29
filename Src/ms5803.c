#include "ms5803.h"
#include "stm32f4xx_hal.h"
#include "USART.h"
#include <math.h>

#define MS5803_ON  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)
#define MS5803_OFF HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET)

uint16_t MS5803_C1;
uint16_t MS5803_C2;
uint16_t MS5803_C3;
uint16_t MS5803_C4;
uint16_t MS5803_C5;
uint16_t MS5803_C6;

uint32_t MS5803_D1;
uint32_t MS5803_D2;

_MS5803 ms5803;

static int d1d2_convert_step = 0;

SPI_HandleTypeDef *MS5803_Handler;

float ms5803_lowpass_0N3D_maxflat_filter_pressure(float raw)
{
	//输入输出缓存
	static float out[4];
	int i;
	//滤波系数
	float cd[4] = { 1, -2.87438796994731f, 2.75654128930279f, -0.881920488787064f };	//H(Z)分母系数
	float cn0 =  0.000232830568416609f;	//H(Z)分子系数
	//滤波过程
	out[0] = (-cd[1] * out[1] - cd[2] * out[2] - cd[3] * out[3] + cn0 * raw) / cd[0];
	for (i = 4; i >= 2; i--)
	{
		out[i - 1] = out[i - 2];
	}
	//超前滤波
	out[0] = 2 * out[0] - out[3];
	return (float)out[0];
}

void MS5803_Init(SPI_HandleTypeDef *hspi)
{
	MS5803_Handler = hspi;
	MS5803_reset();
	//C1
	uint8_t MS5803_C1_buffer[2];//
	MS5803_ReadRegs(MS5803_PROM_1, MS5803_C1_buffer, 2);
	MS5803_C1 = ((uint16_t)(*(MS5803_C1_buffer) << 8) | *(MS5803_C1_buffer + 1));
	//C2
	uint8_t MS5803_C2_buffer[2];//
	MS5803_ReadRegs(MS5803_PROM_2, MS5803_C2_buffer, 2);
	MS5803_C2 = ((uint16_t)(*(MS5803_C2_buffer) << 8) | *(MS5803_C2_buffer + 1));
	//C3
	uint8_t MS5803_C3_buffer[2];//
	MS5803_ReadRegs(MS5803_PROM_3, MS5803_C3_buffer, 2);
	MS5803_C3 = ((uint16_t)(*(MS5803_C3_buffer) << 8) | *(MS5803_C3_buffer + 1));
	//C4
	uint8_t MS5803_C4_buffer[2];//
	MS5803_ReadRegs(MS5803_PROM_4, MS5803_C4_buffer, 2);
	MS5803_C4 = ((uint16_t)(*(MS5803_C4_buffer) << 8) | *(MS5803_C4_buffer + 1));
	//C5
	uint8_t MS5803_C5_buffer[2];//
	MS5803_ReadRegs(MS5803_PROM_5, MS5803_C5_buffer, 2);
	MS5803_C5 = ((uint16_t)(*(MS5803_C5_buffer) << 8) | *(MS5803_C5_buffer + 1));
	//C6
	uint8_t MS5803_C6_buffer[2];//
	MS5803_ReadRegs(MS5803_PROM_6, MS5803_C6_buffer, 2);
	MS5803_C6 = ((uint16_t)(*(MS5803_C6_buffer) << 8) | *(MS5803_C6_buffer + 1));
}

void MS5803_process(void)
{
	MS5803_read_data();
	MS5803_caculate();
}

void MS5803_data_push(void)
{

}

void MS5803_caculate(void)
{
	//气压高度计算需要的变量与参数
	static int ms5803_start_cnt;
	const float _R = 8.314f;	//气体常数
	const float _G = 9.80665f;	//重力加速度
	const float _M = 29.0f;		//空气分子量
	const float _K0 = 273.15;	//热力学温度


	int64_t dT = MS5803_D2 - MS5803_C5 * 256;
	int64_t TEMP = 2000 + dT * MS5803_C6 / (8388608);
	int64_t T2;
	int64_t OFF2;
	int64_t SENS2;
	int64_t OFF;
	int64_t SENS;
	int64_t P;
	OFF = (int64_t)MS5803_C2 * (65536) + (int64_t)(MS5803_C4 * dT) / (128);
	SENS = MS5803_C1 * (32768) + (MS5803_C3 * dT) / (256);
	if(TEMP < 2000)
	{
		T2 = (dT * dT) / 2147483648;
		OFF2 = 3 * (TEMP - 2000) * (TEMP - 2000);
		SENS2 = 7 * (TEMP - 2000) * (TEMP - 2000) / 8;
		if(TEMP < -1500)
		{
			SENS2 = SENS2 + 2 * (TEMP + 1500) * (TEMP + 1500);
		}
	}
	else
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
		if(TEMP >= 4500)
		{
			SENS2 = SENS2 - (TEMP - 4500) * (TEMP - 4500) / 8;
		}
	}
	TEMP = TEMP - T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;
	P = (MS5803_D1 * SENS / (2097152) - OFF) / (32768);
	ms5803.temperature = TEMP*0.01f;//��
	ms5803.pressure = P*0.01f;//mbar
	//ms5803.pressure = ms5803_lowpass_0N3D_maxflat_filter_pressure(P*0.01f);//mbar
	//计算气压高度
	if(ms5803_start_cnt<200)
	{
		ms5803_start_cnt ++ ;
	}
	if(ms5803_start_cnt>=200 && ms5803_start_cnt<203)
	{
		ms5803.pressure_launch += ms5803.pressure;
		ms5803_start_cnt ++ ;
	}
	if(ms5803_start_cnt==203)
	{
		ms5803_start_cnt = 10000 ;
		ms5803.pressure_launch = ms5803.pressure_launch / 3.0f;
	}
	if(ms5803_start_cnt == 10000)
	{
		ms5803.height_from_launch_point = -1000.0f*(_R*(ms5803.temperature+_K0)/(_M*_G))*log(ms5803.pressure/ms5803.pressure_launch);
		ms5803.height_from_launch_point = ms5803_lowpass_0N3D_maxflat_filter_pressure(ms5803.height_from_launch_point);
	}
	
}

void MS5803_read_data(void)
{
	//D1
	if(d1d2_convert_step==0)
	{
		MS5803_WriteReg(MS5803_CVTD1_OSR4096);
		d1d2_convert_step++;
		return;
	}
	if(d1d2_convert_step==1)
	{
		uint8_t MS5803_D1_buffer[3];
		MS5803_ReadRegs(MS5803_ADC_READ,MS5803_D1_buffer,3);
		MS5803_D1 = ((uint32_t)(*(MS5803_D1_buffer) << 16) | *(MS5803_D1_buffer + 1) << 8 | *(MS5803_D1_buffer));
		MS5803_WriteReg(MS5803_CVTD2_OSR4096);
		d1d2_convert_step++;
		return;
	}
	if(d1d2_convert_step==2)
	{
		uint8_t MS5803_D2_buffer[3];
		MS5803_ReadRegs(MS5803_ADC_READ,MS5803_D2_buffer,3);
		MS5803_D2 = ((uint32_t)(*(MS5803_D2_buffer) << 16) | *(MS5803_D2_buffer + 1) << 8 | *(MS5803_D2_buffer));
		d1d2_convert_step = 0;
	}
	if(d1d2_convert_step<0 || d1d2_convert_step>2)
	{
		d1d2_convert_step = 0;
	}
}

uint8_t MS5803_ReadReg(uint8_t ReadAddr) {
	MS5803_ON;
	uint8_t ReadData = 0;
	uint8_t tx = ReadAddr;
	HAL_SPI_Transmit(MS5803_Handler, &tx, 1, 100);
	HAL_SPI_Receive(MS5803_Handler, &ReadData, 1, 100);
	MS5803_OFF;
	return ReadData;
}

void MS5803_ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, uint8_t Bytes) {
	MS5803_ON;
	uint8_t tx = ReadAddr;
	HAL_SPI_Transmit(MS5803_Handler, &tx, 1, 100);
	HAL_SPI_Receive(MS5803_Handler, ReadBuf, Bytes, 100);
	MS5803_OFF;
}

void MS5803_WriteReg(uint8_t WriteData) {
	MS5803_ON;
	HAL_SPI_Transmit(MS5803_Handler, &WriteData, 1, 100);
	MS5803_OFF;
}

void MS5803_reset(void) {
	MS5803_WriteReg(MS5803_RESET);
	HAL_Delay(50);
}


