#include "ms5803.h"
#include "stm32f4xx_hal.h"
#include "USART.h"

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

double MS5803_pressure;
double MS5803_temperature;

static int d1d2_convert_step = 0;

SPI_HandleTypeDef *MS5803_Handler;

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
	MS5803_temperature = TEMP*0.01f;//¡æ
	MS5803_pressure = P*0.01f;//mbar
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


