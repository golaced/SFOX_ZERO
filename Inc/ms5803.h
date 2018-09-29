#ifndef _MS5803_H_
#define _MS5803_H_

#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

#define MS5803_RESET			((uint8_t)0x1E)
#define MS5803_CVTD1_OSR256		((uint8_t)0x40)
#define MS5803_CVTD1_OSR512		((uint8_t)0x42)
#define MS5803_CVTD1_OSR1024	((uint8_t)0x44)
#define MS5803_CVTD1_OSR2048	((uint8_t)0x46)
#define MS5803_CVTD1_OSR4096	((uint8_t)0x48)
#define MS5803_CVTD2_OSR256		((uint8_t)0x50)
#define MS5803_CVTD2_OSR512		((uint8_t)0x52)
#define MS5803_CVTD2_OSR1024	((uint8_t)0x54)
#define MS5803_CVTD2_OSR2048	((uint8_t)0x56)
#define MS5803_CVTD2_OSR4096	((uint8_t)0x58)
#define MS5803_ADC_READ			((uint8_t)0x00)
#define MS5803_PROM_0			((uint8_t)0xA0)
#define MS5803_PROM_1			((uint8_t)0xA2)
#define MS5803_PROM_2			((uint8_t)0xA4)
#define MS5803_PROM_3			((uint8_t)0xA6)
#define MS5803_PROM_4			((uint8_t)0xA8)
#define MS5803_PROM_5			((uint8_t)0xAA)
#define MS5803_PROM_6			((uint8_t)0xAC)
#define MS5803_PROM_7			((uint8_t)0xAE)

extern SPI_HandleTypeDef *MS5803_Handler;

typedef struct{
    double pressure;
    double temperature;
    float pressure_launch;
    float height_from_launch_point;
}_MS5803;

extern _MS5803 ms5803;

// extern uint16_t MS5803_C1;
// extern uint16_t MS5803_C2;
// extern uint16_t MS5803_C3;
// extern uint16_t MS5803_C4;
// extern uint16_t MS5803_C5;
// extern uint16_t MS5803_C6;

// extern uint32_t MS5803_D1;
// extern uint32_t MS5803_D2;

void MS5803_Init(SPI_HandleTypeDef *hspi);
uint8_t MS5803_ReadReg(uint8_t ReadAddr);
void MS5803_ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, uint8_t Bytes);
void MS5803_WriteReg(uint8_t WriteData);
void MS5803_reset(void);
void MS5803_read_data(void);
void MS5803_caculate(void);
void MS5803_data_push(void);
void MS5803_process(void);

#endif
