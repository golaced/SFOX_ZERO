#ifndef _USART_CFG_H_
#define _USART1_CFG_H_

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

extern uint8_t aRxBuffer1[1];
extern uint8_t aRxBuffer2[1];
extern uint8_t aRxBuffer3[1];
extern uint8_t aRxBuffer5[1];
extern uint8_t aRxBuffer6[1];

extern void USART1_DMA_send_data(uint8_t *pdata, uint16_t Length);
extern void USART2_DMA_send_data(uint8_t *pdata, uint16_t Length);
extern void USART3_DMA_send_data(uint8_t *pdata, uint16_t Length);
extern void USART4_DMA_send_data(uint8_t *pdata, uint16_t Length);
extern void USART6_DMA_send_data(uint8_t *pdata, uint16_t Length);
extern void USART_init(void);


#endif
