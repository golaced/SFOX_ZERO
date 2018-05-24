#include <USART.h>
#include "app_ano.h"
#include "usart_cfg.h"
#include "app_ros.h"
#include "uwb1000.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

uint8_t aRxBuffer1[1];
uint8_t aRxBuffer2[1];
uint8_t aRxBuffer3[1];
uint8_t aRxBuffer5[1];
uint8_t aRxBuffer6[1];


void USART_init(void)
{
	if (HAL_UART_Receive_IT(&huart1, aRxBuffer1, 1) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);
	if (HAL_UART_Receive_IT(&huart2, aRxBuffer2, 1) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);
	if (HAL_UART_Receive_IT(&huart3, aRxBuffer3, 1) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);
        if (HAL_UART_Receive_IT(&huart5, aRxBuffer5, 1) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);
	if (HAL_UART_Receive_IT(&huart6, aRxBuffer6, 1) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART1)
  {
	  HAL_UART_Receive_IT(&huart1,aRxBuffer1,1);//开启下一次接收中断
	  ANO_data_receive_prepara(aRxBuffer1[0]);
  }
  if(huart->Instance == USART2)
  {
	  HAL_UART_Receive_IT(&huart2,aRxBuffer2,1);//开启下一次接收中断
          uwb1000_buffer_fill(aRxBuffer2[0]);
  }
  if(huart->Instance == USART3)
  {
	  HAL_UART_Receive_IT(&huart3,aRxBuffer3,1);//开启下一次接收中断
  }
  if(huart->Instance == UART5)
  {
	  HAL_UART_Receive_IT(&huart5,aRxBuffer5,1);//开启下一次接收中断
          ros_data_receive_prepara(aRxBuffer5[0]);
  }
  if(huart->Instance == USART6)
  {
	  HAL_UART_Receive_IT(&huart6,aRxBuffer6,1);//开启下一次接收中断
  }
}

//DMA发送函数
void USART1_DMA_send_data(uint8_t *pdata, uint16_t Length)
{
	HAL_UART_Transmit_DMA(&huart1, pdata, Length);
}

void USART2_DMA_send_data(uint8_t *pdata, uint16_t Length)
{
	HAL_UART_Transmit_DMA(&huart2, pdata, Length);
}

void USART3_DMA_send_data(uint8_t *pdata, uint16_t Length)
{
	HAL_UART_Transmit_DMA(&huart3, pdata, Length);
}

void USART4_DMA_send_data(uint8_t *pdata, uint16_t Length)
{
	HAL_UART_Transmit_DMA(&huart4, pdata, Length);
}

void USART6_DMA_send_data(uint8_t *pdata, uint16_t Length)
{
	HAL_UART_Transmit_DMA(&huart6, pdata, Length);
}

