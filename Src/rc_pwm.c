#include "rc_pwm.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

//timer1
uint32_t timer1_channel1_PE9_IC1Value1;
uint32_t timer1_channel1_PE9_IC1Value2;
uint32_t timer1_channel1_PE9_DiffCapture;
//
uint32_t timer1_channel2_PE11_IC1Value1;
uint32_t timer1_channel2_PE11_IC1Value2;
uint32_t timer1_channel2_PE11_DiffCapture;
//
uint32_t timer1_channel3_PE13_IC1Value1;
uint32_t timer1_channel3_PE13_IC1Value2;
uint32_t timer1_channel3_PE13_DiffCapture;

uint32_t timer1_channel4_PE14_IC1Value1;
uint32_t timer1_channel4_PE14_IC1Value2;
uint32_t timer1_channel4_PE14_DiffCapture;

//timer2
uint32_t timer2_channel1_PA5_IC1Value1;
uint32_t timer2_channel1_PA5_IC1Value2;
uint32_t timer2_channel1_PA5_DiffCapture;

uint32_t timer2_channel2_PA1_IC1Value1;
uint32_t timer2_channel2_PA1_IC1Value2;
uint32_t timer2_channel2_PA1_DiffCapture;

uint32_t timer2_channel3_PB10_IC1Value1;
uint32_t timer2_channel3_PB10_IC1Value2;
uint32_t timer2_channel3_PB10_DiffCapture;

uint32_t timer2_channel4_PB11_IC1Value1;
uint32_t timer2_channel4_PB11_IC1Value2;
uint32_t timer2_channel4_PB11_DiffCapture;

int RC_PWM[8];

//function

void RC_PWM_init(void)
{
	 HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
	 HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
	 HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
	 HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);

	 HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	 HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	 HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
	 HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4);

	//空余时间，捕获遥控器通道
  	HAL_Delay(500);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1)
	{
		//
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			//��ȡ��ʱIO�ڵĵ�ƽ,Ϊ��,����εĲ�����������
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9) == GPIO_PIN_SET)
			{
				/* Get the 1st Input Capture value */
				timer1_channel1_PE9_IC1Value1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
			}
			else
			{  //�½���
				/* Get the 2st Input Capture value */
				timer1_channel1_PE9_IC1Value2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
								//2����1,˵����ͬһ������������,ֱ�Ӽ�
				if(timer1_channel1_PE9_IC1Value2 > timer1_channel1_PE9_IC1Value1)
				{
					timer1_channel1_PE9_DiffCapture = timer1_channel1_PE9_IC1Value2 - timer1_channel1_PE9_IC1Value1;
				}
				else
				{ //2С��1,����һ������������,�����ڼ�1�ļ���,�ټ�ȥ2,�ó���ʵ����
					timer1_channel1_PE9_DiffCapture = (__HAL_TIM_GET_AUTORELOAD(&htim1) -timer1_channel1_PE9_IC1Value1 + 1) + timer1_channel1_PE9_IC1Value2;
				}
				RC_PWM[7] = (int)(timer1_channel1_PE9_DiffCapture * 0.25f);
			}
		}
		//
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			//��ȡ��ʱIO�ڵĵ�ƽ,Ϊ��,����εĲ�����������
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11) == GPIO_PIN_SET)
			{
				/* Get the 1st Input Capture value */
				timer1_channel2_PE11_IC1Value1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
			}
			else
			{  //�½���
				/* Get the 2st Input Capture value */
				timer1_channel2_PE11_IC1Value2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
								//2����1,˵����ͬһ������������,ֱ�Ӽ�
				if(timer1_channel2_PE11_IC1Value2 > timer1_channel2_PE11_IC1Value1)
				{
					timer1_channel2_PE11_DiffCapture = timer1_channel2_PE11_IC1Value2 - timer1_channel2_PE11_IC1Value1;
				}
				else
				{ //2С��1,����һ������������,�����ڼ�1�ļ���,�ټ�ȥ2,�ó���ʵ����
					timer1_channel2_PE11_DiffCapture = (__HAL_TIM_GET_AUTORELOAD(&htim1) -timer1_channel2_PE11_IC1Value1 + 1) + timer1_channel2_PE11_IC1Value2;
				}
				RC_PWM[6] = (int)(timer1_channel2_PE11_DiffCapture * 0.25f);
			}
		}
		//
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			//��ȡ��ʱIO�ڵĵ�ƽ,Ϊ��,����εĲ�����������
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13) == GPIO_PIN_SET)
			{
				/* Get the 1st Input Capture value */
				timer1_channel3_PE13_IC1Value1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);
			}
			else
			{  //�½���
				/* Get the 2st Input Capture value */
				timer1_channel3_PE13_IC1Value2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);
								//2����1,˵����ͬһ������������,ֱ�Ӽ�
				if(timer1_channel3_PE13_IC1Value2 > timer1_channel3_PE13_IC1Value1)
				{
					timer1_channel3_PE13_DiffCapture = timer1_channel3_PE13_IC1Value2 - timer1_channel3_PE13_IC1Value1;
				}
				else
				{ //2С��1,����һ������������,�����ڼ�1�ļ���,�ټ�ȥ2,�ó���ʵ����
					timer1_channel3_PE13_DiffCapture = (__HAL_TIM_GET_AUTORELOAD(&htim1) -timer1_channel3_PE13_IC1Value1 + 1) + timer1_channel3_PE13_IC1Value2;
				}
				RC_PWM[5] = (int)(timer1_channel3_PE13_DiffCapture * 0.25f);
			}
		}
		//
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			//��ȡ��ʱIO�ڵĵ�ƽ,Ϊ��,����εĲ�����������
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14) == GPIO_PIN_SET)
			{
				/* Get the 1st Input Capture value */
				timer1_channel4_PE14_IC1Value1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4);
			}
			else
			{  //�½���
				/* Get the 2st Input Capture value */
				timer1_channel4_PE14_IC1Value2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4);
								//2����1,˵����ͬһ������������,ֱ�Ӽ�
				if(timer1_channel4_PE14_IC1Value2 > timer1_channel4_PE14_IC1Value1)
				{
					timer1_channel4_PE14_DiffCapture = timer1_channel4_PE14_IC1Value2 - timer1_channel4_PE14_IC1Value1;
				}
				else
				{ //2С��1,����һ������������,�����ڼ�1�ļ���,�ټ�ȥ2,�ó���ʵ����
					timer1_channel4_PE14_DiffCapture = (__HAL_TIM_GET_AUTORELOAD(&htim1) -timer1_channel4_PE14_IC1Value1 + 1) + timer1_channel4_PE14_IC1Value2;
				}
				RC_PWM[4] = (int)(timer1_channel4_PE14_DiffCapture * 0.25f);
			}
		}
	}

	//
	if(htim->Instance==TIM2)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			//��ȡ��ʱIO�ڵĵ�ƽ,Ϊ��,����εĲ�����������
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5) == GPIO_PIN_SET)
			{
				/* Get the 1st Input Capture value */
				timer2_channel1_PA5_IC1Value1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
			}
			else
			{  //�½���
				/* Get the 2st Input Capture value */
				timer2_channel1_PA5_IC1Value2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
								//2����1,˵����ͬһ������������,ֱ�Ӽ�
				if(timer2_channel1_PA5_IC1Value2 > timer2_channel1_PA5_IC1Value1)
				{
					timer2_channel1_PA5_DiffCapture = timer2_channel1_PA5_IC1Value2 - timer2_channel1_PA5_IC1Value1;
				}
				else
				{ //2С��1,����һ������������,�����ڼ�1�ļ���,�ټ�ȥ2,�ó���ʵ����
					timer2_channel1_PA5_DiffCapture = (__HAL_TIM_GET_AUTORELOAD(&htim2) -timer2_channel1_PA5_IC1Value1 + 1) + timer2_channel1_PA5_IC1Value2;
				}
				RC_PWM[3] = (int)(timer2_channel1_PA5_DiffCapture * 0.25f);
			}
		}
		//
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			//��ȡ��ʱIO�ڵĵ�ƽ,Ϊ��,����εĲ�����������
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1) == GPIO_PIN_SET)
			{
				/* Get the 1st Input Capture value */
				timer2_channel2_PA1_IC1Value1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
			}
			else
			{  //�½���
				/* Get the 2st Input Capture value */
				timer2_channel2_PA1_IC1Value2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
								//2����1,˵����ͬһ������������,ֱ�Ӽ�
				if(timer2_channel2_PA1_IC1Value2 > timer2_channel2_PA1_IC1Value1)
				{
					timer2_channel2_PA1_DiffCapture = timer2_channel2_PA1_IC1Value2 - timer2_channel2_PA1_IC1Value1;
				}
				else
				{ //2С��1,����һ������������,�����ڼ�1�ļ���,�ټ�ȥ2,�ó���ʵ����
					timer2_channel2_PA1_DiffCapture = (__HAL_TIM_GET_AUTORELOAD(&htim2) -timer2_channel2_PA1_IC1Value1 + 1) + timer2_channel2_PA1_IC1Value2;
				}
				RC_PWM[2] = (int)(timer2_channel2_PA1_DiffCapture * 0.25f);
			}
		}
		//
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			//��ȡ��ʱIO�ڵĵ�ƽ,Ϊ��,����εĲ�����������
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10) == GPIO_PIN_SET)
			{
				/* Get the 1st Input Capture value */
				timer2_channel3_PB10_IC1Value1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
			}else
			{  //�½���
				/* Get the 2st Input Capture value */
				timer2_channel3_PB10_IC1Value2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
								//2����1,˵����ͬһ������������,ֱ�Ӽ�
				if(timer2_channel3_PB10_IC1Value2 > timer2_channel3_PB10_IC1Value1)
				{
					timer2_channel3_PB10_DiffCapture = timer2_channel3_PB10_IC1Value2 - timer2_channel3_PB10_IC1Value1;
				}
				else
				{ //2С��1,����һ������������,�����ڼ�1�ļ���,�ټ�ȥ2,�ó���ʵ����
					timer2_channel3_PB10_DiffCapture = (__HAL_TIM_GET_AUTORELOAD(&htim2) -timer2_channel3_PB10_IC1Value1 + 1) + timer2_channel3_PB10_IC1Value2;
				}
				RC_PWM[1] = (int)(timer2_channel3_PB10_DiffCapture * 0.25f);
			}
		}
		//
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			//��ȡ��ʱIO�ڵĵ�ƽ,Ϊ��,����εĲ�����������
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11) == GPIO_PIN_SET)
			{
				/* Get the 1st Input Capture value */
				timer2_channel4_PB11_IC1Value1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);
			}
			else
			{  //�½���
				/* Get the 2st Input Capture value */
				timer2_channel4_PB11_IC1Value2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);
								//2����1,˵����ͬһ������������,ֱ�Ӽ�
				if(timer2_channel4_PB11_IC1Value2 > timer2_channel4_PB11_IC1Value1)
				{
					timer2_channel4_PB11_DiffCapture = timer2_channel4_PB11_IC1Value2 - timer2_channel4_PB11_IC1Value1;
				}
				else
				{ //2С��1,����һ������������,�����ڼ�1�ļ���,�ټ�ȥ2,�ó���ʵ����
					timer2_channel4_PB11_DiffCapture = (__HAL_TIM_GET_AUTORELOAD(&htim2) -timer2_channel4_PB11_IC1Value1 + 1) + timer2_channel4_PB11_IC1Value2;
				}
				RC_PWM[0] = (int)(timer2_channel4_PB11_DiffCapture * 0.25f);
			}
		}
	}
}
