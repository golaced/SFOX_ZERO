#include "motor_pwm.h"
#include "RC_PWM.h"

int motor1_output;
int motor2_output;
int motor3_output;
int motor4_output;

void motor_esc_init(void)
{
	int esc_max = 2000;
	int esc_min = 1000;
	//判断遥控器油门位置，如果油门置于高位，则进入ESC行程校正：1000~2000
	if(RC_PWM[2]>=1900)
	{
		motor_pwm_set_value(esc_max,esc_max,esc_max,esc_max);
		HAL_Delay(4000);
		motor_pwm_set_value(esc_min,esc_min,esc_min,esc_min);
		HAL_Delay(3000);
	}
	//校正结束后，遥控器油门通道需要手动拉低，防止电机猛转
	while(RC_PWM[2]>=1500)
	{
		;
	}
	//判断是否有遥控器数据，如果没有，将油门置于最低点
	while((RC_PWM[0]+RC_PWM[1]+RC_PWM[2]+RC_PWM[3])==0)
	{
		motor_pwm_set_value(esc_min,esc_min,esc_min,esc_min);
		HAL_Delay(1000);
	}
}

void motor_pwm_set_value(int motor1, int motor2, int motor3, int motor4)
{
	timer4_channel4_PD15_PWM_setvalue(motor1);
	timer4_channel3_PD14_PWM_setvalue(motor2);
	timer4_channel2_PD13_PWM_setvalue(motor3);
	timer4_channel1_PD12_PWM_setvalue(motor4);
}

void timer3_channel1_PA6_PWM_setvalue(uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void timer3_channel2_PA7_PWM_setvalue(uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void timer3_channel3_PB0_PWM_setvalue(uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}
void timer3_channel4_PB1_PWM_setvalue(uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void timer4_channel1_PD12_PWM_setvalue(uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}
void timer4_channel2_PD13_PWM_setvalue(uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}
void timer4_channel3_PD14_PWM_setvalue(uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
void timer4_channel4_PD15_PWM_setvalue(uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}
