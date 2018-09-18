#ifndef _MOTOR_PWM_H_
#define _MOTOR_PWM_H_

#include "stm32f4xx_hal.h"

//400Hz   Couter Period:2500���� ���� 0.0025ms
//��ÿ��������1�������ӳ�1/1000����
//��ͨ������:�������壬���1msͣת��2ms��������ת
//���������ƣ���С1000�����2000
// #define PWM_MIN_OUT 1000
// #define PWM_MAX_OUT 2000

extern int motor1_output;
extern int motor2_output;
extern int motor3_output;
extern int motor4_output;

extern int motor_esc_init_finish;

extern void motor_pwm_set_value(int motor1, int motor2, int motor3, int motor4);

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern void motor_esc_init(void);

void timer3_channel1_PA6_PWM_setvalue(uint16_t value);
void timer3_channel2_PA7_PWM_setvalue(uint16_t value);
void timer3_channel3_PB0_PWM_setvalue(uint16_t value);
void timer3_channel4_PB1_PWM_setvalue(uint16_t value);

void timer4_channel1_PD12_PWM_setvalue(uint16_t value);
void timer4_channel2_PD13_PWM_setvalue(uint16_t value);
void timer4_channel3_PD14_PWM_setvalue(uint16_t value);
void timer4_channel4_PD15_PWM_setvalue(uint16_t value);

#endif
