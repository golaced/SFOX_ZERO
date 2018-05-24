#ifndef _APP_ANO_H_
#define _APP_ANO_H_

#include "stm32f4xx_hal.h"

extern void app_ano_parameter_init(void);

extern float rol_p, rol_i, rol_d;
extern float pit_p, pit_i, pit_d;
extern float yaw_p, yaw_i, yaw_d;

extern float alt_p, alt_i, alt_d;
extern float pos_p, pos_i, pos_d;
extern float pid6_p, pid6_i, pid6_d;

extern float pid7_p, pid7_i, pid7_d;
extern float pid8_p, pid8_i, pid8_d;
extern float pid9_p, pid9_i, pid9_d;

extern float pid10_p, pid10_i, pid10_d;
extern float pid11_p, pid11_i, pid11_d;
extern float pid12_p, pid12_i, pid12_d;

extern float pid13_p, pid13_i, pid13_d;
extern float pid14_p, pid14_i, pid14_d;
extern float pid15_p, pid15_i, pid15_d;

extern float pid16_p, pid16_i, pid16_d;
extern float pid17_p, pid17_i, pid17_d;
extern float pid18_p, pid18_i, pid18_d;

void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum);
void ANO_send_15_data(
		int16_t a_x, int16_t a_y, int16_t a_z,
		int16_t g_x, int16_t g_y, int16_t g_z,
		int16_t m_x, int16_t m_y, int16_t m_z,
	float angle_rol, float angle_pit, float angle_yaw,
	int32_t alt, uint8_t fly_model, uint8_t armed);
int32_t ANO_usart_send_data(uint8_t *dataToSend, uint8_t length);
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf, uint8_t num);
extern void ANO_data_receive_prepara(uint8_t data);
extern int ano_process(int mode);

#endif
