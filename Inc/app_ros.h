#ifndef _APP_ROS_H_
#define _APP_ROS_H_

#include "stm32f4xx_hal.h"

extern int32_t ros_a_x, ros_a_y, ros_a_z;
extern int32_t ros_b_x, ros_b_y, ros_b_z;
extern int32_t ros_c_x, ros_c_y, ros_c_z;
extern int32_t ros_d_x, ros_d_y, ros_d_z;
extern int32_t ros_e_x, ros_e_y, ros_e_z;

int32_t ros_usart_send_data(uint8_t *dataToSend, uint8_t length);

void data_trans_with_ano(void);

int32_t ros_data_receive_analyse(uint8_t *data_buf, uint8_t num);

void ros_send_15_data(
  int32_t a_x, int32_t a_y, int32_t a_z,
  int32_t b_x, int32_t b_y, int32_t b_z,
  int32_t c_x, int32_t c_y, int32_t c_z,
  int32_t d_x, int32_t d_y, int32_t d_z,
  int32_t e_x, int32_t e_y, int32_t e_z
 );

void ros_sending(void);

extern int app_ros_thread(void);
extern void ros_data_receive_prepara(uint8_t data);

#endif
