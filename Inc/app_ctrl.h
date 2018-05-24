#ifndef _APP_CTRL_H_
#define _APP_CTRL_H_

#include "stm32f4xx_hal.h"

extern void app_ctrl_thread(float dT);
void attitude_ctrl(void);
void height_ctrl(void);
void horizontal_ctrl(void);
void remote_control_data_obtain(void);
void gyro_data_obtain(void);
void write_motor(uint16_t throttle, int roll, int pitch, int yaw);

#endif
