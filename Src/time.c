#include "time.h"
#include "stm32f4xx_hal.h"

volatile unsigned long long sysTickUptime;
float cycle_T[20][3];
unsigned long long time_consume[GET_TIME_NUM][2];
int app_ano_time_comsume_us;
int app_backend_time_comsume_us;
int mpu9250_time_comsume_us;
Det_t det_t_s;

float get_cycle_time(int item)
{
  cycle_T[item][OLD] = cycle_T[item][NOW];	//上一次的时间
  cycle_T[item][NOW] = get_sys_time_us()/1000000.0f; //本次的时间
  cycle_T[item][NEW] = ( ( cycle_T[item][NOW] - cycle_T[item][OLD] ) );//间隔的时间（周期）
  return cycle_T[item][NEW];
}

unsigned int get_sys_time_us(void)
{
  register unsigned int ms;
  unsigned int s_load;
  unsigned int s_val;
  unsigned int value;
  s_load =SysTick->LOAD ;
  s_val=SysTick->VAL;

  ms = sysTickUptime;
  value = ms * 1000 + (s_load - s_val) * 1000 / s_load;
  return value;
}

void time_inc_tick(void)
{
  sysTickUptime++;
}