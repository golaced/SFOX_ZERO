#ifndef _TIME_H_
#define _TIME_H_

#define GET_TIME_NUM 	10		//设置获取时间的数组数量

//时间计数
enum
{
  NOW = 0,
  OLD,
  NEW,
};

enum
{
  app_ano_time_index = 0,
  app_backend_time_index,
  mpu9250_time_index,
};

typedef struct
{
	float det_t_app_ano_s;
        float det_t_app_backend_s;
        float det_t_mpu9250_s;
	
}Det_t;
extern Det_t det_t_s;
extern unsigned long long time_consume[GET_TIME_NUM][2];
extern int app_ano_time_comsume_us;
extern int app_backend_time_comsume_us;
extern int mpu9250_time_comsume_us;

extern volatile unsigned long long sysTickUptime;
extern float cycle_T[20][3];

extern float get_cycle_time(int item);
extern unsigned int get_sys_time_us(void);
extern void time_inc_tick(void);

#endif
