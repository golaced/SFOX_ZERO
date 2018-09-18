#ifndef _TIME_H_
#define _TIME_H_

#define GET_TIME_NUM 	10		//���û�ȡʱ�����������

//ʱ�����
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
  app_uwb_time_index,
};

typedef struct
{
	float det_t_app_ano_s;
  float det_t_app_backend_s;
  float det_t_mpu9250_s;
  float det_t_app_uwb_s;
	
}Det_t;
extern Det_t det_t_s;
extern unsigned long long time_consume[GET_TIME_NUM][2];
extern int app_ano_time_consume_us;
extern int app_backend_time_consume_us;
extern int mpu9250_time_consume_us;
extern int app_uwb_time_consume_us;

extern volatile unsigned long long sysTickUptime;
extern float cycle_T[20][3];

extern float get_cycle_time(int item);
extern unsigned int get_sys_time_us(void);
extern void time_inc_tick(void);

#endif
