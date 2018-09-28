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
  normal_task_time_index = 0,
  realtime_task_time_index,
  gps_m8n_task_time_index,
  gps_m8n_update_time_index,
  mpu9250_process_time_index,
  app_ins_time_index,
  app_ctrl_time_index,
};

typedef struct
{
  float det_t_normal_task_s;
  float det_t_realtime_task_s;
  float det_t_gps_m8n_s;
  float det_t_gps_m8n_update_s;
  float det_t_mpu9250_process_s;
  float det_t_app_ins_s;
  float det_t_app_ctrl_s;

}Det_t;

extern Det_t det_t_s;
extern unsigned long long time_consume[GET_TIME_NUM][2];
extern int normal_time_consume_us;
extern int real_time_consume_us;


extern volatile unsigned long long sysTickUptime;
extern float cycle_T[20][3];

extern float get_cycle_time(int item);
extern unsigned int get_sys_time_us(void);
extern void time_inc_tick(void);

#endif
