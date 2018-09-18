#include "app_ros.h"
#include "usart_cfg.h"
#include "mpu9250.h"
#include "uwb1000.h"
#include "app_ins.h"

#define BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 0) )
#define BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )

float ros_a_x, ros_a_y, ros_a_z;
float ros_b_x, ros_b_y, ros_b_z;
float ros_c_x, ros_c_y, ros_c_z;
float ros_d_x, ros_d_y, ros_d_z;
float ros_e_x, ros_e_y, ros_e_z;

static uint8_t ros_data_to_send[200];	//
static uint8_t RxBuffer[200];		//

static int state;
static int _data_len, _data_cnt;
static int data_from_ros_updata;

void ros_sending(void)
{
  ros_send_15_data(
  (int32_t)(accx_raw_mps*1000),(int32_t)(accy_raw_mps*1000),(int32_t)(accz_raw_mps*1000),
  (int32_t)(gyrox_raw_dps*1000),(int32_t)(gyroy_raw_dps*1000),(int32_t)(gyroz_raw_dps*1000),
  (int32_t)(magx_raw_uT),(int32_t)(magy_raw_uT),(int32_t)(magz_raw_uT),
  (int32_t)(anchor[0].distance_mm),(int32_t)(anchor[1].distance_mm),(int32_t)(anchor[2].distance_mm),
  (int32_t)(anchor[3].distance_mm),(int32_t)(anchor[4].distance_mm),(int32_t)(anchor[5].distance_mm));
}

int32_t ros_usart_send_data(uint8_t *dataToSend, uint8_t length)
{
  USART4_DMA_send_data(dataToSend, length);
  return 0;
}

void ros_data_receive_prepara(uint8_t data)
{
  if (state == 0 && data == 0xBB)
  {
    state = 1;
    RxBuffer[0] = data;
  }
  else if (state == 1 && data == 0xBB)
  {
    state = 2;
    RxBuffer[1] = data;
  }
  else if (state == 2 && data == 0X02)
  {
    state = 3;
    RxBuffer[2] = data;
  }
  else if (state == 3 && data == 60)
  {
    state = 4;
    RxBuffer[3] = data;
    _data_len = data;
    _data_cnt = 0;
  }
  else if (state == 4 && _data_len>0)
  {
    _data_len--;
    RxBuffer[4 + _data_cnt++] = data;
    if (_data_len == 0)
            state = 5;
  }
  else if (state == 5)
  {
    state = 0;
    RxBuffer[4 + _data_cnt] = data;
    data_from_ros_updata = 1;
  }
  else
  {
    state = 0;
  }
  
  if (data_from_ros_updata == 1)
  {
    if(ros_data_receive_analyse(RxBuffer, _data_cnt + 5)==0)
    {
      data_from_ros_updata = 1;
    }
  }
}


void data_trans_with_ros()
{
  /*---------------obtain data from ANO_DT-----------*/
  if (data_from_ros_updata == 1)
  {
    if(ros_data_receive_analyse(RxBuffer, _data_cnt + 5)==0)
    {
      data_from_ros_updata = 1;
    }
    return;
  }
}

int32_t ros_data_receive_analyse(uint8_t *data_buf, uint8_t num)
{
  //�ж�sum
  uint8_t sum = 0;
  for(uint8_t i = 0; i < (num-1);i ++)
  {
    sum += data_buf[i];
  }
  if (!(sum == data_buf[num-1]))
  {
          return -1;
  }
  //�ж�֡ͷ
  if (!(*(data_buf) == 0xBB && *(data_buf + 1) == 0xBB))
  {
          return -1;
  }
  //��������
  if (*(data_buf + 2) == 0X02)
  {
    ros_a_x = 0.001f*(int32_t)(data_buf[4]<<24 | data_buf[5]<<16 | data_buf[6]<<8 | data_buf[7]);
    ros_a_y = 0.001f*(int32_t)(data_buf[8]<<24 | data_buf[9]<<16 | data_buf[10]<<8 | data_buf[11]);
    ros_a_z = 0.001f*(int32_t)(data_buf[12]<<24 | data_buf[13]<<16 | data_buf[14]<<8 | data_buf[15]);

    ros_b_x = 0.001f*(int32_t)(data_buf[16]<<24 | data_buf[17]<<16 | data_buf[18]<<8 | data_buf[19]);
    ros_b_y = 0.001f*(int32_t)(data_buf[20]<<24 | data_buf[21]<<16 | data_buf[22]<<8 | data_buf[23]);
    ros_b_z = 0.001f*(int32_t)(data_buf[24]<<24 | data_buf[25]<<16 | data_buf[26]<<8 | data_buf[27]);

    ros_c_x = 0.001f*(int32_t)(data_buf[28]<<24 | data_buf[29]<<16 | data_buf[30]<<8 | data_buf[31]);
    ros_c_y = 0.001f*(int32_t)(data_buf[32]<<24 | data_buf[33]<<16 | data_buf[34]<<8 | data_buf[35]);
    ros_c_z = 0.001f*(int32_t)(data_buf[36]<<24 | data_buf[37]<<16 | data_buf[38]<<8 | data_buf[39]);

    ros_d_x = 0.001f*(int32_t)(data_buf[40]<<24 | data_buf[41]<<16 | data_buf[42]<<8 | data_buf[43]);
    ros_d_y = 0.001f*(int32_t)(data_buf[44]<<24 | data_buf[45]<<16 | data_buf[46]<<8 | data_buf[47]);
    ros_d_z = 0.001f*(int32_t)(data_buf[48]<<24 | data_buf[49]<<16 | data_buf[50]<<8 | data_buf[51]);

    ros_e_x = 0.001f*(int32_t)(data_buf[52]<<24 | data_buf[53]<<16 | data_buf[54]<<8 | data_buf[55]);
    ros_e_y = 0.001f*(int32_t)(data_buf[56]<<24 | data_buf[57]<<16 | data_buf[58]<<8 | data_buf[59]);
    ros_e_z = 0.001f*(int32_t)(data_buf[60]<<24 | data_buf[61]<<16 | data_buf[62]<<8 | data_buf[63]);

    return 0;
  }
  return -1;
}

void ros_send_15_data(
  int32_t a_x, int32_t a_y, int32_t a_z,
  int32_t b_x, int32_t b_y, int32_t b_z,
  int32_t c_x, int32_t c_y, int32_t c_z,
  int32_t d_x, int32_t d_y, int32_t d_z,
  int32_t e_x, int32_t e_y, int32_t e_z)
{
  uint8_t _cnt = 0;
  __IO int32_t _temp;
  ros_data_to_send[_cnt++] = 0xAA;
  ros_data_to_send[_cnt++] = 0xAA;
  ros_data_to_send[_cnt++] = 0x02;
  ros_data_to_send[_cnt++] = 0;

  _temp = a_x;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);
  _temp = a_y;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);
  _temp = a_z;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);

  _temp = b_x;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);
  _temp = b_y;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);
  _temp = b_z;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);

  _temp = c_x;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);
  _temp = c_y;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);
  _temp = c_z;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);

  _temp = d_x;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);
  _temp = d_y;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);
  _temp = d_z;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);

  _temp = e_x;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);
  _temp = e_y;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);
  _temp = e_z;
  ros_data_to_send[_cnt++] = BYTE3(_temp);
  ros_data_to_send[_cnt++] = BYTE2(_temp);
  ros_data_to_send[_cnt++] = BYTE1(_temp);
  ros_data_to_send[_cnt++] = BYTE0(_temp);

  ros_data_to_send[3] = _cnt - 4;

  uint8_t sum = 0;
  for (uint8_t i = 0; i < _cnt; i++)
          sum += ros_data_to_send[i];
  ros_data_to_send[_cnt++] = sum;
  ros_data_to_send[_cnt++] = 0x0A;

  ros_usart_send_data(ros_data_to_send, _cnt);
}



int app_ros_thread(void)
{
  ros_sending();
  data_trans_with_ros();
  return 0;
}
