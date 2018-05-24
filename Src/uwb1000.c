#include "uwb1000.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

uint8_t uwb1000_buffer[UWB1000_BUF_LEN];
uint8_t ANCHOR_INDEX[2];
uint8_t anchor_index;
uint8_t data_parse_state;
struct ANCHOR anchor[ANCHOR_MAX_NUM];
struct ANCHOR home_anchor;

static int _data_len;//the number of comma
static int _data_cnt;
static int _buf_index;
static int _header_index;
static int _comma_index;
//static int _buf_head_index;
static bool parse_ready;

int uwb1000_process(void)
{
	uwb1000_data_push();
	return 0;
}

void uwb1000_data_prepare(void)
{
  uint8_t *pstr;
  static uint8_t Sbuf;
  _header_index = uwb1000_str_find(uwb1000_buffer, (uint8_t*)"$UWB,");
  _comma_index = 0;
  _buf_index = 0;
  pstr = uwb1000_buffer + _header_index + 4;
  do
  {
          Sbuf = *pstr++;
          switch (Sbuf)
          {
          case ',':_comma_index++;  //通过逗号的数目来进行状态分类
                  _buf_index = 0;
                  break;
          default:
                  switch (_comma_index)
                  {
                          //
                          case 0:
                          {
                                  ANCHOR_INDEX[_buf_index] = Sbuf;
                                  if (*pstr == ',')
                                  {
                                          ANCHOR_INDEX[_buf_index + 1] = '\0';
                                          anchor_index = atoi((char*)ANCHOR_INDEX)-1;
                                          anchor[anchor_index].sourceAddress = anchor_index;
                                          anchor[anchor_index].update = 1;
                                  }
                                  break;
                          }
                          //
                          case 1:
                          {
                                  anchor[anchor_index].DISTANCE[_buf_index] = Sbuf;
                                  if (*pstr == ',')
                                  {
                                          anchor[anchor_index].DISTANCE[_buf_index + 1] = '\0';
                                          anchor[anchor_index].distance_mm = atoi((char*)anchor[anchor_index].DISTANCE);
                                  }
                                  break;
                          }
                          //
                          case 2:
                          {
                                  anchor[anchor_index].RSSI[_buf_index] = Sbuf;
                                  if (*pstr == ',')
                                  {
                                          anchor[anchor_index].RSSI[_buf_index + 1] = '\0';
                                          anchor[anchor_index].rssi = atoi((char*)anchor[anchor_index].RSSI);
                                  }
                                  break;
                          }
                          default:break;
                  }
                  _buf_index++;	//
                  break;
          }
  } while (Sbuf != '*');//until obtain '*'
}

int data_push()
{
  
  return 0;
}

void uwb1000_buffer_fill(uint8_t data)
{
  if (data_parse_state == 0 && data == '$')
  {
          data_parse_state = 1;
          uwb1000_buffer[0] = data;
  }
  else if (data_parse_state == 1 && data == 'U')
  {
          data_parse_state = 2;
          uwb1000_buffer[1] = data;
  }
  else if (data_parse_state == 2 && data == 'W')
  {
          data_parse_state = 3;
          uwb1000_buffer[2] = data;
  }
  else if (data_parse_state == 3 && data == 'B')
  {
          data_parse_state = 4;
          uwb1000_buffer[3] = data;
          _data_len = UWB1000_NUM_COMMA;
          _data_cnt = 0;
  }
  else if (data_parse_state == 4 && _data_len >= 0)
  {
          _data_cnt++;
          uwb1000_buffer[3 + _data_cnt] = data;

          if (data == ',')
          {
                  _data_len--;
                  if (_data_len == 0)
                  {
                          uwb1000_buffer[4 + _data_cnt + 1] = '*';
                          parse_ready = true;
                          data_parse_state = 0;
                  }
          }
  }
  else
          data_parse_state = 0;
  if (parse_ready == true)
  {
          uwb1000_data_prepare();
          parse_ready = false;
  }
}

int uwb1000_str_find(uint8_t *str, uint8_t *ptr)
{
  uint16_t index = 0;
  uint8_t *STemp = NULL;
  uint8_t *PTemp = NULL;
  uint8_t *MTemp = NULL;
  if (0 == str || 0 == ptr)
          return 0;
  for (STemp = str; *STemp != '\0'; STemp++)	 //依次查找字符串
  {
          index++;  //当前偏移量加1
          MTemp = STemp; //指向当前字符串
                                     //比较
          for (PTemp = ptr; *PTemp != '\0'; PTemp++)
          {
                  if (*PTemp != *MTemp)
                          break;
                  MTemp++;
          }
          if (*PTemp == '\0')  //出现了所要查找的字符，退出
                  break;
  }
  return index;
}
