#ifndef UWB1000_H_
#define UWB1000_H_

#include <stdint.h>

#define UWB1000_BUF_LEN 25
#define UWB1000_NUM_COMMA 4
#define ANCHOR_MAX_NUM 10

struct  ANCHOR
{
      int sourceAddress;
      uint8_t DISTANCE[8];
      int distance_mm;
      uint8_t RSSI[5];
      float rssi;
      double latitude;
      double longitude;
      float altitude;
      int local_x;
      int local_y;
      int local_z;
      int update;
      int home_anchor;//yes:1 no:0
};

extern uint8_t uwb1000_buffer[UWB1000_BUF_LEN];
extern uint8_t ANCHOR_INDEX[2];
extern uint8_t anchor_index;
extern uint8_t data_parse_state;
extern struct ANCHOR anchor[ANCHOR_MAX_NUM];
extern struct ANCHOR home_anchor;

void uwb1000_data_prepare(void);
int uwb1000_str_find(uint8_t *str, uint8_t *ptr);
int uwb1000_data_push();
void uwb1000_buffer_fill(uint8_t data);
int uwb1000_process(void);

#endif
