#ifndef __UWB_INFO_H__ 
#define __UWB_INFO_H__

#include "stm32f4xx_hal.h"

#define UWB_BUF_LEN 20
#define UWB_MAX_BUF 50

#define UWB_UPDATE_SIGNAL (1 << 0)

typedef struct 
{
  int32_t raw_x;
  int32_t raw_y;

  float out_x;
  float out_y;
} uwb_data_t;

extern uwb_data_t uwb_data;

extern uint8_t uwb_buff[];

void uwb_receive_callback(uwb_data_t* uwb_data, uint8_t* buf);

#endif // !1
