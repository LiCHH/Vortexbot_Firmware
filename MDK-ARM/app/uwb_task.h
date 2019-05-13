#ifndef __UWB_TASK_H__
#define __UWB_TASK_H__

#include "stm32f4xx_hal.h"

#define UWB_TASK_PERIOD 25

typedef struct 
{
  float x;
  float y;
} uwb_info_t;

void uwb_task(void const *argu);

#endif // !__UWB_TASK_H__
