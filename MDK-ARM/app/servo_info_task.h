#ifndef __SERVO_INFO_TASK_H__
#define __SERVO_INFO_TASK_H__ 

#include "stm32f4xx_hal.h"


typedef struct 
{
  int16_t curr_position;
  int16_t last_position;
  int16_t curr_speed;
  int16_t last_speed;

  float angle;

} servo_info_t;

extern servo_info_t servo_infos[];

void servo_info_task(void const *argu);
#endif // 
