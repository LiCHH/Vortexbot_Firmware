#ifndef __ODOM_TASK_H__
#define __ODOM_TASK_H__

#include "stm32f4xx_hal.h"

#define ODOM_TASK_PERIOD 5

typedef struct 
{
  float x;
  float y;
  float theta;

  float trans;
  float rot;
  float last_theta;
  float last_rot;

  float last_motor_angle;
  float curr_motor_angle;

} odom_t;

void odomTaskInit(void);
void odom_task(void const* argu);

extern odom_t odom;

#endif // 
