#ifndef __LOCALIZATION_TASK_H__
#define __LOCALIZATION_TASK_H__

#include "stm32f4xx_hal.h"

// #define USE_UWB

typedef struct
{
  float last_trans;
  float curr_trans;
  float delta_trans;

  float last_theta;
  float curr_theta;

  float rot1;
  float rot2;

  float odom_x;
  float odom_y;
  float odom_theta;

  float last_odom_x;
  float last_odom_y;
  float last_odom_theta;

} odometry_t;

typedef struct
{
  float x;
  float y;
  float theta;
} localization_t;

extern odometry_t odom;
extern localization_t localization;

#endif // !1