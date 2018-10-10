/**
 * @file chassis_task.h
 * @version 1.0
 * date October 2018
 *
 * @brief control chassis action
 *
 */

#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "stm32f4xx_hal.h"

typedef struct
{
  int16_t wheel_spd_fdb[4];
  int16_t wheel_spd_ref[4]; // 空载最高在0x0400左右，即455rpm

  int16_t wheel_current[4];
} chassis_t;

void chassis_task(void);
void chassis_param_init(void);
void send_control_msgs(void);

extern chassis_t chassis;

#endif // __CHASSIS_TASK_H__
