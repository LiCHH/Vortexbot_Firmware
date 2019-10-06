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

#define CHASSIS_TIMER_PERIOD 20

typedef enum
{
  CHASSIS_RELAX       = 0,
  CHASSIS_STOP        = 1,
  OMNI_DIRECTIONAL    = 2,
  FORWARD_DIRECTIONAL = 3,
  DIFFERENTIAL        = 4,
  CAR_LIKE            = 5
} chassis_mode_e;

typedef struct
{
  float vx;
  float vy;
  float vw;

  float mv_direction;

  float power_ratio;

  chassis_mode_e ctrl_mode;
  chassis_mode_e last_ctrl_mode;

  int16_t driving_spd_fdb[4];
  int16_t driving_spd_ref[4]; // 空载最高在521*36rpm左右，即输出轴521rpm
  int16_t driving_pos_fdb[4];
  int16_t driving_pos_ref[4]; // 空载最高在521*36rpm左右，即输出轴521rpm
  int16_t driving_current[4];

  int16_t steer_spd_fdb[4];
  int16_t steer_spd_ref[4];
  int16_t steer_pos_fdb[4];
  int16_t steer_pos_ref[4];
  int16_t steer_current[4];

} chassis_t;

void chassis_task(void const *argu);
void chassis_param_init(void);
void send_control_msgs(void);

extern chassis_t chassis;

#endif // __CHASSIS_TASK_H__
