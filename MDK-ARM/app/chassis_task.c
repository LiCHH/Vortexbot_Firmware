/**
 * @file chassis_task.c
 * @version 1.0
 * date October 2018
 *
 * @brief control chassis action
 *
 */

#include "chassis_task.h"
#include "stdlib.h"
#include "pid.h"
#include "string.h"
#include "bsp_can.h"

chassis_t chassis;

void get_chassis_info(void)
{
  for(int i = 0; i < 4; ++i)
  {
    chassis.wheel_spd_fdb[i] = moto_chassis[i].speed_rpm;
    chassis.wheel_pos_fdb[i] = moto_chassis[i].total_angle;
  }
}

void chassis_task(void)
{
  get_chassis_info();
  for(int i = 0; i < 4; ++i)
  {
    pid_calc(&pid_chassis_pos[i], chassis.wheel_pos_fdb[i], chassis.wheel_pos_ref[i]);
    chassis.wheel_spd_ref[i] = pid_chassis_pos[i].out;
    pid_calc(&pid_driving_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
    chassis.wheel_current[i] = pid_driving_spd[i].out;
  }
  send_control_msgs();
}

void chassis_param_init(void)
{
  memset(&chassis, 0, sizeof(chassis));

  for (int i = 0; i < 4; i++)
  {
    PID_struct_init(&pid_driving_spd[i], POSITION_PID, 10000, 500, 4.5f, 0.0f, 0.f);
    PID_struct_init(&pid_steer_spd[i], POSITION_PID, 10000, 500, 4.5f, 0.0f, 0.f);
    PID_struct_init(&pid_steer_pos[i], POSITION_PID, 600, 5, 3.f, 0.03f, 0.f);
  }
}

void send_control_msgs(void)
{
  send_chassis_current(chassis.wheel_current[0], chassis.wheel_current[1], chassis.wheel_current[2], chassis.wheel_current[3]);
}
