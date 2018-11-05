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
  for (int i = 0; i < 4; ++i)
  {
    chassis.driving_spd_fdb[i]   = motor_driving[i].speed_rpm;
    
    chassis.steer_spd_fdb[i]     = motor_steer[i].speed_rpm;
    chassis.steer_pos_fdb[i]     = motor_steer[i].total_angle;
  }
}

void chassis_task(void)
{
  get_chassis_info();
  for (int i = 0; i < 4; ++i)
  {
    // Steering DOF's double loop PID control
    pid_calc(&pid_steer_pos[i], chassis.steer_pos_fdb[i], chassis.steer_pos_ref[i]);
    chassis.steer_spd_ref[i] = pid_steer_pos[i].out;
    // chassis.steer_spd_ref[i] = chassis.steer_pos_ref[i];
    pid_calc(&pid_steer_spd[i], chassis.steer_spd_fdb[i], chassis.steer_spd_ref[i]);
    chassis.steer_current[i] = pid_steer_spd[i].out;

    // Driving DOF's speed loop PID control
    pid_calc(&pid_driving_spd[i], chassis.driving_spd_fdb[i], chassis.driving_spd_ref[i]);
    chassis.driving_current[i] = pid_driving_spd[i].out;
  }
  send_control_msgs();
}

void chassis_param_init(void)
{
  memset(&chassis, 0, sizeof(chassis));

  for (int i = 0; i < 4; i++)
  {
    PID_struct_init(&pid_driving_spd[i], POSITION_PID, 10000, 500, 4.5f, 0.05f, 0.f);
    PID_struct_init(&pid_steer_spd[i], POSITION_PID, 10000, 500, 4.5f, 0.03f, 0.f);
    PID_struct_init(&pid_steer_pos[i], POSITION_PID, 17000, 80, 3.f, 0.03f, 0.f);
  }
}

void send_control_msgs(void)
{
  send_chassis_current(CAN_LOW_ID, chassis.driving_current[0], chassis.driving_current[1], chassis.driving_current[2], chassis.driving_current[3]);
  send_chassis_current(CAN_HIGH_ID, chassis.steer_current[0], chassis.steer_current[1], chassis.steer_current[2], chassis.steer_current[3]);
}
