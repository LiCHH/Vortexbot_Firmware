/**
 * @file chassis_task.c
 * @version 1.0
 * date October 2018
 *
 * @brief control chassis action
 *
 */

#include "math.h"

#include "sys_config.h"
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
    chassis.driving_spd_fdb[i] = motor_driving[i].speed_rpm;

    chassis.steer_spd_fdb[i] = motor_steer[i].speed_rpm;
    chassis.steer_pos_fdb[i] = motor_steer[i].total_angle;
  }
}

void chassis_task(void const *argu)
{
  get_chassis_info();

  switch(chassis.ctrl_mode)
  {
    case OMNI_DIRECTIONAL:
    {
      omnidirection_handler();
    } break;

    case DIFFERENTIAL:
    {
      differential_handler();
    } break;

    case CAR_LIKE:
    {
      carlike_handler();
    } break;

    default:
    {
    } break;
  }

  for (int i = 0; i < 4; ++i)
  {
    // Steering DOF's double loop PID control
    pid_calc(&pid_steer_pos[i], chassis.steer_pos_fdb[i], chassis.steer_pos_ref[i]);
    chassis.steer_spd_ref[i] = pid_steer_pos[i].out;
    pid_calc(&pid_steer_spd[i], chassis.steer_spd_fdb[i], chassis.steer_spd_ref[i]);
    chassis.steer_current[i] = pid_steer_spd[i].out;

    // Driving DOF's speed loop PID control
    pid_calc(&pid_driving_spd[i], chassis.driving_spd_fdb[i], chassis.driving_spd_ref[i]);
    chassis.driving_current[i] = pid_driving_spd[i].out;
  }
  send_control_msgs();
}

static void omnidirection_handler(void)
{
  float angle;
  if (fabs(chassis.vx) < FLOAT_THRESHOLD)
    angle = 90;
  else
    // TODO: check the output of atan2
    angle = atan2(chassis.vy, chassis.vx) * RAD_TO_ANG;

  chassis.steer_pos_ref[fr_motor] = FR_BL_FLAG * angle + STEER_FR_OFFSET;
  chassis.steer_pos_ref[bl_motor] = FR_BL_FLAG * angle + STEER_BL_OFFSET;
  chassis.steer_pos_ref[fl_motor] = FL_BR_FLAG * angle + STEER_FL_OFFSET;
  chassis.steer_pos_ref[br_motor] = FL_BR_FLAG * angle + STEER_BR_OFFSET;

  // TODO: check spd and where to get vx, vy
  int16_t spd_ref = sqrt(chassis.vx * chassis.vx + chassis.vy * chassis.vy);
  for (int i = 0; i < 4; ++i)
  {
    chassis.driving_spd_ref[i] = spd_ref;
  }
}

static void differential_handler(void)
{

}

static void carlike_handler(void)
{}

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
