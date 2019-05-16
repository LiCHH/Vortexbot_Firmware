/**
 * @file chassis_task.c
 * @version 1.0
 * date October 2018
 *
 * @brief control chassis action
 *
 */
#include "math.h"
#include "stdlib.h"
#include "string.h"

#include "cmsis_os.h"

#include "sys_config.h"
#include "chassis_task.h"
#include "pid.h"
#include "bsp_can.h"
#include "remote_ctrl.h"
#include "vortexbot_info.h"
// #include "steer_ctrl.h"
#include "dm_motor_drive.h"

#include "test_ctrl.h"

chassis_t chassis;

static void omnidirection_handler(void);
static void differential_handler(void);
static void carlike_handler(void);
static void stop_handler(void);

void get_chassis_info(void)
{
  for (int i = 0; i < 4; ++i)
  {
    chassis.driving_spd_fdb[i] = motor_driving[i].speed_rpm;

    // chassis.steer_spd_fdb[i] = motor_steer[i].speed_rpm;
    // chassis.steer_pos_fdb[i] = motor_steer[i].total_angle;
  }
}

void chassis_task(void const *argu)
{
  get_chassis_info();

  switch (chassis.ctrl_mode)
  {
  case OMNI_DIRECTIONAL:
  {
    omnidirection_handler();
  }
  break;

  case DIFFERENTIAL:
  {
    differential_handler();
  }
  break;

  case CAR_LIKE:
  {
    carlike_handler();
  }
  break;

  default:
  {
    stop_handler();
  }
  break;
  }

  for (int i = 0; i < 4; ++i)
  {
    // Steering DOF's double loop PID control
    // pid_calc(&pid_steer_pos[i], chassis.steer_pos_fdb[i], chassis.steer_pos_ref[i]);
    // chassis.steer_spd_ref[i] = pid_steer_pos[i].out;
    // pid_calc(&pid_steer_spd[i], chassis.steer_spd_fdb[i], chassis.steer_spd_ref[i]);
    // chassis.steer_current[i] = pid_steer_spd[i].out;

    // Driving DOF's speed loop PID control
    pid_calc(&pid_driving_spd[i], chassis.driving_spd_fdb[i], chassis.driving_spd_ref[i]);
    chassis.driving_current[i] = pid_driving_spd[i].out;
  }
  send_control_msgs();
}

static void omnidirection_handler(void)
{
  // if (chassis.last_ctrl_mode == CHASSIS_STOP || chassis.last_ctrl_mode == CHASSIS_RELAX)
  // {
  //   chassis.steer_pos_ref[fr_motor] = (FR_BL_POS_F * OMNI_INIT_ANGLE + STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
  //   chassis.steer_pos_ref[bl_motor] = (FR_BL_POS_F * OMNI_INIT_ANGLE + STEER_BL_OFFSET) * MOTOR_REDUCTION_RATIO;
  //   chassis.steer_pos_ref[fl_motor] = (FL_BR_POS_F * OMNI_INIT_ANGLE + STEER_FL_OFFSET) * MOTOR_REDUCTION_RATIO;
  //   chassis.steer_pos_ref[br_motor] = (FL_BR_POS_F * OMNI_INIT_ANGLE + STEER_BR_OFFSET) * MOTOR_REDUCTION_RATIO;
  // }

  // else
  // {

  if (bot_mode == MANUL_CONTROL_MODE)
  {
    chassis.power_ratio = (float)(rc_info.r_rocker_ud - ROCKER_MIN) / ROCKER_RANGE;
    //! 防止抖动
    chassis.power_ratio = (chassis.power_ratio > 0.05f) ? chassis.power_ratio : 0;
    chassis.vx = rc_info.l_rocker_ud;
    chassis.vy = rc_info.l_rocker_lr;
  }

  float angle;
  if (fabs(chassis.vy) < FLOAT_THRESHOLD)
  {
    angle = 0;
  }
  else if (fabs(chassis.vx) < FLOAT_THRESHOLD)
    angle = SIGN(chassis.vy) * 90;
  else
    angle = atan2(chassis.vy, fabs(chassis.vx)) * RAD_TO_DEG;

  // set_servo_pos(FR_BL_POS_F * OMNI_INIT_ANGLE + angle,
  //               FR_BL_POS_F * OMNI_INIT_ANGLE + angle,
  //               FL_BR_POS_F * OMNI_INIT_ANGLE + angle,
  //               FL_BR_POS_F * OMNI_INIT_ANGLE + angle);

  chassis.steer_pos_ref[fr_motor] = FR_BL_POS_F * OMNI_INIT_ANGLE + 180 - angle; //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[bl_motor] = FR_BL_POS_F * OMNI_INIT_ANGLE + 180 - angle; //+ STEER_BL_OFFSET) * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[fl_motor] = FL_BR_POS_F * OMNI_INIT_ANGLE + 180 - angle; //+ STEER_FL_OFFSET) * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[br_motor] = FL_BR_POS_F * OMNI_INIT_ANGLE + 180 - angle; //+ STEER_BR_OFFSET) * MOTOR_REDUCTION_RATIO;

  setDMMotorBuf(fr_motor, chassis.steer_pos_ref[fr_motor] * 100, 0);
  setDMMotorBuf(bl_motor, chassis.steer_pos_ref[bl_motor] * 100, 0);
  setDMMotorBuf(fl_motor, chassis.steer_pos_ref[fl_motor] * 100, 0);
  setDMMotorBuf(br_motor, chassis.steer_pos_ref[br_motor] * 100, 0);


  int16_t spd_ref = 0;
  if (!(fabs(chassis.vx) < FLOAT_THRESHOLD && fabs(chassis.vy) < FLOAT_THRESHOLD))
    spd_ref = SIGN(chassis.vx) * MOTOR_SPEED_MAX * chassis.power_ratio * MOTOR_REDUCTION_RATIO;
  chassis.driving_spd_ref[fr_motor] = FR_BR_SPD_F * spd_ref;
  chassis.driving_spd_ref[br_motor] = FR_BR_SPD_F * spd_ref;
  chassis.driving_spd_ref[fl_motor] = FL_BL_SPD_F * spd_ref;
  chassis.driving_spd_ref[bl_motor] = FL_BL_SPD_F * spd_ref;

  // }
}

static void differential_handler(void)
{
}

static void carlike_handler(void)
{
}

void chassis_param_init(void)
{
  memset(&chassis, 0, sizeof(chassis));

  for (int i = 0; i < 4; i++)
  {
    PID_struct_init(&pid_driving_spd[i], POSITION_PID, 10000, 500, 2.5f, 0.03f, 0.f);
    PID_struct_init(&pid_steer_spd[i], POSITION_PID, 10000, 500, 4.5f, 0.01f, 0.f);
    PID_struct_init(&pid_steer_pos[i], POSITION_PID, 17000, 80, 3.f, 0.01f, 0.f);
  }

  chassis.steer_pos_ref[fr_motor] = 180; //STEER_FR_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[bl_motor] = 180; //STEER_BL_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[fl_motor] = 180; //STEER_FL_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[br_motor] = 180; //STEER_BR_OFFSET * MOTOR_REDUCTION_RATIO;
  // set_servo_pos();
}

void send_control_msgs(void)
{
  send_chassis_current(CAN_LOW_ID, chassis.driving_current[0], chassis.driving_current[1], chassis.driving_current[2], chassis.driving_current[3]);
  // send_chassis_current(CAN_HIGH_ID, chassis.steer_current[0], chassis.steer_current[1], chassis.steer_current[2], chassis.steer_current[3]);
  // send_servo_packet();
  for(uint8_t i = 0; i < 4; ++i) {
    sendDMMotor(i);
  }

}

static void stop_handler(void)
{
  chassis.steer_pos_ref[fr_motor] = 180; //STEER_FR_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[bl_motor] = 180; //STEER_BL_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[fl_motor] = 180; //STEER_FL_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[br_motor] = 180; //STEER_BR_OFFSET * MOTOR_REDUCTION_RATIO;
  // set_servo_pos();
  send_control_msgs();

  memset(&chassis.driving_spd_ref, 0, sizeof(chassis.driving_spd_ref));
}
