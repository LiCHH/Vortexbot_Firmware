#include "motion_task.h"

#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "cmsis_os.h"

#include "sys_config.h"
#include "pid.h"
#include "bsp_can.h"
#include "remote_ctrl.h"
#include "vortexbot_info.h"
#include "dm_motor_drive.h"
#include "test_ctrl.h"
#include "bsp_uart.h"
#include "odom_task.h"
#include "attitude_control.h"

static void omnidirection_handler(void);
static void differential_handler(void);
static void carlike_handler(void);
static void forward_handler(void);
static void stop_handler(void);
static void attitude_control_handler(void);

void motion_task(void const *argu) {
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

  case FORWARD_DIRECTIONAL:
  {
    forward_handler();
  }
  break;

  case ATTITUDE_CONTROL:
  {
    attitude_control_handler();
  }
  break;

  default:
  {
    stop_handler();
  }
  break;
  }

}

static void omnidirection_handler(void)
{
  //! 判断当前是否为静止状态
  static int stop_flag = 0;
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
  else if (fabs(chassis.vx) < FLOAT_THRESHOLD) {
    angle = SIGN(chassis.vy) * 90;
  }
  else {
    angle = atan2(chassis.vy, fabs(chassis.vx)) * RAD_TO_DEG;
  }
    
  chassis.steer_pos_ref[fr_motor] = 100 * (FR_POS_F * OMNI_INIT_FRONT_ANGLE + STEER_INIT_ANGLE_FR - angle + STEER_FR_OFFSET); //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[fl_motor] = 100 * (FL_POS_F * OMNI_INIT_FRONT_ANGLE + STEER_INIT_ANGLE_FL - angle + STEER_FL_OFFSET); //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[bl_motor] = 100 * (BL_POS_F * OMNI_INIT_BACK_ANGLE + STEER_INIT_ANGLE_BL - angle + STEER_BL_OFFSET); //+ STEER_BL_OFFSET) * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[br_motor] = 100 * (BR_POS_F * OMNI_INIT_BACK_ANGLE + STEER_INIT_ANGLE_BR - angle + STEER_BR_OFFSET); //+ STEER_BR_OFFSET) * MOTOR_REDUCTION_RATIO;

  //! test code
  // sprintf(test_buf, "vx: %.2f vy: %.2f %.2f\r\n", chassis.vx, chassis.vy, angle);
  // HAL_UART_Transmit(&TEST_HUART, test_buf, 40, 10);

  setDMMotorBuf(fr_motor , chassis.steer_pos_ref[fr_motor]);
  setDMMotorBuf(fl_motor , chassis.steer_pos_ref[fl_motor]);
  setDMMotorBuf(bl_motor, chassis.steer_pos_ref[bl_motor]);
  setDMMotorBuf(br_motor, chassis.steer_pos_ref[br_motor]);

  int16_t spd_ref = 0;
  if (!(fabs(chassis.vx) < FLOAT_THRESHOLD && fabs(chassis.vy) < FLOAT_THRESHOLD)) {
    spd_ref = SIGN(chassis.vx) * MOTOR_SPEED_MAX * chassis.power_ratio * MOTOR_REDUCTION_RATIO;
    stop_flag = 1;

    chassis.driving_spd_ref[fr_motor]  = FR_SPD_F  * spd_ref;
    chassis.driving_spd_ref[fl_motor]  = FL_SPD_F  * spd_ref;
    chassis.driving_spd_ref[br_motor] = BR_SPD_F * spd_ref;
    chassis.driving_spd_ref[bl_motor] = BL_SPD_F * spd_ref;

  }
  else {
    //! 如果是静止状态，则加入位置环，以保证在墙上不滑动
    if(stop_flag) {
      for(int i = 0; i < 4; ++i) {
        chassis.driving_pos_ref[i] = motor_driving[i].total_angle;
      }
      stop_flag = 0;
    }
    for(int i = 0; i < 4; ++i) {
      pid_calc(&pid_driving_pos[i], chassis.driving_pos_fdb[i], chassis.driving_pos_ref[i]);
      chassis.driving_spd_ref[i] = pid_driving_pos[i].out;
    }
  }

  chassis.mv_direction = -angle;
  // }
}

static void differential_handler(void)
{
}

static void carlike_handler(void)
{
}

void chassis_task_init(void)
{
  memset(&chassis, 0, sizeof(chassis));

  for (int i = 0; i < 4; i++)
  {
    PID_struct_init(&pid_driving_spd[i], POSITION_PID, 10000, 500, 4.f, 0.01f, 0.f);
    PID_struct_init(&pid_driving_pos[i], POSITION_PID, 17000, 80, 3.f, 0.01f, 0.01f);
    PID_struct_init(&pid_steer_spd[i], POSITION_PID, 10000, 500, 4.5f, 0.01f, 0.f);
    PID_struct_init(&pid_steer_pos[i], POSITION_PID, 17000, 80, 3.f, 0.01f, 0.f);
  }

  // DMMotorAngleInit();
  //! 增加45°偏置可以比较顺利地初始化
  chassis.steer_pos_ref[fr_motor] = 100 * (STEER_INIT_ANGLE_FR + STEER_FR_OFFSET); //STEER_FR_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[fl_motor] = 100 * (STEER_INIT_ANGLE_FL + STEER_FL_OFFSET); //STEER_FR_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[bl_motor] = 100 * (STEER_INIT_ANGLE_BL + STEER_BL_OFFSET); //STEER_BL_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[br_motor] = 100 * (STEER_INIT_ANGLE_BR + STEER_BR_OFFSET); //STEER_BR_OFFSET * MOTOR_REDUCTION_RATIO;
  setDMMotorBuf(fr_motor , chassis.steer_pos_ref[fr_motor]);
  setDMMotorBuf(fl_motor , chassis.steer_pos_ref[fl_motor]);
  setDMMotorBuf(bl_motor, chassis.steer_pos_ref[bl_motor]);
  setDMMotorBuf(br_motor, chassis.steer_pos_ref[br_motor]);
  // setDMMotorBufWithDirection(f_motor , chassis.steer_pos_ref[f_motor] * 100, init_rotate_direction[f_motor]);
  // setDMMotorBufWithDirection(bl_motor, chassis.steer_pos_ref[bl_motor] * 100, init_rotate_direction[bl_motor]);
  // setDMMotorBufWithDirection(br_motor, chassis.steer_pos_ref[br_motor] * 100, init_rotate_direction[br_motor]);
  for(int i = 0; i < 4; ++i) {
    sendDMMotor(i);
    HAL_Delay(1);
  }

  attitude_control_init();
}

static void stop_handler(void)
{
  chassis.steer_pos_ref[fr_motor] = 100 * (STEER_INIT_ANGLE_FR + STEER_FR_OFFSET); //STEER_FR_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[fl_motor] = 100 * (STEER_INIT_ANGLE_FL + STEER_FL_OFFSET); //STEER_FR_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[bl_motor] = 100 * (STEER_INIT_ANGLE_BL + STEER_BL_OFFSET); //STEER_BL_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[br_motor] = 100 * (STEER_INIT_ANGLE_BR + STEER_BR_OFFSET); //STEER_BR_OFFSET * MOTOR_REDUCTION_RATIO;
  setDMMotorBuf(fr_motor , chassis.steer_pos_ref[fr_motor]);
  setDMMotorBuf(fl_motor , chassis.steer_pos_ref[fl_motor]);
  setDMMotorBuf(bl_motor, chassis.steer_pos_ref[bl_motor]);
  setDMMotorBuf(br_motor, chassis.steer_pos_ref[br_motor]);

  memset(&chassis.driving_spd_ref, 0, sizeof(chassis.driving_spd_ref));
  // send_control_msgs();
}

static void forward_handler(void)
{
  static int stop_flag = 0;
  chassis.steer_pos_ref[fr_motor] = 100 * (STEER_INIT_ANGLE_FR + STEER_FR_OFFSET); //STEER_FR_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[fl_motor] = 100 * (STEER_INIT_ANGLE_FL + STEER_FL_OFFSET); //STEER_FR_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[bl_motor] = 100 * (STEER_INIT_ANGLE_BL + STEER_BL_OFFSET); //STEER_BL_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[br_motor] = 100 * (STEER_INIT_ANGLE_BR + STEER_BR_OFFSET); //STEER_BR_OFFSET * MOTOR_REDUCTION_RATIO;

  setDMMotorBuf(fr_motor , chassis.steer_pos_ref[fr_motor]);
  setDMMotorBuf(fl_motor , chassis.steer_pos_ref[fl_motor]);
  setDMMotorBuf(bl_motor, chassis.steer_pos_ref[bl_motor]);
  setDMMotorBuf(br_motor, chassis.steer_pos_ref[br_motor]);

  if (bot_mode == MANUL_CONTROL_MODE)
  {
    chassis.power_ratio = (float)(rc_info.r_rocker_ud - ROCKER_MIN) / ROCKER_RANGE;
    //! 防止抖动
    chassis.power_ratio = (chassis.power_ratio > 0.05f) ? chassis.power_ratio : 0;
    chassis.vx = rc_info.l_rocker_ud;
  }

  int16_t spd_ref = 0;
  if (fabs(chassis.vx) > FLOAT_THRESHOLD) {
    spd_ref = SIGN(chassis.vx) * MOTOR_SPEED_MAX * chassis.power_ratio * MOTOR_REDUCTION_RATIO;
    stop_flag = 1;
    chassis.driving_spd_ref[fr_motor]  = FR_SPD_F  * spd_ref;
    chassis.driving_spd_ref[fl_motor]  = -FL_SPD_F  * spd_ref;
    chassis.driving_spd_ref[br_motor] = BR_SPD_F * spd_ref;
    chassis.driving_spd_ref[bl_motor] = -BL_SPD_F * spd_ref;
  }
  else {
    if(stop_flag) {
      for(int i = 0; i < 4; ++i) {
        chassis.driving_pos_ref[i] = motor_driving[i].total_angle; // + motor_driving[i].speed_rpm * 60.0 * DEG_TO_RAD;
      }
      stop_flag = 0;
    }
    for(int i = 0; i < 4; ++i) {
      pid_calc(&pid_driving_pos[i], chassis.driving_pos_fdb[i], chassis.driving_pos_ref[i]);
      chassis.driving_spd_ref[i] = pid_driving_pos[i].out;
    }
    return;
  }
}

static void attitude_control_handler(void)
{
  //! 判断当前是否为静止状态
  static int stop_flag = 0;

  if (bot_mode == MANUL_CONTROL_MODE)
  {
    chassis.power_ratio = (float)(rc_info.r_rocker_ud - ROCKER_MIN) / ROCKER_RANGE;
    //! 防止抖动
    chassis.power_ratio = (chassis.power_ratio > 0.05f) ? chassis.power_ratio : 0;
    chassis.vx = rc_info.l_rocker_ud;
    chassis.vy = rc_info.l_rocker_lr;
  }

  if (!(fabs(chassis.vx) < FLOAT_THRESHOLD && fabs(chassis.vy) < FLOAT_THRESHOLD)) {
    float angle_ref;
    int16_t angle[4];
    if (fabs(chassis.vy) < FLOAT_THRESHOLD) {
      angle_ref = 0;
    } else if (fabs(chassis.vx) < FLOAT_THRESHOLD) {
      angle_ref = -SIGN(chassis.vy) * 90;
    } else {
      angle_ref = -atan2(chassis.vy, fabs(chassis.vx)) * RAD_TO_DEG;
    }
    chassis.mv_direction = angle_ref;

    double spd_ref = 0;
    int16_t spd[4];
    spd_ref = SIGN(chassis.vx) * MOTOR_SPEED_MAX * chassis.power_ratio * MOTOR_REDUCTION_RATIO;

    attitude_control(spd_ref, angle_ref, spd, angle);
    
    chassis.steer_pos_ref[fr_motor] = FR_POS_F * OMNI_INIT_FRONT_ANGLE + STEER_INIT_ANGLE_FR + angle[fr_motor] + STEER_FR_OFFSET; //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[fl_motor] = FL_POS_F * OMNI_INIT_FRONT_ANGLE + STEER_INIT_ANGLE_FL + angle[fl_motor] + STEER_FL_OFFSET; //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[bl_motor] = BL_POS_F * OMNI_INIT_BACK_ANGLE + STEER_INIT_ANGLE_BL + angle[bl_motor] + STEER_BL_OFFSET; //+ STEER_BL_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[br_motor] = BR_POS_F * OMNI_INIT_BACK_ANGLE + STEER_INIT_ANGLE_BR + angle[br_motor] + STEER_BR_OFFSET; //+ STEER_BR_OFFSET) * MOTOR_REDUCTION_RATIO;

    setDMMotorBuf(fr_motor , chassis.steer_pos_ref[fr_motor] * 100);
    setDMMotorBuf(fl_motor , chassis.steer_pos_ref[fl_motor] * 100);
    setDMMotorBuf(bl_motor, chassis.steer_pos_ref[bl_motor] * 100);
    setDMMotorBuf(br_motor, chassis.steer_pos_ref[br_motor] * 100);

    chassis.driving_spd_ref[fr_motor]  = FR_SPD_F  * spd[fr_motor];
    chassis.driving_spd_ref[fl_motor]  = FL_SPD_F  * spd[fl_motor];
    chassis.driving_spd_ref[br_motor] = BR_SPD_F * spd[bl_motor];
    chassis.driving_spd_ref[bl_motor] = BL_SPD_F * spd[br_motor];

    stop_flag = 1;
  }
  else {
    chassis.steer_pos_ref[fr_motor]  = FR_POS_F * OMNI_INIT_FRONT_ANGLE + STEER_INIT_ANGLE_FR + STEER_FR_OFFSET; //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[fl_motor]  = FL_POS_F * OMNI_INIT_FRONT_ANGLE + STEER_INIT_ANGLE_FL + STEER_FL_OFFSET; //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[bl_motor] = BL_POS_F * OMNI_INIT_BACK_ANGLE + STEER_INIT_ANGLE_BL + STEER_BL_OFFSET; //+ STEER_BL_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[br_motor] = BR_POS_F * OMNI_INIT_BACK_ANGLE + STEER_INIT_ANGLE_BR + STEER_BR_OFFSET; //+ STEER_BR_OFFSET) * MOTOR_REDUCTION_RATIO;
    //! 如果是静止状态，则加入位置环，以保证在墙上不滑动
    if(stop_flag) {
      for(int i = 0; i < 4; ++i) {
        chassis.driving_pos_ref[i] = motor_driving[i].total_angle;
      }
      stop_flag = 0;
    }
    for(int i = 0; i < 4; ++i) {
      pid_calc(&pid_driving_pos[i], chassis.driving_pos_fdb[i], chassis.driving_pos_ref[i]);
      chassis.driving_spd_ref[i] = pid_driving_pos[i].out;
    }
  }
}
