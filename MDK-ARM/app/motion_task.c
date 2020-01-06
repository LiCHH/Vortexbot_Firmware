#include "motion_task.h"

#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "cmsis_os.h"
#include "arm_math.h"

#include "sys_config.h"
#include "pid.h"
#include "bsp_can.h"
#include "remote_ctrl.h"
#include "vortexbot_info.h"
#include "dm_motor_drive.h"
#include "test_ctrl.h"
#include "bsp_uart.h"
#include "odom_task.h"
#include "swmode_task.h"
#include "attitude_control.h"

static void omnidirection_handler(void);
static void versatile_control_handler(void);
static void differential_handler(void);
static void carlike_handler(void);
static void forward_handler(void);
static void stop_handler(void);
static void attitude_control_handler(void);

uint32_t motion_time_start, motion_time_end;
int motion_time_ms;

void motion_task(void const *argu) {
  // motion_time_ms = motion_time_end - motion_time_start;
  // sprintf(test_buf, "%02d %01d\r\n", motion_time_ms, chassis.ctrl_mode);
  // HAL_UART_Transmit(&TEST_HUART, test_buf, 6, 5);

  // motion_time_start = HAL_GetTick();
  
  switch (chassis.ctrl_mode)
  {
  case OMNI_DIRECTIONAL:
  {
    taskENTER_CRITICAL();
    omnidirection_handler();
    taskEXIT_CRITICAL();
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
    taskENTER_CRITICAL();
    attitude_control_handler();
    taskEXIT_CRITICAL();
  }
  break;
  
  case VERSATILE_CONTROL:
  {
    taskENTER_CRITICAL();
    versatile_control_handler();
    taskEXIT_CRITICAL();
  }
  break;

  default:
  {
    stop_handler();
  }
  break;
  }
  // motion_time_end = HAL_GetTick();
}


static void differential_handler(void)
{
}

static void carlike_handler(void)
{
}


static void stop_handler(void)
{
  chassis.steer_pos_ref[fr_motor] = 100 * (STEER_INIT_ANGLE_FR + STEER_FR_OFFSET); //STEER_FR_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[fl_motor] = 100 * (STEER_INIT_ANGLE_FL + STEER_FL_OFFSET); //STEER_FR_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[bl_motor] = 100 * (STEER_INIT_ANGLE_BL + STEER_BL_OFFSET); //STEER_BL_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[br_motor] = 100 * (STEER_INIT_ANGLE_BR + STEER_BR_OFFSET); //STEER_BR_OFFSET * MOTOR_REDUCTION_RATIO;
  setDMMotorBuf(fr_motor, chassis.steer_pos_ref[fr_motor]);
  setDMMotorBuf(fl_motor, chassis.steer_pos_ref[fl_motor]);
  setDMMotorBuf(bl_motor, chassis.steer_pos_ref[bl_motor]);
  setDMMotorBuf(br_motor, chassis.steer_pos_ref[br_motor]);

  memset(&chassis.driving_spd_ref, 0, sizeof(chassis.driving_spd_ref));
  // send_control_msgs();
}

static void forward_handler(void)
{
  static int stop_flag = 0;
  static int16_t spd_ref = 0;

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
    chassis.vx = rb_info.vx;
    spd_ref = SIGN(chassis.vx) * MOTOR_SPEED_MAX * rb_info.power_ratio * MOTOR_REDUCTION_RATIO;
  }

  if (fabs(chassis.vx) > FLOAT_THRESHOLD) {
    stop_flag = 1;
    chassis.driving_spd_ref[fr_motor]  = FR_SPD_F  * spd_ref;
    chassis.driving_spd_ref[fl_motor]  = -FL_SPD_F  * spd_ref;
    chassis.driving_spd_ref[br_motor] = BR_SPD_F * spd_ref;
    chassis.driving_spd_ref[bl_motor] = -BL_SPD_F * spd_ref;
  }
  else {
    if(stop_flag || global_stop_flag) {
      for(int i = 0; i < 4; ++i) {
        chassis.driving_pos_ref[i] = motor_driving[i].total_angle; // + motor_driving[i].speed_rpm * 60.0 * DEG_TO_RAD;
      }
      stop_flag = 0;
      global_stop_flag = 0;
    }
    for(int i = 0; i < 4; ++i) {
      pid_calc(&pid_driving_pos[i], chassis.driving_pos_fdb[i], chassis.driving_pos_ref[i]);
      chassis.driving_spd_ref[i] = pid_driving_pos[i].out;
    }
    return;
  }
}

void versatile_control(float vx, float vy, float vw, float* omegas, float* phis) {
  static float vxs[4], vys[4];

  vxs[fr_motor] = vx + 0 * BODY_RADIUS * vw;
  vys[fr_motor] = vy + 1 * BODY_RADIUS * vw;
  vxs[fl_motor] = vx - BODY_RADIUS * vw;
  vys[fl_motor] = vy + BODY_RADIUS * vw;
  vxs[bl_motor] = vx - 0.816 * BODY_RADIUS * vw;
  vys[bl_motor] = vy - 0.5 * BODY_RADIUS * vw;
  vxs[br_motor] = vx + 0.816 * BODY_RADIUS * vw;
  vys[br_motor] = vy - 0.5 * BODY_RADIUS * vw;

  for(int i = 0; i < 4; ++i) {
    // sprintf(test_buf, "fuck1\r\n");
    // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)test_buf, 7, 5);
    phis[i] = atan2(vys[i], vxs[i]);

    // sprintf(test_buf, "fuck2\r\n");
    // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)test_buf, 7, 5);
    // memset(test_buf, 0, 20);
    // sprintf(test_buf, "%.2f %.2f %.2f\r\n", vys[i], vxs[i], phis[i]);
    // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)test_buf, 20, 10);

    omegas[i] = sqrt(vxs[i] * vxs[i] + vys[i] * vys[i]);
    if(fabs(phis[i]) >= PI / 2) {
      phis[i] += PI;
      // sprintf(test_buf, "fuck3\r\n");
      // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)test_buf, 7, 5);
      RESTRICT_ANGLE_RAD(phis[i]);
      // sprintf(test_buf, "fuck4\r\n");
      // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)test_buf, 7, 5);
      omegas[i] = -omegas[i];
    }
  }
}

static void versatile_control_handler(void) {
  //! 判断当前是否为静止状态
  static int stop_flag = 0;

  static float spds[4], phis[4];

  if (bot_mode == MANUL_CONTROL_MODE)
  {
    chassis.vx = rb_info.vx;
    chassis.vy = rb_info.vy;
    chassis.vw = rb_info.vw;
  }

  //! test code
  // memset(test_buf, 0, 20);
  // sprintf(test_buf, "%.2f %5d\r\n", chassis.vw, rc_info.r_rocker_lr);
  // HAL_UART_Transmit(&TEST_HUART, test_buf, 15, 10);

  if (fabs(chassis.vx) > 0.01 || fabs(chassis.vy) > 0.01 || fabs(chassis.vw) > 0.01) {

    versatile_control(chassis.vx, chassis.vy, chassis.vw, spds, phis);
    
    chassis.steer_pos_ref[fr_motor] = 100 * (FR_POS_F * OMNI_INIT_FRONT_ANGLE + STEER_INIT_ANGLE_FR + phis[fr_motor] * RAD_TO_DEG + STEER_FR_OFFSET); //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[fl_motor] = 100 * (FL_POS_F * OMNI_INIT_FRONT_ANGLE + STEER_INIT_ANGLE_FL + phis[fl_motor] * RAD_TO_DEG + STEER_FL_OFFSET); //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[bl_motor] = 100 * (BL_POS_F * OMNI_INIT_BACK_ANGLE + STEER_INIT_ANGLE_BL + phis[bl_motor] * RAD_TO_DEG + STEER_BL_OFFSET); //+ STEER_BL_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[br_motor] = 100 * (BR_POS_F * OMNI_INIT_BACK_ANGLE + STEER_INIT_ANGLE_BR + phis[br_motor] * RAD_TO_DEG + STEER_BR_OFFSET); //+ STEER_BR_OFFSET) * MOTOR_REDUCTION_RATIO;
    setDMMotorBuf(fr_motor, chassis.steer_pos_ref[fr_motor]);
    setDMMotorBuf(fl_motor, chassis.steer_pos_ref[fl_motor]);
    setDMMotorBuf(bl_motor, chassis.steer_pos_ref[bl_motor]);
    setDMMotorBuf(br_motor, chassis.steer_pos_ref[br_motor]);

    //! test code
    // memset(test_buf, 0, 30);
    // sprintf(test_buf, "%03d %03d %03d %03d\r\n", chassis.steer_pos_ref[0]/100, chassis.steer_pos_ref[1]/100
    //                                            , chassis.steer_pos_ref[2]/100, chassis.steer_pos_ref[3]/100);
    // HAL_UART_Transmit(&TEST_HUART, test_buf, 18, 10);

    chassis.driving_spd_ref[fr_motor] = FR_SPD_F * spds[fr_motor] * LINSPD_TO_RPM * MOTOR_REDUCTION_RATIO;
    chassis.driving_spd_ref[fl_motor] = FL_SPD_F * spds[fl_motor] * LINSPD_TO_RPM * MOTOR_REDUCTION_RATIO;
    chassis.driving_spd_ref[bl_motor] = BL_SPD_F * spds[bl_motor] * LINSPD_TO_RPM * MOTOR_REDUCTION_RATIO;
    chassis.driving_spd_ref[br_motor] = BR_SPD_F * spds[br_motor] * LINSPD_TO_RPM * MOTOR_REDUCTION_RATIO;
    
    //! test code
    // memset(test_buf, 0, 10);
    // sprintf(test_buf, "%d %d\r\n", chassis.steer_pos_ref[0], chassis.steer_pos_ref[1]);
    // sprintf(test_buf, "%.2f %.2f\r\n", phis[0]*RAD_TO_DEG, phis[1]*RAD_TO_DEG);
    // HAL_UART_Transmit(&TEST_HUART, test_buf, 20, 15);

    stop_flag = 1;
  }
  else {
    //! 如果是静止状态，则加入位置环，以保证在墙上不滑动
    if(stop_flag || global_stop_flag) {
      for(int i = 0; i < 4; ++i) {
        chassis.driving_pos_ref[i] = motor_driving[i].total_angle;
      }
      stop_flag = 0;
      global_stop_flag = 0;
    }

    chassis.steer_pos_ref[fr_motor] = 100 * (FR_POS_F * OMNI_INIT_FRONT_ANGLE + STEER_INIT_ANGLE_FR + STEER_FR_OFFSET); //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[fl_motor] = 100 * (FL_POS_F * OMNI_INIT_FRONT_ANGLE + STEER_INIT_ANGLE_FL + STEER_FL_OFFSET); //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[bl_motor] = 100 * (BL_POS_F * OMNI_INIT_BACK_ANGLE + STEER_INIT_ANGLE_BL + STEER_BL_OFFSET); //+ STEER_BL_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[br_motor] = 100 * (BR_POS_F * OMNI_INIT_BACK_ANGLE + STEER_INIT_ANGLE_BR + STEER_BR_OFFSET); //+ STEER_BR_OFFSET) * MOTOR_REDUCTION_RATIO;
    setDMMotorBuf(fr_motor, chassis.steer_pos_ref[fr_motor]);
    setDMMotorBuf(fl_motor, chassis.steer_pos_ref[fl_motor]);
    setDMMotorBuf(bl_motor, chassis.steer_pos_ref[bl_motor]);
    setDMMotorBuf(br_motor, chassis.steer_pos_ref[br_motor]);

    for(int i = 0; i < 4; ++i) {
      pid_calc(&pid_driving_pos[i], chassis.driving_pos_fdb[i], chassis.driving_pos_ref[i]);
      chassis.driving_spd_ref[i] = pid_driving_pos[i].out;
    }
  }
}

static void attitude_control_handler(void) {
  //! 判断当前是否为静止状态
  static int stop_flag = 0;

  static float spds[4], phis[4];

  if (bot_mode == MANUL_CONTROL_MODE)
  {
    chassis.vx = cos(rb_info.ref_direction) * rb_info.vx + sin(rb_info.ref_direction) * rb_info.vy;
    chassis.vy = -sin(rb_info.ref_direction) * rb_info.vx + cos(rb_info.ref_direction) * rb_info.vy;
    // chassis.vx = cos(3*PI/4) * rb_info.vx + sin(3*PI/4) * rb_info.vy;
    // chassis.vy = -sin(3*PI/4) * rb_info.vx + cos(3*PI/4) * rb_info.vy;
    // chassis.vw = rb_info.vw;
    chassis.vw = 0;
  }

  //! test code
  // memset(test_buf, 0, 20);
  // sprintf(test_buf, "%.2f %5d\r\n", chassis.vw, rc_info.r_rocker_lr);
  // HAL_UART_Transmit(&TEST_HUART, test_buf, 15, 10);

  if (fabs(chassis.vx) > 0.01 || fabs(chassis.vy) > 0.01) {
    chassis.vw = attitude_control();

    versatile_control(chassis.vx, chassis.vy, chassis.vw, spds, phis);
    
    chassis.steer_pos_ref[fr_motor] = 100 * (FR_POS_F * OMNI_INIT_FRONT_ANGLE + STEER_INIT_ANGLE_FR + phis[fr_motor] * RAD_TO_DEG + STEER_FR_OFFSET); //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[fl_motor] = 100 * (FL_POS_F * OMNI_INIT_FRONT_ANGLE + STEER_INIT_ANGLE_FL + phis[fl_motor] * RAD_TO_DEG + STEER_FL_OFFSET); //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[bl_motor] = 100 * (BL_POS_F * OMNI_INIT_BACK_ANGLE + STEER_INIT_ANGLE_BL + phis[bl_motor] * RAD_TO_DEG + STEER_BL_OFFSET); //+ STEER_BL_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[br_motor] = 100 * (BR_POS_F * OMNI_INIT_BACK_ANGLE + STEER_INIT_ANGLE_BR + phis[br_motor] * RAD_TO_DEG + STEER_BR_OFFSET); //+ STEER_BR_OFFSET) * MOTOR_REDUCTION_RATIO;
    setDMMotorBuf(fr_motor, chassis.steer_pos_ref[fr_motor]);
    setDMMotorBuf(fl_motor, chassis.steer_pos_ref[fl_motor]);
    setDMMotorBuf(bl_motor, chassis.steer_pos_ref[bl_motor]);
    setDMMotorBuf(br_motor, chassis.steer_pos_ref[br_motor]);

    // memset(test_buf, 0, 30);
    // sprintf(test_buf, "%03d %03d %03d %03d\r\n", chassis.steer_pos_ref[0]/100, chassis.steer_pos_ref[1]/100
    //                                            , chassis.steer_pos_ref[2]/100, chassis.steer_pos_ref[3]/100);
    // HAL_UART_Transmit(&TEST_HUART, test_buf, 18, 10);

    chassis.driving_spd_ref[fr_motor] = FR_SPD_F * spds[fr_motor] * LINSPD_TO_RPM * MOTOR_REDUCTION_RATIO;
    chassis.driving_spd_ref[fl_motor] = FL_SPD_F * spds[fl_motor] * LINSPD_TO_RPM * MOTOR_REDUCTION_RATIO;
    chassis.driving_spd_ref[bl_motor] = BL_SPD_F * spds[bl_motor] * LINSPD_TO_RPM * MOTOR_REDUCTION_RATIO;
    chassis.driving_spd_ref[br_motor] = BR_SPD_F * spds[br_motor] * LINSPD_TO_RPM * MOTOR_REDUCTION_RATIO;
    
    //! test code
    // memset(test_buf, 0, 10);
    // sprintf(test_buf, "%d %d\r\n", chassis.steer_pos_ref[0], chassis.steer_pos_ref[1]);
    // sprintf(test_buf, "%.2f %.2f\r\n", phis[0]*RAD_TO_DEG, phis[1]*RAD_TO_DEG);
    // HAL_UART_Transmit(&TEST_HUART, test_buf, 20, 15);

    stop_flag = 1;
  }
  else {
    //! 如果是静止状态，则加入位置环，以保证在墙上不滑动
    if(stop_flag || global_stop_flag) {
      for(int i = 0; i < 4; ++i) {
        chassis.driving_pos_ref[i] = motor_driving[i].total_angle;
      }
      stop_flag = 0;
      global_stop_flag = 0;
    }
    static float ref_angle;
    ref_angle = rb_info.ref_direction;
    // ref_angle = 3.f / 4.f * PI;
    if(fabs(ref_angle) > PI / 2) {
      ref_angle += PI;
      RESTRICT_ANGLE_RAD(ref_angle);
    }

    chassis.steer_pos_ref[fr_motor] = 100 * (-ref_angle * RAD_TO_DEG + STEER_INIT_ANGLE_FR + STEER_FR_OFFSET); //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[fl_motor] = 100 * (-ref_angle * RAD_TO_DEG + STEER_INIT_ANGLE_FL + STEER_FL_OFFSET); //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[bl_motor] = 100 * (-ref_angle * RAD_TO_DEG + STEER_INIT_ANGLE_BL + STEER_BL_OFFSET); //+ STEER_BL_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[br_motor] = 100 * (-ref_angle * RAD_TO_DEG + STEER_INIT_ANGLE_BR + STEER_BR_OFFSET); //+ STEER_BR_OFFSET) * MOTOR_REDUCTION_RATIO;
    setDMMotorBuf(fr_motor, chassis.steer_pos_ref[fr_motor]);
    setDMMotorBuf(fl_motor, chassis.steer_pos_ref[fl_motor]);
    setDMMotorBuf(bl_motor, chassis.steer_pos_ref[bl_motor]);
    setDMMotorBuf(br_motor, chassis.steer_pos_ref[br_motor]);

    for(int i = 0; i < 4; ++i) {
      pid_calc(&pid_driving_pos[i], chassis.driving_pos_fdb[i], chassis.driving_pos_ref[i]);
      chassis.driving_spd_ref[i] = pid_driving_pos[i].out;
    }
  }
}

static void omnidirection_handler(void)
{
  //! 判断当前是否为静止状态
  static int stop_flag = 0;

  static float spds[4], phis[4];

  if (bot_mode == MANUL_CONTROL_MODE)
  {
    chassis.vx = rb_info.vx;
    chassis.vy = rb_info.vy;
    // chassis.vx = cos(3*PI/4) * rb_info.vx + sin(3*PI/4) * rb_info.vy;
    // chassis.vy = -sin(3*PI/4) * rb_info.vx + cos(3*PI/4) * rb_info.vy;
    // chassis.vw = rb_info.vw;
    chassis.vw = 0;
  }

  //! test code
  // memset(test_buf, 0, 20);
  // sprintf(test_buf, "%.2f %5d\r\n", chassis.vw, rc_info.r_rocker_lr);
  // HAL_UART_Transmit(&TEST_HUART, test_buf, 15, 10);

  if (fabs(chassis.vx) > 0.01 || fabs(chassis.vy) > 0.01) {
    chassis.vw = attitude_control();

    versatile_control(chassis.vx, chassis.vy, chassis.vw, spds, phis);
    
    chassis.steer_pos_ref[fr_motor] = 100 * (FR_POS_F * OMNI_INIT_FRONT_ANGLE + STEER_INIT_ANGLE_FR + phis[fr_motor] * RAD_TO_DEG + STEER_FR_OFFSET); //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[fl_motor] = 100 * (FL_POS_F * OMNI_INIT_FRONT_ANGLE + STEER_INIT_ANGLE_FL + phis[fl_motor] * RAD_TO_DEG + STEER_FL_OFFSET); //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[bl_motor] = 100 * (BL_POS_F * OMNI_INIT_BACK_ANGLE + STEER_INIT_ANGLE_BL + phis[bl_motor] * RAD_TO_DEG + STEER_BL_OFFSET); //+ STEER_BL_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[br_motor] = 100 * (BR_POS_F * OMNI_INIT_BACK_ANGLE + STEER_INIT_ANGLE_BR + phis[br_motor] * RAD_TO_DEG + STEER_BR_OFFSET); //+ STEER_BR_OFFSET) * MOTOR_REDUCTION_RATIO;
    setDMMotorBuf(fr_motor, chassis.steer_pos_ref[fr_motor]);
    setDMMotorBuf(fl_motor, chassis.steer_pos_ref[fl_motor]);
    setDMMotorBuf(bl_motor, chassis.steer_pos_ref[bl_motor]);
    setDMMotorBuf(br_motor, chassis.steer_pos_ref[br_motor]);

    //! test code
    // memset(test_buf, 0, 30);
    // sprintf(test_buf, "%03d %03d %03d %03d\r\n", chassis.steer_pos_ref[0]/100, chassis.steer_pos_ref[1]/100
    //                                            , chassis.steer_pos_ref[2]/100, chassis.steer_pos_ref[3]/100);
    // HAL_UART_Transmit(&TEST_HUART, test_buf, 18, 10);

    chassis.driving_spd_ref[fr_motor] = FR_SPD_F * spds[fr_motor] * LINSPD_TO_RPM * MOTOR_REDUCTION_RATIO;
    chassis.driving_spd_ref[fl_motor] = FL_SPD_F * spds[fl_motor] * LINSPD_TO_RPM * MOTOR_REDUCTION_RATIO;
    chassis.driving_spd_ref[bl_motor] = BL_SPD_F * spds[bl_motor] * LINSPD_TO_RPM * MOTOR_REDUCTION_RATIO;
    chassis.driving_spd_ref[br_motor] = BR_SPD_F * spds[br_motor] * LINSPD_TO_RPM * MOTOR_REDUCTION_RATIO;
    
    //! test code
    // memset(test_buf, 0, 10);
    // sprintf(test_buf, "%d %d\r\n", chassis.steer_pos_ref[0], chassis.steer_pos_ref[1]);
    // sprintf(test_buf, "%.2f %.2f\r\n", phis[0]*RAD_TO_DEG, phis[1]*RAD_TO_DEG);
    // HAL_UART_Transmit(&TEST_HUART, test_buf, 20, 15);

    stop_flag = 1;
  }
  else {
    //! 如果是静止状态，则加入位置环，以保证在墙上不滑动
    if(stop_flag || global_stop_flag) {
      for(int i = 0; i < 4; ++i) {
        chassis.driving_pos_ref[i] = motor_driving[i].total_angle;
      }
      stop_flag = 0;
      global_stop_flag = 0;
    }
    static float ref_angle;
    ref_angle = rb_info.ref_direction;
    // ref_angle = 3.f / 4.f * PI;
    if(fabs(ref_angle) > PI / 2) {
      ref_angle += PI;
      RESTRICT_ANGLE_RAD(ref_angle);
    }

    chassis.steer_pos_ref[fr_motor] = 100 * (FR_POS_F * OMNI_INIT_FRONT_ANGLE - ref_angle * RAD_TO_DEG + STEER_INIT_ANGLE_FR + STEER_FR_OFFSET); //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[fl_motor] = 100 * (FL_POS_F * OMNI_INIT_FRONT_ANGLE - ref_angle * RAD_TO_DEG + STEER_INIT_ANGLE_FL + STEER_FL_OFFSET); //+ STEER_FR_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[bl_motor] = 100 * (BL_POS_F * OMNI_INIT_BACK_ANGLE  - ref_angle * RAD_TO_DEG + STEER_INIT_ANGLE_BL + STEER_BL_OFFSET); //+ STEER_BL_OFFSET) * MOTOR_REDUCTION_RATIO;
    chassis.steer_pos_ref[br_motor] = 100 * (BR_POS_F * OMNI_INIT_BACK_ANGLE  - ref_angle * RAD_TO_DEG + STEER_INIT_ANGLE_BR + STEER_BR_OFFSET); //+ STEER_BR_OFFSET) * MOTOR_REDUCTION_RATIO;
    setDMMotorBuf(fr_motor, chassis.steer_pos_ref[fr_motor]);
    setDMMotorBuf(fl_motor, chassis.steer_pos_ref[fl_motor]);
    setDMMotorBuf(bl_motor, chassis.steer_pos_ref[bl_motor]);
    setDMMotorBuf(br_motor, chassis.steer_pos_ref[br_motor]);

    for(int i = 0; i < 4; ++i) {
      pid_calc(&pid_driving_pos[i], chassis.driving_pos_fdb[i], chassis.driving_pos_ref[i]);
      chassis.driving_spd_ref[i] = pid_driving_pos[i].out;
    }
  }
}