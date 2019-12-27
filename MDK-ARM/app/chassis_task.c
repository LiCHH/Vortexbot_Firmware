/**
 * @file chassis_task.c
 * @version 1.0
 * date October 2018
 *
 * @brief control chassis action
 *
 */
#include "chassis_task.h"

// #include "math.h"
// #include "stdlib.h"
// #include "string.h"
#include "cmsis_os.h"

#include "sys_config.h"
#include "pid.h"
#include "bsp_can.h"
#include "test_ctrl.h"
#include "bsp_uart.h"
#include "odom_task.h"
#include "attitude_control.h"
#include "dm_motor_drive.h"
#include "test_ctrl.h"

chassis_t chassis;

void get_chassis_info(void)
{
  for (int i = 0; i < 4; ++i)
  {
    chassis.driving_spd_fdb[i] = motor_driving[i].speed_rpm;
    chassis.driving_pos_fdb[i] = motor_driving[i].total_angle;
  }
  // sprintf(test_buf, "%5.2f %5.2f %5.2f %5.2f\r\n", chassis.driving_pos_fdb[0]/12960.f, chassis.driving_pos_fdb[1]/12960.f,
  //                                              chassis.driving_pos_fdb[2]/12960.f, chassis.driving_pos_fdb[3]/12960.f);

  // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)test_buf, 40, 10);
}

void send_control_msgs(void)
{
  taskENTER_CRITICAL();
  send_chassis_current(CAN_LOW_ID, chassis.driving_current[0], chassis.driving_current[1], chassis.driving_current[2], chassis.driving_current[3]);
  taskEXIT_CRITICAL();
  // for(int i = 0; i < 4; ++i) {
  //   sendDMMotor(i);
  // }
  // sendDMMotor(0);
}

void chassis_task(void const *argu)
{
  uint32_t chassis_wake_time = osKernelSysTick();
  while(1) {
    get_chassis_info();
    for (int i = 0; i < 4; ++i)
    {
      // Driving DOF's speed loop PID control
      pid_calc(&pid_driving_spd[i], chassis.driving_spd_fdb[i], chassis.driving_spd_ref[i]);
      chassis.driving_current[i] = pid_driving_spd[i].out;
    }
    send_control_msgs();
    osDelayUntil(&chassis_wake_time, CHASSIS_TIMER_PERIOD);
  }
}

void chassis_steer_task(void const *argu) {
  uint32_t chassis_steer_wake_time = osKernelSysTick();
  while(1) {
    for(int i = 0; i < 4; ++i) {
      sendDMMotor(i);
    }
    osDelayUntil(&chassis_steer_wake_time, CHASSIS_STEER_PERIOD);
  }
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
  chassis.steer_pos_ref[fr_motor] = 100 * (STEER_INIT_ANGLE_FR + STEER_FR_OFFSET); //STEER_FR_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[fl_motor] = 100 * (STEER_INIT_ANGLE_FL + STEER_FL_OFFSET); //STEER_FR_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[bl_motor] = 100 * (STEER_INIT_ANGLE_BL + STEER_BL_OFFSET); //STEER_BL_OFFSET * MOTOR_REDUCTION_RATIO;
  chassis.steer_pos_ref[br_motor] = 100 * (STEER_INIT_ANGLE_BR + STEER_BR_OFFSET); //STEER_BR_OFFSET * MOTOR_REDUCTION_RATIO;
  setDMMotorBuf(fr_motor , chassis.steer_pos_ref[fr_motor]);
  setDMMotorBuf(fl_motor , chassis.steer_pos_ref[fl_motor]);
  setDMMotorBuf(bl_motor, chassis.steer_pos_ref[bl_motor]);
  setDMMotorBuf(br_motor, chassis.steer_pos_ref[br_motor]);

  for(int i = 0; i < 4; ++i) {
    sendDMMotor(i);
    HAL_Delay(1);
  }

  attitude_control_init();
}
