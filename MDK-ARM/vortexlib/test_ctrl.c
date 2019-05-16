/**
 * @file test_ctrl.c
 * @version 1.0
 * date October 2018
 *
 * @brief test control message from pc handle
 *
 */

#include "test_ctrl.h"
#include "sys_config.h"
#include "stdlib.h"
#include "string.h"
#include "usart.h"
#include "pid.h"
#include "bsp_can.h"
#include "chassis_task.h"
// #include "steer_ctrl.h"

testctrl_info_t tc;

uint8_t test_buf[TEST_BUF_LEN];

void testctrl_callback_handler(testctrl_info_t *tc, uint8_t *buf)
{
  // chassis.steer_pos_ref[0] = 2048;//buf[1] | (buf[0] << 8);
  // chassis.steer_pos_ref[1] = 2048;//buf[3] | (buf[2] << 8);
  // chassis.steer_pos_ref[2] = 4096;//buf[5] | (buf[4] << 8);
  // chassis.steer_pos_ref[3] = 4096;//buf[7] | (buf[6] << 8);

 

  // int16_t temp[4];
  // temp[0] = chassis.steer_pos_fdb[0];
  // temp[1] = chassis.steer_pos_fdb[1];
  // temp[2] = chassis.steer_pos_fdb[2];
  // temp[3] = chassis.driving_spd_ref[3];
  
  // HAL_GPIO_TogglePin(LEDA_GPIO_Port, LEDA_Pin);
  // // Can be replace by HAL_GPIO_TogglePin
  // // static int32_t flag = 0;
  // // if (!flag)
  // // {
  // //   HAL_GPIO_WritePin(LEDA_GPIO_Port, LEDA_Pin, GPIO_PIN_SET);
  // //   flag = 1;
  // // }
  // // else
  // // {
  // //   HAL_GPIO_WritePin(LEDA_GPIO_Port, LEDA_Pin, GPIO_PIN_RESET);
  // //   flag = 0;
  // // }
  // HAL_UART_Transmit(&STEER_HUART, (uint8_t *)buf, 8, 100);
  // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)buf, 8, 100);
}
