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

testctrl_info_t tc;

uint8_t test_buf[TEST_BUF_LEN];

void testctrl_callback_handler(testctrl_info_t *tc, uint8_t *buf)
{
  chassis.wheel_spd_ref[0] = buf[1] | (buf[0] << 8);
  chassis.wheel_spd_ref[1] = buf[3] | (buf[2] << 8);
  chassis.wheel_spd_ref[2] = buf[5] | (buf[4] << 8);
  chassis.wheel_spd_ref[3] = buf[7] | (buf[6] << 8);

 

  int16_t temp[4];
  temp[0] = chassis.wheel_spd_ref[0];
  temp[1] = chassis.wheel_spd_ref[1];
  temp[2] = chassis.wheel_spd_ref[2];
  temp[3] = chassis.wheel_spd_ref[3];
  
  static int32_t flag = 0;
  if (!flag)
  {
    HAL_GPIO_WritePin(LEDA_GPIO_Port, LEDA_Pin, GPIO_PIN_SET);
    flag = 1;
  }
  else
  {
    HAL_GPIO_WritePin(LEDA_GPIO_Port, LEDA_Pin, GPIO_PIN_RESET);
    flag = 0;
  }
  HAL_UART_Transmit_DMA(&TEST_HUART, (uint8_t *)temp, sizeof(temp));
}
