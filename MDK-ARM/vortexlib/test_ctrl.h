/**
 * @file test_ctrl.h
 * @version 1.0
 * date October 2018
 *
 * @brief test control message from pc handle
 *
 */

#ifndef __TEST_CTRL_H__
#define __TEST_CTRL_H__

#include "stm32f4xx_hal.h"

#define TEST_BUF_LEN 8
#define TEST_MAX_LEN 20

typedef __packed struct
{
  int16_t motor1;
  int16_t motor2;
  int16_t motor3;
  int16_t motor4;
} testctrl_info_t;

extern testctrl_info_t tc;
extern uint8_t test_buf[];

void testctrl_callback_handler(testctrl_info_t *tc, uint8_t *buf);

#endif
