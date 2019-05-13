/** @file steer_ctrl.h
 *  @version 1.0
 *  @date Nov 2018
 *
 *  @brief control steer servo through uart messages
 *
 *  @copyright 2018 Vortex Lab. All rights reserved.
 *
 */

#ifndef __STEER_CTRL_H__
#define __STEER_CTRL_H__

#include "stm32f4xx_hal.h"

#include "sys_config.h"
#include "usart.h"
#include "servo_info_task.h"

#define STEER_CTRL_MAX_SIZE 50

#define SERVO_POS_RANGE 4192
#define SERVO_SPD_RANGE 3000
#define SERVO_ACC_RANGE 100

#define SERVO_BUF_LEN 100

typedef enum
{
  SERVO_ID     = 0x05,
  TARGET_POS   = 0x2A,
  RUNNING_TIME = 0x2C,
  RUNNING_SPD  = 0x2E,
  CURR_POS     = 0x38,
  CURR_SPD     = 0x3A
} servo_addr_e;

typedef enum
{
  FRAME_HEADER_1   = 0xFF,
  FRAME_HEADER_2   = 0xFF,

  SERVO_PING       = 0x01,
  SERVO_READ       = 0x02,
  SERVO_WRITE      = 0x03,
  SERVO_REG_WRITE  = 0x04,
  SERVO_ACTION     = 0x05,
  SERVO_SYCN_WRITE = 0x83,
  SERVO_RESET      = 0x06

} servo_cmd_e;

typedef enum
{
  FR_SERVO  = 0x01,
  FL_SERVO  = 0x02,
  BL_SERVO  = 0x03,
  BR_SERVO  = 0x04,
  ALL_SERVO = 0xFE
} servo_id_e;

typedef __packed struct
{
  uint8_t servo_id;
  uint8_t pos_low;
  uint8_t pos_high;
  uint8_t runtime_low;
  uint8_t runtime_high;
  uint8_t spd_low;
  uint8_t spd_high;
} single_servo_t;

typedef __packed struct
{
  uint8_t header_1;
  uint8_t header_2;
  uint8_t servo_id;
  uint8_t total_length;
  uint8_t cmd_type;
  uint8_t start_addr;
  uint8_t single_length;
  single_servo_t ctrl_info[4];
  uint8_t check_sum;

} servo_sync_ctrl_t;

typedef __packed struct
{
  uint8_t header_1;
  uint8_t header_2;
  uint8_t servo_id;
  uint8_t data_length;
  uint8_t cmd_type;
  uint8_t start_addr;
  uint8_t pos_low;
  uint8_t pos_high;
  uint8_t time_low;
  uint8_t time_high;
  uint8_t spd_low;
  uint8_t spd_high;
  uint8_t check_sum;
} servo_async_ctrl_t;

typedef __packed struct 
{
  uint8_t header_1;
  uint8_t header_2;
  uint8_t servo_id;
  uint8_t data_length;
  uint8_t cmd_type;
  uint8_t start_addr;
  uint8_t data_num;
  uint8_t check_sum;
} servo_request_t;


extern uint8_t servo_buf[];

extern servo_sync_ctrl_t servo_packet;
extern servo_async_ctrl_t single_packet;
extern servo_request_t request_packet;

extern uint8_t info_received;
extern uint8_t receive_fail;


void send_servo_packet(void);
void servo_init(void);
void set_servo_pos(void);
void send_request(uint8_t id);

void steer_callback_handler(servo_info_t *servo, uint8_t *buf);

#endif
