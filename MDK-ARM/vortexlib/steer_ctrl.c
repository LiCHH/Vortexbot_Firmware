/** @file steer_ctrl.c
 *  @version 1.0
 *  @date Nov 2018
 *
 *  @brief control steer servo through uart messages
 *
 *  @copyright 2018 Vortex Lab. All rights reserved.
 *
 */

#include "string.h"

#include "steer_ctrl.h"
#include "chassis_task.h"

servo_sync_ctrl_t servo_packet;
servo_async_ctrl_t single_packet;

static uint8_t get_check_sum(uint8_t *pack, uint16_t length)
{
  uint8_t check_sum = 0;
  for (int i = 2; i < length - 1; ++i)
  {
    check_sum += pack[i];
  }
  check_sum = ~(check_sum & 0xFF);
  return check_sum;
}

void send_servo_packet(void)
{
  HAL_UART_Transmit_DMA(&STEER_HUART, (uint8_t *)(&servo_packet), sizeof(servo_packet));
  
  // HAL_UART_Transmit_DMA(&STEER_HUART, (uint8_t *)(&single_packet), sizeof(single_packet));
}

void servo_init(void)
{
  memset(&servo_packet, 0, sizeof(servo_packet));

  servo_packet.header_1 = FRAME_HEADER_1;
  servo_packet.header_2 = FRAME_HEADER_2;
  servo_packet.servo_id = ALL_SERVO;
  servo_packet.total_length = 0x20;
  servo_packet.cmd_type = SERVO_SYCN_WRITE;
  servo_packet.start_addr = TARGET_POS;
  servo_packet.single_length = 0x06;

  for (int i = 0; i < 4; ++i)
  {
    servo_packet.ctrl_info[i].servo_id = i + 1;
    servo_packet.ctrl_info[i].pos_low = 0x00;
    servo_packet.ctrl_info[i].pos_high = 0x08; // 初始位置取中间 2048
    servo_packet.ctrl_info[i].runtime_low = 0x00;
    servo_packet.ctrl_info[i].runtime_high = 0x00;
    servo_packet.ctrl_info[i].spd_low = 0xE8;
    servo_packet.ctrl_info[i].spd_high = 0x03;
  }

  servo_packet.check_sum = get_check_sum((uint8_t *)(&servo_packet), sizeof(servo_packet));

  // memset(&single_packet, 0, sizeof(single_packet));

  // single_packet.header_1 = FRAME_HEADER_1;
  // single_packet.header_2 = FRAME_HEADER_2;
  // single_packet.servo_id = 0x01;
  // single_packet.data_length = 0x09;
  // single_packet.cmd_type = 0x03;
  // single_packet.start_addr = 0x2A;
  // single_packet.pos_low = 0x00;
  // single_packet.pos_high = 0x08;
  // single_packet.time_low = 0x00;
  // single_packet.time_high = 0x00;
  // single_packet.spd_low = 0xE8;
  // single_packet.spd_high = 0x03;
  // single_packet.check_sum = get_check_sum((uint8_t *)&single_packet, sizeof(single_packet));

}

void set_servo_pos(void)
{
  // int fr_angle = INT_LI0MIT((int)(fr / 360.f * SERVO_POS_RANGE), 4192) + STEER_SERVO_OFFSET;
  // int fl_angle = INT_LIMIT((int)(fl / 360.f * SERVO_POS_RANGE), 4192) + STEER_SERVO_OFFSET;
  // int bl_angle = INT_LIMIT((int)(bl / 360.f * SERVO_POS_RANGE), 4192) + STEER_SERVO_OFFSET;
  // int br_angle = INT_LIMIT((int)(br / 360.f * SERVO_POS_RANGE), 4192) + STEER_SERVO_OFFSET;

  int fr_angle = (int)(chassis.steer_pos_ref[fr_motor] / 360.f * SERVO_POS_RANGE);
  int fl_angle = (int)(chassis.steer_pos_ref[fl_motor] / 360.f * SERVO_POS_RANGE);
  int bl_angle = (int)(chassis.steer_pos_ref[bl_motor] / 360.f * SERVO_POS_RANGE);
  int br_angle = (int)(chassis.steer_pos_ref[br_motor] / 360.f * SERVO_POS_RANGE);

  servo_packet.ctrl_info[FR_SERVO - 1].pos_low = fr_angle & 0xFF;
  servo_packet.ctrl_info[FR_SERVO - 1].pos_high = (fr_angle >> 8) & 0xFF;
  servo_packet.ctrl_info[FL_SERVO - 1].pos_low = fl_angle & 0xFF;
  servo_packet.ctrl_info[FL_SERVO - 1].pos_high = (fl_angle >> 8) & 0xFF;
  servo_packet.ctrl_info[BL_SERVO - 1].pos_low = bl_angle & 0xFF;
  servo_packet.ctrl_info[BL_SERVO - 1].pos_high = (bl_angle >> 8) & 0xFF;
  servo_packet.ctrl_info[BR_SERVO - 1].pos_low = br_angle & 0xFF;
  servo_packet.ctrl_info[BR_SERVO - 1].pos_high = (br_angle >> 8) & 0xFF;
  
  servo_packet.check_sum = get_check_sum((uint8_t *)(&servo_packet), sizeof(servo_packet));
}


