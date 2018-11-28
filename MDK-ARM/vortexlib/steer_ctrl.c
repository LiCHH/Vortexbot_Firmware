/** @file steer_ctrl.c
 *  @version 1.0
 *  @date Nov 2018
 *
 *  @brief control steer servo through uart messages
 *
 *  @copyright 2018 Vortex Lab. All rights reserved.
 *
 */

#include "steer_ctrl.h"
#include "string"

servo_sync_ctrl_t servo_packet;

void send_servo_packet(void)
{
  HAL_UART_Transmit_DMA(&STEER_HUART, (uint8_t *)(&servo_packet), sizeof(servo_packet));

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

  for(int i = 0; i < 4; ++i){
    servo_packet.ctrl_info[i].pos_low       = 0x00;
    servo_packet.ctrl_info[i].pos_high      = 0x08; // 初始位置取中间 2048
    servo_packet.ctrl_info[i].runtime_low   = 0x00;
    servo_packet.ctrl_info[i].runtime_high  = 0x00;
    servo_packet.ctrl_info[i].spd_low       = 0x00;
    servo_packet.ctrl_info[i].spd_high      = 0x00;
  }

  servo_packet.check_sum = get_check_sum((uint8_t *)(&servo_packet), sizeof(servo_packet));
}

static uint8_t get_check_sum(uint8_t *pack, uint16_t length)
{
  uint8_t check_sum = 0;
  for(int i = 2; i < length - 1; ++i)
  {
    check_sum += pack[i];
  }
  check_sum = ~(check_sum & 0xFF);
  return check_sum;
}

