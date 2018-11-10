#include "remote_ctrl.h"
#include "usart.h"
#include "sys_config.h"

uint8_t rc_buf[RC_BUFLEN];
rc_info_t rc_info;

/**
 * @brief   handle remote controller information
 * @param   remote control struct and remote uart(dbus) buffer
 * @usage   call after uart idle interrupt
 * @info    r_rocker_lr :
 *          r_rocker_ud :
 *          l_rocker_lr :
 *          l_rocker_ud :
 *          knob_v1     :
 *          knob_v2     :
 *          sa          :
 *          sb          :
 *          sc          :
 *          sd          :
 */
void rc_callback_handler(rc_info_t *rc, uint8_t *buf)
{
  if(buf[0] != 0x0F)
    return;

  rc->r_rocker_lr = (buf[1]      | buf[2] << 8) & 0x07ff;
  rc->r_rocker_lr -= ROCKER_OFFSET;
  rc->l_rocker_ud = (buf[2] >> 3 | buf[3] << 5) & 0x07ff;
  rc->l_rocker_ud -= ROCKER_OFFSET;
  rc->r_rocker_ud = (buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07ff;
  // rc->r_rocker_ud -= ROCKER_OFFSET;
  rc->l_rocker_lr = (buf[5] >> 1 | buf[6] << 7) & 0x07ff;
  rc->l_rocker_lr -= ROCKER_OFFSET;

  rc->knob_v1     = (buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07ff;
  rc->knob_v2     = (buf[9] >> 2 | buf[10] << 6)  & 0x07ff;

  rc->sa          = (buf[11] >> 6) & 0x03;
  rc->sb          = (buf[7]  >> 5) & 0x03;
  rc->sc          = (buf[13] >> 1) & 0x03;
  rc->sd          = (buf[15] >> 4) & 0x03;

  // test code
  // static int count = 0;
  // if(count == 100){
  //   HAL_UART_Transmit_DMA(&TEST_HUART, (uint8_t *)&rc->knob_v2, 2);
  //   count = 0;
  // }
  // count++;
}


