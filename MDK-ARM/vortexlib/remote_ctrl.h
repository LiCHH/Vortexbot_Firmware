/* use for WFLY ET07 */

#ifndef __REMOTE_CTRL_H__
#define __REMOTE_CTRL_H__

#include "stm32f4xx_hal.h"

#define RC_MAX_LEN      50
#define RC_BUFLEN       20

#define ROCKER_MAX      1695
#define ROCKER_MIN      353
#define ROCKER_OFFSET   1024

#define KNOB_V2_MAX     1695
#define KNOB_V2_MIN     353
#define KNOB_V2_OFFSET  1024 

#define KNOB_V1_MIN     353
#define KNOB_V1_MAX     927 
#define KNOB_V1_RANGE   (KNOB_V1_MAX - KNOB_V1_MIN)

typedef enum
{
  SW_UP   = 0x00,
  SW_MID  = 0x02,
  SW_DOWN = 0x03
} sw_e;

typedef __packed struct
{
  int16_t r_rocker_lr;
  int16_t r_rocker_ud;
  int16_t l_rocker_lr;
  int16_t l_rocker_ud;

  int16_t knob_v1;
  int16_t knob_v2;

  uint8_t sa;
  uint8_t sb;
  uint8_t sc;
  uint8_t sd;
} rc_info_t;

extern uint8_t rc_buf[];
extern rc_info_t rc_info;

void rc_callback_handler(rc_info_t *rc, uint8_t *rc_buf);

#endif // !RL_H__
