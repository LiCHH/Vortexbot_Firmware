/* use for WFLY ET07 */

#ifndef __REMOTE_CTRL_H__
#define __REMOTE_CTRL_H__

#include "stm32f4xx_hal.h"
#include "sys_config.h"

#define RC_MAX_LEN      50
#define RC_BUFLEN       20

#define ROCKER_MAX      1695
#define ROCKER_MIN      353
#define ROCKER_OFFSET   1024
#define ROCKER_RANGE    (ROCKER_MAX - ROCKER_MIN)

#define KNOB_V2_MAX     1695
#define KNOB_V2_MIN     353
#define KNOB_V2_OFFSET  1024 

#define KNOB_V1_MIN     353
#define KNOB_V1_MAX     1695 
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
void rc_param_init(void);

///! 均值滤波器
typedef struct
{
  int16_t *datas;
  int16_t sum;
  int num_data;
  int insert_index;
  int16_t filtered_data;
  int data_received;

} filter_t;

#if BOT_ID == 1
#define FILTER_SIZE 13
#define LR_UD_BIAS 128
#define LR_LR_BIAS -104
#elif BOT_ID == 2
#define FILTER_SIZE 9
#define LR_UD_BIAS  193
#define LR_LR_BIAS  -32
#endif

void filter_init(filter_t *filter, int num_datas);
void filter_update(filter_t *filter, int16_t raw_data);

#endif // !RL_H__
