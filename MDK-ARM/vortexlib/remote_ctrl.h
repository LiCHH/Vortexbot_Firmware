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

typedef struct
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

typedef struct 
{
  float vx;
  float vy;
  float vw;

  float ref_direction;
  float power_ratio;
} rb_info_t;

extern uint8_t rc_buf[];
extern rc_info_t rc_info;
extern rb_info_t rb_info;

void rc_callback_handler(uint8_t *rc_buf);
void rc_param_init(void);
void get_bot_velocity(void);

///! 均值滤波器
typedef volatile struct
{
  int16_t num_data;
  int16_t* datas;
  int16_t sum;
  int16_t insert_index;
  int16_t filtered_data;
  int16_t data_received;
} filter_t;

void filter_init(filter_t *filter, int num_datas);
void filter_update(filter_t *filter, int16_t raw_data);

#endif // !RL_H__
