#ifndef __DM_MOTOR_DRIVE_H__
#define __DM_MOTOR_DRIVE_H__

#include "stm32f4xx_hal.h"

typedef enum
{
  READ_ENC = 0x90,
  OPEN_LOOP = 0xA0,
  SPD_CL = 0xA2,
  POS_CL1 = 0xA3,
  POS_CL2 = 0xA4,
  POS_CL3 = 0xA5,
  POS_CL4 = 0xA6
} dm_cmd_e;

typedef enum
{
  
} dm_direction_e;

typedef __packed struct 
{
  uint8_t header;
  uint8_t cmd;
  uint8_t id;
  uint8_t data_length;
  uint8_t check_sum;

} dm_motor_header_t;

typedef __packed struct 
{
  uint8_t direction;

} dm_pos_cl3_t;


#endif // !1
