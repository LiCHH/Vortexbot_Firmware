#ifndef __DM_MOTOR_DRIVE_H__
#define __DM_MOTOR_DRIVE_H__

#include "stm32f4xx_hal.h"

#define STEER_BUF_LEN 30
#define STEER_MAX_LEN 60

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
  CLOCKWISE = 0x00,
  COUNTERCLOCKWISE = 0x01 
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
  uint8_t header;
  uint8_t cmd;
  uint8_t id;
  uint8_t data_length;
  uint8_t check_sum;
} dm_read_enc_t;

typedef __packed struct 
{
  // uint8_t direction;
  uint8_t pos1;
  uint8_t pos2;
  uint8_t pos3;
  uint8_t pos4;
  uint8_t pos5;
  uint8_t pos6;
  uint8_t pos7;
  uint8_t pos8;
  uint8_t check_sum;
} dm_pos_cl1_t;

typedef __packed struct 
{
  // uint8_t direction;
  uint8_t direction;
  uint8_t pos1;
  uint8_t pos2;
  uint8_t pos3;
  uint8_t check_sum;
} dm_pos_cl3_t;

void dmMotorInit(void);
void setDMMotorBuf(int id, int32_t pos);
void sendDMMotor(int id);
void requestDMEncoderInfo(int id);
void DMReceiveHandler(void);
void DMMotorAngleInit(void);
void setDMMotorBufWithDirection(int id, int32_t pos, uint8_t direction);

extern dm_motor_header_t dm_motor_header;
extern dm_read_enc_t dm_read_enc;
extern dm_pos_cl1_t dm_pos_cl1;
extern dm_pos_cl3_t dm_pos_cl3;
extern uint8_t dm_motor_buf[4][16];

extern uint8_t steer_buf[];

#define F_INIT_RAW 141
#define BL_INIT_RAW 181
#define BR_INIT_RAW 180

extern int16_t read_angle[4];
extern int read_flag[4];
extern uint8_t init_rotate_direction[4];

#endif // !1
