#include "dm_motor_drive.h"

#include "string.h"
#include "cmsis_os.h"

#include "sys_config.h"
#include "bsp_uart.h"

dm_motor_header_t dm_motor_header;
dm_pos_cl3_t dm_pos_cl3;
uint8_t dm_motor_buf[4][16];
uint8_t steer_buf[STEER_BUF_LEN];

static uint8_t getCheckSum(uint8_t* pack, int length) {
  int i;
  uint8_t sum = 0;
  for (i = 0; i < length; ++i) {
    sum += pack[i];
  }
  return sum;
}

void dmMotorInit(void) {
  memset(&dm_motor_header, 0, sizeof(dm_motor_header));
  memset(&dm_pos_cl3, 0, sizeof(dm_pos_cl3));
  memset(&dm_motor_buf, 0, sizeof(dm_motor_buf));

  dm_motor_header.header = 0x3E;
  dm_motor_header.cmd = POS_CL1;
  dm_motor_header.data_length = 0x08;
}

void setDMMotorBuf(int id, int32_t pos, uint8_t direction) {
  dm_motor_header.id = (uint8_t)id + 1;
  dm_motor_header.check_sum = getCheckSum((uint8_t *)&dm_motor_header, 4);
  memcpy((uint8_t *)&dm_motor_buf[id], (uint8_t *)&dm_motor_header, 5);

  // dm_pos_cl3.direction = direction;
  dm_pos_cl3.pos1 = 0xFF & pos;
  dm_pos_cl3.pos2 = 0xFF & (pos >> 8);
  dm_pos_cl3.pos3 = 0xFF & (pos >> 16);
  dm_pos_cl3.pos4 = 0x7F & (pos >> 24);
  dm_pos_cl3.pos5 = 0;
  dm_pos_cl3.pos6 = 0;
  dm_pos_cl3.pos7 = 0;
  dm_pos_cl3.pos8 = (0x01 & (pos >> 31)) << 7;
  dm_pos_cl3.check_sum = getCheckSum((uint8_t *)&dm_pos_cl3, 8);
  memcpy((uint8_t *)(&dm_motor_buf[id]) + 5, (uint8_t *)&dm_pos_cl3, 9);
}

void sendDMMotor(int id) {
  //!  unused  id 1
  if(id == 1) return ;

  //! FIXME: don't know why add this make it work for br_wheel
  if(id == 3 || id == 2) {
    HAL_UART_Transmit(&STEER_HUART, (uint8_t *)dm_motor_buf[id], 14, 10);
    HAL_UART_Transmit(&STEER_HUART, (uint8_t *)dm_motor_buf[id], 14, 10); 
    return ;
  }

  HAL_UART_Transmit(&STEER_HUART, (uint8_t *)dm_motor_buf[id], 14, 10); 
 
  // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)dm_motor_buf[id], 14, 10);  
}
