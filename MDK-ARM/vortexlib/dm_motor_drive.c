#include "dm_motor_drive.h"

#include "string.h"
#include "cmsis_os.h"

#include "sys_config.h"
#include "bsp_uart.h"
#include "test_ctrl.h"

dm_motor_header_t dm_motor_header;
dm_read_enc_t dm_read_enc; 
dm_pos_cl1_t dm_pos_cl1;
dm_pos_cl3_t dm_pos_cl3;
uint8_t dm_motor_buf[4][16];
uint8_t steer_buf[STEER_BUF_LEN];
int16_t read_angle[4];
int read_flag[4];
uint8_t init_rotate_direction[4];

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
  memset(&dm_pos_cl1, 0, sizeof(dm_pos_cl1));
  memset(&dm_pos_cl3, 0, sizeof(dm_pos_cl3));
  memset(&dm_motor_buf, 0, sizeof(dm_motor_buf));

  dm_motor_header.header = 0x3E;
  dm_motor_header.cmd = POS_CL1;
  dm_motor_header.data_length = 0x08;

  dm_read_enc.header = 0x3E;
  dm_read_enc.cmd = READ_ENC;
  dm_read_enc.data_length = 0x00;
}

void setDMMotorBuf(int id, int32_t pos) {
//   sprintf(test_buf, "%03d\r\n", pos/100);
//   HAL_UART_Transmit(&TEST_HUART, test_buf, 5, 5);
  dm_motor_header.id = (uint8_t)id + 1;
  dm_motor_header.check_sum = getCheckSum((uint8_t *)&dm_motor_header, 4);
  memcpy((uint8_t *)&dm_motor_buf[id], (uint8_t *)&dm_motor_header, 5);

  // dm_pos_cl3.direction = direction;
  dm_pos_cl1.pos1 = 0xFF & pos;
  dm_pos_cl1.pos2 = 0xFF & (pos >> 8);
  dm_pos_cl1.pos3 = 0xFF & (pos >> 16);
  dm_pos_cl1.pos4 = 0x7F & (pos >> 24);
  dm_pos_cl1.pos5 = 0;
  dm_pos_cl1.pos6 = 0;
  dm_pos_cl1.pos7 = 0;
  dm_pos_cl1.pos8 = (0x01 & (pos >> 31)) << 7;
  dm_pos_cl1.check_sum = getCheckSum((uint8_t *)&dm_pos_cl1, 8);
  memcpy((uint8_t *)(&dm_motor_buf[id]) + 5, (uint8_t *)&dm_pos_cl1, 9);
}

void sendDMMotor(int id) {
  //! FIXME: don't know why add this make it work for br_wheel
  taskENTER_CRITICAL();
  // if(id == 1) 
  //   HAL_UART_Transmit(&STEER_HUART, (uint8_t *)dm_motor_buf[id], 14, 5); 
  HAL_UART_Transmit(&STEER_HUART, (uint8_t *)dm_motor_buf[id], 14, 10); 
  taskEXIT_CRITICAL();
  // osDelay(1);
  HAL_Delay(1);
 
  // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)dm_motor_buf[id], 14, 10);  
}

void requestDMEncoderInfo(int id) {
  // if(id == 1) return;
  dm_read_enc.id = id + 1;
  dm_read_enc.check_sum = getCheckSum((uint8_t *)&dm_read_enc, 4);
  HAL_UART_Transmit(&STEER_HUART, (uint8_t *)&dm_read_enc, 5, 10);
  // HAL_UART_Transmit(&STEER_HUART, (uint8_t *)&dm_read_enc, 5, 10);
  // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)&dm_read_enc, 5, 10);
}

void DMReceiveHandler(void) {
  // if(!read_flag[f_motor] || !read_flag[bl_motor] || !read_flag[br_motor]) {
    // int bias = 0;
    // char output[40];
    // memset(output, 0, 40);
    // sprintf(output, "fuck3\n");
    // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)output, 10, 10);
    // while (bias < STEER_BUF_LEN)
    // {
    //   for(; bias < STEER_BUF_LEN; ++bias) {
    //     if(steer_buf[bias] == 0x3E) break;
    //   }
    //   if(bias == STEER_BUF_LEN) break;
    //   if(getCheckSum((uint8_t *)(steer_buf + 5 + bias), 2) == steer_buf[7 + bias]) {
    //     int id = (int)steer_buf[2 + bias];
    //     int encoder = (steer_buf[6 + bias] << 8) + steer_buf[5 + bias];
    //     double angle = encoder / 4096.f * 360.f;
 
    //     read_flag[id - 1] = 1;
    //     read_angle[id - 1] = angle;
    //   }
    //   bias += 8;
    //   // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)steer_buf, 10, 10);
    //   // sprintf(output, "\r\n");
    //   // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)output, 2, 10);
    // }
    // sprintf(output, "%d %d %d %d\r\n", read_angle[0], read_angle[1], read_angle[2], read_angle[3]);
    // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)output, 40, 10);
    // memset(steer_buf, 0, STEER_BUF_LEN);
  // }
  // HAL_UART_Transmit(&TEST_HUART, (uint8_t*)steer_buf, STEER_BUF_LEN, 10);
  // memset(steer_buf, 0, STEER_BUF_LEN);
  // sprintf(test_buf, "\r\n");
  // HAL_UART_Transmit(&TEST_HUART, (uint8_t*)test_buf, 2, 1);
}

void DMMotorAngleInit(void) {
  char output[30];
  memset(output, 0, sizeof(output));
  memset(read_flag, 0, sizeof(read_flag));
  while (!read_flag[fr_motor] || !read_flag[fl_motor] || !read_flag[bl_motor] || !read_flag[br_motor])
  {
    for(int i = 0; i < 4; ++i) {
      requestDMEncoderInfo(i);
      HAL_Delay(10);
    }
    HAL_Delay(20);
  }
  uint8_t direction = 0;
  int16_t init_angle_raw[4];
  init_angle_raw[fr_motor] = F_INIT_RAW;
  init_angle_raw[fl_motor] = F_INIT_RAW;
  init_angle_raw[bl_motor] = BL_INIT_RAW;
  init_angle_raw[br_motor] = BR_INIT_RAW;
  for(int i = 0; i < 4; ++i) {
    sprintf(output, "fuck5\n");
    HAL_UART_Transmit(&TEST_HUART, (uint8_t *)output, 10, 10);
    if(i == 1) continue;
    if(read_angle[i] > init_angle_raw[i]) {
      direction = CLOCKWISE;
    } else {
      direction = COUNTERCLOCKWISE;
    }
    init_rotate_direction[i] = direction;
    sprintf(output, "id:%d s:%d r:%d d:%d\n", i, init_angle_raw[i], read_angle[i], direction);
    HAL_UART_Transmit(&TEST_HUART, (uint8_t *)output, 30, 10);
  }
  sprintf(output, "finish\n");
  HAL_UART_Transmit(&TEST_HUART, (uint8_t *)output, 10, 10);
  
}

void setDMMotorBufWithDirection(int id, int32_t pos, uint8_t direction) {
  dm_motor_header.id = (uint8_t)id + 1;
  dm_motor_header.data_length = 0x04;
  dm_motor_header.cmd = POS_CL3;
  dm_motor_header.check_sum = getCheckSum((uint8_t *)&dm_motor_header, 4);
  memcpy((uint8_t *)&dm_motor_buf[id], (uint8_t *)&dm_motor_header, 5);

  // dm_pos_cl3.direction = direction;
  dm_pos_cl3.direction = direction;
  dm_pos_cl3.pos1 = 0xFF & pos;
  dm_pos_cl3.pos2 = 0xFF & (pos >> 8);
  dm_pos_cl3.pos3 = (0x7F & (pos >> 16)) | ((0x01 & (pos >> 31)) << 7);
  dm_pos_cl3.check_sum = getCheckSum((uint8_t *)&dm_pos_cl3, 4);
  memcpy((uint8_t *)(&dm_motor_buf[id]) + 5, (uint8_t *)&dm_pos_cl3, 5);

  //! Reset header
  dm_motor_header.cmd = POS_CL1;
  dm_motor_header.data_length = 0x08;
}
