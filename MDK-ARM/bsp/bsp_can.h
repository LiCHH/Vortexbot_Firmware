/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#ifndef __BSP_CAN
#define __BSP_CAN

#ifdef STM32F4
#include "stm32f4xx_hal.h"
#elif defined STM32F1
#include "stm32f1xx_hal.h"
#endif
//#include "mytype.h"
#include "can.h"

/*CAN发送或是接收的ID*/
typedef enum
{
  CAN_HIGH_ID       = 0x1FF,
  CAN_LOW_ID        = 0x200,

  DRIVING_MOTOR1_ID = 0x201,
  DRIVING_MOTOR2_ID = 0x202,
  DRIVING_MOTOR3_ID = 0x203,
  DRIVING_MOTOR4_ID = 0x204,

  STEER_MOTOR1_ID   = 0x205,
  STEER_MOTOR2_ID   = 0x206,
  STEER_MOTOR3_ID   = 0x207,
  STEER_MOTOR4_ID   = 0x208
} can_msg_id_e;

#define FILTER_BUF 5
typedef struct
{
  //Encoder
  uint16_t ecd;      //angle
  uint16_t last_ecd; //last_angle

  int16_t speed_rpm;     //speed_rpm
  int16_t given_current; //given_current

  int32_t round_cnt;   // round_cnt
  int32_t total_ecd;   // total_angle

  int32_t total_angle; // total_ecd / encoder ratio

  uint16_t offset_ecd;
  uint32_t msg_cnt;

  int32_t ecd_raw_rate;
  int32_t rate_buf[FILTER_BUF]; //use in EC60
  uint8_t buf_cut;              //use in EC60
  int32_t filter_rate;          //use in EC60
} moto_measure_t;

typedef struct
{
  union {
    CAN_RxHeaderTypeDef rx;
    CAN_TxHeaderTypeDef tx;
  } header;
  uint8_t data[8];
} can_transmit_t;

/* Extern  ------------------------------------------------------------------*/
extern moto_measure_t motor_driving[];
extern moto_measure_t motor_steer[];

void can_device_init(void);
void can_receive_start(void);

void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef *hcan);
void encoder_data_handler(moto_measure_t *ptr, CAN_HandleTypeDef *hcan);

void send_chassis_current(can_msg_id_e id, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
//void set_moto_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
#endif
