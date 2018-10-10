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

#include "bsp_can.h"
#include "can.h"
#include "sys_config.h"

// test include
#include "chassis_task.h"

moto_measure_t moto_chassis[4] = {0}; //4 chassis moto
can_transmit_t hcan1_tx_msgs = {0};
can_transmit_t hcan1_rx_msgs = {0};

/**
  * @brief   can filter initialization
  * @param   CAN_HandleTypeDef
  * @retval  None
  */
void can_device_init(void)
{
  //can1 &can2 use same filter config
  CAN_FilterTypeDef can_filter;
  // static CanTxMsgTypeDef Tx1Message;
  // static CanRxMsgTypeDef Rx1Message;
  //  static CanTxMsgTypeDef Tx2Message;
  //  static CanRxMsgTypeDef Rx2Message;

  can_filter.SlaveStartFilterBank = 0; // 取代之前的Filter Number 单个can的时候没什么作用
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK; // 屏蔽位模式，可以识别出一组标识符 CAN_FILTERMODE_IDLIST 标识符列表模式，仅可识别出一个标识符
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
  can_filter.FilterBank = 14;
  can_filter.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  //while (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK);

  can_filter.SlaveStartFilterBank = 14;
  //  HAL_CAN_ConfigFilter(&hcan2, &can_filter);
  //while (HAL_CAN_ConfigFilter(&hcan2, &can_filter) != HAL_OK);

  hcan1_tx_msgs.header.tx.StdId = 0x200;
  hcan1_tx_msgs.header.tx.RTR = CAN_RTR_DATA;
  hcan1_tx_msgs.header.tx.IDE = CAN_ID_STD;
  hcan1_tx_msgs.header.tx.DLC = 0x08;

  HAL_CAN_Start(&hcan1);
  // hcan1.pTxMsg = &Tx1Message;
  // hcan1.pRxMsg = &Rx1Message;
  //  hcan2.pTxMsg = &Tx2Message;
  //  hcan2.pRxMsg = &Rx2Message;
}

void can_receive_start(void)
{
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  // HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  //  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
}

uint32_t FlashTimer;
/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
  * @Param		
  * @Retval		None 
  * @Date     2015/11/24
 *******************************************************************************************/
// void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  if (HAL_GetTick() - FlashTimer > 500)
  {
    //		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
    FlashTimer = HAL_GetTick();
  }
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan1_rx_msgs.header.rx, hcan1_rx_msgs.data);

  //ignore can1 or can2.
  switch (hcan1_rx_msgs.header.rx.StdId)
  {
  case CAN_2006Moto1_ID:
  case CAN_2006Moto2_ID:
  case CAN_2006Moto3_ID:
  case CAN_2006Moto4_ID:
  {
    static uint8_t i;
    i = hcan1_rx_msgs.header.rx.StdId - CAN_2006Moto1_ID;
    moto_chassis[i].msg_cnt++ <= 2 ? get_moto_offset(&moto_chassis[i], hcan) : encoder_data_handler(&moto_chassis[i], hcan);
  }
  break;
  }

  // for test
  chassis_task();

  static int flag = 0;
  if (!flag)
  {
    HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
  }
  /*#### add enable can it again to solve can receive only one ID problem!!!####**/
  // __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收3508电机通过CAN发过来的信息
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void encoder_data_handler(moto_measure_t *ptr, CAN_HandleTypeDef *hcan)
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd = (uint16_t)(hcan1_rx_msgs.data[0] << 8 | hcan1_rx_msgs.data[1]);
  ptr->speed_rpm = (int16_t)(hcan1_rx_msgs.data[2] << 8 | hcan1_rx_msgs.data[3]);
  // ptr->real_current = (hcan1_rx_msgs.data[4]<<8 | hcan1_rx_msgs.data[5])*5.f/16384.f;

  // ptr->hall = hcan1_rx_msgs.data[6]; //not use

  //判断正反转
  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }
  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;

  ptr->speed_rpm = (int16_t)(hcan1_rx_msgs.data[2] << 8 | hcan1_rx_msgs.data[3]);
  ptr->given_current = (int16_t)(hcan1_rx_msgs.data[4] << 8 | hcan1_rx_msgs.data[5]);
}

/*this function should be called after system+can init */
/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef *hcan)
{
  ptr->ecd = (uint16_t)(hcan1_rx_msgs.data[0] << 8 | hcan1_rx_msgs.data[1]);
  ptr->offset_ecd = ptr->ecd;
}

void send_chassis_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  // CHASSIS_CAN.pTxMsg->StdId = 0x200;      //标识符
  // CHASSIS_CAN.pTxMsg->IDE = CAN_ID_STD;   //标准标识符 CAN_ID_EXT 拓展标识符
  // CHASSIS_CAN.pTxMsg->RTR = CAN_RTR_DATA; //数据帧 CAN_RTR_REMOTE 遥控帧
  // CHASSIS_CAN.pTxMsg->DLC = 0x08;         //数据段长度
  hcan1_tx_msgs.data[0] = iq1 >> 8;
  hcan1_tx_msgs.data[1] = iq1;
  hcan1_tx_msgs.data[2] = iq2 >> 8;
  hcan1_tx_msgs.data[3] = iq2;
  hcan1_tx_msgs.data[4] = iq3 >> 8;
  hcan1_tx_msgs.data[5] = iq3;
  hcan1_tx_msgs.data[6] = iq4 >> 8;
  hcan1_tx_msgs.data[7] = iq4;
  uint32_t mailbox;
  while (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)) {}
  HAL_CAN_AddTxMessage(&hcan1, &hcan1_tx_msgs.header.tx, hcan1_tx_msgs.data, &mailbox);
  // HAL_CAN_Transmit(&CHASSIS_CAN, 10);
}
