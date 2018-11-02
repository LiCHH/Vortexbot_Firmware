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
#include "bsp_uart.h"
#include "can.h"
#include "sys_config.h"

moto_measure_t moto_chassis[4] = {0};//4 chassis moto


/**
  * @brief   can filter initialization
  * @param   CAN_HandleTypeDef
  * @retval  None
  */
void can_device_init(void)
{
  //can1 &can2 use same filter config
  CAN_FilterConfTypeDef  can_filter;
  static CanTxMsgTypeDef Tx1Message;
  static CanRxMsgTypeDef Rx1Message;
//  static CanTxMsgTypeDef Tx2Message;
//  static CanRxMsgTypeDef Rx2Message;

  can_filter.FilterNumber         = 0;
  can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;  //屏蔽位模式，可以识别出一组标识符 CAN_FILTERMODE_IDLIST 标识符列表模式，仅可识别出一个标识符
  can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;  
  can_filter.FilterIdHigh         = 0x0000;
  can_filter.FilterIdLow          = 0x0000;
  can_filter.FilterMaskIdHigh     = 0x0000;
  can_filter.FilterMaskIdLow      = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
  can_filter.BankNumber           = 14;
  can_filter.FilterActivation     = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  //while (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK);
  
  can_filter.FilterNumber         = 14;
//  HAL_CAN_ConfigFilter(&hcan2, &can_filter);
  //while (HAL_CAN_ConfigFilter(&hcan2, &can_filter) != HAL_OK);
    
  hcan1.pTxMsg = &Tx1Message;
  hcan1.pRxMsg = &Rx1Message;
//  hcan2.pTxMsg = &Tx2Message;
//  hcan2.pRxMsg = &Rx2Message;
}

void can_receive_start(void)
{
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
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
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
    static uint16_t count = 0;
    uint8_t temp[2];
    temp[0] = moto_chassis[0].ecd >> 8;
    temp[1] = moto_chassis[0].ecd;
    if(count == 1000){
        HAL_UART_Transmit_DMA(&TEST_HUART, (uint8_t *)temp, sizeof(temp));
        count = 0;
    } else {
        count += 1;
    }
    
	if(HAL_GetTick() - FlashTimer>500){
//		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		FlashTimer = HAL_GetTick();
	}

	//ignore can1 or can2.
	switch(_hcan->pRxMsg->StdId){
		case CAN_2006Moto1_ID:
		case CAN_2006Moto2_ID:
		case CAN_2006Moto3_ID:
		case CAN_2006Moto4_ID:
			{
				static uint8_t i;
				i = _hcan->pRxMsg->StdId - CAN_2006Moto1_ID;				
				moto_chassis[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[i], _hcan) : encoder_data_handler(&moto_chassis[i], _hcan);
			}
			break;
	}
    
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);


}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收3508电机通过CAN发过来的信息
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void encoder_data_handler(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	ptr->last_ecd = ptr->ecd;
	ptr->ecd = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->speed_rpm  = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
	// ptr->real_current = (hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5])*5.f/16384.f;

	// ptr->hall = hcan->pRxMsg->Data[6]; //not use
	
	//判断正反转
	if(ptr->ecd - ptr->last_ecd > 4096){
		ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }
	else if (ptr->ecd - ptr->last_ecd < -4096){
		ptr->round_cnt ++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else{
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }
	ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;
  
  ptr->speed_rpm = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
  ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4] << 8 | hcan->pRxMsg->Data[5]);
}


/*this function should be called after system+can init */
/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	ptr->ecd = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->offset_ecd = ptr->ecd;
}


void send_chassis_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  CHASSIS_CAN.pTxMsg->StdId = 0x200;      //标识符
  CHASSIS_CAN.pTxMsg->IDE = CAN_ID_STD;   //标准标识符 CAN_ID_EXT 拓展标识符
  CHASSIS_CAN.pTxMsg->RTR = CAN_RTR_DATA; //数据帧 CAN_RTR_REMOTE 遥控帧
  CHASSIS_CAN.pTxMsg->DLC = 0x08;         //数据段长度
  CHASSIS_CAN.pTxMsg->Data[0] = iq1 >> 8;
  CHASSIS_CAN.pTxMsg->Data[1] = iq1;
  CHASSIS_CAN.pTxMsg->Data[2] = iq2 >> 8;
  CHASSIS_CAN.pTxMsg->Data[3] = iq2;
  CHASSIS_CAN.pTxMsg->Data[4] = iq3 >> 8;
  CHASSIS_CAN.pTxMsg->Data[5] = iq3;
  CHASSIS_CAN.pTxMsg->Data[6] = iq4 >> 8;
  CHASSIS_CAN.pTxMsg->Data[7] = iq4;
  HAL_CAN_Transmit(&CHASSIS_CAN, 10);
  static int flag = 0;
  if(!flag){
    HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
  }
  else{
    HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
  }
}
