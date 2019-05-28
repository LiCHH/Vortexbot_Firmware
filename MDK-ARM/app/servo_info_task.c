
#include "servo_info_task.h"

#include "cmsis_os.h"

#include "test_ctrl.h"
#include "bsp_uart.h"
#include "sys_config.h"


#define SERVO_INFO_PERIOD 20

servo_info_t servo_infos[4];

static void update_angle()
{
//  servo_infos[fr_motor].angle = FR_BL_POS_F * OMNI_INIT_ANGLE + 180 - servo_infos[fr_motor].curr_position;
//  servo_infos[bl_motor].angle = FR_BL_POS_F * OMNI_INIT_ANGLE + 180 - servo_infos[bl_motor].curr_position;
//  servo_infos[fr_motor].angle = FL_BR_POS_F * OMNI_INIT_ANGLE + 180 - servo_infos[fl_motor].curr_position;
//  servo_infos[br_motor].angle = FL_BR_POS_F * OMNI_INIT_ANGLE + 180 - servo_infos[br_motor].curr_position;
}

void servo_info_task(void const *argu)
{
  uint32_t servo_wake_time = osKernelSysTick();
  int i;
  while(1) {
    // taskENTER_CRITICAL();
    for(i = 0; i < 4; ++i){
      // send_request(i);
      // sprintf((char *)test_buf, "going to send servo id: %d \r\n", i);
      // // HAL_UART_Transmit_DMA(&TEST_HUART, (uint8_t *)test_buf, 50);
      // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)test_buf, 50, 100);
      //! TODO: Check if these code work!!!
      // while(!info_received) {
      //   if(receive_fail) send_request(i);
      // }
    }
    taskENTER_CRITICAL();
    update_angle();
    taskEXIT_CRITICAL();
    osDelayUntil(&servo_wake_time, SERVO_INFO_PERIOD); 
  }
}
