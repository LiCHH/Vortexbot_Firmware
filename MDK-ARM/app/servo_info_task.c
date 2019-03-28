
#include "servo_info_task.h"

#include "cmsis_os.h"

#include "test_ctrl.h"
#include "bsp_uart.h"

#define SERVO_INFO_PERIOD 20

void servo_info_task(void const *argu)
{
  uint32_t servo_wake_time = osKernelSysTick();
  int i;
  while(1) {
    // taskENTER_CRITICAL();
    for(i = 0; i < 4; ++i){
      send_request(i);
      // sprintf((char *)test_buf, "going to send servo id: %d \r\n", i);
      // // HAL_UART_Transmit_DMA(&TEST_HUART, (uint8_t *)test_buf, 50);
      // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)test_buf, 50, 100);
      //! TODO: Check if these code work!!!
      // while(!info_received) {
      //   if(receive_fail) send_request(i);
      // }
    }
    // taskEXIT_CRITICAL();
    osDelayUntil(&servo_wake_time, SERVO_INFO_PERIOD); 
  }
}
