
#include "servo_info_task.h"

#include "cmsis_os.h"

#define SERVO_INFO_PERIOD 10

void servo_info_task(void const *argu)
{
  uint32_t servo_wake_time = osKernelSysTick();
  while(1) {
    taskENTER_CRITICAL();
    for(int i = 1; i <= 4; ++i){
      send_request(i);
      while(!info_received) {
        if(receive_fail) send_request(i);
      }
    }
    taskEXIT_CRITICAL();
    osDelayUntil(&servo_wake_time, SERVO_INFO_PERIOD); 
  }
}
