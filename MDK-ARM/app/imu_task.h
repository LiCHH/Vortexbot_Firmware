#ifndef  __IMU_TASK_H__
#define __IMU_TASK_H__

#include "stm32f4xx_hal.h"

#include "bsp_imu.h"

#define IMU_TASK_PERIOD 4

typedef struct 
{
  float yaw;
  float last_yaw;
} imu_attitede_t;

extern imu_attitede_t attitude;

void imu_task(void const *argu);
void imu_param_init(void);

#endif // !
