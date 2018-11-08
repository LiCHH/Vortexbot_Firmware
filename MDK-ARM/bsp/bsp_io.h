
#ifndef __BSP_IO_H__
#define __BSP_IO_H__

#include "stm32f4xx_hal.h"
#include "main.h"
#include "tim.h"

#define VORTEX_PORT       TIM4->CCR1    // 设置风扇输出占空比
#define VORTEX_TIM        htim4         // 风扇对应htim
#define VORTEX_CHANNEL    TIM_CHANNEL_1 // 风扇PWM对应通道
#define VORTEX_STOP_DUTY  1000          // 停止时占空比
#define VORTEX_DUTY_RANGE 1000          // 占空比调整范围

void pwm_device_init(void);

void turn_on_vortex(float spd_ratio);
void turn_off_vortex(void);

#endif //  __BSP_IO_H__
