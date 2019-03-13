
#ifndef __BSP_IO_H__
#define __BSP_IO_H__

#include "stm32f4xx_hal.h"
#include "main.h"
#include "tim.h"

///! setup for vortex
#define VORTEX_PORT         TIM4->CCR1    // 设置风扇输出占空比
#define VORTEX_TIM          htim4         // 风扇对应htim
#define VORTEX_CHANNEL      TIM_CHANNEL_1 // 风扇PWM对应通道
#define VORTEX_STOP_DUTY    500           // 停止时占空比
#define VORTEX_DUTY_RANGE   200           // 占空比调整范围
#define VORTEX_DUTY_OFFSET  150           // 占空比到达40%以上时电机开启

///! setup for imu temperature control
#define IMU_TEMP_TIM htim3
#define IMU_TEMP_CHANNEL TIM_CHANNEL_2
#define IMU_PWM_PORT TIM3->CCR2

///! setup for beep
#define BEEP_TIM htim12
#define BEEP_CHANNEL TIM_CHANNEL_1
#define BEEP_TUNE TIM12->ARR
#define BEEP_CTRL TIM12->CCR1

void pwm_device_init(void);

void turn_on_vortex(float spd_ratio);
void turn_off_vortex(void);

void imu_temp_ctrl(uint16_t pwm_pulse);

void beep_ctrl(uint16_t tune, uint16_t ctrl);

#endif //  __BSP_IO_H__
