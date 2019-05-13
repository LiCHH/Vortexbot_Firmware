
#include "bsp_io.h"

void turn_on_vortex(float spd_ratio)
{
  VORTEX_PORT = VORTEX_STOP_DUTY + spd_ratio * VORTEX_DUTY_RANGE + VORTEX_DUTY_OFFSET;
}

void turn_off_vortex(void)
{
  VORTEX_PORT = VORTEX_STOP_DUTY;
}

void imu_temp_ctrl(uint16_t pwm_pulse)
{
  IMU_PWM_PORT = pwm_pulse;
}

void beep_ctrl(uint16_t tune, uint16_t ctrl)
{
  BEEP_TUNE = tune;
  BEEP_CTRL = ctrl;
}

void pwm_device_init(void)
{
  HAL_TIM_PWM_Start(&VORTEX_TIM, VORTEX_CHANNEL);
  VORTEX_PORT = VORTEX_STOP_DUTY;
  HAL_TIM_PWM_Start(&IMU_TEMP_TIM, IMU_TEMP_CHANNEL);
  HAL_TIM_PWM_Start(&BEEP_TIM, BEEP_CHANNEL);
}
