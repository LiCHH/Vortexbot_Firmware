
#include "bsp_io.h"

void turn_on_vortex(float spd_ratio)
{
  VORTEX_PORT = VORTEX_STOP_DUTY + spd_ratio * VORTEX_DUTY_RANGE;
}

void turn_off_vortex(void)
{
  VORTEX_PORT = VORTEX_STOP_DUTY;
}

void pwm_device_init(void)
{
  HAL_TIM_PWM_Start(&VORTEX_TIM, VORTEX_CHANNEL);
  VORTEX_PORT = VORTEX_STOP_DUTY;
}
