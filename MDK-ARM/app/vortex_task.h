
#ifndef __VORTEX_TASK_H__
#define __VORTEX_TASK_H__

#include "stm32f4xx_hal.h"

#define VORTEX_TASK_PERIOD 100

typedef enum
{
  VORTEX_OFF = 0,
  VORTEX_ON  = 1
} vortex_mode_e;

typedef struct
{
  float spd_ratio;

  vortex_mode_e ctrl_mode;
  vortex_mode_e last_ctrl_mode;
} vortex_info_t;

void vortex_task(void const *argu);
void vortex_param_init(void);

extern vortex_info_t vortex_info;

#endif // !1 __VORTEX_TASK_H__
