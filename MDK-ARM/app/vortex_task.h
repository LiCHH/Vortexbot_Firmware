
#ifndef __VORTEX_TASK_H__
#define __VORTEX_TASK_H__

#include "stm32f4xx_hal.h"

typedef enum
{
  VORTEX_OFF,
  VORTEX_ON
} vortex_mode_e;

typedef struct
{
  vortex_mode_e ctrl_mode;
  vortex_mode_e last_ctrl_mode;
} vortex_info_t;

void vortex_task(void);
void vortex_param_init(void);

extern vortex_info_t vortex_info;

#endif // !1 __VORTEX_TASK_H__
