
#ifndef __SWMODE_TASK_H__
#define __SWMODE_TASK_H__

#include "stm32f4xx_hal.h"
#include "remote_ctrl.h"
#include "vortexbot_info.h"


void mode_switch_task(void);

static void get_vortex_bot_mode(void);

#endif // !__SWMODE_TASK_H__ 