
#ifndef __SWMODE_TASK_H__
#define __SWMODE_TASK_H__

#include "stm32f4xx_hal.h"
#include "vortexbot_info.h"

/* Switch mode task period time (ms) */
#define SWMODE_TASK_PERIOD 10

void mode_switch_task(void const *argu);

static void get_vortex_bot_mode(void);
static void get_chassis_mode(void);
static void get_vortex_mode(void);

static void get_last_mode(void);

extern int global_stop_flag;

#endif // !__SWMODE_TASK_H__ 
