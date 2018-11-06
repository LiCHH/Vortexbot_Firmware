
#ifndef __VORTEXBOT_INFO_H__
#define __VORTEXBOT_INFO_H__

#include "stm32f4xx_hal.h"

typedef enum
{
  SAFETY_MODE,
  MANUL_CONTROL_MODE,
  AUTO_CONTROL_MODE
} vortexbot_mode_e;



extern vortexbot_mode_e bot_mode;
extern vortexbot_mode_e last_bot_mode;

#endif // __VORTEXBOT_INFO_H__
