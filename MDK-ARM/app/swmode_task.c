
#include "cmsis_os.h"

#include "swmode_task.h"
#include "chassis_task.h"
#include "vortex_task.h"

/* stack usage monitor */
UBaseType_t mode_stack_surplus;

extern osTimerId chassis_timer_id;
extern osTimerId vortex_timer_id;

void mode_switch_task(void const *argu)
{
  osTimerStart(chassis_timer_id, CHASSIS_TIMER_PERIOD); 
  osTimerStart(vortex_timer_id, VORTEX_TASK_PERIOD); 

  uint32_t mode_wake_time = osKernelSysTick();
  while (1)
  {
    taskENTER_CRITICAL();

    get_last_mode();
    get_vortex_bot_mode();
    get_vortex_mode();
    get_chassis_mode();

    taskEXIT_CRITICAL();

//    mode_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
    osDelayUntil(&mode_wake_time, SWMODE_TASK_PERIOD);
  }
}

static void get_vortex_bot_mode()
{
  switch (rc_info.sb)
  {
  case SW_UP:
  {
    bot_mode = SAFETY_MODE;
  }
  break;

  case SW_MID:
  {
    bot_mode = MANUL_CONTROL_MODE;
  }
  break;

  case SW_DOWN:
  {
    bot_mode = AUTO_CONTROL_MODE;
  }
  break;

  default:
  {
    bot_mode = SAFETY_MODE;
  }
  break;
  }
}

static void chassis_mode_handler(void)
{
  switch (bot_mode)
  {
  case MANUL_CONTROL_MODE:
  {
    if (rc_info.sc == SW_MID)
    {
      chassis.ctrl_mode = OMNI_DIRECTIONAL;
    }
    else if (rc_info.sc == SW_UP)
    {
      chassis.ctrl_mode = FORWARD_DIRECTIONAL;
    }
    else if (rc_info.sc == SW_DOWN)
    {
      chassis.ctrl_mode = ATTITUDE_CONTROL;
    }
    else
    {
      chassis.ctrl_mode = CHASSIS_STOP;
    }
  }
  break;

  case AUTO_CONTROL_MODE:
  {
    chassis.ctrl_mode = CHASSIS_STOP;
  }
  break;

  case SAFETY_MODE:
  {
    chassis.ctrl_mode = CHASSIS_STOP;
  }
  break;

  default:
  {
    chassis.ctrl_mode = CHASSIS_STOP;
  }
  break;
  }
}

static void get_chassis_mode(void)
{
  chassis_mode_handler();
}

static void vortex_mode_handler(void)
{
  switch (rc_info.sa)
  {
  case SW_UP:
  {
    vortex_info.ctrl_mode = VORTEX_OFF;
  }
  break;

  case SW_DOWN:
  {
    vortex_info.ctrl_mode = VORTEX_ON;
  }
  break;

  default:
  {
    vortex_info.ctrl_mode = VORTEX_ON;
  }
  break;
  }
}

static void get_vortex_mode(void)
{
  vortex_mode_handler();
}

static void get_last_mode()
{
  last_bot_mode = bot_mode;
  chassis.last_ctrl_mode = chassis.ctrl_mode;
  vortex_info.last_ctrl_mode = vortex_info.ctrl_mode;
}
