
#include "swmode_task.h"

void mode_switch_task()
{
  get_vortex_bot_mode();
  get_vortex_mode();
  get_chassis_mode();
}

static void get_vortex_bot_mode()
{
  switch (rc_info.sb)
  {
    case SW_UP:
    {
      bot_mode = SAFETY_MODE;
    } break;

    case SW_MID:
    {
      bot_mode = MANUL_CONTROL_MODE;
    } break;

    case SW_DOWN:
    {
      bot_mode = AUTO_CONTROL_MODE;
    } break;

    default:
    {
      bot_mode = SAFETY_MODE;
    } break;
  }
}

static void get_chassis_mode()
{

}