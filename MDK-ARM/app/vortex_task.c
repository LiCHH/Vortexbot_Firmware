
#include "vortex_task.h"
#include "stdlib.h"
#include "string.h"
#include "remote_ctrl.h"
#include "bsp_io.h"
#include "cmsis_os.h"

vortex_info_t vortex_info;

void vortex_task(void const *argu)
{
  switch(vortex_info.ctrl_mode)
  {
    case VORTEX_ON:
    {
      taskENTER_CRITICAL();

      vortex_info.spd_ratio = ((float)rc_info.knob_v1 - KNOB_V1_MIN) / KNOB_V1_RANGE;
      turn_on_vortex(vortex_info.spd_ratio);

      taskEXIT_CRITICAL();
    } break;

    case VORTEX_OFF:
    {
      vortex_info.spd_ratio = 0;
      turn_off_vortex();
    } break;
    
    default:
    {
      turn_off_vortex();
    } break;

  }
} 

void vortex_param_init(void)
{
  memset(&vortex_info, 0, sizeof(vortex_info));
  turn_off_vortex();
}
