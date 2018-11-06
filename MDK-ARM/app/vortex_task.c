
#include "vortex_task.h"

vortex_info_t vortex_info;

void vortex_task(void)
{

} 

void vortex_param_init(void)
{
  vortex_info.last_ctrl_mode = VORTEX_STOP;
  vortex_info.ctrl_mode = VORTEX_STOP;
}