#ifndef __ATTITUDE_CONTROL_H__
#define __ATTITUDE_CONTROL_H__

#include "sys_config.h"

void attitude_control(double speed_ref, double alpha_ref, int16_t* speed_out,
                      int16_t* angle_out);
void attitude_control_init(void);

#endif
