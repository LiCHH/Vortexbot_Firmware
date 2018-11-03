/**
  ******************************************************************************
  * @file		 pid.h
  * @author  Ginger
  * @version V1.0.0
  * @date    2015/11/14
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#ifndef _PID_H
#define _PID_H

#include "stdint.h"

#include "stm32f4xx_hal.h"

enum
{
  LLAST = 0,      //pid.err[0]
  LAST,           //pid.err[1]
  NOW,            //pid.err[2]
};

typedef enum
{
  POSITION_PID,   //位置PID
  DELTA_PID,      //增量式PID
} pid_mode_e;

typedef struct pid_t
{
  float p;
  float i;
  float d;

  float ref;
  float feedback;
  float err[3];

  float pout;
  float iout;
  float dout;
  float out;

  float input_max_err;    //input max err;
  float output_deadband;  //output deadband; 
  
  pid_mode_e pid_mode;
  uint32_t max_out;
  uint32_t integral_limit;

  void (*f_param_init)(struct pid_t *pid, 
                       pid_mode_e      pid_mode,
                       uint32_t      max_output,
                       uint32_t      inte_limit,
                       float         p,
                       float         i,
                       float         d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);
} pid_t;

void PID_struct_init(
    pid_t *pid,
    pid_mode_e mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float kp,
    float ki,
    float kd);
    
float pid_calc(pid_t *pid, float feedback, float ref);

extern pid_t pid_driving_spd[4];
extern pid_t pid_steer_spd[4];
extern pid_t pid_steer_pos[4];
  
#endif
