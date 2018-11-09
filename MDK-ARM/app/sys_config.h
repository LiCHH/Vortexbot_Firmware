
#ifndef __SYS_H__
#define __SYS_H__

#include "stm32f4xx_hal.h"

/* can relevant */
#define CHASSIS_CAN hcan1

/* uart relevant */
#define PC_HUART    huart6
#define TEST_HUART  huart7
#define RC_HUART    huart1

/* math relevant */
#define PI 3.1416f
#define RAD_TO_ANG (180.f / PI)

#define FLOAT_THRESHOLD 0.01

typedef enum{
  fr_motor = 0,
  fl_motor = 1,
  bl_motor = 2,
  br_motor = 3
} motor_seq_e;

#define FR_BL_FLAG 1
#define FL_BR_FLAG -1

#define STEER_FR_OFFSET 0 
#define STEER_FL_OFFSET 0 
#define STEER_BR_OFFSET 0 
#define STEER_BL_OFFSET 0

/* chassis relevant */
/* the ratio of motor encoder value translate to degree */
#define ENCODER_ANGLE_RATIO (8192.0f/360.0f)

#endif
