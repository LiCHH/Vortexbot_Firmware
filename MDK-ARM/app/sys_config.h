
#ifndef __SYS_H__
#define __SYS_H__

#include "stm32f4xx_hal.h"

/* can relevant */
#define CHASSIS_CAN hcan1

/* uart relevant */
#define PC_HUART    huart6
#define TEST_HUART  huart7
#define JUDGE_HUART huart1

/* math relevant */
#define PI 3.1416f


/* chassis relevant */
/* the ratio of motor encoder value translate to degree */
#define ENCODER_ANGLE_RATIO (8192.0f/360.0f)

#endif
