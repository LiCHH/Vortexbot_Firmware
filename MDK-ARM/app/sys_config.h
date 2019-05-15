
#ifndef __SYS_H__
#define __SYS_H__

#include "stm32f4xx_hal.h"
#include "math.h"

/* can relevant */
#define CHASSIS_CAN hcan1

/* uart relevant */
#define PC_HUART huart7
#define TEST_HUART huart7
#define RC_HUART huart1
#define STEER_HUART huart8
#define UWB_HUART huart6

/* math relevant */
#define PI 3.1416f
#define RAD_TO_DEG 57.3f
#define DEG_TO_RAD 0.0175f 

#define RPM_TO_RPS 0.0167f

#define FLOAT_THRESHOLD 50

#define SIGN(x) ((x < 0) ? -1 : 1)
#define INT_LIMIT(x, range) ((abs(x) < range ? x : range * SIGN(x)))

#define RESTRICT_ANGLE(angle) do {  \
  while(angle > 180) angle -= 360;     \
  while(angle < -180) angle += 360;\
} while(0);

typedef enum
{
  fr_motor = 0,
  fl_motor = 1,
  bl_motor = 2,
  br_motor = 3
} motor_seq_e;

#define MOTOR_SPEED_MAX 150

#define MOTOR_REDUCTION_RATIO 36

#define WHEEL_RADIUS 0.045f

#define FR_BL_POS_F -1
#define FL_BR_POS_F 1

#define FR_BR_SPD_F -1
#define FL_BL_SPD_F 1

#define STEER_FR_OFFSET -((float)0x0051 / ENCODER_ANGLE_RATIO / MOTOR_REDUCTION_RATIO)
#define STEER_FL_OFFSET -((float)0x0001 / ENCODER_ANGLE_RATIO / MOTOR_REDUCTION_RATIO)
#define STEER_BR_OFFSET ((float)0x00D5 / ENCODER_ANGLE_RATIO / MOTOR_REDUCTION_RATIO)
#define STEER_BL_OFFSET ((float)0x0061 / ENCODER_ANGLE_RATIO / MOTOR_REDUCTION_RATIO)

#define OMNI_INIT_ANGLE      45
#define STEER_SERVO_OFFSET 2048

/* chassis relevant */
/* the ratio of motor encoder value translate to degree */
#define ENCODER_ANGLE_RATIO 0.0439f // (360.0f / 8192.0)

///! setup robot's initial pitch angle on wall
#define ROBOT_INIT_PITCH 0.f

///! setup imu temperature control
#define DEFAULT_IMU_TEMP 50


#endif
