
#ifndef __SYS_H__
#define __SYS_H__

#include "stm32f4xx_hal.h"
#include "math.h"

#define BOT_ID 3

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

#define FLOAT_THRESHOLD 0.001f

#define SIGN(x) ((x < 0) ? -1 : 1)
#define INT_LIMIT(x, range) ((abs(x) < range ? x : range * SIGN(x)))

#define RESTRICT_ANGLE(angle) do {  \
  while(angle > 180) angle -= 360;  \
  while(angle < -180) angle += 360; \
} while(0);

#define RESTRICT_ANGLE_RAD(angle) do {  \
  while(angle > PI)  angle -= 2 * PI;   \
  while(angle < -PI) angle += 2 * PI;   \
} while(0);

typedef enum
{
  fr_motor = 0,
  fl_motor = 1,
  bl_motor = 2,
  br_motor = 3
} motor_seq_e;

#define MOTOR_SPEED_MAX 300 // 150

#define MOTOR_REDUCTION_RATIO 36

#define WHEEL_RADIUS 0.04f // 0.045f

#define BODY_RADIUS 0.2228f

#define LINSPD_TO_RPM (60 / (2 * PI * WHEEL_RADIUS))

#define ROBOT_LIN_SPD_MAX (MOTOR_SPEED_MAX / LINSPD_TO_RPM)

#define ROBOT_ANG_SPD_MAX (ROBOT_LIN_SPD_MAX / BODY_RADIUS)


#define FR_POS_F   -1
#define FL_POS_F    1
#define BL_POS_F   -1
#define BR_POS_F    1

#define FR_SPD_F   -1
#define FL_SPD_F    1
#define BL_SPD_F    1
#define BR_SPD_F   -1

//! for using M2006 for steer
// #define STEER_FR_OFFSET -((float)0x0051 / ENCODER_ANGLE_RATIO / MOTOR_REDUCTION_RATIO)
// #define STEER_FL_OFFSET -((float)0x0001 / ENCODER_ANGLE_RATIO / MOTOR_REDUCTION_RATIO)
// #define STEER_BR_OFFSET ((float)0x00D5 / ENCODER_ANGLE_RATIO / MOTOR_REDUCTION_RATIO)
// #define STEER_BL_OFFSET ((float)0x0061 / ENCODER_ANGLE_RATIO / MOTOR_REDUCTION_RATIO)

//！ for using DM
#if BOT_ID == 1
#define STEER_F_OFFSET 0.5
#define STEER_BL_OFFSET 5.5
#define STEER_BR_OFFSET -2.5
#elif BOT_ID == 2
#define STEER_F_OFFSET -0.2f
#define STEER_BL_OFFSET 3.f
#define STEER_BR_OFFSET 9.5f
#elif BOT_ID == 3
#define STEER_FR_OFFSET 2.5
#define STEER_FL_OFFSET 6.0
#define STEER_BL_OFFSET 0.8
#define STEER_BR_OFFSET 2.5
#endif

#define OMNI_INIT_FRONT_ANGLE   45
#define OMNI_INIT_BACK_ANGLE    45
#define STEER_INIT_ANGLE_FR     135
#define STEER_INIT_ANGLE_FL     45
#define STEER_INIT_ANGLE_BL     135
#define STEER_INIT_ANGLE_BR     45
#define STEER_SERVO_OFFSET 2048

/* chassis relevant */
/* the ratio of motor encoder value translate to degree */
#define ENCODER_ANGLE_RATIO 0.0439f // (360.0f / 8192.0)

///! setup robot's initial pitch angle on wall
#define ROBOT_INIT_PITCH 90.f

///! setup imu temperature control
#define DEFAULT_IMU_TEMP 50

#endif
