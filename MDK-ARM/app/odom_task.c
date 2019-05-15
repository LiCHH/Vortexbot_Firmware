#include "odom_task.h"

#include "string.h"
#include "cmsis_os.h"

#include "sys_config.h"
#include "imu_task.h"
#include "bsp_can.h"
#include "servo_info_task.h"
#include "kf.h"

odom_t odom;

void odomTaskInit(void) { memset(&odom, 0, sizeof(odom)); }

void odomTask(void const* argu) {
  uint32_t odom_wake_time = osKernelSysTick();
  while (1) {
    odom.last_theta = odom.theta;
    odom.last_rot = odom.rot;
    odom.theta = imu.yaw;
    odom.duration = HAL_GetTick() - odom.last_tick;
    odom.last_tick = HAL_GetTick();
    // odom.trans = (motor_driving[0].speed_rpm + motor_driving[1].speed_rpm +
    //               motor_driving[2].speed_rpm + motor_driving[3].speed_rpm) /
    //              4.f * RPM_TO_RPS * WHEEL_RADIUS * 2 * PI * odom.duration;
    odom.trans = (motor_driving[0].pass_angle + motor_driving[1].pass_angle +
                  motor_driving[2].pass_angle + motor_driving[3].pass_angle) /
                 4 / MOTOR_REDUCTION_RATIO * DEG_TO_RAD * WHEEL_RADIUS;

    odom.rot = (servo_infos[0].angle + servo_infos[1].angle +
                servo_infos[2].angle + servo_infos[3].angle) /
               4;
    kfPredict(odom.last_theta * DEG_TO_RAD, odom.last_rot, odom.trans);
    odom.x = kf.mu_curr.pData[0];
    odom.y = kf.mu_curr.pData[1];

    osDelayUntil(&odom_wake_time, ODOM_TASK_PERIOD);
  }
}
