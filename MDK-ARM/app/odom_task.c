#include "odom_task.h"

#include "string.h"
#include "cmsis_os.h"
#include "math.h"

#include "sys_config.h"
#include "imu_task.h"
#include "bsp_can.h"
#include "servo_info_task.h"
#include "kf.h"
#include "bsp_uart.h"
#include "test_ctrl.h"
#include "chassis_task.h"

odom_t odom;

void odomTaskInit(void) { memset(&odom, 0, sizeof(odom)); }

void odom_task(void const* argu) {
  uint32_t odom_wake_time = osKernelSysTick();
  while (1) {
    odom.last_theta = odom.theta;
    odom.last_rot = odom.rot;
    odom.last_motor_angle = odom.curr_motor_angle;

    // odom.theta = attitude.yaw;
    // TODO: For test
    odom.theta = 0;
    odom.rot = chassis.mv_direction;
    odom.curr_motor_angle =
        (abs(motor_driving[0].total_angle) + abs(motor_driving[2].total_angle) +
         abs(motor_driving[3].total_angle)) /
        3 / MOTOR_REDUCTION_RATIO * DEG_TO_RAD;
    odom.trans = (odom.curr_motor_angle - odom.last_motor_angle) * WHEEL_RADIUS;

    kfPredict(odom.last_theta * DEG_TO_RAD, odom.last_rot * DEG_TO_RAD, odom.trans);
    odom.x = kf.mu_curr.pData[0];
    odom.y = kf.mu_curr.pData[1];

    sprintf((char*)test_buf, "%.2f %.2f %.2f\r\n", odom.x, odom.y, odom.rot);
    HAL_UART_Transmit(&TEST_HUART, test_buf, 20, 10);

    osDelayUntil(&odom_wake_time, ODOM_TASK_PERIOD);
  }
}
