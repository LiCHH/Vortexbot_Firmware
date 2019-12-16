#include "attitude_control.h"

#include <string.h>

#include "pid.h"
#include "bsp_imu.h"
#include "imu_task.h"
#include "bsp_uart.h"
#include "commons.h"

static double radius[4];
static double beta[4];
static double theta[4];
static double phi[4];
static double speed[4];

double yaw_ref = 0;
pid_t yaw_pid;

//! for test
char output[50];

static void calc_velocity(double omega_ref, double speed_ref, double alpha_ref,
                          int16_t* speed_out, int16_t* angle_out) {
  // omega_ref += 0.1f;
  // double speed_body = speed_ref / 60 * 2 * PI * WHEEL_RADIUS / MOTOR_REDUCTION_RATIO;
  // double radius_ref = fabs(speed_body / omega_ref);
  // if(radius_ref < 5 * BODY_RADIUS) {
  //   angle_out[f_motor]  = alpha_ref;
  //   angle_out[bl_motor] = alpha_ref;
  //   angle_out[br_motor] = alpha_ref;

  //   speed_out[f_motor]  = speed_ref;
  //   speed_out[bl_motor] = speed_ref;
  //   speed_out[br_motor] = speed_ref;

  //   return;
  // }

  // double radius_ref_2 = radius_ref * radius_ref;
  // double body_radius_2 = BODY_RADIUS * BODY_RADIUS;
  // int omega_flag = 1;
  // int alpha_flag = 1;
  // //! 角速度小于0
  // if(omega_ref < 0) omega_flag = -1;
  // //! 移动方向角度在±30°以外
  // if(fabs(alpha_ref) > PI / 6.f) alpha_flag = -1;

  // beta[f_motor] = PI * 0.5 - omega_flag * omega_ref;
  // beta[bl_motor] = PI * 0.6667 - omega_flag * beta[f_motor]; 
  // beta[br_motor] = PI * 0.6667 + omega_flag * beta[f_motor]; 

  // radius[f_motor] = sqrt(body_radius_2 + radius_ref_2 - 2 * BODY_RADIUS * radius_ref * cos(beta[f_motor])); 
  // radius[bl_motor] = sqrt(body_radius_2 + radius_ref_2 - 2 * BODY_RADIUS * radius_ref * cos(beta[bl_motor])); 
  // radius[br_motor] = sqrt(body_radius_2 + radius_ref_2 - 2 * BODY_RADIUS * radius_ref * cos(beta[br_motor])); 
  
  // speed[f_motor]  = omega_ref * radius[f_motor]  / (2 * PI * WHEEL_RADIUS) * 60 * MOTOR_REDUCTION_RATIO;
  // speed[bl_motor] = omega_ref * radius[bl_motor] / (2 * PI * WHEEL_RADIUS) * 60 * MOTOR_REDUCTION_RATIO;
  // speed[br_motor] = omega_ref * radius[br_motor] / (2 * PI * WHEEL_RADIUS) * 60 * MOTOR_REDUCTION_RATIO;

  // theta[f_motor] = acos((radius_ref_2 + radius[f_motor] * radius[f_motor] - body_radius_2) / (2 * radius_ref * radius[f_motor]));
  // theta[bl_motor] = acos((radius_ref_2 + radius[bl_motor] * radius[bl_motor] - body_radius_2) / (2 * radius_ref * radius[bl_motor]));
  // theta[br_motor] = acos((radius_ref_2 + radius[br_motor] * radius[br_motor] - body_radius_2) / (2 * radius_ref * radius[br_motor]));

  // phi[f_motor] = alpha_ref + omega_flag * theta[f_motor];
  // if(omega_ref < 0) {
  //   phi[bl_motor] = alpha_ref + omega_flag * theta[bl_motor];
  //   phi[br_motor] = alpha_ref + omega_flag * alpha_flag * theta[br_motor];
  // } else {
  //   phi[br_motor] = alpha_ref + omega_flag * theta[br_motor];
  //   phi[bl_motor] = alpha_ref + omega_flag * alpha_flag * theta[bl_motor];
  // }

  // if(!(is_in(phi[f_motor], -90.f, 90.f) && is_in(phi[bl_motor], -90.f, 90.f) && 
  //      is_in(phi[br_motor], -90.f, 90.f)) ||
  //    !(is_in(speed[f_motor], -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX) &&
  //      is_in(speed[bl_motor], -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX) &&
  //      is_in(speed[br_motor], -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX))) {
  //   angle_out[f_motor]  = alpha_ref;
  //   angle_out[bl_motor] = alpha_ref;
  //   angle_out[br_motor] = alpha_ref;

  //   speed_out[f_motor]  = speed_ref;
  //   speed_out[bl_motor] = speed_ref;
  //   speed_out[br_motor] = speed_ref;
  // } else {
  //   angle_out[f_motor]  = phi[f_motor];
  //   angle_out[bl_motor] = phi[bl_motor];
  //   angle_out[br_motor] = phi[br_motor];

  //   speed_out[f_motor]  = speed[f_motor];
  //   speed_out[bl_motor] = speed[bl_motor];
  //   speed_out[br_motor] = speed[br_motor];
  // }

  // memset(output, 0, sizeof(output));
  // sprintf(output, "sf:%d sl:%d sr:%d af:%d al:%d ar:%d\r\n",
  //         speed_out[f_motor], speed_out[bl_motor], speed_out[br_motor],
  //         angle_out[f_motor], angle_out[bl_motor], angle_out[br_motor]);
  // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)output, 50, 10);
}

void attitude_control(double speed_ref, double alpha_ref, int16_t* speed_out, int16_t* angle_out) {
  // double yaw_raw = attitude.yaw;
  // double omega_ref;
  // pid_calc(&yaw_pid, yaw_raw, yaw_ref);
  // omega_ref = yaw_pid.out;

  // memset(output, 0, sizeof(output));
  // sprintf(output, "sp:%.2f a:%.2f r:%.2f f:%.2f om:%.2f\r\n", speed_ref, alpha_ref, yaw_ref, yaw_raw, omega_ref);
  // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)output, 50, 10);

  // calc_velocity(omega_ref, speed_ref, alpha_ref, speed_out, angle_out);
}

void attitude_control_init(void) {
  // PID_struct_init(&yaw_pid, POSITION_PID, PI / 2, PI / 2, 0.1f, 0.f, 0.0001f);
}
