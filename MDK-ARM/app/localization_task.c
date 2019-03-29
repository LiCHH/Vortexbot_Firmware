#include "localization_task.h"

#include "arm_math.h"

#include "bsp_can.h"
#include "sys_config.h"
#include "servo_info_task.h"
#include "imu_task.h"

odometry_t odom = {0};
localization_t localization = {0};

void localization_task(void const *argu)
{

}

static void omni_odometry_update(void)
{
  //! save old values
  odom.last_trans = odom.curr_trans;
  odom.last_theta = odom.curr_theta;
  odom.last_odom_theta = odom.odom_theta;

  //! get new values
  odom.curr_theta = attitude.yaw;
  odom.curr_trans = (motor_driving[fr_motor].total_angle + 
                     motor_driving[bl_motor].total_angle + 
                     motor_driving[fl_motor].total_angle +
                     motor_driving[br_motor].total_angle) / 4.f / MOTOR_REDUCTION_RATIO;
  odom.rot1 = (servo_infos[fr_motor].angle +
               servo_infos[bl_motor].angle +
               servo_infos[fl_motor].angle +
               servo_infos[br_motor].angle) / 4.f; 
  odom.rot2 = odom.curr_theta - odom.last_theta;
  odom.delta_trans = odom.curr_trans - odom.last_trans;

  #ifdef USE_UWB 
  odom.last_odom_x = localization.x;
  odom.last_odom_y = localization.y;
  odom.last_odom_theta = localization.theta;
  #else
  odom.last_odom_x = odom.odom_x;
  odom.last_odom_y = odom.odom_y;
  odom.last_odom_theta = odom.odom_theta;
  #endif 

  odom.odom_x = odom.last_odom_x + odom.delta_trans * arm_cos_f32(odom.last_theta + odom.rot1);
  odom.odom_y = odom.last_odom_y + odom.delta_trans * arm_sin_f32(odom.last_theta + odom.rot1);
  odom.odom_theta = odom.last_odom_theta + odom.rot2;
}