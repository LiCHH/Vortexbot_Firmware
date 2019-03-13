#include "cmsis_os.h"

#include "imu_task.h"
#include "pid.h"
#include "sys_config.h"
#include "bsp_io.h"

imu_attitede_t attitude;

void imu_temp_keep(void)
{
  imu.temp_ref = DEFAULT_IMU_TEMP;
  pid_calc(&pid_imu_tmp, imu.temp, imu.temp_ref);
  imu_temp_ctrl(pid_imu_tmp.out);
}

void imu_temp_ctrl_init(void)
{
  PID_struct_init(&pid_imu_tmp, POSITION_PID, 300, 150, 10, 1, 0);
}

void imu_param_init()
{
  imu_temp_ctrl_init();
}

void imu_task(void const *argu)
{
  uint32_t imu_wake_time = osKernelSysTick();
  while (1)
  {
    imu_temp_keep();

    mpu_get_data();
    MadgwickUpdate();
    attitude.last_yaw = attitude.yaw;
    attitude.yaw = get_yaw();

    osDelayUntil(&imu_wake_time, IMU_TASK_PERIOD);
  }
}
