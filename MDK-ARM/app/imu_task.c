#include "cmsis_os.h"

#include "imu_task.h"
#include "pid.h"
#include "sys_config.h"
#include "bsp_io.h"
#include "bsp_uart.h"
#include "test_ctrl.h"

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
  // int count = 0;
  while (1)
  {
    imu_temp_keep();

    mpu_get_data();
    MadgwickUpdate();
    attitude.last_yaw = attitude.yaw;
    taskENTER_CRITICAL();
    attitude.yaw = get_yaw();
    taskEXIT_CRITICAL();
    RESTRICT_ANGLE(attitude.yaw);

    //! test code 
    // sprintf((char *)test_buf, "%.2f\r\n", attitude.yaw * RAD_TO_DEG);
    // sprintf((char *)test_buf, "ax: %d, ay: %d, az: %d, mx: %d, my: %d, mz: %d, gx: %d, gy: %d, gz: %d\r\n",
    //                            mpu_data.ax, mpu_data.ay, mpu_data.az,
    //                            mpu_data.mx, mpu_data.my, mpu_data.mz,
    //                            mpu_data.gx, mpu_data.gy, mpu_data.gz);
    // HAL_UART_Transmit(&TEST_HUART, (uint8_t *)test_buf, 10, 5);
    osDelayUntil(&imu_wake_time, IMU_TASK_PERIOD);
  }
}
