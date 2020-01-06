#include "remote_ctrl.h"

#include "stdlib.h"
#include "string.h"
#include "cmsis_os.h"
#include "math.h"

#include "usart.h"
#include "sys_config.h"
#include "test_ctrl.h"
#include "chassis_task.h"
#include "imu_task.h"

uint8_t rc_buf[RC_BUFLEN];
rc_info_t rc_info;
rb_info_t rb_info;

static void get_bot_velocity(void);
static void get_bot_refangle(void);

/**
 * @brief   handle remote controller information
 * @param   remote control struct and remote uart(dbus) buffer
 * @usage   call after uart idle interrupt
 * @info    r_rocker_lr :
 *          r_rocker_ud :
 *          l_rocker_lr :
 *          l_rocker_ud :
 *          knob_v1     :
 *          knob_v2     :
 *          sa          :
 *          sb          :
 *          sc          :
 *          sd          :
 */
void rc_callback_handler(uint8_t *buf) {
  if (buf[0] != 0x0F) return;

  rc_info.r_rocker_lr = (buf[1] | buf[2] << 8) & 0x07ff;
  rc_info.r_rocker_lr -= ROCKER_OFFSET;

  rc_info.l_rocker_ud = (buf[2] >> 3 | buf[3] << 5) & 0x07ff;
  rc_info.l_rocker_ud -= ROCKER_OFFSET;

  rc_info.l_rocker_lr = (buf[5] >> 1 | buf[6] << 7) & 0x07ff;
  rc_info.l_rocker_lr -= ROCKER_OFFSET;

  rc_info.r_rocker_ud = (buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07ff;
  rc_info.r_rocker_ud -= ROCKER_MIN;

  if(rc_info.r_rocker_lr <= 5 && rc_info.r_rocker_lr >= -5) 
    rc_info.r_rocker_lr = 0;
  if(rc_info.l_rocker_lr <= 5 && rc_info.l_rocker_lr >= -5) 
    rc_info.l_rocker_lr = 0;
  if(rc_info.l_rocker_ud <= 5 && rc_info.l_rocker_ud >= -5) 
    rc_info.l_rocker_ud = 0;
  if(rc_info.r_rocker_ud <= 5) 
    rc_info.r_rocker_ud = 0;

  rc_info.knob_v1 = (buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07ff;
  rc_info.knob_v2 = (buf[9] >> 2 | buf[10] << 6) & 0x07ff;

  rc_info.sa = (buf[11] >> 6) & 0x03;
  rc_info.sb = (buf[7] >> 5) & 0x03;
  rc_info.sc = (buf[13] >> 1) & 0x03;
  rc_info.sd = (buf[14] >> 4) & 0x03;

  get_bot_velocity();
  if(rc_info.sd == SW_DOWN) {
    get_bot_refangle();
  }
  
  //! test code
  // sprintf(test_buf, "%04d %04d %04d %04d\r\n", rc_info.l_rocker_lr, rc_info.l_rocker_ud,
  //                                              rc_info.r_rocker_lr, rc_info.r_rocker_ud);
  // HAL_UART_Transmit(&TEST_HUART, test_buf, 22, 5);
}

float total_spd;
int16_t vx, vy, vw;

static void get_bot_velocity(void) {

    rb_info.power_ratio = (float)(rc_info.r_rocker_ud) / ROCKER_RANGE;

    vx = rc_info.l_rocker_ud;
    vy = rc_info.l_rocker_lr;
    vw = rc_info.r_rocker_lr;

    //! test code
    // memset(test_buf, 0, 20);
    // sprintf(test_buf, "v.%.2f %.2f %.2f\r\n", chassis.vx, chassis.vy, chassis.vw);
    // HAL_UART_Transmit(&TEST_HUART, test_buf, 20, 10);

    total_spd = sqrt(vx * vx + vy * vy + vw * vw);
    if(total_spd > 0.01) { 
      rb_info.vx = (float)vx / total_spd * ROBOT_LIN_SPD_MAX * rb_info.power_ratio;
      rb_info.vy = (float)vy / total_spd * ROBOT_LIN_SPD_MAX * rb_info.power_ratio;
      rb_info.vw = (float)vw / total_spd * ROBOT_ANG_SPD_MAX * rb_info.power_ratio;
    } else {
      rb_info.vx = 0;
      rb_info.vy = 0;
      rb_info.vw = 0;
    }
}

static void get_bot_refangle(void) {
  rb_info.ref_direction = attitude.yaw;
}

void filter_init(filter_t *filter, int num_datas) {
  int i = 0;
  filter->datas = malloc(sizeof(int16_t) * num_datas);
  for (i = 0; i < num_datas; ++i) {
    filter->datas[i] = 0;
  }
  filter->num_data = num_datas;
  filter->insert_index = 0;
  filter->data_received = 0;
  filter->sum = 0;
}

void filter_update(filter_t *filter, int16_t raw_data) {
    filter->sum += (raw_data - filter->datas[filter->insert_index]);
    
    if (filter->data_received < filter->num_data) {
      ++filter->data_received;
      filter->filtered_data = filter->sum / filter->data_received;
    } else {
      filter->filtered_data = filter->sum / filter->num_data;
    }
    filter->datas[filter->insert_index] = raw_data;
    filter->insert_index = (filter->insert_index + 1) % filter->num_data;
}

void rc_param_init() {
  memset(&rc_info, 0, sizeof(rc_info));
  rc_info.sa = SW_UP;
  rc_info.sb = SW_UP;
  rc_info.sc = SW_UP;
  rc_info.sd = SW_UP;
  rc_info.knob_v1 = KNOB_V1_MIN;
  rc_info.knob_v2 = KNOB_V2_MIN;
  rc_info.l_rocker_lr = 0;
  rc_info.l_rocker_ud = 0;
  rc_info.r_rocker_ud = ROCKER_MIN;
  rc_info.r_rocker_lr = ROCKER_OFFSET;

  memset(&rb_info, 0, sizeof(rb_info));
}
