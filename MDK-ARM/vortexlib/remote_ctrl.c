#include "remote_ctrl.h"

#include "stdlib.h"
#include "string.h"

#include "usart.h"
#include "sys_config.h"
#include "test_ctrl.h"

uint8_t rc_buf[RC_BUFLEN];
rc_info_t rc_info;

filter_t rr_ud_f, lr_lr_f, lr_ud_f;

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
void rc_callback_handler(rc_info_t *rc, uint8_t *buf) {
  if (buf[0] != 0x0F) return;

  int16_t rr_ud_raw, lr_lr_raw, lr_ud_raw;

  rc->r_rocker_lr = (buf[1] | buf[2] << 8) & 0x07ff;
  rc->r_rocker_lr -= ROCKER_OFFSET;

  lr_ud_raw = (buf[2] >> 3 | buf[3] << 5) & 0x07ff;
  lr_ud_raw -= ROCKER_OFFSET + 193;
  filter_update(&lr_ud_f, lr_ud_raw);
  rc->l_rocker_ud = lr_ud_f.filtered_data;

  rr_ud_raw = (buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07ff;
  filter_update(&rr_ud_f, rr_ud_raw);
  rc->r_rocker_ud = rr_ud_f.filtered_data;
  // rc->r_rocker_ud -= ROCKER_OFFSET;

  lr_lr_raw = (buf[5] >> 1 | buf[6] << 7) & 0x07ff;
  lr_lr_raw -= ROCKER_OFFSET - 41;
  filter_update(&lr_lr_f, lr_lr_raw);
  rc->l_rocker_lr = lr_lr_f.filtered_data;

  rc->knob_v1 = (buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07ff;
  rc->knob_v2 = (buf[9] >> 2 | buf[10] << 6) & 0x07ff;

  rc->sa = (buf[11] >> 6) & 0x03;
  rc->sb = (buf[7] >> 5) & 0x03;
  rc->sc = (buf[13] >> 1) & 0x03;
  rc->sd = (buf[15] >> 4) & 0x03;

  // test code
  // static int count = 0;
  // if (count == 10) {
  //   sprintf((char *)test_buf, "ud raw:%d fil:%d lr raw:%d fil:%d\r\n",
  //           rr_ud_raw, rr_ud_f.filtered_data, lr_lr_raw, lr_lr_f.filtered_data);
  //   HAL_UART_Transmit(&TEST_HUART, test_buf, 50, 10);
  //   count = 0;
  // }
  // count++;
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
  filter_init(&rr_ud_f, FILTER_SIZE);
  filter_init(&lr_ud_f, FILTER_SIZE);
  filter_init(&lr_lr_f, FILTER_SIZE);
}
