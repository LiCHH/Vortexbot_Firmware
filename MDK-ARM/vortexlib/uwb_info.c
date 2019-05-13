#include "uwb_info.h"

#include "cmsis_os.h"

extern TaskHandle_t uwb_task_t;

uint8_t uwb_buff[UWB_BUF_LEN];

uwb_data_t uwb_data;

void uwb_receive_callback(uwb_data_t* uwb_data, uint8_t *buf)
{
  //! TODO: Fill this
  uwb_data->raw_x = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];
  uwb_data->raw_y = buf[4] << 24 | buf[5] << 16 | buf[6] << 8 | buf[7];
  osSignalSet(uwb_task_t, UWB_UPDATE_SIGNAL);
}
