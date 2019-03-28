#include "uwb_info.h"

#include "cmsis_os.h"

extern TaskHandle_t uwb_task_t;

uint8_t uwb_buff[UWB_BUF_LEN];

uwb_data_t uwb_data;

void uwb_receive_callback(uwb_data_t* uwb_data, uint8_t *buf)
{
  //! TODO: Fill this

  osSignalSet(uwb_task_t, UWB_UPDATE_SIGNAL);
}
