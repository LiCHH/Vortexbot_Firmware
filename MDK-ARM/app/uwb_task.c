#include "uwb_task.h"

#include "cmsis_os.h"

#include "uwb_info.h"


void uwb_task(void const *argu)
{
  osEvent event;
  while(1) {
    event = osSignalWait(UWB_UPDATE_SIGNAL, osWaitForever);
    if(event.status == osEventSignal) {
      if(event.value.signals & UWB_UPDATE_SIGNAL) {
        //! TODO: Fill this

      }
    }
  }
}
