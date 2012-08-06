#include "BlinkThread.h"
#include "hal.h"

/*
 * Red LEDs blinker thread, times are in milliseconds.
 */
msg_t BlinkThread(void * arg)
{
  bool & fs_ready = *(static_cast<bool *>(arg));
  chRegSetThreadName("Blink");
  while (1) {
    palTogglePad(IOPORT3, GPIOC_LED_STATUS1);
    chThdSleepMilliseconds(fs_ready ? 125 : 1000);
  }
  return 0;
}
