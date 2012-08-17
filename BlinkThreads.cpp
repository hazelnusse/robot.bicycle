#include "ch.h"
#include "hal.h"

#include "BlinkThreads.h"
#include "SampleAndControl.h"

//  Thread to blink green LED when filesystem is ready
__attribute__((noreturn))
void FileSystemBlinkThread(void * arg)
{
  chRegSetThreadName("fs_blink");
  while (1) {
    palTogglePad(IOPORT3, GPIOC_LED_STATUS1);
    chThdSleepMilliseconds(*static_cast<bool *>(arg) ? 125 : 1000);
  }
}

// Thread to blink Orange LED when Sample and Control thread is active.
__attribute__((noreturn))
void SampleAndControlBlinkThread(__attribute__((unused)) void * arg)
{
  SampleAndControl & sc = SampleAndControl::Instance();
  chRegSetThreadName("sc_blink");
  while (1) {
    palTogglePad(IOPORT3, GPIOC_LED_STATUS2);
    chThdSleepMilliseconds(sc.Enabled() ? 125 : 1000);
  }
}
