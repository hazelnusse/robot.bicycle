#include <cstdint>
#include <cstring>
#include "SystemCommands.h"

#include "chprintf.h"

#include "SampleAndControl.h"
#include "RearWheel.h"
#include "YawRateController.h"

void SystemCommands::disablemotors(BaseSequentialStream*, int, char**)
{
  RearWheel::Instance().turnOff();
  YawRateController::Instance().turnOff();
}

void SystemCommands::reset(BaseSequentialStream*, int, char**)
{
  NVIC_SystemReset();
}

void SystemCommands::status(BaseSequentialStream *chp, int, char**)
{
  uint32_t state = SampleAndControl::Instance().systemState();
  float rw_sp = RearWheel::Instance().RateCommanded(),
        yr_sp = YawRateController::Instance().RateCommanded();


  chprintf(chp, "%u,%f,%f\r\n", state, rw_sp, yr_sp);
}
