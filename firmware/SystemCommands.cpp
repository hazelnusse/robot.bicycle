#include <cstdint>
#include <cstring>
#include "SystemCommands.h"

#include "chprintf.h"

#include "SampleAndControl.h"
#include "RearWheel.h"
#include "YawRateController.h"

void SystemCommands::disablemotors(BaseSequentialStream *chp __attribute__((unused)), int argc __attribute__((unused)), char *argv[] __attribute__((unused)))
{
  RearWheel::Instance().turnOff();
  YawRateController::Instance().turnOff();
}

void SystemCommands::reset(BaseSequentialStream *chp __attribute__((unused)), int argc __attribute__((unused)), char *argv[] __attribute__((unused)))
{
  NVIC_SystemReset();
}

void SystemCommands::status(BaseSequentialStream *chp, int argc __attribute__((unused)), char *argv[] __attribute__((unused)))
{
  uint32_t state = SampleAndControl::Instance().systemState();
  float rw_sp = RearWheel::Instance().RateCommanded(),
        yr_sp = YawRateController::Instance().RateCommanded();


  chprintf(chp, "%u,%f,%f,%s\r\n", state, rw_sp, yr_sp, SampleAndControl::Instance().fileName());
}
