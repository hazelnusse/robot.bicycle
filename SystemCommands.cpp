#include "SystemCommands.h"
#include "RearWheel.h"
#include "YawRateController.h"

void SystemCommands::disablemotors(BaseSequentialStream *chp, int argc, char *argv[])
{
  RearWheel::Instance().turnOff();
  YawRateController::Instance().turnOff();
}

void SystemCommands::reset(BaseSequentialStream *chp, int argc, char *argv[])
{
  NVIC_SystemReset();
}
