#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "ch.h"

namespace calibration {

void fork_encoder_calibration(BaseSequentialStream * chp, int, char **);
void fork_encoder_home(BaseSequentialStream * chp, int, char **);

}

#endif

