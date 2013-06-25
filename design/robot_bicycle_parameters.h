#ifndef ROBOT_BICYCLE_PARAMETERS_H
#define ROBOT_BICYCLE_PARAMETERS_H

#include "bicycle/bicycle.h"
#include "bicycle/wheelassemblygyrostat.h"

namespace bicycle {

extern WheelAssemblyGyrostat rear;
extern WheelAssemblyGyrostat front;
extern double ls;
extern double g;

Bicycle benchmark_bicycle();
Bicycle robot_bicycle();

}

#endif

