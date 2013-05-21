#ifndef SYSTEMCOMMANDS_H
#define SYSTEMCOMMANDS_H

#include "ch.h"

class SystemCommands {
 public:
  static void disable_controllers(BaseSequentialStream *, int, char**);
  static void reset(BaseSequentialStream *, int, char**);
};
#endif
