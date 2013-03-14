#ifndef SYSTEMCOMMANDS_H
#define SYSTEMCOMMANDS_H

#include "ch.h"

class SystemCommands {
 public:
  static void disablemotors(BaseSequentialStream *, int, char**);
  static void reset(BaseSequentialStream *, int, char**);
  static void status(BaseSequentialStream *chp, int, char**);
};
#endif
