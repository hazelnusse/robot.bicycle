#ifndef SYSTEMCOMMANDS_H
#define SYSTEMCOMMANDS_H

#include "ch.h"

class SystemCommands {
 public:
  static void disablemotors(BaseSequentialStream *chp, int argc, char *argv[]);
  static void reset(BaseSequentialStream *chp, int argc, char *argv[]);
};
#endif
