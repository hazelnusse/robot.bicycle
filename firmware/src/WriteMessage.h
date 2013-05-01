#ifndef WRITEMESSAGE_H
#define WRITEMESSAGE_H

#include <cstdint>
#include "ch.h"

union WriteMessage {
  msg_t message;
  struct {
    uint16_t bytes_to_write;
    uint16_t buffer_offset;
  } m;
};

#endif

