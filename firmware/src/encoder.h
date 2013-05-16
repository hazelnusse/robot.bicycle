#ifndef ENCODER_H
#define ENCODER_H

#include <cstdint>
#include "hal.h"

class Encoder {
 public:

  Encoder(stm32_tim_t * timer, int counts_per_revolution);
  float get_angle() const;
  void set_count(uint32_t count);
  uint32_t get_count() const;

 private:
  stm32_tim_t * const timer_;
  const float rad_per_count_;
};

#include "encoder_priv.h"

#endif

