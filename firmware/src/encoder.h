#ifndef ENCODER_H
#define ENCODER_H

#include <cstdint>
#include "ch.h"
#include "hal.h"
#include "constants.h"

namespace hardware {

class Encoder {
 public:
  Encoder(stm32_tim_t * timer, uint16_t counts_per_revolution) :
    timer_(timer), rad_per_count_(constants::two_pi / counts_per_revolution) {}

  float get_angle() const;

  uint32_t get_count() const;
  void set_count(uint32_t count);

  float get_rad_per_count() const;

  bool rotation_direction() const;

 private:
  stm32_tim_t * const timer_;
  const float rad_per_count_;
};

} // namespace hardware

#include "encoder-inl.h"

#endif

