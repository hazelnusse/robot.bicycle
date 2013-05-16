#ifndef ENCODER_PRIV_H
#define ENCODER_PRIV_H

#include "bitband.h"
#include "constants.h"

Encoder::Encoder(stm32_tim_t * timer, int counts_per_revolution)
  : timer_(timer),
    rad_per_count_(constants::two_pi / counts_per_revolution)
{
}

inline float Encoder::get_angle() const
{
  return static_cast<int16_t>(timer_->CNT) * rad_per_count_;
}

inline void Encoder::set_count(uint32_t count)
{
  timer_->CNT = count;
}

inline uint32_t Encoder::get_count() const
{
  return timer_->CNT;
}

#endif

