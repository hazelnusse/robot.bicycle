#ifndef ENCODER_INL_H
#define ENCODER_INL_H

#include "bitband.h"

namespace hardware {

inline
float Encoder::get_angle() const
{
  return static_cast<int16_t>(timer_->CNT) * rad_per_count_;
}

inline
uint32_t Encoder::get_count() const
{
  return timer_->CNT;
}

inline
void Encoder::set_count(uint32_t count)
{
  timer_->CNT = count;
}

inline
float Encoder::get_rad_per_count() const
{
  return rad_per_count_;
}

inline
bool Encoder::rotation_direction() const
{
  return MEM_ADDR(BITBAND(reinterpret_cast<uint32_t>(&(timer_->SR)), (1 << 4)));
}

}
#endif

