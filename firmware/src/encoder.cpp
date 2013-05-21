#include "bitband.h"
#include "constants.h"
#include "encoder.h"

namespace hardware {

Encoder::Encoder(stm32_tim_t * timer, uint16_t counts_per_revolution)
  : timer_(timer),
    rad_per_count_(constants::two_pi / counts_per_revolution)
{
}

float Encoder::get_angle() const
{
  return static_cast<int16_t>(timer_->CNT) * rad_per_count_;
}

void Encoder::set_count(uint32_t count)
{
  timer_->CNT = count;
}

uint32_t Encoder::get_count() const
{
  return timer_->CNT;
}

} // namespace hardware

