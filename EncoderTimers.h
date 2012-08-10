#ifndef ENCODERTIMERS_H
#define ENCODERTIMERS_H

class EncoderTimers {
 public:
  uint32_t Clockticks[3];  // clock ticks between most recent rising edges
  enum direction_t {cw, ccw};
  direction_t Direction[3];
};

#endif
