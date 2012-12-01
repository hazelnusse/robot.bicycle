#include <cmath>
#include "textutilities.h"

float tofloat(const char * s)
{
  float sp = ((s[1] - '0')*10 +
              (s[2] - '0')) +
             ((s[4] - '0')*0.1f +
              (s[5] - '0')*0.01f);
  return (s[0] == '-') ? -sp : sp;
}
