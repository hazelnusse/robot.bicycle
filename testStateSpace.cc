#include <iostream>

#include "DiscreteStateSpace.h"

int main()
{

  static DiscreteStateSpace<float, 1, 1, 2> PID;
  PID.A << 1.0f, 0.0f, 0.0f, 0.867113741721284f;
  PID.B << 0.031250000000000f, 0.295779403066755f;
  PID.C << 0.025234411263784f, 0.295779403066755f;
  PID.D << 0.238135200943393f;
  PID.u << 1.0f;

  for(int i = 0; i < 10; ++i) {
    std::cout << PID.GetOutput() << std::endl;
    PID.Iterate();
  }
}
