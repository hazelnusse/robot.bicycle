#include <cassert>
#include <iostream>
#include "cgains.h"

int main(int argc, char * argv[])
{
  std::array<float, cg::num_gains + 2> rates;
  rates[0] = cg::gains[0].theta_R_dot - 0.1;
  rates[cg::num_gains + 1] = cg::gains[cg::num_gains - 1].theta_R_dot + 0.1;
  for (unsigned int i = 1; i < cg::num_gains + 1; ++i)
    rates[i] = cg::gains[i - 1].theta_R_dot + 0.01;

  const cg::ControllerGains * ar[2];
  // Test end cases
  assert(!cg::lu_bounds(rates[0], ar));
  assert(!cg::lu_bounds(rates[cg::num_gains + 1], ar));
  // Test intermediate cases
  for (unsigned int i = 1; i < cg::num_gains; ++i) {
    assert(cg::lu_bounds(rates[i], ar));
    assert(ar[0] == &cg::gains[i - 1]);
    assert(ar[1] == &cg::gains[i]);
  }
}

