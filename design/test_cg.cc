#include <cassert>
#include <iostream>
#include "cgains.h"

int main(int argc, char** argv) {
  float theta_R_dot = -0.95f;
  const cg::ControllerGains * ar[2];
  if (cg::lu_bounds(theta_R_dot, ar)) {
    std::cout << "Velocity in range" << std::endl;
    assert(ar[0] == &cg::gains[cg::num_gains - 2]);
    assert(ar[1] == &cg::gains[cg::num_gains - 1]);
  } else {
    std::cout << "Velocity outside of range" << std::endl;
  }
}

