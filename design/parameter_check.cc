#include "bicycle/bicycle.h"
#include "robot_bicycle_parameters.h"

int main(int argc, char ** argv)
{
  bicycle::Bicycle rb = bicycle::robot_bicycle();
  std::cout << "Robot Bicycle:" << std::endl;
  std::cout << rb << std::endl << std::endl;
  bicycle::Bicycle bb = bicycle::benchmark_bicycle();
  std::cout << "Benchmark Bicycle:" << std::endl;
  std::cout << bb << std::endl;
}


