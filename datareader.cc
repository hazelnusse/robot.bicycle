#include <iostream>
#include <vector>
#include "SampleReader.h"
#include "SampleConverted.h"

int main(int argc, char *argv[])
{
  if (argc != 2) {
    std::cout << "usage: " << argv[0] << " <filename>" << std::endl;
    return 0;
  }
  SampleReader sr(argv[1]);
  std::vector<SampleConverted> samples(sr.Convert());

  for (auto s : samples) {
    std::cout << s.Gyroscope[0] << ", "
              << s.Gyroscope[1] << ", "
              << s.Gyroscope[2] << ", "
              << s.Accelerometer[0] << ", "
              << s.Accelerometer[1] << ", "
              << s.Accelerometer[2] << std::endl;
  }
}
