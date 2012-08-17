#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <cstdint>
#include <cmath>

#include "Constants.h"
#include "Sample.h"

template <class T>
double norm(T a[3])
{
  double m = a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
  return std::sqrt(m);
}

int main(int argc, char *argv[])
{
  if (argc != 2) {
    std::cout << "usage: " << argv[0] << " <filename>" << std::endl;
    return 0;
  }
  std::ifstream datafile(argv[1], std::ios_base::in |
                                  std::ios_base::binary);

  std::vector<Sample> samples;
  char * membuff;
  std::ifstream::pos_type size;
  if (datafile.is_open()) {
    datafile.seekg(0, std::ios::end);
    size = datafile.tellg();
    membuff = new char[size];
    datafile.seekg(0, std::ios::beg);
    datafile.read(membuff, size);
    std::cout << "Read " << size << " bytes." << std::endl;
    datafile.close();

    if (size % sizeof(Sample)) {
      std::cerr << "Non-integer number of samples in datafile." << std::endl;
    } else {
      unsigned int N = size / sizeof(Sample);
      for (unsigned int i = 0; i < N; ++i) {
        Sample * s = reinterpret_cast<Sample *>(&membuff[i*sizeof(Sample)]);
        samples.push_back(*s);
      }

    }
    delete [] membuff;
  }

  for (std::vector<Sample>::const_iterator it = samples.begin();
       it != samples.end();
       ++it) {
    static const float sf = 2.0 * constants<float>::pi * 36e6 / 200.0;
    static const float current_max = 6.0;
    std::cout << it->systemTime << ", "
              << it->Speed_sp   << ", "
              << sf / (it->rearWheelRate & ~(1 << 31)) << ", "
              << it->CCR_rw / (0x3FFF + 1.0) << ", "
              << it->CCR_rw << std::endl;
  }
  std::cout << samples.size() << " Samples read." << std::endl;
}
