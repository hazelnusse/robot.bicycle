#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <cstdint>
#include <cmath>

#include "sample.h"

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

  std::cout << "  X     Y    Z" << std::endl;
  for (std::vector<Sample>::const_iterator it = samples.begin();
       it != samples.end();
       ++it) {
    //int16_t ar[3];
//    std::cout << it->mag[0] / 256.0 * 9.81 << "   "
//              << it->mag[1] / 256.0 * 9.81 << "   "
//              << it->mag[2] / 256.0 * 9.81 << std::endl;
    std::cout << it->steerAngle << "   "
              << it->steerRate << "   "
              << it->rearWheelRate << "   "
              << it->frontWheelRate << "    "
              << it->systemTime << std::endl;

//    ar[0] = it->gyro[3];
//    ar[1] = 0; // it->acc[1];
//    ar[2] = 0; // it->acc[2];
//    std::cout << norm(ar) / 256.0 * 9.81  << std::endl;
  }
  std::cout << samples.size() << " Samples read." << std::endl;
}
