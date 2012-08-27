#ifndef SAMPLEREADER_H
#define SAMPLEREADER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "Sample.h"
#include "SampleConverted.h"

class SampleReader {
 public:
  SampleReader(const char *);
  std::vector<SampleConverted> Convert();

 private:
  std::vector<Sample> samples_;
  std::vector<SampleConverted> samplesConverted_;
  bool converted_;
};

#endif
