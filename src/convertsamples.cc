#include <iostream>
#include <vector>

#include "SampleReader.h"
#include "SampleConverted.h"

int main(int argc, char *argv[])
{
  if (argc != 3) {
    std::cout << "usage: " << argv[0] << " <input data filename> <output data filename>" << std::endl;
    return 0;
  }
  SampleReader sr(argv[1]);
  std::vector<SampleConverted> samples(sr.Convert());
  std::cout << samples.size() << " samples converted @ "
                              << sizeof(SampleConverted)
                              << " bytes per sample " << std::endl;

  std::ofstream outfile(argv[2], std::ios_base::out | std::ios_base::binary);
  for (unsigned int i = 0; i < samples.size(); ++i) {
    outfile.write(reinterpret_cast<char *>(&samples[i]), sizeof(SampleConverted));
  }
  outfile.close();
}

