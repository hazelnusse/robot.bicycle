#include <iostream>
#include "SampleReader.h"

int main(int argc, char *argv[])
{
  if (argc != 2) {
    std::cout << "usage: " << argv[0] << " <filename>" << std::endl;
    return 0;
  }
  SampleReader sr(argv[1]);

}
