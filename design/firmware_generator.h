#ifndef FIRMWARE_GENERATOR_H
#define FIRMWARE_GENERATOR_H
#include <vector>
#include <string>
#include "control_design_functions.h"

void firmware_generator(const std::vector<model_data> & md);
std::string generate_source(const std::vector<model_data> & md, std::string filename_base);

#endif // FIRMWARE_GENERATOR_H

