#include <fstream>
#include <vector>
#include <string>
#include "firmware_generator.h"
#include "firmware_template.h"
#include "control_design_functions.h"

void firmware_generator(const std::vector<model_data> & md, std::string filename_base)
{
  const int observer_state_size = 2;    // observer state is w
  const int observer_input_size = 4;    // observer inputs are: steer, roll rate, steer rate, steer torque
  const int observer_output_size = 1;   // estimate of roll angle
  const int plant_model_state_size = 4; // roll, steer, roll rate, steer rate

  std::string header_body{};
  header_body += "const uint32_t num_gains = "
              + std::to_string(md.size()) + ";\n";
  header_body += "const uint32_t observer_state_size = "
              + std::to_string(observer_state_size) + ";\n";
  header_body += "const uint32_t observer_input_size = "
              + std::to_string(observer_input_size) + ";\n";
  header_body += "const uint32_t observer_output_size = "
              + std::to_string(observer_output_size) + ";\n";
  header_body += "const uint32_t plant_model_state_size = "
              + std::to_string(plant_model_state_size) + ";\n";

  std::ofstream header_file(filename_base + ".h");
  header_file << firmware_template::preamble
              << header_body << firmware_template::postamble;
  header_file.close();

  std::ofstream source_file(filename_base + ".cpp");
  source_file << generate_source(md, filename_base);
  source_file.close();
}

std::string generate_source(const std::vector<model_data> & md, std::string filename_base)
{
  std::string source_body{};

  return source_body;
}

