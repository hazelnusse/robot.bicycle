#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include "firmware_generator.h"
#include "firmware_template.h"
#include "control_design_functions.h"

void firmware_generator(const std::vector<model_data> & md)
{
  const int observer_state_size = 4;    // observer is a full state estimator
  const int observer_input_size = 3;    // observer inputs are: steer, roll rate,
                                        //                      steer torque
  //const int observer_output_size = 1;   // estimate of roll angle
  const int plant_model_state_size = 4; // roll, steer, roll rate, steer rate
  const int plant_model_input_size = 1; // torque

  std::string header_body{};
  header_body += "const uint32_t num_gains = "
              + std::to_string(md.size()) + ";\n";
  header_body += "const uint32_t observer_state_size = "
              + std::to_string(observer_state_size) + ";\n";
  header_body += "const uint32_t observer_input_size = "
              + std::to_string(observer_input_size) + ";\n";
//  header_body += "const uint32_t observer_output_size = "
//              + std::to_string(observer_output_size) + ";\n";
  header_body += "const uint32_t plant_model_state_size = "
              + std::to_string(plant_model_state_size) + ";\n";
  header_body += "const uint32_t plant_model_input_size = "
              + std::to_string(plant_model_input_size) + ";\n";

  std::ofstream header_file("gain_schedule.h");
  header_file << firmware_template::preamble
              << header_body << firmware_template::postamble;
  header_file.close();

  std::ofstream source_file("fork_schedule.cpp");
  source_file << generate_source(md, "gain_schedule");
  source_file.close();
}

std::string generate_source(const std::vector<model_data> & md, std::string filename_base)
{
  //std::string source_body{};
  Eigen::IOFormat printfmt(Eigen::FullPrecision, 0, "f, ", "f, ", "", "", "{{", "f}}");
  std::ostringstream out;
  out << "#include \"" << filename_base << ".h\"\n\n";
  out << "namespace control {\n\n";
  out << "const std::array<rt_controller_t, " << md.size();
  out << "> GainSchedule::schedule_ {{\n";

  for(auto it = md.begin(); it != md.end(); ++it) {
    out << "\t{\n";
    out << "\t\t" << it->theta_R_dot <<"f, // theta_R_dot\n";
    out << "\t\t{\n"; // start writing out controllers

    out << "\t\t\t{ // StateEstimator\n";
    out << "\t\t\t\t" << it->A_kalman_d.format(printfmt) << ",\n";
    out << "\t\t\t\t" << it->B_kalman_d.format(printfmt) << "\n";
    out << "\t\t\t},\n"; // StateEstimator End

    out << "\t\t\t{ // LQRController\n"; // LQRController Start
    out << "\t\t\t\t" << it->K_lqr_d.format(printfmt) << "\n";
    out << "\t\t\t},\n"; // LQRController End

    out << "\t\t\t{ // PIController\n"; // PIController Start
    out << "\t\t\t\t0, 0\n";
    out << "\t\t\t}\n"; // PIController End

    out << "\t\t}\n"; // controllers end
    if (it + 1 == md.end()) {
      out << "\t}\n";
    } else {
      out << "\t},\n";
    }
  }

  out << "}};\n\n} // namespace control";

  return out.str();
}

