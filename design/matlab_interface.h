#ifndef MATLAB_INTERFACE
#define MATLAB_INTERFACE
#include <string>
#include <Eigen/Dense>
#include "engine.h"

class MatlabInterface {
 public:
  MatlabInterface();
  ~MatlabInterface();
  int eval(const std::string & command);
  int put_Matrix(const Eigen::MatrixXd & m, const std::string & varname, bool display = false);
  Eigen::MatrixXd get_Matrix(const std::string & varname);

 private:
  Engine * ep_;
};

#endif // MATLAB_INTERFACE

