#include <algorithm>
#include <iostream>
#include <array>
#include "matlab_interface.h"

constexpr int buffer_size = 8192;

MatlabInterface::MatlabInterface()
  : ep_{engOpen("matlab -nosplash")}
{
  if (!ep_)
    std::cerr << "Can't start Matlab engine.\n";
}

MatlabInterface::~MatlabInterface()
{
  engClose(ep_);
}

int MatlabInterface::eval(const std::string & command)
{
  std::array<char, buffer_size> buf{};
  engOutputBuffer(ep_, buf.data(), buf.size());
  int result = engEvalString(ep_, command.c_str());
  if (result)
    std::cerr << "Session is no longer running or engine pointer is invalid." << std::endl;

  std::cout << buf.data();
  return result;
}


int MatlabInterface::put_Matrix(const Eigen::MatrixXd & m, const std::string & varname, bool display)
{
  mxArray * ar_ptr = mxCreateDoubleMatrix(m.rows(), m.cols(), mxREAL);
  std::copy(m.data(), m.data() + m.size(), mxGetPr(ar_ptr));
  int result = engPutVariable(ep_, varname.c_str(), ar_ptr);
  if (display) {
    std::string buf;
    buf.resize(buffer_size);
    engOutputBuffer(ep_, const_cast<char *>(buf.data()), buf.size());
    eval(varname.c_str());
    std::cout << buf;
  }
  return result;
}

Eigen::MatrixXd MatlabInterface::get_Matrix(const std::string & varname)
{
  mxArray * ar_ptr = engGetVariable(ep_, varname.c_str());
  if (ar_ptr == nullptr)
    return {};

  Eigen::MatrixXd result(mxGetM(ar_ptr), mxGetN(ar_ptr));
  std::copy(static_cast<double *>(mxGetData(ar_ptr)),
            static_cast<double *>(mxGetData(ar_ptr)) + result.size(), result.data());
  mxDestroyArray(ar_ptr);
  return result;
}
