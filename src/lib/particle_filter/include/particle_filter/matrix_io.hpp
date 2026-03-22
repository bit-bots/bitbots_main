// #########################################################//
// #                                                       #//
// # gaussian_mixture_models  matrix_io.h                  #//
// # Roberto Capobianco  <capobianco@dis.uniroma1.it>      #//
// #                                                       #//
// #########################################################//

#ifndef _MATRIX_IO_HPP_
#define _MATRIX_IO_HPP_

#include <Eigen/Core>
#include <string>

class MatrixIO {
 public:
  MatrixIO() {}
  ~MatrixIO() {}

  void readFromFile(const std::string& filename, Eigen::MatrixXd& matrix);
  void writeToFile(const std::string& filename, const Eigen::MatrixXd& matrix);
};

#endif  // _MATRIX_IO_HPP_
