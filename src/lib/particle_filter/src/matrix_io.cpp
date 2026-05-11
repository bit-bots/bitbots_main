// #########################################################//
// #                                                       #//
// # gaussian_mixture_models  matrix_io.cpp                #//
// # Roberto Capobianco  <capobianco@dis.uniroma1.it>      #//
// #                                                       #//
// #########################################################//

#include <fstream>
#include <iostream>
#include <particle_filter/matrix_io.hpp>
#include <stdexcept>
#include <string>

void MatrixIO::readFromFile(const std::string& filename, Eigen::MatrixXd& matrix) {
  std::ifstream input_file;
  std::string line;
  int rows = 0;
  int cols = 0;

  input_file.open(filename);

  if (input_file.fail()) {
    throw std::runtime_error("Impossible to open the file");
  }

  int row_idx = 0;

  while (!input_file.fail() && !input_file.eof()) {
    std::getline(input_file, line);
    std::istringstream iss(line);
    std::string buffer;

    iss >> buffer;
    iss.clear();
    iss.seekg(0, std::ios::beg);

    if (buffer.empty() || (buffer[0] == '#' && buffer != "#size:")) {
      continue;
    }

    if (buffer == "#size:" && rows == 0 && cols == 0) {
      std::string str;
      iss >> str;
      iss >> rows;
      iss >> cols;
      matrix = Eigen::MatrixXd::Zero(rows, cols);
      row_idx = 0;
    } else {
      double value;

      for (int i = 0; i < cols && !iss.eof(); ++i) {
        iss >> value;

        matrix(row_idx, i) = value;
      }

      ++row_idx;
    }
  }

  input_file.close();
}

void MatrixIO::writeToFile(const std::string& filename, const Eigen::MatrixXd& matrix) {
  std::ofstream output_file;

  output_file.open(filename);

  if (output_file.fail()) {
    throw std::runtime_error("Impossible to open the file");
  }

  output_file << "#file automatically generated" << std::endl;
  output_file << "#size: " << matrix.rows() << " " << matrix.cols() << std::endl;

  output_file << matrix << std::endl;

  output_file.close();
}
