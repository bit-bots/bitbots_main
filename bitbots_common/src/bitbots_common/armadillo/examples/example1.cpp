#include <iostream>

#include "armadillo"

using namespace arma;
using namespace std;


int main(int argc, char** argv)
  {
  cout << "Armadillo version: " << arma_version::as_string() << endl;
  
  // directly specify the matrix size (elements are uninitialised)
  mat A(2,3);
  
  // .n_rows = number of rows    (read only)
  // .n_cols = number of columns (read only)
  cout << "A.n_rows = " << A.n_rows << endl;
  cout << "A.n_cols = " << A.n_cols << endl;
  
  // directly access an element (indexing starts at 0)
  A(1,2) = 456.0;
  
  A.print("A:");
  
  // scalars are treated as a 1x1 matrix,
  // hence the code below will set A to have a size of 1x1
  A = 5.0;
  A.print("A:");
  
  // if you want a matrix with all elements set to a particular value
  // the .fill() member function can be used
  A.set_size(3,3);
  A.fill(5.0);
  A.print("A:");
  
  
  mat B;
  
  // endr indicates "end of row"
  B << 0.555950 << 0.274690 << 0.540605 << 0.798938 << endr
    << 0.108929 << 0.830123 << 0.891726 << 0.895283 << endr
    << 0.948014 << 0.973234 << 0.216504 << 0.883152 << endr
    << 0.023787 << 0.675382 << 0.231751 << 0.450332 << endr;
  
  // print to the cout stream
  // with an optional string before the contents of the matrix
  B.print("B:");
  
  // the << operator can also be used to print the matrix
  // to an arbitrary stream (cout in this case) 
  cout << "B:" << endl << B << endl;
  
  // save to disk
  B.save("B.txt", raw_ascii);
  
  // load from disk
  mat C;
  C.load("B.txt");
  
  C += 2.0 * B;
  C.print("C:");
  
  
  // submatrix types:
  //
  // .submat(first_row, first_column, last_row, last_column)
  // .row(row_number)
  // .col(column_number)
  // .cols(first_column, last_column)
  // .rows(first_row, last_row)
  
  cout << "C.submat(0,0,3,1) =" << endl;
  cout << C.submat(0,0,3,1) << endl;
  
  // generate the identity matrix
  mat D = eye<mat>(4,4);
  
  D.submat(0,0,3,1) = C.cols(1,2);
  D.print("D:");
  
  // transpose
  cout << "trans(B) =" << endl;
  cout << trans(B) << endl;
  
  // maximum from each column (traverse along rows)
  cout << "max(B) =" << endl;
  cout << max(B) << endl;
  
  // maximum from each row (traverse along columns)
  cout << "max(B,1) =" << endl;
  cout << max(B,1) << endl;
  
  // maximum value in B
  cout << "max(max(B)) = " << max(max(B)) << endl;
  
  // sum of each column (traverse along rows)
  cout << "sum(B) =" << endl;
  cout << sum(B) << endl;
  
  // sum of each row (traverse along columns)
  cout << "sum(B,1) =" << endl;
  cout << sum(B,1) << endl;
  
  // sum of all elements
  cout << "sum(sum(B)) = " << sum(sum(B)) << endl;
  cout << "accu(B)     = " << accu(B) << endl;
  
  // trace = sum along diagonal
  cout << "trace(B)    = " << trace(B) << endl;
  
  // random matrix -- values are uniformly distributed in the [0,1] interval
  mat E = randu<mat>(4,4);
  E.print("E:");
  
  cout << endl;
  
  // row vectors are treated like a matrix with one row
  rowvec r;
  r << 0.59499 << 0.88807 << 0.88532 << 0.19968;
  r.print("r:");
  
  // column vectors are treated like a matrix with one column
  colvec q;
  q << 0.81114 << 0.06256 << 0.95989 << 0.73628;
  q.print("q:");
  
  // dot or inner product
  cout << "as_scalar(r*q) = " << as_scalar(r*q) << endl;
  
  
  // outer product
  cout << "q*r =" << endl;
  cout << q*r << endl;
  
  // multiply-and-accumulate operation
  // (no temporary matrices are created)
  cout << "accu(B % C) = " << accu(B % C) << endl;
  
  // sum of three matrices (no temporary matrices are created)
  mat F = B + C + D;
  F.print("F:");
  
  // imat specifies an integer matrix
  imat AA;
  imat BB;
  
  AA << 1 << 2 << 3 << endr << 4 << 5 << 6 << endr << 7 << 8 << 9;
  BB << 3 << 2 << 1 << endr << 6 << 5 << 4 << endr << 9 << 8 << 7;
  
  // comparison of matrices (element-wise)
  // output of a relational operator is a umat
  umat ZZ = (AA >= BB);
  ZZ.print("ZZ =");
  
  
  // 2D field of arbitrary length row vectors
  // (fields can also store abitrary objects, e.g. instances of std::string)
  field<rowvec> xyz(3,2);
  
  xyz(0,0) = randu(1,2);
  xyz(1,0) = randu(1,3);
  xyz(2,0) = randu(1,4);
  xyz(0,1) = randu(1,5);
  xyz(1,1) = randu(1,6);
  xyz(2,1) = randu(1,7);
  
  cout << "xyz:" << endl;
  cout << xyz << endl;
  
  
  // cubes ("3D matrices")
  cube Q( B.n_rows, B.n_cols, 2 );
  
  Q.slice(0) = B;
  Q.slice(1) = 2.0 * B;
  
  Q.print("Q:");
  
  
  return 0;
  }

