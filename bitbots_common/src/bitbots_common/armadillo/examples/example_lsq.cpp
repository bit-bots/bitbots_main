// Tutorial for linear least square fitting
// Author: Pierre Moulon
// Date: 8 December 2009
// Objective:
// Fit a 2D line with to a set of points
// Specifically, find a and b in the model y = ax + b
// 
// Direct application of the example in:
// http://en.wikipedia.org/wiki/Linear_least_squares#Motivational_example


#include <iostream>

#include "armadillo"

using namespace arma;
using namespace std;



int main(int argc, char** argv)
  {
  // points to which we will fit the line
  mat data = "1 6; 2 5; 3 7; 4 10";

  cout << "Points used for the estimation:" << endl;
  cout << data << endl;

  // Build matrices to solve Ax = b problem:
  vec b(data.n_rows);
  mat C(data.n_rows, 2);

  for(u32 i=0; i<data.n_rows; ++i)
    {
    b(i)   = data(i,1);
    
    C(i,0) = 1;
    C(i,1) = data(i,0);
    }
  
  cout << "b:" << endl;
  cout << b << endl;
  
  cout << "Constraint matrix:" << endl;
  cout << C << endl;
  
  // Compute least-squares solution:
  vec solution = solve(C,b);
  
  // solution should be "3.5; 1.4"
  cout << "solution:" << endl;
  cout << solution << endl;
  

  cout << "Reprojection error:" << endl;
  
  for(u32 i=0; i<data.n_rows; ++i)
    {
    cout << "  residual: " << ( data(i,1) - (solution(0) + solution(1) * data(i,0)) ) << endl;
    }
    
  return 0;
  }

