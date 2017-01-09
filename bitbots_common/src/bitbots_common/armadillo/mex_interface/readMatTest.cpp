// Demonstration of how to connect Armadillo with Matlab mex functions.
// Version 0.5
// 
// Copyright (C) 2014 George Yammine
// Copyright (C) 2014 Conrad Sanderson
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include "armaMex.hpp"


void
mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
  {
  // Read the matrix from the file inData.mat
  mat fromFile = armaReadMatFromFile("inData.mat");
  
  fromFile.print();
  
  mat tmp(4,6);
  tmp.randu();
  
  // Write the matrix tmp as outData in the file outData.mat
  armaWriteMatToFile("outData.mat", tmp, "outData");
  }
