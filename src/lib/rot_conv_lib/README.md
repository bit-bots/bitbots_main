# Rotations Conversion Library

**Author:** Philipp Allgeuer

**Version:** 1.2.0

**Date:** 22/01/18

## General Overview

The Rotations Conversion Library (RCL) is a collection of functions that address common computations and numerical handling of rotations in 3D Euclidean space. The rotation representations that are supported are rotation matrices (`Rotmat`), Quaternions (`Quat`), intrinsic ZYX Euler angles (`Euler`), fused angles (`Fused`) and tilt angles (`Tilt`). In addition to the core competency of being able to convert between each of the representations, operations such as inversion, ZYX yaw extraction, fused yaw extraction, renormalisation, equality detection, vector rotation and much more have been implemented. Tilt vector addition and the relative and absolute tilt phase spaces are also supported. Unit tests for each function are provided to ensure that the library performs exactly as it should. The implementation of the Rotations Conversion Library is based on the very related Matlab/Octave Rotations Library (MORL), which is a highly tested reference implementation for the required conversion algorithms (see https://github.com/AIS-Bonn/matlab_octave_rotations_lib).

Please note that the ***fused angles*** and ***tilt angles*** rotation representations are the invention of the author, and have been specifically designed to overcome the many limitations and peculiarities of the Euler angles representation. More information can be found in the IROS paper *"Fused Angles: A Representation of Body Orientation for Balance"* (at time of writing available online at http://www.ais.uni-bonn.de/~pallgeuer/papers/IROS_2015_fused.pdf). The tilt phase space is also an invention of the author.

## Getting Started

This library is implemented as a collection of platform-independent C++ source files. To get started just clone the `rot_conv_lib` repository to the desired location on your computer.

There are three ways of using the library:

1. Directly include the source files in your project, and build them with the rest of your project.

2. Build a static library (e.g. `*.a` or `*.lib`) of the source code and link your project to it.

3. Build a dynamic library (e.g. `*.so` or `*.dll`) of the source code and link your project to it.

Due to the small and efficient nature of the library, one of the first two options is recommended. Very minimal benefit is expected from building a dynamic library.

## Compiling the Code

If you want to compile the unit tests as well (do this first):

~~~
git clone https://github.com/google/googletest googletest
~~~

Compile and run the library and sample code:

~~~
mkdir build
cd build
cmake ..
make
./rot_conv_sample_direct
~~~

To run the unit tests:

~~~
./test_rot_conv
~~~

To restore the initial clean state:

~~~
rm -rf build googletest
~~~

## Notes on Numerical Stability

Although as much as possible has been done to try to avoid the problems associated with numerical stability and accuracy, the extent to which this is possible is limited by the presence of singularities in the rotation representations, floating point errors, and the use of (unavoidable and required) functions of high numerical sensitivity. For example:

~~~
Code:
double eps = std::numeric_limits<double>::epsilon();
std::cout << "Epsilon is " << eps << std::endl;
std::cout << "Expect acos(1) = 0, but with an epsilon of error it is " << acos(1.0 - eps) << std::endl;
std::cout << "Expect sqrt(0) = 0, but with an epsilon of error it is " << sqrt(0.0 + eps) << std::endl;

Output:
Epsilon is 2.22045e-16
Expect acos(1) = 0, but with an epsilon of error it is 2.10734e-08
Expect sqrt(0) = 0, but with an epsilon of error it is 1.49012e-08
~~~

This means that very tiny floating point errors in required expressions such as

~~~
double alpha = acos(2.0*(q.w()*q.w() + q.z()*q.z()) - 1.0)
double calpha = sqrt(1.0 - (sth*sth + sphi*sphi))
~~~

can quickly turn into comparatively larger errors in the output variables, for certain *highly specific* input scenarios.

## Quick Help

Every function of the library is well-commented. A summary of the five rotation representations is shown in the following table.

**Representation** | **Code** | **Format** | **Universal set**
--- |:---:| --- | ---
ZYX Euler angles | `Euler` |  (&psi;, &theta;, &phi;) =<br>`(yaw, pitch, roll)` | (-&pi;,&pi;] &times; \[-&pi;/2,&pi;/2\] &times; (-&pi;,&pi;]
Fused angles | `Fused` | (&psi;, &theta;, &phi;, h) =<br>`(fused yaw, fused pitch, fused roll, hemi)` | (-&pi;,&pi;] &times; \[-&pi;/2,&pi;/2\] &times; \[-&pi;/2,&pi;/2\] &times; {-1,1}
Quaternion | `Quat` | `(w, x, y, z)` | &#x211a;
Rotation matrix | `Rotmat` | `3x3 matrix` | SO(3)
Tilt angles | `Tilt` | (&psi;, &gamma;, &alpha;) =<br>`(fused yaw, tilt axis angle, tilt angle)` | (-&pi;,&pi;] &times; (-&pi;,&pi;] &times; \[0,&pi;\]

The ***fused angles*** and ***tilt angles*** rotation representations are the invention of the author (see *General Overview*).

## Where To Get More Help?

If a look into the source code does not resolve an issue you have with the library, then you can contact the author at the email address given in the *Bugs and Improvements* section.

## Bugs and Improvements

I welcome all feedback, suggestions and bug reports. If you improve or fix anything about the library then I encourage you to let me know so that the library can be improved for everyone!

**Email:** `pallgeuer[at]ais.uni-bonn.de`
