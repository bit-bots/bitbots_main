#ifndef LEGIK_HPP
#define LEGIK_HPP

/*****************************************************************************/
#include <vector>
#include <string>
#include <cmath>

#define IKDEBUG(command)  
#define IKMSG(command) command

namespace LegIK {

/*****************************************************************************/

class Vector3D : public std::vector<double> {
 public:
  Vector3D();
  Vector3D(double x1, double x2, double x3);
  Vector3D(const Vector3D & other);
  double length();
  void normalize();

  friend Vector3D operator*(double x, const Vector3D & v);
  friend Vector3D operator+(const Vector3D & v1, const Vector3D & v2);
  friend Vector3D operator-(const Vector3D & v1, const Vector3D & v2);
  friend double scalar_prod(const Vector3D & v1, const Vector3D & v2);
  friend Vector3D vect_prod(const Vector3D & v1, const Vector3D & v2);

  std::string pp() const; 
};

/*****************************************************************************/

class Frame3D : public std::vector<Vector3D> {
 public:
  /* canonical frame */
  Frame3D();
  Frame3D(const Frame3D & other);
  /* Ordre habituel des angles d'euler : precession, nutation et rotation propre */
  static Frame3D from_euler(double euler_psi, double euler_theta, double euler_phi);
  static Frame3D from_vectors(Vector3D e1, Vector3D e2, Vector3D e3);
  std::string pp() const;
};

/*****************************************************************************/

class Position {
 public:
  double theta[6];
  Position();
  Position(double theta0, double theta1, double theta2, 
	   double theta3, double theta4, double theta5);
  
};

/*****************************************************************************/

class IK {
  double L[3];
 public:
  IK(double L0, double L1, double L2);
  bool compute(Vector3D C, Frame3D orientation, Position & result);
};

/*****************************************************************************/

class IKTest {
 public:
  static bool test(double L0, double L1, double L2, 
		   Vector3D C, 
		   double euler_psi, double euler_theta, double euler_phi, 
		   Position result,
		   double epsilon);

  static bool test_suite();
};

/*****************************************************************************/
/*****************************************************************************/

}

#endif

