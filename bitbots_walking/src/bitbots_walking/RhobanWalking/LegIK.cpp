/*****************************************************************************/
#include "LegIK.hpp"
#include <cmath>
#include <vector>
#include <iostream>
#include <sstream>

using namespace std;

#define DEG2RAD(x) ((x) * M_PI / 180.0)

namespace LegIK {

/*****************************************************************************/
#define ik_global_epsilon 0.0000001

namespace IKTools {

  inline double val_abs(double x) {
    return (x >= 0) ? x : -x;
  }

  inline bool is_zero(double x) {
    return val_abs(x) < ik_global_epsilon;
  }

  inline void bound(double min, double max, double & x) {
    if (x < min) x = min;
    if (x > max) x = max;
  }

  inline int sign(double x) { 
    return (x >= 0) ? 1 : -1; 
  }
}

using namespace IKTools;

/*****************************************************************************/

Vector3D::Vector3D() {
  for (int i=0; i<3; i++) this->push_back(0.0);
}

Vector3D::Vector3D(double x1, double x2, double x3) {
  push_back(x1);
  push_back(x2);
  push_back(x3);
}

Vector3D::Vector3D(const Vector3D & other) : 
  vector<double>(other)
{}

double Vector3D::length() {
  return sqrt((*this)[0]*(*this)[0] + (*this)[1]*(*this)[1] + (*this)[2]*(*this)[2]);
}

void Vector3D::normalize() {
  double l = length();
  if (is_zero(l)) return;
  *this = (1.0 / l) * *this;
}

Vector3D operator+ (const Vector3D & v1, const Vector3D & v2) {
  Vector3D result;
  for(int i=0; i<3; i++) 
    result[i] = v1[i] + v2[i];
  return result;
}

Vector3D operator- (const Vector3D & v1, const Vector3D & v2) {
  Vector3D result;
  for(int i=0; i<3; i++) 
    result[i] = v1[i] - v2[i];
  return result;
}

Vector3D operator * (double x, const Vector3D & v) {
  Vector3D result;
  for(int i=0; i<3; i++) 
    result[i] = x * v[i];
  return result;
}

double scalar_prod(const Vector3D & v1, const Vector3D & v2) {
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

Vector3D vect_prod(const Vector3D & v1, const Vector3D & v2) {
  return Vector3D(v1[1]*v2[2] - v1[2]*v2[1],
		  v1[2]*v2[0] - v1[0]*v2[2],
		  v1[0]*v2[1] - v1[1]*v2[0]);
}

std::string Vector3D::pp() const {
  char str[256];
  sprintf(str, "(%6.3f, %6.3f, %6.3f)", (*this)[0], (*this)[1], (*this)[2]);
  return string(str);
}

/*****************************************************************************/

Frame3D::Frame3D() {
  push_back(Vector3D(1,0,0));
  push_back(Vector3D(0,1,0));
  push_back(Vector3D(0,0,1));
}

Frame3D::Frame3D(const Frame3D & other) : 
  vector<Vector3D>(other) 
{}

Frame3D Frame3D::from_euler(double psi, double theta, double phi) {
  Frame3D res;
  res[0] = Vector3D(cos(phi)*cos(psi) - sin(phi)*cos(theta)*sin(psi),
		    cos(phi)*sin(psi) + sin(phi)*cos(theta)*cos(psi),
		    sin(phi)*sin(theta));
  res[1] = Vector3D(-sin(phi)*cos(psi) - cos(phi)*cos(theta)*sin(psi),
		    -sin(phi)*sin(psi) + cos(phi)*cos(theta)*cos(psi),
		    cos(phi)*sin(theta));
  res[2] = Vector3D(sin(theta)*sin(psi),
		    -sin(theta)*cos(psi),
		    cos(theta));
  return res;
}

Frame3D Frame3D::from_vectors(Vector3D e1, Vector3D e2, Vector3D e3) {
  Frame3D res;
  res[0] = e1;
  res[1] = e2;
  res[2] = e3;
  return res;
}

string Frame3D::pp() const {
  stringstream str;
  str << (*this)[0].pp() << endl;
  str << (*this)[1].pp() << endl;
  str << (*this)[2].pp() << endl;
  return str.str();
}

/*****************************************************************************/

Position::Position() {
  for (int i=0; i<6; i++) theta[i] = 0.0;
}

Position::Position(double theta0, double theta1, double theta2, 
		   double theta3, double theta4, double theta5) {
  theta[0] = theta0;
  theta[1] = theta1;
  theta[2] = theta2;
  theta[3] = theta3;
  theta[4] = theta4;
  theta[5] = theta5;
}

/*****************************************************************************/

IK::IK(double L0, double L1, double L2) {
  L[0] = L0; L[1] = L1; L[2] = L2;
}

bool IK::compute(Vector3D C, Frame3D orientation, Position & result) {
  if (is_zero(L[0]) || is_zero(L[1]))
    return false;

  Vector3D e1(1,0,0), e2(0,1,0), e3(0,0,1);

  /* step 1 : calcul de B */
  Vector3D B = C + L[2] * orientation[2];  
  double B_len = B.length();
  if (B[2] >= 0 || is_zero(B_len)) return false;
  IKDEBUG(printf("  step 1 ok.\n"));

  /* step 2 : calcul de phi */
  Vector3D phi;
  if (!is_zero(orientation[0][2])) {
    double a_pp = 1.0;
    double b_pp = -B[2] / orientation[0][2];
    phi = (a_pp * B) + (b_pp * orientation[0]);
    phi.normalize();
  } 
  else {
    phi = orientation[0]; 
    phi.normalize(); 
  }

  /* phi est orienté vers l'avant ou sur la gauche */
  double phi_e1 = scalar_prod(phi, e1);
  if (!(phi_e1 > 0 || (is_zero(phi_e1) && scalar_prod(phi, e2) >= 0)))
    phi = -1.0 * phi;
  IKDEBUG(printf("  step 2 ok.\n"));

  /* step 3 : calcul de \theta_0 */
  result.theta[0] = atan2(phi[1], phi[0]);
  IKDEBUG(printf("  step 3 ok.\n"));

  /* step 4 : calcul de G */
  Vector3D G = scalar_prod(B,phi) * phi;
  IKDEBUG(printf("  step 4 ok.\n"));

  /* step 5 : calcul de \theta_1 */
  Vector3D zeta = -1.0 * vect_prod(phi, e3);
  result.theta[1] = atan2(scalar_prod(B-G, zeta), -B[2]);
  IKDEBUG(printf("  step 5 ok.\n"));

  /* step 6 : calcul de \theta_3 */
  double q = (L[0]*L[0] + L[1]*L[1] - B_len*B_len) / (2 * L[0] * L[1]);
  if (q < (-1.0 - ik_global_epsilon) || q > (1.0 + ik_global_epsilon)) return false;
  bound(-1.0, 1.0, q);
  result.theta[3] = M_PI - acos(q);
  IKDEBUG(printf("  step 6 ok.\n"));

  /* step 7 : calcul de \omega */
  Vector3D omega(-sin(result.theta[0])*sin(result.theta[1]),
		 cos(result.theta[0])*sin(result.theta[1]),
		 -cos(result.theta[1]));
  IKDEBUG(printf("  step 7 ok.\n"));

  /* step 8 : calcul de alpha */
  q = scalar_prod(B,omega) / B_len;
  bound(-1.0, 1.0, q); /* on a toujours |q| <= 1 */
  double alpha = sign(scalar_prod(vect_prod(B, omega), zeta)) * acos(q); 
  IKDEBUG(printf("  step 8 ok.\n"));

  /* step 9 : calcul de l'angle (A \Omega B) */
  q = (L[0]*L[0] + B_len*B_len - L[1]*L[1]) / (2 * L[0] * B_len);
  if (q < (-1.0 - ik_global_epsilon) || q > (1.0 + ik_global_epsilon)) return false;
  bound(-1.0, 1.0, q);
  double A_omega_B = acos(q);
  IKDEBUG(printf("  step 9 ok.\n"));

  /* step 10 : calcul de theta_2 */
  result.theta[2] = alpha + A_omega_B;
  IKDEBUG(printf("  step 10 ok.\n"));
  
  /* step 11 : calcul de theta_4 */
  q = scalar_prod(phi, orientation[0]);
  bound(-1.0, 1.0, q);
  double beta = -sign(scalar_prod(vect_prod(phi, orientation[0]), zeta)) * acos(q);
  result.theta[4] = beta + result.theta[3] - result.theta[2];
  IKDEBUG(printf("  step 11 ok.\n"));

  /* step 12 : calcul de theta_5 */
  Vector3D tau = vect_prod(phi, omega);
  q = scalar_prod(tau, orientation[1]);
  bound(-1.0, 1.0, q);
  result.theta[5] = sign(scalar_prod(vect_prod(tau, orientation[1]), orientation[0])) * acos(q);
  IKDEBUG(printf("  step 12 ok.\n"));

  return true;
}

/*****************************************************************************/

bool IKTest::test(double L0, double L1, double L2, 
		  Vector3D C, 
		  double euler_psi, double euler_theta, double euler_phi, 
		  Position result,
		  double epsilon) {
  IK ik(L0,L1,L2);
  Position res;
  ik.compute(C,Frame3D::from_euler(euler_psi, euler_theta, euler_phi), res);
  for (int i=0; i<6; i++)
    if (val_abs(res.theta[i] - result.theta[i]) > epsilon) return false;
  return true;
}

#define LAUNCH(test) if (test) {    \
    printf("test result: OK\n");    \
    test_result =  false;           \
  }                                 \
  else printf("test result: KO\n"); \

bool IKTest::test_suite() {
  printf("-- IK basic test --\n");
  bool test_result = true;

  printf("-- Test 1: \n");
  LAUNCH(test(1.0, 1.0, 0.0, 
	      Vector3D(0.0, 0.0, -sqrt(2)), 
	      0.0, 0.0, 0.0,
	      Position(0.0,0.0,DEG2RAD(45.0),DEG2RAD(90.0),DEG2RAD(45.0),0.0), 
	      0.01))

  printf("-- Test 2: \n");
  LAUNCH(test(1.0, 1.0, 0.2, 
	      Vector3D(0.0, 0.0, -2.2), 
	      0.0, 0.0, 0.0,
	      Position(DEG2RAD(0.0),DEG2RAD(0.0),DEG2RAD(0.0),DEG2RAD(0.0),DEG2RAD(0.0),DEG2RAD(0.0)), 
	      0.01))

  printf("-- Test 3: \n");
  LAUNCH(test(1.0, 1.0, 0.2, 
	      Vector3D(0.0, 0.0, -2.2), 
	      DEG2RAD(15.0), 0.0, 0.0,
	      Position(DEG2RAD(15.0),DEG2RAD(0.0),DEG2RAD(0.0),DEG2RAD(0.0),DEG2RAD(0.0),DEG2RAD(0.0)), 
	      0.01))

  printf("-- Test 4: \n");
  LAUNCH(test(1.0, 1.0, 0.2, 
	      Vector3D(0.0, 0.0, -sqrt(2)-0.2), 
	      DEG2RAD(45), 0.0, 0.0,
	      Position(DEG2RAD(45),0.0,DEG2RAD(45.0),DEG2RAD(90.0),DEG2RAD(45.0),0.0), 
	      0.01))

  printf("-- Test 5: \n");
  LAUNCH(test(1.0, 1.0, 0.2, 
	      Vector3D(0.0, 0.0, -sqrt(2)-0.2), 
	      DEG2RAD(-45), 0.0, 0.0,
	      Position(DEG2RAD(-45),0.0,DEG2RAD(45.0),DEG2RAD(90.0),DEG2RAD(45.0),0.0), 
	      0.01))

  printf("-- Test 6: \n");
  LAUNCH(test(1.0, 1.0, 0.2, 
	      Vector3D(0.0, 0.0, -sqrt(2)-0.2), 
	      DEG2RAD(0), 0.0, 0.0,
	      Position(DEG2RAD(0.0),DEG2RAD(0.0),DEG2RAD(45.0),DEG2RAD(90.0),DEG2RAD(45.0),DEG2RAD(0.0)),
	      0.01))
  return test_result;
}

/*****************************************************************************/
/*****************************************************************************/

}

