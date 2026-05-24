// Sample program that uses the Rotations Conversion Library
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rot_conv/rot_conv.h>
#include <rot_conv/rot_conv_extras.h>
#include <iostream>
#include <ctime>

// Namespaces
using namespace rot_conv;

// Main function
int main(int argc, char **argv)
{
	// Initialise random numbers
	std::srand(std::time(0));

	// Print header
	std::cout << "Rotations Conversion Library Sample" << std::endl << "===================================" << std::endl;

	// Print the identity rotations
	std::cout << "The identity rotation matrix is:" << std::endl << Rotmat::Identity() << std::endl;
	std::cout << "The identity quaternion is " << Quat::Identity() << std::endl;
	std::cout << "The identity Euler angles is " << EulerAngles::Identity() << std::endl;
	std::cout << "The identity fused angles is " << FusedAngles::Identity() << std::endl;
	std::cout << "The identity tilt angles is " << TiltAngles::Identity() << std::endl;
	std::cout << std::endl;

	// Construct the same rotation of each type
	FusedAngles f(3.0*((2.0*std::rand())/RAND_MAX - 1.0), 0.75*((2.0*std::rand())/RAND_MAX - 1.0), 0.75*((2.0*std::rand())/RAND_MAX - 1.0), true);
	Rotmat R = RotmatFromFused(f);
	Quat q = QuatFromFused(f);
	EulerAngles e = EulerFromFused(f);
	TiltAngles t = TiltFromFused(f);

	// Print the rotations
	std::cout << "The following rotations are all the same (obtained by conversions from a random fused angles rotation):" << std::endl;
	std::cout << "Rotation matrix R = " << std::endl << R << std::endl;
	std::cout << "Quaternion   q = " << q << std::endl;
	std::cout << "Euler angles e = " << e << std::endl;
	std::cout << "Fused angles f = " << f << std::endl;
	std::cout << "Tilt angles  t = " << t << std::endl;
	std::cout << std::endl;

	// Yaw of rotations
	std::cout << "So for starters they should all have the same Euler and fused yaws:" << std::endl;
	std::cout << "Rotmat: EYaw = " << EYawOfRotmat(R) << ", FYaw = " << FYawOfRotmat(R) << std::endl;
	std::cout << "Quat:   EYaw = " << EYawOfQuat(q) << ", FYaw = " << FYawOfQuat(q) << std::endl;
	std::cout << "Euler:  EYaw = " << EYawOfEuler(e) << ", FYaw = " << FYawOfEuler(e) << std::endl;
	std::cout << "Fused:  EYaw = " << EYawOfFused(f) << ", FYaw = " << FYawOfFused(f) << std::endl;
	std::cout << "Tilt:   EYaw = " << EYawOfTilt(t) << ", FYaw = " << FYawOfTilt(t) << std::endl;
	std::cout << std::endl;

	// Apply rotations to vector
	Vec3 v(1.0, 2.0, 3.0);
	std::cout << "We also expect the rotations to have the same effect on an input vector [" << v.transpose() << "]" << std::endl;
	std::cout << "Applying Rotmat gives vout = [" << RotmatRotVec(R, v).transpose() << "]" << std::endl;
	std::cout << "Applying Quat   gives vout = [" << QuatRotVec(q, v).transpose() << "]" << std::endl;
	std::cout << "Applying Euler  gives vout = [" << EulerRotVec(e, v).transpose() << "]" << std::endl;
	std::cout << "Applying Fused  gives vout = [" << FusedRotVec(f, v).transpose() << "]" << std::endl;
	std::cout << "Applying Tilt   gives vout = [" << TiltRotVec(t, v).transpose() << "]" << std::endl;
	std::cout << std::endl;

	// Check equality
	std::cout << "But we can perform some conversions and check equality directly:" << std::endl;
	std::cout << "RotmatEqual(R, RotmatFromQuat(q)) = " << (RotmatEqual(R, RotmatFromQuat(q)) ? "true" : "false") << std::endl;
	std::cout << "QuatEqual(q, QuatFromEuler(e))    = " << (QuatEqual(q, QuatFromEuler(e)) ? "true" : "false") << std::endl;
	std::cout << "EulerEqual(e, EulerFromRotmat(R)) = " << (EulerEqual(e, EulerFromRotmat(R)) ? "true" : "false") << std::endl;
	std::cout << "FusedEqual(f, FusedFromTilt(t))   = " << (FusedEqual(f, FusedFromTilt(t)) ? "true" : "false") << std::endl;
	std::cout << "TiltEqual(t, TiltFromQuat(q))     = " << (TiltEqual(t, TiltFromQuat(q)) ? "true" : "false") << std::endl;
	std::cout << std::endl;

	// Remove the fused yaw components of the rotations
	Rotmat Rny = RotmatNoFYaw(R);
	Quat qny = QuatNoFYaw(q);
	EulerAngles eny = EulerNoFYaw(e);
	FusedAngles fny = FusedNoFYaw(f);
	TiltAngles tny = TiltNoFYaw(t);

	// Remove fused yaw component
	std::cout << "We can also for example remove the fused yaw component of the rotations:" << std::endl;
	std::cout << "Output Rotmat:" << std::endl << Rny << " has a fused yaw of " << FYawOfRotmat(Rny) << std::endl;
	std::cout << "Output Quat  " << qny << " has a fused yaw of " << FYawOfQuat(qny) << std::endl;
	std::cout << "Output Euler " << eny << " has a fused yaw of " << FYawOfEuler(eny) << std::endl;
	std::cout << "Output Fused " << fny << " has a fused yaw of " << FYawOfFused(fny) << std::endl;
	std::cout << "Output Tilt  " << tny << " has a fused yaw of " << FYawOfTilt(tny) << std::endl;
	std::cout << std::endl;

	// Rotation inverses
	std::cout << "The inverse rotations are:" << std::endl;
	std::cout << "Inverse Rotmat:" << std::endl << RotmatInv(R) << std::endl;
	std::cout << "Inverse Quat  is " << QuatInv(q) << std::endl;
	std::cout << "Inverse Euler is " << EulerInv(e) << std::endl;
	std::cout << "Inverse Fused is " << FusedInv(f) << std::endl;
	std::cout << "Inverse Tilt  is " << TiltInv(t) << std::endl;
	std::cout << std::endl;

	// Composition of rotations
	std::cout << "Composing the rotations with themselves twice gives:" << std::endl;
	std::cout << "Cubed Rotmat:" << std::endl << ComposeRotmat(R, R, R) << std::endl;
	std::cout << "Cubed Quat  is " << ComposeQuat(q, q, q) << std::endl;
	std::cout << "Cubed Euler is " << ComposeEuler(e, e, e) << std::endl;
	std::cout << "Cubed Fused is " << ComposeFused(f, f, f) << std::endl;
	std::cout << "Cubed Tilt  is " << ComposeTilt(t, t, t) << std::endl;
	std::cout << std::endl;

	// Final message
	std::cout << "But it does not stop there..." << std::endl;
	std::cout << "Check out the library header(s) for a full list of available functions!" << std::endl;
	std::cout << std::endl;
}
// EOF