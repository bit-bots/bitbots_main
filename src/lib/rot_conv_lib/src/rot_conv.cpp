// Rotations conversion library
// File: rot_conv.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rot_conv/rot_conv.h>
#include <Eigen/Eigenvalues>
#include <cfloat>

// Defines
#define M_2PI (2.0*M_PI)

// Rotations conversion namespace
namespace rot_conv
{
	// ################################
	// #### Rotation normalisation ####
	// ################################

	// Normalise: Rotation matrix
	void NormaliseRotmat(Rotmat& R)
	{
		// Find the closest orthogonal matrix to the input rotation matrix
		Rotmat nonOrth = R.transpose() * R;
		R *= Eigen::SelfAdjointEigenSolver<Rotmat>(nonOrth).operatorInverseSqrt();

		// Filter out invalid left hand coordinate systems
		if(R.determinant() < 0.0)
			R.setIdentity();
	}

	// Normalise: Quaternion
	void NormaliseQuat(Quat& q, double normTol)
	{
		// Normalise the quaternion
		double normsq = QuatNormSq(q);
		if(normsq <= normTol*normTol)
			q.setIdentity();
		else
			q /= sqrt(normsq);
	}

	// Normalise: Vector
	void NormaliseVec(Vec3& v, double normTol, const Vec3& vdef)
	{
		// Normalise the vector
		double normsq = VecNormSq(v);
		if(normsq <= normTol*normTol)
			v = vdef;
		else
			v /= sqrt(normsq);
	}

	// ##########################
	// #### Random rotations ####
	// ##########################

	// Random: Vector
	Vec3 RandVec(double maxNorm)
	{
		// Generate the required random vector
		double desNorm = (maxNorm * std::rand()) / RAND_MAX;
		Vec3 vec((2.0*std::rand()) / RAND_MAX - 1.0, (2.0*std::rand()) / RAND_MAX - 1.0, (2.0*std::rand()) / RAND_MAX - 1.0);
		double vecNorm = VecNorm(vec);
		if(vecNorm > 0.0)
			vec *= desNorm / vecNorm;
		return vec;
	}

	// Random: Unit vector
	Vec3 RandUnitVec()
	{
		// Generate the required random vector
		Vec3 vec((2.0*std::rand()) / RAND_MAX - 1.0, (2.0*std::rand()) / RAND_MAX - 1.0, (2.0*std::rand()) / RAND_MAX - 1.0);
		NormaliseVec(vec);
		return vec;
	}

	// Random: Rotation matrix
	void RandRotmat(Rotmat& R)
	{
		// Generate the required random rotation
		Quat q;
		RandQuat(q);
		RotmatFromQuat(q, R);
	}

	// Random: Quaternion
	void RandQuat(Quat& q)
	{
		// Generate random rotation components
		q.w() = (2.0*std::rand()) / RAND_MAX - 1.0;
		q.x() = (2.0*std::rand()) / RAND_MAX - 1.0;
		q.y() = (2.0*std::rand()) / RAND_MAX - 1.0;
		q.z() = (2.0*std::rand()) / RAND_MAX - 1.0;

		// Normalise the quaternion
		NormaliseQuat(q);
	}

	// Random: Euler angles
	void RandEuler(EulerAngles& e)
	{
		// Generate the required random rotation
		e.yaw = RandAng();
		e.pitch = 0.5*RandAng();
		e.roll = RandAng();
	}

	// Random: Fused angles
	void RandFused(FusedAngles& f)
	{
		// Generate the required random rotation
		double lambda1 = 0.25*RandAng();
		double lambda2 = 0.25*RandAng();
		f.fusedYaw = RandAng();
		f.fusedPitch = lambda1 + lambda2;
		f.fusedRoll = lambda1 - lambda2;
		f.hemi = (std::rand() % 2 == 0);
	}

	// Random: Tilt angles
	void RandTilt(TiltAngles& t)
	{
		// Generate the required random rotation
		t.fusedYaw = RandAng();
		t.tiltAxisAngle = RandAng();
		t.tiltAngle = (M_PI * std::rand()) / RAND_MAX;
	}

	// ##########################################
	// #### Rotation checking and validation ####
	// ##########################################

	// Check and validate: Rotation matrix
	bool ValidateRotmat(Rotmat& R, double tol)
	{
		// Make a copy of the input
		Rotmat Rorig = R;

		// Normalise the rotation matrix
		NormaliseRotmat(R);

		// Return whether the rotation matrix was valid within the given tolerance
		return (R - Rorig).isZero(tol);
	}

	// Check and validate: Quaternion
	bool ValidateQuat(Quat& q, double tol, bool unique)
	{
		// Make a copy of the input
		Quat qorig = q;

		// Normalise the quaternion
		NormaliseQuat(q);

		// Make the quaternion unique
		if(unique && q.w() < 0.0)
			q = -q;

		// Return whether the quaternion was valid within the given tolerance
		return QuatEqualExact(q, qorig, tol);
	}

	// Check and validate: Euler angles
	bool ValidateEuler(EulerAngles& e, double tol, bool unique)
	{
		// Make a copy of the input
		EulerAngles eorig = e;

		// Wrap the pitch to (-pi,pi] and then collapse it to the [-pi/2,pi/2] interval
		internal::picutVar(e.pitch);
		if(fabs(e.pitch) > M_PI_2)
		{
			e.yaw += M_PI;
			e.pitch = (e.pitch >= 0.0 ? M_PI - e.pitch : -M_PI - e.pitch);
			e.roll += M_PI;
		}

		// Make the positive and negative gimbal lock representations unique
		if(unique)
		{
			double spitch = sin(e.pitch);
			if(fabs(spitch - 1.0) <= tol)
			{
				e.roll -= e.yaw;
				e.yaw = 0.0;
			}
			else if(fabs(spitch + 1.0) <= tol)
			{
				e.roll += e.yaw;
				e.yaw = 0.0;
			}
		}

		// Wrap yaw and roll to (-pi,pi]
		internal::picutVar(e.yaw);
		internal::picutVar(e.roll);

		// Return whether the Euler angles were valid within the given tolerance
		return (fabs(e.yaw - eorig.yaw) <= tol && fabs(e.pitch - eorig.pitch) <= tol && fabs(e.roll - eorig.roll) <= tol);
	}

	// Check and validate: Fused angles
	bool ValidateFused(FusedAngles& f, double tol, bool unique)
	{
		// Make a copy of the input
		FusedAngles forig = f;

		// Wrap the angles to (-pi,pi]
		internal::picutVar(f.fusedYaw);
		internal::picutVar(f.fusedPitch);
		internal::picutVar(f.fusedRoll);

		// Coerce the L1 norm
		double L1Norm = fabs(f.fusedPitch) + fabs(f.fusedRoll);
		if(L1Norm > M_PI_2)
		{
			double scale = M_PI_2 / L1Norm;
			f.fusedPitch *= scale;
			f.fusedRoll *= scale;
		}

		// Make the representation unique if required
		if(unique)
		{
			double spitch = sin(f.fusedPitch);
			double sroll = sin(f.fusedRoll);
			double sineSum = spitch*spitch + sroll*sroll;
			if(sineSum >= 1.0 - tol)
				f.hemi = true;
			L1Norm = fabs(f.fusedPitch) + fabs(f.fusedRoll);
			if(L1Norm <= tol && !f.hemi)
				f.fusedYaw = 0.0;
		}

		// Return whether the fused angles were valid within the given tolerance
		return (fabs(f.fusedYaw - forig.fusedYaw) <= tol && fabs(f.fusedPitch - forig.fusedPitch) <= tol && fabs(f.fusedRoll - forig.fusedRoll) <= tol && f.hemi == forig.hemi);
	}

	// Check and validate: Tilt angles
	bool ValidateTilt(TiltAngles& t, double tol, bool unique)
	{
		// Make a copy of the input
		TiltAngles torig = t;

		// Wrap the angles to (-pi,pi]
		internal::picutVar(t.fusedYaw);
		internal::picutVar(t.tiltAxisAngle);
		internal::picutVar(t.tiltAngle);

		// Handle the case of a negative tilt angle
		if(t.tiltAngle < 0.0)
		{
			t.tiltAxisAngle = (t.tiltAxisAngle > 0.0 ? -M_PI + t.tiltAxisAngle : M_PI + t.tiltAxisAngle);
			t.tiltAngle = -t.tiltAngle;
		}

		// Make the representation unique if required
		if(unique)
		{
			double ctilt = cos(t.tiltAngle);
			bool near0 = (fabs(ctilt - 1.0) <= tol);
			bool near180 = (fabs(ctilt + 1.0) <= tol);
			if(near0 || near180)
				t.tiltAxisAngle = 0.0;
			if(near180)
				t.fusedYaw = 0.0;
		}

		// Return whether the tilt angles were valid within the given tolerance
		return (fabs(t.fusedYaw - torig.fusedYaw) <= tol && fabs(t.tiltAxisAngle - torig.tiltAxisAngle) <= tol && fabs(t.tiltAngle - torig.tiltAngle) <= tol);
	}

	// ###########################
	// #### Rotation equality ####
	// ###########################

	// Check equality: Rotation matrix
	bool RotmatEqual(const Rotmat& Ra, const Rotmat& Rb, double tol)
	{
		// Return whether none of the elements of the rotation matrices differ by more than the tolerance
		return (Ra - Rb).isZero(tol);
	}

	// Check equality: Rotation matrix (exact)
	bool RotmatEqualExact(const Rotmat& Ra, const Rotmat& Rb, double tol)
	{
		// Return whether none of the elements of the rotation matrices differ by more than the tolerance
		return (Ra - Rb).isZero(tol);
	}

	// Check equality: Quaternion
	bool QuatEqual(const Quat& qa, const Quat& qb, double tol)
	{
		// Return whether to the specified tolerance the quaternions are the same
		bool isSame = (fabs(qa.w() - qb.w()) <= tol && fabs(qa.x() - qb.x()) <= tol && fabs(qa.y() - qb.y()) <= tol && fabs(qa.z() - qb.z()) <= tol);
		bool isOpp  = (fabs(qa.w() + qb.w()) <= tol && fabs(qa.x() + qb.x()) <= tol && fabs(qa.y() + qb.y()) <= tol && fabs(qa.z() + qb.z()) <= tol);
		return (isSame || isOpp);
	}

	// Check equality: Quaternion (exact)
	bool QuatEqualExact(const Quat& qa, const Quat& qb, double tol)
	{
		// Return whether to the specified tolerance the quaternions are the same
		return (fabs(qa.w() - qb.w()) <= tol && fabs(qa.x() - qb.x()) <= tol && fabs(qa.y() - qb.y()) <= tol && fabs(qa.z() - qb.z()) <= tol);
	}

	// Check equality: Euler angles
	bool EulerEqual(const EulerAngles& ea, const EulerAngles& eb, double tol)
	{
		// Convert both Euler angles to their unique representations
		EulerAngles eau = ea, ebu = eb;
		ValidateEuler(eau, tol, true);
		ValidateEuler(ebu, tol, true);

		// Handle angle wrapping issues
		if(fabs(eau.yaw - ebu.yaw) > M_PI)
		{
			if(eau.yaw > ebu.yaw)
				ebu.yaw += M_2PI;
			else
				eau.yaw += M_2PI;
		}
		if(fabs(eau.roll - ebu.roll) > M_PI)
		{
			if(eau.roll > ebu.roll)
				ebu.roll += M_2PI;
			else
				eau.roll += M_2PI;
		}

		// Return whether to the specified tolerance the Euler angles are the same
		return (fabs(eau.yaw - ebu.yaw) <= tol && fabs(sin(eau.pitch) - sin(ebu.pitch)) <= tol && fabs(eau.roll - ebu.roll) <= tol); // The pitch suffers from the numerical insensitivity of asin, so the sine thereof is checked
	}

	// Check equality: Euler angles (exact)
	bool EulerEqualExact(const EulerAngles& ea, const EulerAngles& eb, double tol)
	{
		// Return whether to the specified tolerance the Euler angles are the same
		return (fabs(ea.yaw - eb.yaw) <= tol && fabs(ea.pitch - eb.pitch) <= tol && fabs(ea.roll - eb.roll) <= tol);
	}

	// Check equality: Fused angles
	bool FusedEqual(const FusedAngles& fa, const FusedAngles& fb, double tol)
	{
		// Convert both fused angles to their unique representations
		FusedAngles fau = fa, fbu = fb;
		ValidateFused(fau, tol, true);
		ValidateFused(fbu, tol, true);

		// Handle angle wrapping issues
		if(fabs(fau.fusedYaw - fbu.fusedYaw) > M_PI)
		{
			if(fau.fusedYaw > fbu.fusedYaw)
				fbu.fusedYaw += M_2PI;
			else
				fau.fusedYaw += M_2PI;
		}

		// Return whether to the specified tolerance the fused angles are the same
		return (fabs(fau.fusedYaw - fbu.fusedYaw) <= tol && fabs(sin(fau.fusedPitch) - sin(fbu.fusedPitch)) <= tol && fabs(sin(fau.fusedRoll) - sin(fbu.fusedRoll)) <= tol && fau.hemi == fbu.hemi); // The fused pitch and roll suffer from the numerical insensitivity of asin, so the sine's thereof are checked
	}

	// Check equality: Fused angles (exact)
	bool FusedEqualExact(const FusedAngles& fa, const FusedAngles& fb, double tol)
	{
		// Return whether to the specified tolerance the fused angles are the same
		return (fabs(fa.fusedYaw - fb.fusedYaw) <= tol && fabs(fa.fusedPitch - fb.fusedPitch) <= tol && fabs(fa.fusedRoll - fb.fusedRoll) <= tol && fa.hemi == fb.hemi);
	}

	// Check equality: Tilt angles
	bool TiltEqual(const TiltAngles& ta, const TiltAngles& tb, double tol)
	{
		// Convert both tilt angles to their unique representations
		TiltAngles tau = ta, tbu = tb;
		ValidateTilt(tau, tol, true);
		ValidateTilt(tbu, tol, true);

		// Handle angle wrapping issues
		if(fabs(tau.fusedYaw - tbu.fusedYaw) > M_PI)
		{
			if(tau.fusedYaw > tbu.fusedYaw)
				tbu.fusedYaw += M_2PI;
			else
				tau.fusedYaw += M_2PI;
		}

		// Return whether to the specified tolerance the fused angles are the same
		double stilta = sin(tau.tiltAngle);
		double stiltb = sin(tbu.tiltAngle);
		double stiltasq = stilta*stilta;
		double stiltbsq = stiltb*stiltb;
		return (fabs(tau.fusedYaw - tbu.fusedYaw) <= tol && fabs(stiltasq*cos(tau.tiltAxisAngle) - stiltbsq*cos(tbu.tiltAxisAngle)) <= tol && fabs(stiltasq*sin(tau.tiltAxisAngle) - stiltbsq*sin(tbu.tiltAxisAngle)) <= tol && fabs(cos(tau.tiltAngle) - cos(tbu.tiltAngle)) <= tol); // The tilt angle suffers from the numerical insensitivity of acos, so the cosine thereof is checked / The tilt axis angle has a singularity when the tilt angle is zero, so two geometrically relevant terms are checked instead of the tilt axis angle directly
	}

	// Check equality: Tilt angles (exact)
	bool TiltEqualExact(const TiltAngles& ta, const TiltAngles& tb, double tol)
	{
		// Return whether to the specified tolerance the tilt angles are the same
		return (fabs(ta.fusedYaw - tb.fusedYaw) <= tol && fabs(ta.tiltAxisAngle - tb.tiltAxisAngle) <= tol && fabs(ta.tiltAngle - tb.tiltAngle) <= tol);
	}

	// #########################
	// #### Yaw of rotation ####
	// #########################

	// Euler yaw of: Rotation matrix
	double EYawOfRotmat(const Rotmat& R)
	{
		// Calculate and return the Euler ZYX yaw of the rotation
		return atan2(R.coeff(1,0), R.coeff(0,0));
	}

	// Fused yaw of: Rotation matrix
	double FYawOfRotmat(const Rotmat& R)
	{
		// Calculate, wrap and return the fused yaw
		double fusedYaw, trace = R.coeff(0,0) + R.coeff(1,1) + R.coeff(2,2);
		if(trace >= 0.0)
			fusedYaw = 2.0*atan2(R.coeff(1,0) - R.coeff(0,1), 1.0 + trace);
		else if(R.coeff(2,2) >= R.coeff(1,1) && R.coeff(2,2) >= R.coeff(0,0))
			fusedYaw = 2.0*atan2(1.0 - R.coeff(0,0) - R.coeff(1,1) + R.coeff(2,2), R.coeff(1,0) - R.coeff(0,1));
		else if(R.coeff(1,1) >= R.coeff(0,0))
			fusedYaw = 2.0*atan2(R.coeff(2,1) + R.coeff(1,2), R.coeff(0,2) - R.coeff(2,0));
		else
			fusedYaw = 2.0*atan2(R.coeff(0,2) + R.coeff(2,0), R.coeff(2,1) - R.coeff(1,2));
		if(fusedYaw > M_PI) fusedYaw -= M_2PI;   // fusedYaw is now in [-2*pi,pi]
		if(fusedYaw <= -M_PI) fusedYaw += M_2PI; // fusedYaw is now in (-pi,pi]
		return fusedYaw;
	}

	// Euler yaw of: Quaternion
	double EYawOfQuat(const Quat& q)
	{
		// Calculate and return the Euler ZYX yaw of the rotation
		return atan2(q.w()*q.z() + q.x()*q.y(), 0.5 - (q.y()*q.y() + q.z()*q.z()));
	}

	// Fused yaw of: Quaternion
	double FYawOfQuat(const Quat& q)
	{
		// Calculate, wrap and return the fused yaw
		double fusedYaw = 2.0*atan2(q.z(), q.w()); // Output of atan2 is [-pi,pi], so this expression is in [-2*pi,2*pi]
		if(fusedYaw > M_PI) fusedYaw -= M_2PI;     // fusedYaw is now in [-2*pi,pi]
		if(fusedYaw <= -M_PI) fusedYaw += M_2PI;   // fusedYaw is now in (-pi,pi]
		return fusedYaw;
	}

	// Fused yaw of: Euler angles
	double FYawOfEuler(const EulerAngles& e)
	{
		// Calculate and return the fused yaw of the rotation
		return FYawOfRotmat(RotmatFromEuler(e));
	}

	// Euler yaw of: Fused angles
	double EYawOfFused(const FusedAngles& f)
	{
		// Precalculate the sin values
		double sth  = sin(f.fusedPitch);
		double sphi = sin(f.fusedRoll);

		// Calculate the sine sum criterion
		double crit = sth*sth + sphi*sphi;

		// Calculate the cosine of the tilt angle alpha
		double calpha;
		if(crit >= 1.0)
			calpha = 0.0;
		else
			calpha = (f.hemi ? sqrt(1.0-crit) : -sqrt(1.0-crit));

		// Calculate the tilt axis gamma
		double gamma = atan2(sth,sphi);

		// Precalculate trigonometric terms
		double psigam = f.fusedYaw + gamma;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);
		double cgam = cos(gamma);
		double sgam = sin(gamma);
		double A = sgam*calpha;

		// Calculate and return the Euler ZYX yaw of the rotation
		return atan2(cgam*spsigam - A*cpsigam, cgam*cpsigam + A*spsigam);
	}

	// Euler yaw of: Tilt angles
	double EYawOfTilt(const TiltAngles& t)
	{
		// Precalculate trigonometric terms
		double psigam = t.fusedYaw + t.tiltAxisAngle;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);
		double cgam = cos(t.tiltAxisAngle);
		double sgam = sin(t.tiltAxisAngle);
		double calpha = cos(t.tiltAngle);
		double A = sgam*calpha;

		// Calculate and return the Euler ZYX yaw of the rotation
		return atan2(cgam*spsigam - A*cpsigam, cgam*cpsigam + A*spsigam);
	}

	// ##################################
	// #### Remove yaw from rotation ####
	// ##################################

	// Remove Euler yaw from: Rotation matrix
	void RotmatNoEYaw(const Rotmat& R, Rotmat& Rout)
	{
		// Calculate the Euler ZYX yaw of the input
		double EYaw = EYawOfRotmat(R);

		// Precalculate trigonometric terms
		double cEYaw = cos(EYaw);
		double sEYaw = sin(EYaw);

		// Construct the Euler ZYX yaw component of the rotation
		Rotmat REYawTrans;
		REYawTrans << cEYaw, sEYaw, 0.0,
		              -sEYaw, cEYaw, 0.0,
		              0.0, 0.0, 1.0;

		// Remove the Euler ZYX yaw component of the rotation
		Rout = REYawTrans * R;
	}

	// Remove fused yaw from: Rotation matrix
	void RotmatNoFYaw(const Rotmat& R, Rotmat& Rout)
	{
		// Calculate the fused yaw of the input
		double FYaw = FYawOfRotmat(R);

		// Precalculate trigonometric terms
		double cFYaw = cos(FYaw);
		double sFYaw = sin(FYaw);

		// Construct the fused yaw component of the rotation
		Rotmat RFYawTrans;
		RFYawTrans << cFYaw, sFYaw, 0.0,
		              -sFYaw, cFYaw, 0.0,
		              0.0, 0.0, 1.0;

		// Remove the fused yaw component of the rotation
		Rout = RFYawTrans * R;
	}

	// Remove Euler yaw from: Quaternion
	void QuatNoEYaw(const Quat& q, Quat& qout)
	{
		// Calculate the Euler ZYX yaw of the input
		double EYaw = EYawOfQuat(q);

		// Construct the Euler ZYX yaw component of the rotation
		double hcEYaw = cos(0.5*EYaw);
		double hsEYaw = sin(0.5*EYaw);

		// Remove the Euler ZYX yaw component of the rotation
		qout.w() = hcEYaw*q.w() + hsEYaw*q.z();
		qout.x() = hcEYaw*q.x() + hsEYaw*q.y();
		qout.y() = hcEYaw*q.y() - hsEYaw*q.x();
		qout.z() = hcEYaw*q.z() - hsEYaw*q.w();
	}

	// Remove fused yaw from: Quaternion
	void QuatNoFYaw(const Quat& q, Quat& qout)
	{
		// Calculate the fused yaw of the input
		double FYaw = FYawOfQuat(q);

		// Construct the fused yaw component of the rotation
		double hcFYaw = cos(0.5*FYaw);
		double hsFYaw = sin(0.5*FYaw);

		// Remove the fused yaw component of the rotation
		qout.w() = hcFYaw*q.w() + hsFYaw*q.z();
		qout.x() = hcFYaw*q.x() + hsFYaw*q.y();
		qout.y() = hcFYaw*q.y() - hsFYaw*q.x();
		qout.z() = hcFYaw*q.z() - hsFYaw*q.w();
	}

	// Remove yaw from: Euler angles
	void EulerNoFYaw(const EulerAngles& e, EulerAngles& eout)
	{
		// Calculate the fused yaw of the input
		double FYaw = FYawOfEuler(e);

		// Remove the fused yaw component of the rotation
		eout.yaw = e.yaw - FYaw;
		eout.pitch = e.pitch;
		eout.roll = e.roll;
	}

	// Remove yaw from: Fused angles
	void FusedNoEYaw(const FusedAngles& f, FusedAngles& fout)
	{
		// Calculate the Euler ZYX yaw of the input
		double EYaw = EYawOfFused(f);

		// Remove the Euler ZYX yaw component of the rotation
		fout.fusedYaw = f.fusedYaw - EYaw;
		fout.fusedPitch = f.fusedPitch;
		fout.fusedRoll = f.fusedRoll;
		fout.hemi = f.hemi;
	}

	// Remove yaw from: Tilt angles
	void TiltNoEYaw(const TiltAngles& t, TiltAngles& tout)
	{
		// Calculate the Euler ZYX yaw of the input
		double EYaw = EYawOfTilt(t);

		// Remove the Euler ZYX yaw component of the rotation
		tout.fusedYaw = t.fusedYaw - EYaw;
		tout.tiltAxisAngle = t.tiltAxisAngle;
		tout.tiltAngle = t.tiltAngle;
	}

	// #################################
	// #### Rotation with given yaw ####
	// #################################

	// Rotation with given Euler yaw: Rotation matrix
	void RotmatWithEYaw(const Rotmat& R, double eulerYaw, Rotmat& Rout)
	{
		// Calculate the Euler ZYX yaw of the input
		double EYaw = EYawOfRotmat(R);

		// Precalculate trigonometric terms
		double deltaEYaw = eulerYaw - EYaw;
		double cdEYaw = cos(deltaEYaw);
		double sdEYaw = sin(deltaEYaw);

		// Construct the yaw adjustment rotation
		Rotmat REYawAdj;
		REYawAdj << cdEYaw, -sdEYaw, 0.0,
		            sdEYaw, cdEYaw, 0.0,
		            0.0, 0.0, 1.0;

		// Adjust the Euler ZYX yaw component of the rotation
		Rout = REYawAdj * R;
	}

	// Rotation with given fused yaw: Rotation matrix
	void RotmatWithFYaw(const Rotmat& R, double fusedYaw, Rotmat& Rout)
	{
		// Calculate the fused yaw of the input
		double FYaw = FYawOfRotmat(R);

		// Precalculate trigonometric terms
		double deltaFYaw = fusedYaw - FYaw;
		double cdFYaw = cos(deltaFYaw);
		double sdFYaw = sin(deltaFYaw);

		// Construct the yaw adjustment rotation
		Rotmat RFYawAdj;
		RFYawAdj << cdFYaw, -sdFYaw, 0.0,
		            sdFYaw, cdFYaw, 0.0,
		            0.0, 0.0, 1.0;

		// Adjust the fused yaw component of the rotation
		Rout = RFYawAdj * R;
	}

	// Rotation with given Euler yaw: Quaternion
	void QuatWithEYaw(const Quat& q, double eulerYaw, Quat& qout)
	{
		// Calculate the Euler ZYX yaw of the input
		double EYaw = EYawOfQuat(q);

		// Construct the components of the Euler ZYX yaw adjustment rotation
		double hdeltaEYaw = 0.5*(eulerYaw - EYaw);
		double hcdEYaw = cos(hdeltaEYaw);
		double hsdEYaw = sin(hdeltaEYaw);

		// Adjust the Euler ZYX yaw component of the rotation
		qout.w() = hcdEYaw*q.w() - hsdEYaw*q.z();
		qout.x() = hcdEYaw*q.x() - hsdEYaw*q.y();
		qout.y() = hcdEYaw*q.y() + hsdEYaw*q.x();
		qout.z() = hcdEYaw*q.z() + hsdEYaw*q.w();
	}

	// Rotation with given fused yaw: Quaternion
	void QuatWithFYaw(const Quat& q, double fusedYaw, Quat& qout)
	{
		// Calculate the fused yaw of the input
		double FYaw = FYawOfQuat(q);

		// Construct the components of the fused yaw adjustment rotation
		double hdeltaFYaw = 0.5*(fusedYaw - FYaw);
		double hcdFYaw = cos(hdeltaFYaw);
		double hsdFYaw = sin(hdeltaFYaw);

		// Adjust the fused yaw component of the rotation
		qout.w() = hcdFYaw*q.w() - hsdFYaw*q.z();
		qout.x() = hcdFYaw*q.x() - hsdFYaw*q.y();
		qout.y() = hcdFYaw*q.y() + hsdFYaw*q.x();
		qout.z() = hcdFYaw*q.z() + hsdFYaw*q.w();
	}

	// Rotation with given fused yaw: Euler angles
	void EulerWithFYaw(const EulerAngles& e, double fusedYaw, EulerAngles& eout)
	{
		// Calculate the fused yaw of the input
		double FYaw = FYawOfEuler(e);

		// Adjust the fused yaw component of the rotation
		eout.yaw = e.yaw + (fusedYaw - FYaw);
		eout.pitch = e.pitch;
		eout.roll = e.roll;
	}

	// Rotation with given Euler yaw: Fused angles
	void FusedWithEYaw(const FusedAngles& f, double eulerYaw, FusedAngles& fout)
	{
		// Calculate the Euler ZYX yaw of the input
		double EYaw = EYawOfFused(f);

		// Adjust the Euler ZYX yaw component of the rotation
		fout.fusedYaw = f.fusedYaw + (eulerYaw - EYaw);
		fout.fusedPitch = f.fusedPitch;
		fout.fusedRoll = f.fusedRoll;
		fout.hemi = f.hemi;
	}

	// Rotation with given Euler yaw: Tilt angles
	void TiltWithEYaw(const TiltAngles& t, double eulerYaw, TiltAngles& tout)
	{
		// Calculate the Euler ZYX yaw of the input
		double EYaw = EYawOfTilt(t);

		// Adjust the Euler ZYX yaw component of the rotation
		tout.fusedYaw = t.fusedYaw + (eulerYaw - EYaw);
		tout.tiltAxisAngle = t.tiltAxisAngle;
		tout.tiltAngle = t.tiltAngle;
	}

	// ###########################
	// #### Rotation inverses ####
	// ###########################

	// Inverse: Rotation matrix
	void RotmatInv(const Rotmat& R, Rotmat& Rinv)
	{
		// Calculate the inverse of the rotation
		Rinv = R.transpose();
	}

	// Inverse: Quaternion
	void QuatInv(const Quat& q, Quat& qinv)
	{
		// Calculate the inverse of the rotation
		qinv.w() = q.w();
		qinv.x() = -q.x();
		qinv.y() = -q.y();
		qinv.z() = -q.z();
	}

	// Inverse: Euler angles
	void EulerInv(const EulerAngles& e, EulerAngles& einv)
	{
		// Precalculate the required sin and cos values
		double cpsi = cos(e.yaw);
		double spsi = sin(e.yaw);
		double cth = cos(e.pitch);
		double sth = sin(e.pitch);
		double cphi = cos(e.roll);
		double sphi = sin(e.roll);

		// Calculate the sine of the inverse pitch angle
		double sthinv = -(cpsi*sth*cphi + spsi*sphi);
		sthinv = (sthinv >= 1.0 ? 1.0 : (sthinv <= -1.0 ? -1.0 : sthinv)); // Coerce sthinv to [-1,1]

		// Calculate the required inverse Euler angles representation
		einv.yaw = atan2(cpsi*sth*sphi - spsi*cphi, cpsi*cth);
		einv.pitch = asin(sthinv);
		einv.roll = atan2(spsi*sth*cphi - cpsi*sphi, cth*cphi);
	}

	// Inverse: Fused angles
	void FusedInv(const FusedAngles& f, FusedAngles& finv)
	{
		// Precalculate the sin values
		double sth  = sin(f.fusedPitch);
		double sphi = sin(f.fusedRoll);

		// Calculate the sine of the tilt angle alpha
		double crit = sth*sth + sphi*sphi;
		double salpha = (crit >= 1.0 ? 1.0 : sqrt(crit));

		// Calculate the tilt axis gamma
		double gamma = atan2(sth,sphi);

		// Precalculate trigonometric values
		double psigam = f.fusedYaw + gamma;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);

		// Calculate the inverse fused pitch and roll
		double thinv = asin(-salpha*spsigam);
		double phinv = asin(-salpha*cpsigam);

		// Construct the inverse fused angles rotation
		finv.fusedYaw = -f.fusedYaw;
		finv.fusedPitch = thinv;
		finv.fusedRoll = phinv;
		finv.hemi = f.hemi;
	}

	// Inverse: Tilt angles
	void TiltInv(const TiltAngles& t, TiltAngles& tinv)
	{
		// Calculate the inverse tilt axis angle
		double gammainv = internal::picut(t.fusedYaw + t.tiltAxisAngle - M_PI);

		// Construct the inverse tilt angles rotation
		tinv.fusedYaw = -t.fusedYaw;
		tinv.tiltAxisAngle = gammainv;
		tinv.tiltAngle = t.tiltAngle;
	}

	// ##########################
	// #### Vector rotations ####
	// ##########################

	// Rotate vector by: Rotation matrix
	Vec3 RotmatRotVec(const Rotmat& R, const Vec3& v)
	{
		// Return the required vector
		return R*v;
	}

	// Rotate vector by: Rotation matrix (in-place)
	void RotmatRotVecInPlace(const Rotmat& R, Vec3& v)
	{
		// Calculate the required vector
		v = R*v;
	}

	// Rotate pure z-vector by: Rotation matrix
	Vec3 RotmatRotVecPureZ(const Rotmat& R, double vz)
	{
		// Return the required vector
		return R.col(2)*vz;
	}

	// Rotate vector by: Quaternion
	Vec3 QuatRotVec(const Quat& q, const Vec3& v)
	{
		// Precalculate an intermediate vector term
		double tx = 2.0*(q.y()*v.z() - v.y()*q.z());
		double ty = 2.0*(q.z()*v.x() - v.z()*q.x());
		double tz = 2.0*(q.x()*v.y() - v.x()*q.y());

		// Calculate and return the required vector
		Vec3 vout = v;
		vout.x() += q.w()*tx + q.y()*tz - ty*q.z();
		vout.y() += q.w()*ty + q.z()*tx - tz*q.x();
		vout.z() += q.w()*tz + q.x()*ty - tx*q.y();
		return vout;
	}

	// Rotate vector by: Quaternion (in-place)
	void QuatRotVecInPlace(const Quat& q, Vec3& v)
	{
		// Precalculate an intermediate vector term
		double tx = 2.0*(q.y()*v.z() - v.y()*q.z());
		double ty = 2.0*(q.z()*v.x() - v.z()*q.x());
		double tz = 2.0*(q.x()*v.y() - v.x()*q.y());

		// Calculate the required vector
		v.x() += q.w()*tx + q.y()*tz - ty*q.z();
		v.y() += q.w()*ty + q.z()*tx - tz*q.x();
		v.z() += q.w()*tz + q.x()*ty - tx*q.y();
	}

	// Rotate pure z-vector by: Quaternion
	Vec3 QuatRotVecPureZ(const Quat& q, double vz)
	{
		// Calculate and return the required vector
		return Vec3(vz*2.0*(q.x()*q.z() + q.y()*q.w()), vz*2.0*(q.y()*q.z() - q.x()*q.w()), vz*(1.0 - 2.0*(q.x()*q.x() + q.y()*q.y())));
	}

	// Rotate vector by: Euler angles
	Vec3 EulerRotVec(const EulerAngles& e, const Vec3& v)
	{
		// Return the required vector
		return RotmatFromEuler(e)*v;
	}

	// Rotate vector by: Euler angles (in-place)
	void EulerRotVecInPlace(const EulerAngles& e, Vec3& v)
	{
		// Calculate the required vector
		v = RotmatFromEuler(e)*v;
	}

	// Rotate pure z-vector by: Euler angles
	Vec3 EulerRotVecPureZ(const EulerAngles& e, double vz)
	{
		// Precalculate the trigonometric values
		double cpsi = cos(e.yaw);
		double spsi = sin(e.yaw);
		double cth  = cos(e.pitch);
		double sth  = sin(e.pitch);
		double cphi = cos(e.roll);
		double sphi = sin(e.roll);

		// Calculate and return the required vector
		return Vec3(vz*(cpsi*sth*cphi + spsi*sphi), vz*(spsi*sth*cphi - cpsi*sphi), vz*(cth*cphi));
	}

	// Rotate vector by: Fused angles
	Vec3 FusedRotVec(const FusedAngles& f, const Vec3& v)
	{
		// Return the required vector
		return RotmatFromFused(f)*v;
	}

	// Rotate vector by: Fused angles (in-place)
	void FusedRotVecInPlace(const FusedAngles& f, Vec3& v)
	{
		// Calculate the required vector
		v = RotmatFromFused(f)*v;
	}

	// Rotate pure z-vector by: Fused angles
	Vec3 FusedRotVecPureZ(const FusedAngles& f, double vz)
	{
		// Precalculate the sine values
		double sth = sin(f.fusedPitch);
		double sphi = sin(f.fusedRoll);

		// Calculate the sine sum criterion
		double crit = sth*sth + sphi*sphi;

		// Calculate the tilt angle alpha
		double calpha, salpha;
		if(crit >= 1.0)
		{
			calpha = 0.0;
			salpha = 1.0;
		}
		else
		{
			calpha = (f.hemi ? sqrt(1.0 - crit) : -sqrt(1.0 - crit));
			salpha = sqrt(crit);
		}

		// Calculate the tilt axis angle gamma
		double gamma = atan2(sth,sphi);

		// Precalculate terms involved in the vector expression
		double psigam = f.fusedYaw + gamma;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);

		// Calculate and return the required vector
		return Vec3(vz*salpha*spsigam, -vz*salpha*cpsigam, vz*calpha);
	}

	// Rotate vector by: Tilt angles
	Vec3 TiltRotVec(const TiltAngles& t, const Vec3& v)
	{
		// Return the required vector
		return RotmatFromTilt(t)*v;
	}

	// Rotate vector by: Tilt angles (in-place)
	void TiltRotVecInPlace(const TiltAngles& t, Vec3& v)
	{
		// Return the required vector
		v = RotmatFromTilt(t)*v;
	}

	// Rotate pure z-vector by: Tilt angles
	Vec3 TiltRotVecPureZ(const TiltAngles& t, double vz)
	{
		// Precalculate terms involved in the vector expression
		double calpha = cos(t.tiltAngle);
		double salpha = sin(t.tiltAngle);
		double psigam = t.fusedYaw + t.tiltAxisAngle;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);

		// Calculate and return the required vector
		return Vec3(vz*salpha*spsigam, -vz*salpha*cpsigam, vz*calpha);
	}

	// #########################################
	// #### Rotation about global unit axis ####
	// #########################################

	// Rotate about global x-axis: Rotation matrix
	void RotmatRotGlobalX(const Rotmat& R, double angle, Rotmat& Rout)
	{
		// Precalculate trigonometric values
		double cang = cos(angle);
		double sang = sin(angle);

		// Calculate the required rotated rotation
		Rout << R.coeff(0,0), R.coeff(0,1), R.coeff(0,2),
		        R.coeff(1,0)*cang - R.coeff(2,0)*sang, R.coeff(1,1)*cang - R.coeff(2,1)*sang, R.coeff(1,2)*cang - R.coeff(2,2)*sang,
		        R.coeff(2,0)*cang + R.coeff(1,0)*sang, R.coeff(2,1)*cang + R.coeff(1,1)*sang, R.coeff(2,2)*cang + R.coeff(1,2)*sang;
	}

	// Rotate about global y-axis: Rotation matrix
	void RotmatRotGlobalY(const Rotmat& R, double angle, Rotmat& Rout)
	{
		// Precalculate trigonometric values
		double cang = cos(angle);
		double sang = sin(angle);

		// Calculate the required rotated rotation
		Rout << R.coeff(0,0)*cang + R.coeff(2,0)*sang, R.coeff(0,1)*cang + R.coeff(2,1)*sang, R.coeff(0,2)*cang + R.coeff(2,2)*sang,
		        R.coeff(1,0), R.coeff(1,1), R.coeff(1,2),
		        R.coeff(2,0)*cang - R.coeff(0,0)*sang, R.coeff(2,1)*cang - R.coeff(0,1)*sang, R.coeff(2,2)*cang - R.coeff(0,2)*sang;
	}

	// Rotate about global z-axis: Rotation matrix
	void RotmatRotGlobalZ(const Rotmat& R, double angle, Rotmat& Rout)
	{
		// Precalculate trigonometric values
		double cang = cos(angle);
		double sang = sin(angle);

		// Calculate the required rotated rotation
		Rout << R.coeff(0,0)*cang - R.coeff(1,0)*sang, R.coeff(0,1)*cang - R.coeff(1,1)*sang, R.coeff(0,2)*cang - R.coeff(1,2)*sang,
		        R.coeff(1,0)*cang + R.coeff(0,0)*sang, R.coeff(1,1)*cang + R.coeff(0,1)*sang, R.coeff(1,2)*cang + R.coeff(0,2)*sang,
		        R.coeff(2,0), R.coeff(2,1), R.coeff(2,2);
	}

	// Rotate about global x-axis: Quaternion
	void QuatRotGlobalX(const Quat& q, double angle, Quat& qout)
	{
		// Precalculate trigonometric values
		double hcang = cos(0.5*angle);
		double hsang = sin(0.5*angle);

		// Calculate the required rotated rotation
		qout.w() = hcang*q.w() - hsang*q.x();
		qout.x() = hcang*q.x() + hsang*q.w();
		qout.y() = hcang*q.y() - hsang*q.z();
		qout.z() = hcang*q.z() + hsang*q.y();
	}

	// Rotate about global y-axis: Quaternion
	void QuatRotGlobalY(const Quat& q, double angle, Quat& qout)
	{
		// Precalculate trigonometric values
		double hcang = cos(0.5*angle);
		double hsang = sin(0.5*angle);

		// Calculate the required rotated rotation
		qout.w() = hcang*q.w() - hsang*q.y();
		qout.x() = hcang*q.x() + hsang*q.z();
		qout.y() = hcang*q.y() + hsang*q.w();
		qout.z() = hcang*q.z() - hsang*q.x();
	}

	// Rotate about global z-axis: Quaternion
	void QuatRotGlobalZ(const Quat& q, double angle, Quat& qout)
	{
		// Precalculate trigonometric values
		double hcang = cos(0.5*angle);
		double hsang = sin(0.5*angle);

		// Calculate the required rotated rotation
		qout.w() = hcang*q.w() - hsang*q.z();
		qout.x() = hcang*q.x() - hsang*q.y();
		qout.y() = hcang*q.y() + hsang*q.x();
		qout.z() = hcang*q.z() + hsang*q.w();
	}

	// Rotate about global x-axis: Euler angles
	void EulerRotGlobalX(const EulerAngles& e, double angle, EulerAngles& eout)
	{
		// Calculate the required rotated rotation
		Quat q = QuatFromEuler(e), qout;
		QuatRotGlobalX(q, angle, qout);
		EulerFromQuat(qout, eout);
	}

	// Rotate about global y-axis: Euler angles
	void EulerRotGlobalY(const EulerAngles& e, double angle, EulerAngles& eout)
	{
		// Calculate the required rotated rotation
		Quat q = QuatFromEuler(e), qout;
		QuatRotGlobalY(q, angle, qout);
		EulerFromQuat(qout, eout);
	}

	// Rotate about global x-axis: Fused angles
	void FusedRotGlobalX(const FusedAngles& f, double angle, FusedAngles& fout)
	{
		// Calculate the required rotated rotation
		Quat q = QuatFromFused(f), qout;
		QuatRotGlobalX(q, angle, qout);
		FusedFromQuat(qout, fout);
	}

	// Rotate about global y-axis: Fused angles
	void FusedRotGlobalY(const FusedAngles& f, double angle, FusedAngles& fout)
	{
		// Calculate the required rotated rotation
		Quat q = QuatFromFused(f), qout;
		QuatRotGlobalY(q, angle, qout);
		FusedFromQuat(qout, fout);
	}

	// Rotate about global x-axis: Tilt angles
	void TiltRotGlobalX(const TiltAngles& t, double angle, TiltAngles& tout)
	{
		// Calculate the required rotated rotation
		Quat q = QuatFromTilt(t), qout;
		QuatRotGlobalX(q, angle, qout);
		TiltFromQuat(qout, tout);
	}

	// Rotate about global y-axis: Tilt angles
	void TiltRotGlobalY(const TiltAngles& t, double angle, TiltAngles& tout)
	{
		// Calculate the required rotated rotation
		Quat q = QuatFromTilt(t), qout;
		QuatRotGlobalY(q, angle, qout);
		TiltFromQuat(qout, tout);
	}

	// #####################################
	// #### Conversions from axis angle ####
	// #####################################

	// Conversion: Axis angle (unit axis) --> Rotation matrix
	void RotmatFromAxis(UnitAxis axis, double angle, Rotmat& R)
	{
		// Precalculate trigonometric values
		double cang = cos(angle);
		double sang = sin(angle);

		// Calculate the required rotation matrix
		if(axis == X_AXIS)
		{
			R << 1.0, 0.0, 0.0,
			     0.0, cang, -sang,
			     0.0, sang, cang;
		}
		else if(axis == Y_AXIS)
		{
			R << cang, 0.0, sang,
			     0.0, 1.0, 0.0,
			     -sang, 0.0, cang;
		}
		else // Z_AXIS
		{
			R << cang, -sang, 0.0,
			     sang, cang, 0.0,
			     0.0, 0.0, 1.0;
		}
	}

	// Conversion: Axis angle --> Rotation matrix
	void RotmatFromAxis(const Vec3& axis, double angle, Rotmat& R)
	{
		// Precalculate trigonometric values
		double cang = cos(angle);
		double sang = sin(angle);
		double C = 1.0 - cang;

		// Precalculate values
		double xC = axis.x()*C;
		double yC = axis.y()*C;
		double zC = axis.z()*C;
		double xxC = axis.x()*xC;
		double yyC = axis.y()*yC;
		double zzC = axis.z()*zC;
		double xyC = axis.x()*yC;
		double yzC = axis.y()*zC;
		double zxC = axis.z()*xC;
		double xs = axis.x()*sang;
		double ys = axis.y()*sang;
		double zs = axis.z()*sang;

		// Calculate the required rotation matrix
		R << xxC + cang, xyC - zs, zxC + ys,
		     xyC + zs, yyC + cang, yzC - xs,
		     zxC - ys, yzC + xs, zzC + cang;
	}

	// Conversion: Axis angle (unit axis) --> Quaternion
	void QuatFromAxis(UnitAxis axis, double angle, Quat& q)
	{
		// Precalculate trigonometric values
		double hcang = cos(0.5*angle);
		double hsang = sin(0.5*angle);

		// Calculate the required quaternion
		if(axis == X_AXIS)
		{
			q.w() = hcang;
			q.x() = hsang;
			q.y() = 0.0;
			q.z() = 0.0;
		}
		else if(axis == Y_AXIS)
		{
			q.w() = hcang;
			q.x() = 0.0;
			q.y() = hsang;
			q.z() = 0.0;
		}
		else // Z_AXIS
		{
			q.w() = hcang;
			q.x() = 0.0;
			q.y() = 0.0;
			q.z() = hsang;
		}
	}

	// Conversion: Axis angle --> Quaternion
	void QuatFromAxis(const Vec3& axis, double angle, Quat& q)
	{
		// Precalculate trigonometric values
		double hcang = cos(0.5*angle);
		double hsang = sin(0.5*angle);

		// Calculate the required quaternion
		double normsq = VecNormSq(axis);
		if(normsq <= 0.0)
			q.setIdentity();
		else
		{
			double scale = hsang / sqrt(normsq);
			q.w() = hcang;
			q.x() = axis.x() * scale;
			q.y() = axis.y() * scale;
			q.z() = axis.z() * scale;
		}
	}

	// Conversion: Axis angle (unit axis) --> Euler angles
	void EulerFromAxis(UnitAxis axis, double angle, EulerAngles& e)
	{
		// Wrap the rotation angle to (-pi,pi]
		internal::picutVar(angle);

		// Calculate the required Euler angles
		if(axis == X_AXIS)
		{
			e.yaw = 0.0;
			e.pitch = 0.0;
			e.roll = angle;
		}
		else if(axis == Y_AXIS)
		{
			if(fabs(angle) <= M_PI_2)
			{
				e.yaw = 0.0;
				e.pitch = angle;
				e.roll = 0.0;
			}
			else
			{
				e.yaw = M_PI;
				e.pitch = (angle >= M_PI_2 ? M_PI - angle : -M_PI - angle);
				e.roll = M_PI;
			}
		}
		else // Z_AXIS
		{
			e.yaw = angle;
			e.pitch = 0.0;
			e.roll = 0.0;
		}
	}

	// Conversion: Axis angle --> Euler angles
	void EulerFromAxis(const Vec3& axis, double angle, EulerAngles& e)
	{
		// Calculate the required Euler angles via the quaternion space
		Quat q;
		QuatFromAxis(axis, angle, q);
		EulerFromQuat(q, e);
	}

	// Conversion: Axis angle (unit axis) --> Fused angles
	void FusedFromAxis(UnitAxis axis, double angle, FusedAngles& f)
	{
		// Wrap the rotation angle to (-pi,pi]
		internal::picutVar(angle);

		// Calculate the required fused angles
		if(axis == X_AXIS)
		{
			if(fabs(angle) <= M_PI_2)
			{
				f.fusedYaw = 0.0;
				f.fusedPitch = 0.0;
				f.fusedRoll = angle;
				f.hemi = true;
			}
			else
			{
				f.fusedYaw = 0.0;
				f.fusedPitch = 0.0;
				f.fusedRoll = (angle >= M_PI_2 ? M_PI - angle : -M_PI - angle);
				f.hemi = false;
			}
		}
		else if(axis == Y_AXIS)
		{
			if(fabs(angle) <= M_PI_2)
			{
				f.fusedYaw = 0.0;
				f.fusedPitch = angle;
				f.fusedRoll = 0.0;
				f.hemi = true;
			}
			else
			{
				f.fusedYaw = 0.0;
				f.fusedPitch = (angle >= M_PI_2 ? M_PI - angle : -M_PI - angle);
				f.fusedRoll = 0.0;
				f.hemi = false;
			}
		}
		else // Z_AXIS
		{
			f.fusedYaw = angle;
			f.fusedPitch = 0.0;
			f.fusedRoll = 0.0;
			f.hemi = true;
		}
	}

	// Conversion: Axis angle --> Fused angles
	void FusedFromAxis(const Vec3& axis, double angle, FusedAngles& f)
	{
		// Calculate the required fused angles via the quaternion space
		Quat q;
		QuatFromAxis(axis, angle, q);
		FusedFromQuat(q, f);
	}

	// Conversion: Axis angle (unit axis) --> Tilt angles
	void TiltFromAxis(UnitAxis axis, double angle, TiltAngles& t)
	{
		// Wrap the rotation angle to (-pi,pi]
		internal::picutVar(angle);

		// Calculate the required fused angles
		if(axis == X_AXIS)
		{
			t.fusedYaw = 0.0;
			if(angle >= 0.0)
			{
				t.tiltAxisAngle = 0.0;
				t.tiltAngle = angle;
			}
			else
			{
				t.tiltAxisAngle = M_PI;
				t.tiltAngle = -angle;
			}
		}
		else if(axis == Y_AXIS)
		{
			t.fusedYaw = 0.0;
			if(angle >= 0.0)
			{
				t.tiltAxisAngle = M_PI_2;
				t.tiltAngle = angle;
			}
			else
			{
				t.tiltAxisAngle = -M_PI_2;
				t.tiltAngle = -angle;
			}
		}
		else // Z_AXIS
		{
			t.fusedYaw = angle;
			t.tiltAxisAngle = 0.0;
			t.tiltAngle = 0.0;
		}
	}

	// Conversion: Axis angle --> Tilt angles
	void TiltFromAxis(const Vec3& axis, double angle, TiltAngles& t)
	{
		// Calculate the required tilt angles via the quaternion space
		Quat q;
		QuatFromAxis(axis, angle, q);
		TiltFromQuat(q, t);
	}

	// Conversion: Axis angle (unit axis) --> Z vector
	void ZVecFromAxis(UnitAxis axis, double angle, ZVec& z)
	{
		// Precalculate trigonometric values
		double cang = cos(angle);
		double sang = sin(angle);

		// Calculate the required Z vector
		if(axis == X_AXIS)
		{
			z.x() = 0.0;
			z.y() = sang;
			z.z() = cang;
		}
		else if(axis == Y_AXIS)
		{
			z.x() = -sang;
			z.y() = 0.0;
			z.z() = cang;
		}
		else // Z_AXIS
		{
			z.x() = 0.0;
			z.y() = 0.0;
			z.z() = 1.0;
		}
	}

	// Conversion: Axis angle --> Z vector
	void ZVecFromAxis(const Vec3& axis, double angle, ZVec& z)
	{
		// Precalculate trigonometric values
		double cang = cos(angle);
		double sang = sin(angle);
		double zC = axis.z()*(1.0 - cang);

		// Calculate the required Z vector
		z.x() = axis.x()*zC - axis.y()*sang;
		z.y() = axis.y()*zC + axis.x()*sang;
		z.z() = axis.z()*zC + cang;
	}

	// ############################################
	// #### Conversions from rotation matrices ####
	// ############################################

	//
	// Conversion: Rotation matrix --> Quaternion
	//

	// Conversion: Rotation matrix --> Quaternion
	void QuatFromRotmat(const Rotmat& R, Quat& q)
	{
		// Perform the required conversion in a numerically stable manner
		double r, s, t = R.coeff(0,0) + R.coeff(1,1) + R.coeff(2,2);
		if(t >= 0.0)
		{
			r = sqrt(1.0 + t);
			s = 0.5/r;
			q.w() = 0.5*r;
			q.x() = s*(R.coeff(2,1) - R.coeff(1,2));
			q.y() = s*(R.coeff(0,2) - R.coeff(2,0));
			q.z() = s*(R.coeff(1,0) - R.coeff(0,1));
		}
		else if(R.coeff(2,2) >= R.coeff(1,1) && R.coeff(2,2) >= R.coeff(0,0))
		{
			r = sqrt(1.0 - (R.coeff(0,0) + R.coeff(1,1) - R.coeff(2,2)));
			s = 0.5/r;
			q.w() = s*(R.coeff(1,0) - R.coeff(0,1));
			q.x() = s*(R.coeff(0,2) + R.coeff(2,0));
			q.y() = s*(R.coeff(2,1) + R.coeff(1,2));
			q.z() = 0.5*r;
		}
		else if(R.coeff(1,1) >= R.coeff(0,0))
		{
			r = sqrt(1.0 - (R.coeff(0,0) - R.coeff(1,1) + R.coeff(2,2)));
			s = 0.5/r;
			q.w() = s*(R.coeff(0,2) - R.coeff(2,0));
			q.x() = s*(R.coeff(1,0) + R.coeff(0,1));
			q.y() = 0.5*r;
			q.z() = s*(R.coeff(2,1) + R.coeff(1,2));
		}
		else
		{
			r = sqrt(1.0 + (R.coeff(0,0) - R.coeff(1,1) - R.coeff(2,2)));
			s = 0.5/r;
			q.w() = s*(R.coeff(2,1) - R.coeff(1,2));
			q.x() = 0.5*r;
			q.y() = s*(R.coeff(1,0) + R.coeff(0,1));
			q.z() = s*(R.coeff(0,2) + R.coeff(2,0));
		}
	}

	//
	// Conversion: Rotation matrix --> Euler angles
	//

	// Conversion: Rotation matrix --> Euler angles
	void EulerFromRotmat(const Rotmat& R, double& yaw, double& pitch, double& roll)
	{
		// Calculate the sine of the pitch angle
		double sth = -R.coeff(2,0);
		sth = (sth >= 1.0 ? 1.0 : (sth <= -1.0 ? -1.0 : sth)); // Coerce sth to [-1,1]

		// Calculate the required Euler angles
		yaw = atan2(R.coeff(1,0), R.coeff(0,0));
		pitch = asin(sth);
		roll = atan2(R.coeff(2,1), R.coeff(2,2));
	}

	//
	// Conversion: Rotation matrix --> Fused angles
	//

	// Conversion: Rotation matrix --> Fused angles (2D)
	void FusedFromRotmat(const Rotmat& R, double& fusedPitch, double& fusedRoll)
	{
		// Calculate the fused pitch and roll
		double stheta = -R.coeff(2,0);
		double sphi   = R.coeff(2,1);
		stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
		sphi   = (sphi   >= 1.0 ? 1.0 : (sphi   <= -1.0 ? -1.0 : sphi  )); // Coerce sphi   to [-1,1]
		fusedPitch = asin(stheta);
		fusedRoll  = asin(sphi);
	}

	// Conversion: Rotation matrix --> Fused angles (3D)
	void FusedFromRotmat(const Rotmat& R, double& fusedYaw, double& fusedPitch, double& fusedRoll)
	{
		// Calculate the fused yaw, pitch and roll
		fusedYaw = FYawOfRotmat(R);
		FusedFromRotmat(R, fusedPitch, fusedRoll);
	}

	// Conversion: Rotation matrix --> Fused angles (4D)
	void FusedFromRotmat(const Rotmat& R, double& fusedYaw, double& fusedPitch, double& fusedRoll, bool& hemi)
	{
		// Calculate the fused yaw, pitch and roll
		fusedYaw = FYawOfRotmat(R);
		FusedFromRotmat(R, fusedPitch, fusedRoll);

		// Calculate the hemisphere of the rotation
		hemi = (R.coeff(2,2) >= 0.0);
	}

	//
	// Conversion: Rotation matrix --> Tilt angles
	//

	// Conversion: Rotation matrix --> Tilt angles (2D)
	void TiltFromRotmat(const Rotmat& R, double& tiltAxisAngle, double& tiltAngle)
	{
		// Calculate the tilt axis angle
		tiltAxisAngle = atan2(-R.coeff(2,0), R.coeff(2,1));

		// Calculate the tilt angle
		double calpha = R.coeff(2,2);
		calpha = (calpha >= 1.0 ? 1.0 : (calpha <= -1.0 ? -1.0 : calpha)); // Coerce calpha to [-1,1]
		tiltAngle = acos(calpha);
	}

	// Conversion: Rotation matrix --> Tilt angles (3D)
	void TiltFromRotmat(const Rotmat& R, double& fusedYaw, double& tiltAxisAngle, double& tiltAngle)
	{
		// Calculate the fused yaw, tilt axis angle and tilt angle
		fusedYaw = FYawOfRotmat(R);
		TiltFromRotmat(R, tiltAxisAngle, tiltAngle);
	}

	// ######################################
	// #### Conversions from quaternions ####
	// ######################################

	//
	// Conversion: Quaternion --> Axes
	//

	// Conversion: Quaternion --> X-axis
	void AxisXFromQuat(const Quat& q, Vec3& axis)
	{
		// Construct the required axis
		axis.x() = 1.0 - 2.0*(q.y()*q.y() + q.z()*q.z());
		axis.y() = 2.0*(q.x()*q.y() + q.z()*q.w());
		axis.z() = 2.0*(q.x()*q.z() - q.y()*q.w());
	}

	// Conversion: Quaternion --> Y-axis
	void AxisYFromQuat(const Quat& q, Vec3& axis)
	{
		// Construct the required axis
		axis.x() = 2.0*(q.x()*q.y() - q.z()*q.w());
		axis.y() = 1.0 - 2.0*(q.x()*q.x() + q.z()*q.z());
		axis.z() = 2.0*(q.y()*q.z() + q.x()*q.w());
	}

	// Conversion: Quaternion --> Z-axis
	void AxisZFromQuat(const Quat& q, Vec3& axis)
	{
		// Construct the required axis
		axis.x() = 2.0*(q.x()*q.z() + q.y()*q.w());
		axis.y() = 2.0*(q.y()*q.z() - q.x()*q.w());
		axis.z() = 1.0 - 2.0*(q.x()*q.x() + q.y()*q.y());
	}

	//
	// Conversion: Quaternion --> Rotation matrix
	//

	// Conversion: Quaternion --> Rotation matrix
	void RotmatFromQuat(const Quat& q, Rotmat& R)
	{
		// Construct the required rotation matrix
		R << 1.0 - 2.0*(q.y()*q.y() + q.z()*q.z()),       2.0*(q.x()*q.y() - q.z()*q.w()),       2.0*(q.x()*q.z() + q.y()*q.w()),
		           2.0*(q.x()*q.y() + q.z()*q.w()), 1.0 - 2.0*(q.x()*q.x() + q.z()*q.z()),       2.0*(q.y()*q.z() - q.x()*q.w()),
		           2.0*(q.x()*q.z() - q.y()*q.w()),       2.0*(q.y()*q.z() + q.x()*q.w()), 1.0 - 2.0*(q.x()*q.x() + q.y()*q.y());
	}

	//
	// Conversion: Quaternion --> Euler angles
	//

	// Conversion: Quaternion --> Euler angles
	void EulerFromQuat(const Quat& q, double& yaw, double& pitch, double& roll)
	{
		// Calculate the sine of the pitch angle
		double sth = 2.0*(q.w()*q.y() - q.x()*q.z());
		sth = (sth >= 1.0 ? 1.0 : (sth <= -1.0 ? -1.0 : sth)); // Coerce sth to [-1,1]

		// Calculate the required Euler angles
		double qysq = q.y()*q.y();
		yaw = atan2(q.x()*q.y() + q.z()*q.w(), 0.5 - (qysq + q.z()*q.z()));
		pitch = asin(sth);
		roll = atan2(q.y()*q.z() + q.x()*q.w(), 0.5 - (q.x()*q.x() + qysq));
	}

	//
	// Conversion: Quaternion --> Fused angles
	//

	// Conversion: Quaternion --> Fused angles (2D)
	void FusedFromQuat(const Quat& q, double& fusedPitch, double& fusedRoll)
	{
		// Calculate the fused pitch and roll
		double stheta = 2.0*(q.y()*q.w() - q.x()*q.z());
		double sphi   = 2.0*(q.y()*q.z() + q.x()*q.w());
		stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
		sphi   = (sphi   >= 1.0 ? 1.0 : (sphi   <= -1.0 ? -1.0 : sphi  )); // Coerce sphi   to [-1,1]
		fusedPitch = asin(stheta);
		fusedRoll  = asin(sphi);
	}

	// Conversion: Quaternion --> Fused angles (3D)
	void FusedFromQuat(const Quat& q, double& fusedYaw, double& fusedPitch, double& fusedRoll)
	{
		// Calculate the fused yaw, pitch and roll
		FusedFromQuat(q, fusedYaw);
		FusedFromQuat(q, fusedPitch, fusedRoll);
	}

	// Conversion: Quaternion --> Fused angles (4D)
	void FusedFromQuat(const Quat& q, double& fusedYaw, double& fusedPitch, double& fusedRoll, bool& hemi)
	{
		// Calculate the fused yaw, pitch and roll
		FusedFromQuat(q, fusedYaw);
		FusedFromQuat(q, fusedPitch, fusedRoll);

		// Calculate the hemisphere of the rotation
		hemi = (0.5 - (q.x()*q.x() + q.y()*q.y()) >= 0.0);
	}

	//
	// Conversion: Quaternion --> Tilt angles
	//

	// Conversion: Quaternion --> Tilt angles (2D)
	void TiltFromQuat(const Quat& q, double& tiltAxisAngle, double& tiltAngle)
	{
		// Calculate the tilt axis angle
		tiltAxisAngle = atan2(q.w()*q.y() - q.x()*q.z(), q.w()*q.x() + q.y()*q.z());

		// Calculate the tilt angle
		double calpha = 1.0 - 2.0*(q.x()*q.x() + q.y()*q.y());
		calpha = (calpha >= 1.0 ? 1.0 : (calpha <= -1.0 ? -1.0 : calpha)); // Coerce calpha to [-1,1]
		tiltAngle = acos(calpha);
	}

	// Conversion: Quaternion --> Tilt angles (3D)
	void TiltFromQuat(const Quat& q, double& fusedYaw, double& tiltAxisAngle, double& tiltAngle)
	{
		// Calculate the fused yaw, tilt axis angle and tilt angle
		FusedFromQuat(q, fusedYaw);
		TiltFromQuat(q, tiltAxisAngle, tiltAngle);
	}

	//
	// Conversion: Quaternion --> Z vector
	//

	// Conversion: Quaternion --> Z vector
	void ZVecFromQuat(const Quat& q, ZVec& z)
	{
		// Calculate the required Z vector
		z.x() = 2.0*(q.x()*q.z() - q.y()*q.w());
		z.y() = 2.0*(q.y()*q.z() + q.x()*q.w());
		z.z() = 1.0 - 2.0*(q.x()*q.x() + q.y()*q.y());
	}

	// #######################################
	// #### Conversions from Euler angles ####
	// #######################################

	// Conversion: Euler angles --> Rotation matrix
	Rotmat RotmatFromEuler(double yaw, double pitch, double roll)
	{
		// Precalculate the trigonometric values
		double cpsi = cos(yaw);
		double spsi = sin(yaw);
		double cth  = cos(pitch);
		double sth  = sin(pitch);
		double cphi = cos(roll);
		double sphi = sin(roll);

		// Calculate and return the required rotation matrix
		Rotmat R;
		R << cpsi*cth, cpsi*sth*sphi - spsi*cphi, cpsi*sth*cphi + spsi*sphi,
		     spsi*cth, spsi*sth*sphi + cpsi*cphi, spsi*sth*cphi - cpsi*sphi,
		         -sth,                  cth*sphi,                  cth*cphi;
		return R;
	}

	// Conversion: Euler angles --> Quaternion
	Quat QuatFromEuler(double yaw, double pitch, double roll)
	{
		// Halve the Euler angles
		double hpsi = 0.5*yaw;
		double hth = 0.5*pitch;
		double hphi = 0.5*roll;

		// Precalculate the trigonometric values
		double hcpsi = cos(hpsi);
		double hspsi = sin(hpsi);
		double hcth  = cos(hth);
		double hsth  = sin(hth);
		double hcphi = cos(hphi);
		double hsphi = sin(hphi);

		// Calculate and return the required quaternion
		return Quat(hcphi*hcth*hcpsi + hsphi*hsth*hspsi, hsphi*hcth*hcpsi - hcphi*hsth*hspsi, hcphi*hsth*hcpsi + hsphi*hcth*hspsi, hcphi*hcth*hspsi - hsphi*hsth*hcpsi); // Order: (w,x,y,z)
	}

	// Conversion: Euler angles --> Fused angles
	FusedAngles FusedFromEuler(double yaw, double pitch, double roll)
	{
		// Construct a fused angles object
		FusedAngles f;

		// Calculation of the fused yaw in a numerically stable manner requires the complete rotation matrix representation
		Rotmat R = RotmatFromEuler(yaw, pitch, roll);

		// Calculate the fused yaw
		f.fusedYaw = FYawOfRotmat(R);

		// Calculate the fused pitch
		f.fusedPitch = pitch; // ZYX Euler pitch is equivalent to fused pitch!

		// Calculate the fused roll
		double sphi = R.coeff(2,1);
		sphi = (sphi >= 1.0 ? 1.0 : (sphi <= -1.0 ? -1.0 : sphi)); // Coerce sphi to [-1,1]
		f.fusedRoll  = asin(sphi);

		// See which hemisphere we're in
		f.hemi = (R.coeff(2,2) >= 0.0);

		// Return the calculated fused angles
		return f;
	}

	// Conversion: Euler angles --> Tilt angles
	TiltAngles TiltFromEuler(double yaw, double pitch, double roll)
	{
		// Construct a tilt angles object
		TiltAngles t;

		// Calculation of the fused yaw in a numerically stable manner requires the complete rotation matrix representation
		Rotmat R = RotmatFromEuler(yaw, pitch, roll);

		// Calculate the fused yaw
		t.fusedYaw = FYawOfRotmat(R);

		// Calculate the tilt axis angle
		t.tiltAxisAngle = atan2(-R.coeff(2,0), R.coeff(2,1));

		// Calculate the tilt angle
		double calpha = R.coeff(2,2);
		calpha = (calpha >= 1.0 ? 1.0 : (calpha <= -1.0 ? -1.0 : calpha)); // Coerce calpha to [-1,1]
		t.tiltAngle = acos(calpha);

		// Return the calculated tilt angles
		return t;
	}

	// Conversion: Euler angles --> Z vector
	ZVec ZVecFromEuler(double pitch, double roll)
	{
		// Precalculate the trigonometric values
		double cth  = cos(pitch);
		double sth  = sin(pitch);
		double cphi = cos(roll);
		double sphi = sin(roll);

		// Calculate and return the required Z vector
		return ZVec(-sth, cth*sphi, cth*cphi);
	}

	// #######################################
	// #### Conversions from fused angles ####
	// #######################################

	//
	// Conversion: Fused angles --> Rotation matrix
	//

	// Conversion: Fused angles (2D) --> Rotation matrix
	Rotmat RotmatFromFused(double fusedPitch, double fusedRoll) // Assume: fusedYaw = 0, hemi = true
	{
		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the sine sum criterion
		double crit = sth*sth + sphi*sphi;
		double calpha = (crit >= 1.0 ? 0.0 : sqrt(1.0 - crit));

		// Calculate the tilt axis angle gamma
		double gamma = atan2(sth,sphi);

		// Precalculate terms involved in the rotation matrix expression
		double calphabar = 1.0 - calpha;
		double cgam = cos(gamma);
		double sgam = sin(gamma);
		double A = calpha + calphabar*cgam*cgam;
		double B = calpha + calphabar*sgam*sgam;
		double C = calphabar*cgam*sgam;

		// Calculate and return the required rotation matrix
		Rotmat R;
		R <<    A,    C,    sth,
		        C,    B,  -sphi,
		     -sth, sphi, calpha;
		return R;
	}

	// Conversion: Fused angles (3D/4D) --> Rotation matrix
	Rotmat RotmatFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi)
	{
		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the sine sum criterion
		double crit = sth*sth + sphi*sphi;

		// Calculate the tilt angle alpha
		double calpha, salpha;
		if(crit >= 1.0)
		{
			calpha = 0.0;
			salpha = 1.0;
		}
		else
		{
			calpha = (hemi ? sqrt(1.0 - crit) : -sqrt(1.0 - crit));
			salpha = sqrt(crit);
		}

		// Calculate the tilt axis angle gamma
		double gamma = atan2(sth,sphi);

		// Precalculate terms involved in the rotation matrix expression
		double cgam = cos(gamma);
		double sgam = sin(gamma);
		double psigam = fusedYaw + gamma;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);
		double A = cgam*cpsigam;
		double B = sgam*cpsigam;
		double C = cgam*spsigam;
		double D = sgam*spsigam;

		// Calculate and return the required rotation matrix
		Rotmat R;
		R << A + D*calpha, B - C*calpha,  salpha*spsigam,
		     C - B*calpha, D + A*calpha, -salpha*cpsigam,
		          -sth,          sphi,    calpha;
		return R;
	}

	//
	// Conversion: Fused angles --> Quaternion
	//

	// Conversion: Fused angles (2D) --> Quaternion
	Quat QuatFromFused(double fusedPitch, double fusedRoll) // Assume: fusedYaw = 0, hemi = true
	{
		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the sine sum criterion
		double crit = sth*sth + sphi*sphi;

		// Calculate the tilt angle alpha
		double alpha = (crit >= 1.0 ? M_PI_2 : acos(sqrt(1.0 - crit)));
		double halpha = 0.5*alpha;
		double chalpha = cos(halpha);
		double shalpha = sin(halpha);

		// Calculate the tilt axis angle gamma
		double gamma = atan2(sth,sphi);
		double cgamma = cos(gamma);
		double sgamma = sin(gamma);

		// Return the required quaternion orientation (a rotation about (cgamma, sgamma, 0) by angle alpha)
		return Quat(chalpha, cgamma*shalpha, sgamma*shalpha, 0.0); // Order: (w,x,y,z)
	}

	// Conversion: Fused angles (3D/4D) --> Quaternion
	Quat QuatFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi)
	{
		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the sine sum criterion
		double crit = sth*sth + sphi*sphi;

		// Calculate the tilt angle alpha
		double alpha;
		if(crit >= 1.0)
			alpha = M_PI_2;
		else
			alpha = acos(hemi ? sqrt(1.0 - crit) : -sqrt(1.0 - crit));

		// Calculate the tilt axis angle gamma
		double gamma = atan2(sth,sphi);

		// Evaluate the required intermediate angles
		double halpha = 0.5*alpha;
		double hpsi = 0.5*fusedYaw;
		double hgampsi = gamma + hpsi;

		// Precalculate trigonometric terms involved in the quaternion expression
		double chalpha = cos(halpha);
		double shalpha = sin(halpha);
		double chpsi = cos(hpsi);
		double shpsi = sin(hpsi);
		double chgampsi = cos(hgampsi);
		double shgampsi = sin(hgampsi);

		// Calculate and return the required quaternion
		return Quat(chalpha*chpsi, shalpha*chgampsi, shalpha*shgampsi, chalpha*shpsi); // Order: (w,x,y,z)
	}

	//
	// Conversion: Fused angles --> Euler angles
	//

	// Conversion: Fused angles (2D) --> Euler angles
	EulerAngles EulerFromFused(double fusedPitch, double fusedRoll) // Assume: fusedYaw = 0, hemi = true
	{
		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the cosine of the tilt angle
		double crit = sth*sth + sphi*sphi;
		double calpha = (crit >= 1.0 ? 0.0 : sqrt(1.0 - crit));

		// Calculate the tilt axis angle gamma
		double gamma = atan2(sth,sphi);

		// Precalculate terms
		double cgam = cos(gamma);
		double sgam = sin(gamma);
		double A = cgam*(1.0 - calpha);

		// Calculate and return the required Euler angles
		return EulerAngles(atan2(A*sgam, calpha + A*cgam), fusedPitch, atan2(sphi, calpha));
	}

	// Conversion: Fused angles (3D/4D) --> Euler angles
	EulerAngles EulerFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi)
	{
		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the cosine of the tilt angle
		double crit = sth*sth + sphi*sphi;
		double calpha = 0.0;
		if(crit < 1.0)
			calpha = (hemi ? sqrt(1.0 - crit) : -sqrt(1.0 - crit));

		// Calculate the tilt axis angle gamma
		double gamma = atan2(sth,sphi);

		// Precalculate terms
		double cgam = cos(gamma);
		double sgam = sin(gamma);
		double psigam = fusedYaw + gamma;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);
		double A = sgam*calpha;

		// Calculate and return the required Euler angles
		return EulerAngles(atan2(cgam*spsigam - A*cpsigam, cgam*cpsigam + A*spsigam), fusedPitch, atan2(sphi, calpha));
	}

	//
	// Conversion: Fused angles --> Tilt angles
	//

	// Conversion: Fused angles (2D) --> Tilt angles
	TiltAngles TiltFromFused(double fusedPitch, double fusedRoll) // Assume: fusedYaw = 0, hemi = true
	{
		// Construct a tilt angles object
		TiltAngles t;

		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the cosine of the tilt angle
		double crit = sth*sth + sphi*sphi;
		double calpha = (crit >= 1.0 ? 0.0 : sqrt(1.0 - crit));

		// Calculate and return the tilt angles representation
		t.fusedYaw = 0.0;
		t.tiltAxisAngle = atan2(sth,sphi);
		t.tiltAngle = acos(calpha);
		return t;
	}

	// Conversion: Fused angles (3D/4D) --> Tilt angles
	TiltAngles TiltFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi)
	{
		// Construct a tilt angles object
		TiltAngles t;

		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the cosine of the tilt angle
		double crit = sth*sth + sphi*sphi;
		double calpha = 0.0;
		if(crit < 1.0)
			calpha = (hemi ? sqrt(1.0 - crit) : -sqrt(1.0 - crit));

		// Calculate and return the tilt angles representation
		t.fusedYaw = fusedYaw;
		t.tiltAxisAngle = atan2(sth,sphi);
		t.tiltAngle = acos(calpha);
		return t;
	}

	// Conversion: Fused angles (2D) --> Tilt angle component
	double TiltAngleFromFused(double fusedPitch, double fusedRoll)
	{
		// Precalculate the sine values
		double sth = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the cosine of the tilt angle
		double crit = sth*sth + sphi*sphi;
		double calpha = (crit >= 1.0 ? 0.0 : sqrt(1.0 - crit));

		// Calculate and return the tilt angle component of the tilt angles representation
		return acos(calpha);
	}

	//
	// Conversion: Fused angles --> Z vector
	//

	// Conversion: Fused angles --> Z vector
	ZVec ZVecFromFused(double fusedPitch, double fusedRoll, bool hemi)
	{
		// Precalculate the sin values
		double sth  = sin(fusedPitch);
		double sphi = sin(fusedRoll);

		// Calculate the cosine of the tilt angle
		double crit = sth*sth + sphi*sphi;
		double calpha = 0.0;
		if(crit < 1.0)
			calpha = (hemi ? sqrt(1.0 - crit) : -sqrt(1.0 - crit));

		// Return the required Z vector
		return ZVec(-sth, sphi, calpha);
	}

	// ######################################
	// #### Conversions from tilt angles ####
	// ######################################

	//
	// Conversion: Tilt angles --> Rotation matrix
	//

	// Conversion: Tilt angles (2D) --> Rotation matrix
	Rotmat RotmatFromTilt(double tiltAxisAngle, double tiltAngle) // Assume: fusedYaw = 0
	{
		// Precalculate terms involved in the rotation matrix expression
		double cgam = cos(tiltAxisAngle);
		double sgam = sin(tiltAxisAngle);
		double calpha = cos(tiltAngle);
		double salpha = sin(tiltAngle);
		double calphabar = 1.0 - calpha;
		double sth = salpha*sgam;
		double sphi = salpha*cgam;
		double A = calpha + calphabar*cgam*cgam;
		double B = calpha + calphabar*sgam*sgam;
		double C = calphabar*cgam*sgam;

		// Calculate and return the required rotation matrix
		Rotmat R;
		R <<    A,    C,    sth,
		        C,    B,  -sphi,
		     -sth, sphi, calpha;
		return R;
	}

	// Conversion: Tilt angles (3D) --> Rotation matrix
	Rotmat RotmatFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle)
	{
		// Precalculate terms involved in the rotation matrix expression
		double cgam = cos(tiltAxisAngle);
		double sgam = sin(tiltAxisAngle);
		double calpha = cos(tiltAngle);
		double salpha = sin(tiltAngle);
		double psigam = fusedYaw + tiltAxisAngle;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);
		double A = cgam*cpsigam;
		double B = sgam*cpsigam;
		double C = cgam*spsigam;
		double D = sgam*spsigam;

		// Calculate and return the required rotation matrix
		Rotmat R;
		R << A + D*calpha, B - C*calpha,  salpha*spsigam,
		     C - B*calpha, D + A*calpha, -salpha*cpsigam,
		     -sgam*salpha,  cgam*salpha,  calpha;
		return R;
	}

	//
	// Conversion: Tilt angles --> Quaternion
	//

	// Conversion: Tilt angles (2D) --> Quaternion
	Quat QuatFromTilt(double tiltAxisAngle, double tiltAngle) // Assume: fusedYaw = 0
	{
		// Precalculate the required angles
		double halpha = 0.5*tiltAngle;

		// Precalculate the required trigonometric values
		double chalpha = cos(halpha);
		double shalpha = sin(halpha);
		double cgamma = cos(tiltAxisAngle);
		double sgamma = sin(tiltAxisAngle);

		// Return the required quaternion orientation
		return Quat(chalpha, shalpha*cgamma, shalpha*sgamma, 0.0); // Order: (w,x,y,z)
	}

	// Conversion: Tilt angles (3D) --> Quaternion
	Quat QuatFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle)
	{
		// Precalculate the required angles
		double hpsi = 0.5*fusedYaw;
		double halpha = 0.5*tiltAngle;
		double hgampsi = tiltAxisAngle + hpsi;

		// Precalculate the required trigonometric values
		double chpsi = cos(hpsi);
		double shpsi = sin(hpsi);
		double chalpha = cos(halpha);
		double shalpha = sin(halpha);
		double chgampsi = cos(hgampsi);
		double shgampsi = sin(hgampsi);

		// Return the required quaternion orientation
		return Quat(chalpha*chpsi, shalpha*chgampsi, shalpha*shgampsi, chalpha*shpsi); // Order: (w,x,y,z)
	}

	//
	// Conversion: Tilt angles --> Euler angles
	//

	// Conversion: Tilt angles (2D) --> Euler angles
	EulerAngles EulerFromTilt(double tiltAxisAngle, double tiltAngle) // Assume: fusedYaw = 0
	{
		// Precalculate terms
		double cgam = cos(tiltAxisAngle);
		double sgam = sin(tiltAxisAngle);
		double calpha = cos(tiltAngle);
		double salpha = sin(tiltAngle);
		double sth = sgam*salpha;
		double sphi = cgam*salpha;
		double A = cgam*(1.0 - calpha);

		// Calculate and return the required Euler angles
		return EulerAngles(atan2(A*sgam, calpha + A*cgam), asin(sth), atan2(sphi, calpha));
	}

	// Conversion: Tilt angles (3D) --> Euler angles
	EulerAngles EulerFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle)
	{
		// Precalculate terms
		double cgam = cos(tiltAxisAngle);
		double sgam = sin(tiltAxisAngle);
		double calpha = cos(tiltAngle);
		double salpha = sin(tiltAngle);
		double sth = sgam*salpha;
		double sphi = cgam*salpha;
		double psigam = fusedYaw + tiltAxisAngle;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);
		double A = sgam*calpha;

		// Calculate and return the required Euler angles
		return EulerAngles(atan2(cgam*spsigam - A*cpsigam, cgam*cpsigam + A*spsigam), asin(sth), atan2(sphi, calpha));
	}

	//
	// Conversion: Tilt angles --> Fused angles
	//

	// Conversion: Tilt angles (2D) --> Fused angles
	FusedAngles FusedFromTilt(double tiltAxisAngle, double tiltAngle) // Assume: fusedYaw = 0
	{
		// Calculate and return the fused angles representation
		return FusedFromTilt(0.0, tiltAxisAngle, tiltAngle);
	}

	// Conversion: Tilt angles (3D) --> Fused angles
	FusedAngles FusedFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle)
	{
		// Construct a fused angles object
		FusedAngles f;

		// Precalculate terms
		double cgam = cos(tiltAxisAngle);
		double sgam = sin(tiltAxisAngle);
		double salpha = sin(tiltAngle);

		// Calculate and return the fused angles representation
		f.fusedYaw = fusedYaw;
		f.fusedPitch = asin(salpha*sgam);
		f.fusedRoll = asin(salpha*cgam);
		f.hemi = (tiltAngle <= M_PI_2);
		return f;
	}

	//
	// Conversion: Tilt angles --> Z vector
	//

	// Conversion: Tilt angles --> Z vector
	ZVec ZVecFromTilt(double tiltAxisAngle, double tiltAngle)
	{
		// Precalculate the required trigonometric terms
		double cgamma = cos(tiltAxisAngle);
		double sgamma = sin(tiltAxisAngle);
		double calpha = cos(tiltAngle);
		double salpha = sin(tiltAngle);

		// Return the required Z vector
		return ZVec(-salpha*sgamma, salpha*cgamma, calpha);
	}

	// ####################################
	// #### Conversions from Z vectors ####
	// ####################################

	//
	// Conversion: Z vector --> Rotation matrix (zero fused yaw)
	//

	// Conversion: Z vector --> Rotation matrix
	void RotmatFromZVec(const ZVec& z, Rotmat& R)
	{
		// Perform the conversion via a quaternion
		Quat q;
		QuatFromZVec(z, q);
		RotmatFromQuat(q, R);
	}

	//
	// Conversion: Z vector --> Quaternion (zero fused yaw)
	//

	// Conversion: Z vector --> Quaternion
	void QuatFromZVec(const ZVec& z, Quat& q)
	{
		// Calculate the z component
		q.z() = 0.0; // Zero fused yaw is equivalent to a quaternion z component of zero!

		// Calculate the w component
		double wsq = 0.5*(1.0 + z.z());
		wsq = (wsq >= 1.0 ? 1.0 : (wsq <= 0.0 ? 0.0 : wsq)); // Coerce wsq to [0,1]
		q.w() = sqrt(wsq);

		// Calculate the x and y components
		double xsqplusysq = 1.0 - wsq;
		double xtilde = z.y();
		double ytilde = -z.x();
		double xytildenormsq = xtilde*xtilde + ytilde*ytilde;
		if(xytildenormsq <= 0.0)
		{
			q.x() = sqrt(xsqplusysq);
			q.y() = 0.0;
		}
		else
		{
			double factor = sqrt(xsqplusysq / xytildenormsq);
			q.x() = factor * xtilde;
			q.y() = factor * ytilde;
		}
	}

	//
	// Conversion: Z vector --> Euler angles (zero Euler yaw)
	//

	// Conversion: Z vector --> Euler angles
	void EulerFromZVec(const ZVec& z, double& pitch, double& roll)
	{
		// Calculate the pitch
		double stheta = -z.x();
		stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
		pitch = asin(stheta);

		// Calculate the roll
		roll = atan2(z.y(), z.z());
	}

	//
	// Conversion: Z vector --> Fused angles (zero fused yaw)
	//

	// Conversion: Z vector --> Fused angles (2D)
	void FusedFromZVec(const ZVec& z, double& fusedPitch, double& fusedRoll)
	{
		// Calculate the fused pitch and roll
		double stheta = -z.x();
		double sphi   = z.y();
		stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
		sphi   = (sphi   >= 1.0 ? 1.0 : (sphi   <= -1.0 ? -1.0 : sphi  )); // Coerce sphi   to [-1,1]
		fusedPitch = asin(stheta);
		fusedRoll  = asin(sphi);
	}

	// Conversion: Z vector --> Fused angles (3D)
	void FusedFromZVec(const ZVec& z, double& fusedPitch, double& fusedRoll, bool& hemi)
	{
		// Calculate the fused pitch and roll
		FusedFromZVec(z, fusedPitch, fusedRoll);

		// Calculate the hemisphere
		hemi = (z.z() >= 0.0);
	}

	//
	// Conversion: Z vector --> Tilt angles (zero fused yaw)
	//

	// Conversion: Z vector --> Tilt angles (2D)
	void TiltFromZVec(const ZVec& z, double& tiltAxisAngle, double& tiltAngle)
	{
		// Calculate the tilt axis angle
		tiltAxisAngle = atan2(-z.x(), z.y());

		// Calculate the tilt angle
		double calpha = z.z();
		calpha = (calpha >= 1.0 ? 1.0 : (calpha <= -1.0 ? -1.0 : calpha)); // Coerce calpha to [-1,1]
		tiltAngle = acos(calpha);
	}

	// #########################################
	// #### Conversions from yaw and z-axis ####
	// #########################################

	// Conversion: Yaw and z-axis (BzG) --> Rotation matrix
	void RotmatFromFYawBzG(double fusedYaw, const Vec3& BzG, Rotmat& RGB)
	{
		// Calculate the quaternion representation of the required rotation
		Quat qGB;
		QuatFromFYawBzG(fusedYaw, BzG, qGB);

		// Return the required rotation matrix representation
		RotmatFromQuat(qGB, RGB);
	}

	// Conversion: Yaw and z-axis (GzB) --> Rotation matrix
	void RotmatFromFYawGzB(double fusedYaw, const Vec3& GzB, Rotmat& RGB)
	{
		// Calculate the quaternion representation of the required rotation
		Quat qGB;
		QuatFromFYawGzB(fusedYaw, GzB, qGB);

		// Return the required rotation matrix representation
		RotmatFromQuat(qGB, RGB);
	}

	// Conversion: Yaw and z-axis (BzG) --> Quaternion
	void QuatFromFYawBzG(double fusedYaw, const Vec3& BzG, Quat& qGB)
	{
		// Precalculate trigonometric terms
		double chpsi = cos(0.5*fusedYaw);
		double shpsi = sin(0.5*fusedYaw);

		// Calculate the w and z components
		double wsqpluszsq = 0.5*(1 + BzG.z());
		wsqpluszsq = (wsqpluszsq >= 1.0 ? 1.0 : (wsqpluszsq <= 0.0 ? 0.0 : wsqpluszsq)); // Coerce wsqpluszsq to [0,1]
		double wznorm = sqrt(wsqpluszsq);
		qGB.w() = wznorm * chpsi;
		qGB.z() = wznorm * shpsi;

		// Calculate the x and y components
		double xsqplusysq = 1.0 - wsqpluszsq;
		double xtilde = BzG.x()*qGB.z() + BzG.y()*qGB.w();
		double ytilde = BzG.y()*qGB.z() - BzG.x()*qGB.w();
		double xytildenormsq = xtilde*xtilde + ytilde*ytilde;
		if(xytildenormsq <= 0.0)
		{
			qGB.x() = sqrt(xsqplusysq);
			qGB.y() = 0.0;
		}
		else
		{
			double factor = sqrt(xsqplusysq / xytildenormsq);
			qGB.x() = factor * xtilde;
			qGB.y() = factor * ytilde;
		}
	}

	// Conversion: Yaw and z-axis (GzB) --> Quaternion
	void QuatFromFYawGzB(double fusedYaw, const Vec3& GzB, Quat& qGB)
	{
		// Precalculate trigonometric terms
		double chpsi = cos(0.5*fusedYaw);
		double shpsi = sin(0.5*fusedYaw);

		// Calculate the w and z components
		double wsqpluszsq = 0.5*(1 + GzB.z());
		wsqpluszsq = (wsqpluszsq >= 1.0 ? 1.0 : (wsqpluszsq <= 0.0 ? 0.0 : wsqpluszsq)); // Coerce wsqpluszsq to [0,1]
		double wznorm = sqrt(wsqpluszsq);
		qGB.w() = wznorm * chpsi;
		qGB.z() = wznorm * shpsi;

		// Calculate the x and y components
		double xsqplusysq = 1.0 - wsqpluszsq;
		double xtilde = GzB.x()*qGB.z() - GzB.y()*qGB.w();
		double ytilde = GzB.y()*qGB.z() + GzB.x()*qGB.w();
		double xytildenormsq = xtilde*xtilde + ytilde*ytilde;
		if(xytildenormsq <= 0.0)
		{
			qGB.x() = sqrt(xsqplusysq);
			qGB.y() = 0.0;
		}
		else
		{
			double factor = sqrt(xsqplusysq / xytildenormsq);
			qGB.x() = factor * xtilde;
			qGB.y() = factor * ytilde;
		}
	}

	// Conversion: Yaw and z-axis --> Euler angles
	void EulerFromFYawBzG(double fusedYaw, const Vec3& BzG, EulerAngles& eGB)
	{
		// Calculate the Euler pitch
		double stheta = -BzG.x();
		stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
		eGB.pitch = asin(stheta);

		// Calculate the Euler roll
		eGB.roll = atan2(BzG.y(), BzG.z());

		// Calculate the Euler ZYX yaw
		if(stheta == 0.0 && BzG.y() == 0.0)
			eGB.yaw = fusedYaw;
		else
		{
			double cphi = cos(eGB.roll);
			double sphi = sin(eGB.roll);
			eGB.yaw = fusedYaw + atan2(sphi, stheta*cphi) - atan2(BzG.y(), stheta);
		}
		internal::picutVar(eGB.yaw);
	}

	// Conversion: Yaw and z-axis --> Euler angles
	void EulerFromFYawGzB(double fusedYaw, const Vec3& GzB, EulerAngles& eGB)
	{
		// Precalculate trigonometric terms
		double cfyaw = cos(fusedYaw);
		double sfyaw = sin(fusedYaw);

		// Calculate the Euler pitch
		double stheta = cfyaw*GzB.x() + sfyaw*GzB.y();
		stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
		eGB.pitch = asin(stheta);

		// Calculate the Euler roll
		double sfphi = sfyaw*GzB.x() - cfyaw*GzB.y();
		eGB.roll = atan2(sfphi, GzB.z());

		// Calculate the Euler ZYX yaw
		if(stheta == 0.0 && sfphi == 0.0)
			eGB.yaw = fusedYaw;
		else
		{
			double cphi = cos(eGB.roll);
			double sphi = sin(eGB.roll);
			eGB.yaw = fusedYaw + atan2(sphi, stheta*cphi) - atan2(sfphi, stheta);
		}
		internal::picutVar(eGB.yaw);
	}

	// Conversion: Yaw and z-axis --> Fused angles
	void FusedFromFYawBzG(double fusedYaw, const Vec3& BzG, FusedAngles& fGB)
	{
		// Transcribe and wrap the fused yaw
		fGB.fusedYaw = internal::picut(fusedYaw);

		// Calculate the fused pitch and roll
		double stheta = -BzG.x();
		double sphi   = BzG.y();
		stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
		sphi   = (sphi   >= 1.0 ? 1.0 : (sphi   <= -1.0 ? -1.0 : sphi  )); // Coerce sphi   to [-1,1]
		fGB.fusedPitch = asin(stheta);
		fGB.fusedRoll  = asin(sphi);

		// Calculate the hemisphere
		fGB.hemi = (BzG.z() >= 0.0);
	}

	// Conversion: Yaw and z-axis --> Fused angles
	void FusedFromFYawGzB(double fusedYaw, const Vec3& GzB, FusedAngles& fGB)
	{
		// Transcribe and wrap the fused yaw
		fGB.fusedYaw = internal::picut(fusedYaw);

		// Precalculate trigonometric terms
		double cpsi = cos(fGB.fusedYaw);
		double spsi = sin(fGB.fusedYaw);

		// Calculate the fused pitch and roll
		double stheta = cpsi*GzB.x() + spsi*GzB.y();
		double sphi   = spsi*GzB.x() - cpsi*GzB.y();
		stheta = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta)); // Coerce stheta to [-1,1]
		sphi   = (sphi   >= 1.0 ? 1.0 : (sphi   <= -1.0 ? -1.0 : sphi  )); // Coerce sphi   to [-1,1]
		fGB.fusedPitch = asin(stheta);
		fGB.fusedRoll  = asin(sphi);

		// Calculate the hemisphere
		fGB.hemi = (GzB.z() >= 0.0);
	}

	// Conversion: Yaw and z-axis --> Tilt angles
	void TiltFromFYawBzG(double fusedYaw, const Vec3& BzG, TiltAngles& tGB)
	{
		// Transcribe and wrap the fused yaw
		tGB.fusedYaw = internal::picut(fusedYaw);

		// Calculate the tilt axis angle
		tGB.tiltAxisAngle = atan2(-BzG.x(), BzG.y());

		// Calculate the tilt angle
		double calpha = BzG.z();
		calpha = (calpha >= 1.0 ? 1.0 : (calpha <= -1.0 ? -1.0 : calpha)); // Coerce calpha to [-1,1]
		tGB.tiltAngle = acos(calpha);
	}

	// Conversion: Yaw and z-axis --> Tilt angles
	void TiltFromFYawGzB(double fusedYaw, const Vec3& GzB, TiltAngles& tGB)
	{
		// Transcribe and wrap the fused yaw
		tGB.fusedYaw = internal::picut(fusedYaw);

		// Calculate the tilt axis angle
		if(GzB.x() == 0.0 && GzB.y() == 0.0)
			tGB.tiltAxisAngle = 0.0;
		else
			tGB.tiltAxisAngle = internal::picut(atan2(GzB.x(), -GzB.y()) - tGB.fusedYaw);

		// Calculate the tilt angle
		double calpha = GzB.z();
		calpha = (calpha >= 1.0 ? 1.0 : (calpha <= -1.0 ? -1.0 : calpha)); // Coerce calpha to [-1,1]
		tGB.tiltAngle = acos(calpha);
	}

	// ########################################
	// #### Spherical Linear Interpolation ####
	// ########################################

	// Slerp: Quaternion
	Quat QuatSlerp(const Quat& q0, const Quat& q1, double u)
	{
		// Calculate the dot product of the two quaternions
		double dprod = q0.w()*q1.w() + q0.x()*q1.x() + q0.y()*q1.y() + q0.z()*q1.z();

		// Adjust for the situation that two quaternions in different hemispheres are being interpolated
		double q1sign = 1.0;
		if(dprod < 0.0)
		{
			dprod = -dprod;
			q1sign = -1.0;
		}

		// If q0 and q1 are very close then just use linear interpolation, otherwise use spherical linear interpolation
		Quat qu;
		if(dprod >= 1.0 - 5e-9) // A dot product within this tolerance of unity produces a negligible amount of error if using linear interpolation instead
		{
			// Perform the required interpolation
			qu = (1.0 - u)*q0 + (u*q1sign)*q1;
		}
		else
		{
			// Calculate half the angle between the two quaternions
			double htheta = acos(dprod);

			// Perform the required interpolation
			qu = sin((1.0 - u)*htheta)*q0 + (sin(u*htheta)*q1sign)*q1;
		}

		// Normalise the interpolated quaternion
		NormaliseQuat(qu);

		// Return the interpolated quaternion
		return qu;
	}

	// Slerp: Quaternion scaling
	Quat QuatSlerp(const Quat& q, double u)
	{
		// Ensure the w component is non-negative
		Quat qu = (q.w() < 0.0 ? -q : q);

		// If q is a very small rotation then just use linear interpolation, otherwise use spherical linear interpolation
		if(qu.w() >= 1.0 - 5e-9) // A w component within this tolerance of unity produces a negligible amount of error if using linear interpolation instead
		{
			// Perform the required interpolation
			qu *= u;
			qu.w() += (1.0 - u);
		}
		else
		{
			// Calculate half the angle magnitude of the quaternion
			double htheta = acos(qu.w());

			// Perform the required interpolation
			qu *= sin(u*htheta);
			qu.w() += sin((1.0 - u)*htheta);
		}

		// Normalise the interpolated quaternion
		NormaliseQuat(qu);

		// Return the interpolated quaternion
		return qu;
	}

	// Slerp: Unit vector
	Vec3 VecSlerp(const Vec3& v0, const Vec3& v1, double u)
	{
		// Normalise the input vectors
		Vec3 v0hat = NormalisedVec(v0);
		Vec3 v1hat = NormalisedVec(v1);

		// Calculate the dot product of the two vectors
		double dprod = v0hat.x()*v1hat.x() + v0hat.y()*v1hat.y() + v0hat.z()*v1hat.z();

		// If v0hat and v1hat are very close then just use linear interpolation, otherwise use spherical linear interpolation
		Vec3 vu;
		if(dprod >= 1.0 - 5e-9) // A dot product within this tolerance of unity produces a negligible amount of error if using linear interpolation instead
		{
			// Perform the required interpolation
			vu = (1.0 - u)*v0hat + u*v1hat;
		}
		else
		{
			// Calculate half the angle between the two quaternions
			double htheta = acos(dprod);

			// Perform the required interpolation
			vu = sin((1.0 - u)*htheta)*v0hat + sin(u*htheta)*v1hat;
		}

		// Normalise the interpolated vector
		NormaliseVec(vu);

		// Return the interpolated vector
		return vu;
	}

	// ##############################
	// #### Tilt phase functions ####
	// ##############################

	// Conversion: Tilt phase velocity 2D --> Angular velocity (assumes zero pzVel)
	void AngFromTiltPhaseVel(const TiltPhaseVel2D& pdot, const TiltAngles& t, AngVel& angVel)
	{
		// Precalculate trigonometric values
		double cgamma = cos(t.tiltAxisAngle);
		double sgamma = sin(t.tiltAxisAngle);
		double psigam = t.fusedYaw + t.tiltAxisAngle;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);

		// Precalculate additional terms
		double S, C;
		if(t.tiltAngle == 0.0)
		{
			S = 1.0;
			C = 0.0;
		}
		else
		{
			S = sin(t.tiltAngle) / t.tiltAngle;
			C = (1.0 - cos(t.tiltAngle)) / t.tiltAngle;
		}

		// Calculate the tilt velocity parameters
		double alphadot = pdot.pxVel*cgamma + pdot.pyVel*sgamma;
		double agammadot = pdot.pyVel*cgamma - pdot.pxVel*sgamma; // = alpha*gammadot

		// Calculate the required angular velocity
		angVel.x() = cpsigam*alphadot - S*agammadot*spsigam;
		angVel.y() = spsigam*alphadot + S*agammadot*cpsigam;
		angVel.z() = C*agammadot;
	}

	// Conversion: Tilt phase velocity 3D --> Angular velocity
	void AngFromTiltPhaseVel(const TiltPhaseVel3D& pdot, const TiltAngles& t, AngVel& angVel)
	{
		// Calculate the required angular velocity
		TiltPhaseVel2D pdot2D = pdot;
		AngFromTiltPhaseVel(pdot2D, t, angVel);
		angVel.z() += pdot.pzVel;
	}

	// #######################
	// #### Miscellaneous ####
	// #######################

	// Conversion: Split yaw and tilt --> Quaternion
	// Calculates qHB for the frame B that has a given fused yaw relative to G and tilt rotation component relative to H
	// qGH       ==> Relative quaternion rotation between G and H
	// fusedYawG ==> Desired fused yaw of B relative to G
	// qH        ==> Specification of the desired tilt rotation component of B relative to H, can be any qHC that has the
	//               same tilt rotation component as is desired for qHB
	Quat QuatHFromFYawGTiltH(const Quat& qGH, double fusedYawG, const Quat& qH)
	{
		// Precalculate trigonometric values
		double chpsi = cos(0.5*fusedYawG);
		double shpsi = sin(0.5*fusedYawG);

		// Construct the base components of the solution
		double a = qGH.x()*qH.x() + qGH.y()*qH.y();
		double b = qGH.x()*qH.y() - qGH.y()*qH.x();
		double c = qGH.w()*qH.z() + qGH.z()*qH.w();
		double d = qGH.w()*qH.w() - qGH.z()*qH.z();
		double A = d - a;
		double B = b - c;
		double C = b + c;
		double D = d + a;
		double G = D*chpsi - B*shpsi;
		double H = A*shpsi - C*chpsi;
		double F = sqrt(G*G + H*H);

		// Construct and return the output quaternion
		if(F < 64.0*DBL_EPSILON)
			return qH;
		else
		{
			double chphi = G/F;
			double shphi = H/F;
			return Quat(chphi*qH.w() - qH.z()*shphi, chphi*qH.x() - qH.y()*shphi, chphi*qH.y() + qH.x()*shphi, chphi*qH.z() + qH.w()*shphi); // Order: (w,x,y,z)
		}
	}
}
// EOF