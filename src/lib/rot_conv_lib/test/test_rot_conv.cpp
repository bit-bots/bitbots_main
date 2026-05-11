// Unit test for the rotations conversion package
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rot_conv/rot_conv.h>
#include <gtest/gtest.h>

// Defines
#define M_2PI (2.0*M_PI)

// Tolerances
#define TOL_HIGH  1e-14
#define TOL_MED   1e-12
#define TOL_LOW   1e-10

// Namespaces
using namespace rot_conv;

// ##########################
// #### Helper functions ####
// ##########################

// Test perfect equality of two vectors
void ExpectEqual(const Vec3& va, const Vec3& vb)
{
	// Expect the rotations to be equal
	EXPECT_EQ(va.x(), vb.x());
	EXPECT_EQ(va.y(), vb.y());
	EXPECT_EQ(va.z(), vb.z());
}

// Test equality of two vectors
void ExpectNear(const Vec3& va, const Vec3& vb, double tol = -1.0)
{
	// Expect the vectors to be nearly equal
	if(tol >= 0.0)
	{
		EXPECT_NEAR(va.x(), vb.x(), tol);
		EXPECT_NEAR(va.y(), vb.y(), tol);
		EXPECT_NEAR(va.z(), vb.z(), tol);
	}
	else
	{
		EXPECT_DOUBLE_EQ(va.x(), vb.x());
		EXPECT_DOUBLE_EQ(va.y(), vb.y());
		EXPECT_DOUBLE_EQ(va.z(), vb.z());
	}
}

// Test perfect equality of two rotation matrices
void ExpectEqual(const Rotmat& Ra, const Rotmat& Rb)
{
	// Expect the rotations to be equal
	EXPECT_EQ(Ra.coeff(0,0), Rb.coeff(0,0));
	EXPECT_EQ(Ra.coeff(0,1), Rb.coeff(0,1));
	EXPECT_EQ(Ra.coeff(0,2), Rb.coeff(0,2));
	EXPECT_EQ(Ra.coeff(1,0), Rb.coeff(1,0));
	EXPECT_EQ(Ra.coeff(1,1), Rb.coeff(1,1));
	EXPECT_EQ(Ra.coeff(1,2), Rb.coeff(1,2));
	EXPECT_EQ(Ra.coeff(2,0), Rb.coeff(2,0));
	EXPECT_EQ(Ra.coeff(2,1), Rb.coeff(2,1));
	EXPECT_EQ(Ra.coeff(2,2), Rb.coeff(2,2));
}

// Test whether two rotation matrices are almost equal
void ExpectNear(const Rotmat& Ra, const Rotmat& Rb, double tol = -1.0)
{
	// Expect the rotations to be nearly equal
	if(tol >= 0.0)
	{
		EXPECT_NEAR(Ra.coeff(0,0), Rb.coeff(0,0), tol);
		EXPECT_NEAR(Ra.coeff(0,1), Rb.coeff(0,1), tol);
		EXPECT_NEAR(Ra.coeff(0,2), Rb.coeff(0,2), tol);
		EXPECT_NEAR(Ra.coeff(1,0), Rb.coeff(1,0), tol);
		EXPECT_NEAR(Ra.coeff(1,1), Rb.coeff(1,1), tol);
		EXPECT_NEAR(Ra.coeff(1,2), Rb.coeff(1,2), tol);
		EXPECT_NEAR(Ra.coeff(2,0), Rb.coeff(2,0), tol);
		EXPECT_NEAR(Ra.coeff(2,1), Rb.coeff(2,1), tol);
		EXPECT_NEAR(Ra.coeff(2,2), Rb.coeff(2,2), tol);
	}
	else
	{
		EXPECT_DOUBLE_EQ(Ra.coeff(0,0), Rb.coeff(0,0));
		EXPECT_DOUBLE_EQ(Ra.coeff(0,1), Rb.coeff(0,1));
		EXPECT_DOUBLE_EQ(Ra.coeff(0,2), Rb.coeff(0,2));
		EXPECT_DOUBLE_EQ(Ra.coeff(1,0), Rb.coeff(1,0));
		EXPECT_DOUBLE_EQ(Ra.coeff(1,1), Rb.coeff(1,1));
		EXPECT_DOUBLE_EQ(Ra.coeff(1,2), Rb.coeff(1,2));
		EXPECT_DOUBLE_EQ(Ra.coeff(2,0), Rb.coeff(2,0));
		EXPECT_DOUBLE_EQ(Ra.coeff(2,1), Rb.coeff(2,1));
		EXPECT_DOUBLE_EQ(Ra.coeff(2,2), Rb.coeff(2,2));
	}
}

// Test perfect equality of two quaternions
void ExpectEqual(const Quat& qa, const Quat& qb)
{
	// Expect the rotations to be equal
	EXPECT_EQ(qa.w(), qb.w());
	EXPECT_EQ(qa.x(), qb.x());
	EXPECT_EQ(qa.y(), qb.y());
	EXPECT_EQ(qa.z(), qb.z());
}

// Test equality of two quaternions
void ExpectNear(const Quat& qa, const Quat& qb, double tol = -1.0)
{
	// Expect the rotations to be nearly equal
	if(tol >= 0.0)
	{
		EXPECT_NEAR(qa.w(), qb.w(), tol);
		EXPECT_NEAR(qa.x(), qb.x(), tol);
		EXPECT_NEAR(qa.y(), qb.y(), tol);
		EXPECT_NEAR(qa.z(), qb.z(), tol);
	}
	else
	{
		EXPECT_DOUBLE_EQ(qa.w(), qb.w());
		EXPECT_DOUBLE_EQ(qa.x(), qb.x());
		EXPECT_DOUBLE_EQ(qa.y(), qb.y());
		EXPECT_DOUBLE_EQ(qa.z(), qb.z());
	}
}

// Test perfect equality of two Euler angles
void ExpectEqual(const EulerAngles& ea, const EulerAngles& eb)
{
	// Expect the rotations to be equal
	EXPECT_EQ(ea.yaw, eb.yaw);
	EXPECT_EQ(ea.pitch, eb.pitch);
	EXPECT_EQ(ea.roll, eb.roll);
}

// Test equality of two Euler angles
void ExpectNear(const EulerAngles& ea, const EulerAngles& eb, double tol = -1.0)
{
	// Expect the rotations to be nearly equal
	if(tol >= 0.0)
	{
		EXPECT_NEAR(ea.yaw, eb.yaw, tol);
		EXPECT_NEAR(ea.pitch, eb.pitch, tol);
		EXPECT_NEAR(ea.roll, eb.roll, tol);
	}
	else
	{
		EXPECT_DOUBLE_EQ(ea.yaw, eb.yaw);
		EXPECT_DOUBLE_EQ(ea.pitch, eb.pitch);
		EXPECT_DOUBLE_EQ(ea.roll, eb.roll);
	}
}

// Test perfect equality of two fused angles
void ExpectEqual(const FusedAngles& fa, const FusedAngles& fb)
{
	// Expect the rotations to be equal
	EXPECT_EQ(fa.fusedYaw, fb.fusedYaw);
	EXPECT_EQ(fa.fusedPitch, fb.fusedPitch);
	EXPECT_EQ(fa.fusedRoll, fb.fusedRoll);
	EXPECT_EQ(fa.hemi, fb.hemi);
}

// Test equality of two fused angles
void ExpectNear(const FusedAngles& fa, const FusedAngles& fb, double tol = -1.0)
{
	// Expect the rotations to be nearly equal
	if(tol >= 0.0)
	{
		EXPECT_NEAR(fa.fusedYaw, fb.fusedYaw, tol);
		EXPECT_NEAR(fa.fusedPitch, fb.fusedPitch, tol);
		EXPECT_NEAR(fa.fusedRoll, fb.fusedRoll, tol);
		EXPECT_TRUE(fa.hemi == fb.hemi);
	}
	else
	{
		EXPECT_DOUBLE_EQ(fa.fusedYaw, fb.fusedYaw);
		EXPECT_DOUBLE_EQ(fa.fusedPitch, fb.fusedPitch);
		EXPECT_DOUBLE_EQ(fa.fusedRoll, fb.fusedRoll);
		EXPECT_TRUE(fa.hemi == fb.hemi);
	}
}

// Test perfect equality of two tilt angles
void ExpectEqual(const TiltAngles& ta, const TiltAngles& tb)
{
	// Expect the rotations to be equal
	EXPECT_EQ(ta.fusedYaw, tb.fusedYaw);
	EXPECT_EQ(ta.tiltAxisAngle, tb.tiltAxisAngle);
	EXPECT_EQ(ta.tiltAngle, tb.tiltAngle);
}

// Test equality of two tilt angles
void ExpectNear(const TiltAngles& ta, const TiltAngles& tb, double tol = -1.0)
{
	// Expect the rotations to be nearly equal
	if(tol >= 0.0)
	{
		EXPECT_NEAR(ta.fusedYaw, tb.fusedYaw, tol);
		EXPECT_NEAR(ta.tiltAxisAngle, tb.tiltAxisAngle, tol);
		EXPECT_NEAR(ta.tiltAngle, tb.tiltAngle, tol);
	}
	else
	{
		EXPECT_DOUBLE_EQ(ta.fusedYaw, tb.fusedYaw);
		EXPECT_DOUBLE_EQ(ta.tiltAxisAngle, tb.tiltAxisAngle);
		EXPECT_DOUBLE_EQ(ta.tiltAngle, tb.tiltAngle);
	}
}

// ########################
// #### Rotation types ####
// ########################

// Test the Rotmat type
TEST(RotConvTest, testRotmatType)
{
	// Test the type
	Rotmat Ra;
	Ra.setIdentity();
	EXPECT_EQ(1.0, Ra.coeff(0,0));
	EXPECT_EQ(0.0, Ra.coeff(0,1));
	EXPECT_EQ(0.0, Ra.coeff(0,2));
	EXPECT_EQ(0.0, Ra.coeff(1,0));
	EXPECT_EQ(1.0, Ra.coeff(1,1));
	EXPECT_EQ(0.0, Ra.coeff(1,2));
	EXPECT_EQ(0.0, Ra.coeff(2,0));
	EXPECT_EQ(0.0, Ra.coeff(2,1));
	EXPECT_EQ(1.0, Ra.coeff(2,2));
}

// Test the Quat type
TEST(RotConvTest, testQuatType)
{
	// Test the type
	Quat qa(1.0, 2.0, 3.0, 4.0);
	EXPECT_EQ(1.0, qa.w());
	EXPECT_EQ(2.0, qa.x());
	EXPECT_EQ(3.0, qa.y());
	EXPECT_EQ(4.0, qa.z());
	qa.setIdentity();
	EXPECT_EQ(1.0, qa.w());
	EXPECT_EQ(0.0, qa.x());
	EXPECT_EQ(0.0, qa.y());
	EXPECT_EQ(0.0, qa.z());
}

// Test the EulerAngles struct
TEST(RotConvTest, testEulerAnglesStruct)
{
	// Test the struct
	EulerAngles ea; // This should compile and leave the members uninitialized
	EulerAngles eb(1.0);
	EXPECT_EQ(1.0, eb.yaw);
	EXPECT_EQ(0.0, eb.pitch);
	EXPECT_EQ(0.0, eb.roll);
	EulerAngles ec(2.0, -1.0, 1.5);
	EXPECT_EQ(2.0, ec.yaw);
	EXPECT_EQ(-1.0, ec.pitch);
	EXPECT_EQ(1.5, ec.roll);
	ea.set(0.5, 1.0, -1.5);
	EXPECT_EQ(0.5, ea.yaw);
	EXPECT_EQ(1.0, ea.pitch);
	EXPECT_EQ(-1.5, ea.roll);
	ea.setIdentity();
	EXPECT_EQ(0.0, ea.yaw);
	EXPECT_EQ(0.0, ea.pitch);
	EXPECT_EQ(0.0, ea.roll);
	EXPECT_EQ(0.0, EulerAngles::Identity().yaw);
	EXPECT_EQ(0.0, EulerAngles::Identity().pitch);
	EXPECT_EQ(0.0, EulerAngles::Identity().roll);
}

// Test the FusedAngles struct
TEST(RotConvTest, testFusedAnglesStruct)
{
	// Test the struct
	FusedAngles fa; // This should compile and leave the members uninitialized
	FusedAngles fg(0.7);
	EXPECT_EQ(0.7, fg.fusedYaw);
	EXPECT_EQ(0.0, fg.fusedPitch);
	EXPECT_EQ(0.0, fg.fusedRoll);
	EXPECT_TRUE(true == fg.hemi);
	FusedAngles fb(1.0, 2.0);
	EXPECT_EQ(0.0, fb.fusedYaw);
	EXPECT_EQ(1.0, fb.fusedPitch);
	EXPECT_EQ(2.0, fb.fusedRoll);
	EXPECT_TRUE(true == fb.hemi);
	FusedAngles ff(2.0, 1.0, false);
	EXPECT_EQ(0.0, ff.fusedYaw);
	EXPECT_EQ(2.0, ff.fusedPitch);
	EXPECT_EQ(1.0, ff.fusedRoll);
	EXPECT_TRUE(false == ff.hemi);
	FusedAngles fc(4.0, 3.0, 2.0);
	EXPECT_EQ(4.0, fc.fusedYaw);
	EXPECT_EQ(3.0, fc.fusedPitch);
	EXPECT_EQ(2.0, fc.fusedRoll);
	EXPECT_TRUE(true == fc.hemi);
	FusedAngles fd(1.0, 2.0, 3.0, false);
	EXPECT_EQ(1.0, fd.fusedYaw);
	EXPECT_EQ(2.0, fd.fusedPitch);
	EXPECT_EQ(3.0, fd.fusedRoll);
	EXPECT_TRUE(false == fd.hemi);
	fd.setIdentity();
	EXPECT_EQ(0.0, fd.fusedYaw);
	EXPECT_EQ(0.0, fd.fusedPitch);
	EXPECT_EQ(0.0, fd.fusedRoll);
	EXPECT_TRUE(true == fd.hemi);
	fd.set(1.5, -2.0, 3.0, false);
	EXPECT_EQ(1.5, fd.fusedYaw);
	EXPECT_EQ(-2.0, fd.fusedPitch);
	EXPECT_EQ(3.0, fd.fusedRoll);
	EXPECT_TRUE(false == fd.hemi);
	fd.set(-1.0, 2.0, 1.5);
	EXPECT_EQ(-1.0, fd.fusedYaw);
	EXPECT_EQ(2.0, fd.fusedPitch);
	EXPECT_EQ(1.5, fd.fusedRoll);
	EXPECT_TRUE(true == fd.hemi);
	FusedAngles fe(3.0);
	EXPECT_EQ(3.0, fe.fusedYaw);
	EXPECT_EQ(0.0, fe.fusedPitch);
	EXPECT_EQ(0.0, fe.fusedRoll);
	EXPECT_TRUE(true == fe.hemi);
	EXPECT_EQ(0.0, FusedAngles::Identity().fusedYaw);
	EXPECT_EQ(0.0, FusedAngles::Identity().fusedPitch);
	EXPECT_EQ(0.0, FusedAngles::Identity().fusedRoll);
	EXPECT_TRUE(true == FusedAngles::Identity().hemi);
}

// Test the TiltAngles struct
TEST(RotConvTest, testTiltAnglesStruct)
{
	// Test the struct
	TiltAngles ta; // This should compile and leave the members uninitialized
	TiltAngles te(0.4);
	EXPECT_EQ(0.4, te.fusedYaw);
	EXPECT_EQ(0.0, te.tiltAxisAngle);
	EXPECT_EQ(0.0, te.tiltAngle);
	TiltAngles tb(1.0, 2.0);
	EXPECT_EQ(0.0, tb.fusedYaw);
	EXPECT_EQ(1.0, tb.tiltAxisAngle);
	EXPECT_EQ(2.0, tb.tiltAngle);
	TiltAngles tc(4.0, 3.0, 2.0);
	EXPECT_EQ(4.0, tc.fusedYaw);
	EXPECT_EQ(3.0, tc.tiltAxisAngle);
	EXPECT_EQ(2.0, tc.tiltAngle);
	tc.setIdentity();
	EXPECT_EQ(0.0, tc.fusedYaw);
	EXPECT_EQ(0.0, tc.tiltAxisAngle);
	EXPECT_EQ(0.0, tc.tiltAngle);
	tc.set(1.5, -2.0, 0.3);
	EXPECT_EQ(1.5, tc.fusedYaw);
	EXPECT_EQ(-2.0, tc.tiltAxisAngle);
	EXPECT_EQ(0.3, tc.tiltAngle);
	TiltAngles td(-2.0);
	EXPECT_EQ(-2.0, td.fusedYaw);
	EXPECT_EQ(0.0, td.tiltAxisAngle);
	EXPECT_EQ(0.0, td.tiltAngle);
	EXPECT_EQ(0.0, TiltAngles::Identity().fusedYaw);
	EXPECT_EQ(0.0, TiltAngles::Identity().tiltAxisAngle);
	EXPECT_EQ(0.0, TiltAngles::Identity().tiltAngle);
}

// ##########################################
// #### Rotation checking and validation ####
// ##########################################

// Test checking and validation: Rotation matrix
TEST(RotConvTest, testCheckValidateRotmat)
{
	// Define rotations
	Rotmat Ra, Rao, Rb, Rbo, Rc, Rco, Rtmp, Rout;
	Ra  << 0.5184924958405515, -0.0607647897101671, 0.8529203786987448, 0.8201040096413277, -0.2470343320581230, -0.5161428602187713, 0.2420639284122036, 0.9670996221259091, -0.0782519999266893;
	Rao << 0.5184924956811956, -0.0607647895817123, 0.8529203786223987, 0.8201040095588041, -0.2470343321255222, -0.5161428603176355, 0.2420639283905362, 0.9670996221167640, -0.0782519998067399;
	Rb  << 0.6469156566545853, 0.9004440976767099, -0.2368830858139832, 0.3896572459516341, -0.9311078389941825, 0.5310335762980047, -0.3658010398782789, -0.1225112806872035, 0.5903998022741264;
	Rbo.setIdentity();
	Rc  << -0.6262547908912428, 0.2926260202225293, -0.4479498460028433, -0.0204712084235379, 0.4187296617161451, 0.3594053537073496, -0.1088275985782010, 0.5093733639647218, 0.3101960079476813;
	Rco << -0.8208576638530812, 0.2776245171136335, -0.4991165426942650, -0.4772097080263116, 0.1467411821115800, 0.8664513373740804, 0.3137890856270149, 0.9494164802496288, 0.0120314908140614;

	// Test effect of tolerance on a nearly orthonormal rotation matrix
	Rtmp = Ra;
	EXPECT_FALSE(CheckRotmat(Rtmp));
	EXPECT_FALSE(CheckRotmat(Rtmp, Rout));
	ExpectNear(Rao, Rout, TOL_HIGH);
	EXPECT_FALSE(ValidateRotmat(Rtmp));
	ExpectNear(Rao, Rtmp, TOL_HIGH);
	Rtmp = Ra;
	double tol = ROT_CONV_DEFAULT_TOL*10.0;
	EXPECT_TRUE(CheckRotmat(Rtmp, tol));
	EXPECT_TRUE(CheckRotmat(Rtmp, Rout, tol));
	ExpectNear(Rao, Rout, TOL_HIGH);
	EXPECT_TRUE(ValidateRotmat(Rtmp, tol));
	ExpectNear(Rao, Rtmp, TOL_HIGH);

	// Test completely invalid rotation matrices
	Rtmp = Rb;
	EXPECT_FALSE(CheckRotmat(Rtmp));
	EXPECT_FALSE(CheckRotmat(Rtmp, Rout));
	ExpectNear(Rbo, Rout, TOL_HIGH);
	EXPECT_FALSE(ValidateRotmat(Rtmp));
	ExpectNear(Rbo, Rtmp, TOL_HIGH);
	Rtmp = Rc;
	EXPECT_FALSE(CheckRotmat(Rtmp));
	EXPECT_FALSE(CheckRotmat(Rtmp, Rout));
	ExpectNear(Rco, Rout, TOL_MED);
	EXPECT_FALSE(ValidateRotmat(Rtmp));
	ExpectNear(Rco, Rtmp, TOL_MED);
}

// Test checking and validation: Quaternion
TEST(RotConvTest, testCheckValidateQuat)
{
	// Define rotations
	Quat qtmp, qout;
	Quat qa(-0.4919425823202622, -0.5555358142415039, -0.0023853600625694, 0.6703482419253991);
	Quat qao(-0.4919425821726794, -0.5555358140748431, -0.0023853600618538, 0.6703482417242946);
	Quat qb(-0.2347146536221927, 0.5178711549954271, 1.0972301679700580, 1.6727097576073957);
	Quat qbo(-0.1128598432472697, 0.2490123921668100, 0.5275905140656608, 0.8043032598451260);
	Quat qbu(0.1128598432472697, -0.2490123921668100, -0.5275905140656608, -0.8043032598451260);

	// Test effect of tolerance on a nearly valid quaternion
	qtmp = qa;
	EXPECT_FALSE(CheckQuat(qtmp));
	EXPECT_FALSE(CheckQuat(qtmp, qout));
	ExpectNear(qao, qout, TOL_HIGH);
	EXPECT_FALSE(ValidateQuat(qtmp));
	ExpectNear(qao, qtmp, TOL_HIGH);
	qtmp = qa;
	double tol = ROT_CONV_DEFAULT_TOL*10.0;
	EXPECT_TRUE(CheckQuat(qtmp, tol));
	EXPECT_TRUE(CheckQuat(qtmp, qout, tol));
	ExpectNear(qao, qout, TOL_HIGH);
	EXPECT_TRUE(ValidateQuat(qtmp, tol));
	ExpectNear(qao, qtmp, TOL_HIGH);

	// Test completely invalid quaternions
	qtmp = qb;
	EXPECT_FALSE(CheckQuat(qtmp));
	EXPECT_FALSE(CheckQuat(qtmp, qout));
	ExpectNear(qbo, qout, TOL_HIGH);
	EXPECT_FALSE(ValidateQuat(qtmp));
	ExpectNear(qbo, qtmp, TOL_HIGH);
	qtmp = qb;
	EXPECT_FALSE(CheckQuat(qtmp, qout, ROT_CONV_DEFAULT_TOL, true));
	ExpectNear(qbu, qout, TOL_HIGH);
	EXPECT_FALSE(ValidateQuat(qtmp, ROT_CONV_DEFAULT_TOL, true));
	ExpectNear(qbu, qtmp, TOL_HIGH);
}

// Test checking and validation: Euler angles
TEST(RotConvTest, testCheckValidateEuler)
{
	// Define rotations
	EulerAngles etmp, eout;
	EulerAngles ea(8.2578951086046644, 8.0831853071795869, 8.9803281972962505);
	EulerAngles eao(-1.1668828521647150, 1.3415926535897924, -0.4444497634731288);
	EulerAngles eb(0.7532515739280932, 1.5707763267948966, -1.8481340284399175);
	EulerAngles ebu(0.0000000000000000, 1.5707763267948966, -2.6013856023680102);

	// Test some invalid Euler angles
	etmp = ea;
	EXPECT_FALSE(CheckEuler(etmp));
	EXPECT_FALSE(CheckEuler(etmp, eout));
	ExpectNear(eao, eout, TOL_HIGH);
	EXPECT_FALSE(ValidateEuler(etmp));
	ExpectNear(eao, etmp, TOL_HIGH);

	// Test tolerances and uniqueness
	etmp = eb;
	EXPECT_TRUE(CheckEuler(etmp, ROT_CONV_DEFAULT_TOL));
	EXPECT_TRUE(CheckEuler(etmp, ROT_CONV_DEFAULT_TOL*10.0));
	EXPECT_TRUE(CheckEuler(etmp, eout, ROT_CONV_DEFAULT_TOL));
	ExpectNear(eb, eout, TOL_HIGH);
	EXPECT_TRUE(CheckEuler(etmp, eout, ROT_CONV_DEFAULT_TOL*10.0));
	ExpectNear(eb, eout, TOL_HIGH);
	EXPECT_TRUE(ValidateEuler(etmp, ROT_CONV_DEFAULT_TOL));
	ExpectNear(eb, etmp, TOL_HIGH);
	etmp = eb;
	EXPECT_TRUE(ValidateEuler(etmp, ROT_CONV_DEFAULT_TOL*10.0));
	ExpectNear(eb, etmp, TOL_HIGH);
	etmp = eb;
	EXPECT_TRUE(CheckEuler(etmp, ROT_CONV_DEFAULT_TOL, true));
	EXPECT_FALSE(CheckEuler(etmp, ROT_CONV_DEFAULT_TOL*10.0, true));
	EXPECT_TRUE(CheckEuler(etmp, eout, ROT_CONV_DEFAULT_TOL, true));
	ExpectNear(eb, eout, TOL_HIGH);
	EXPECT_FALSE(CheckEuler(etmp, eout, ROT_CONV_DEFAULT_TOL*10.0, true));
	ExpectNear(ebu, eout, TOL_HIGH);
	EXPECT_TRUE(ValidateEuler(etmp, ROT_CONV_DEFAULT_TOL, true));
	ExpectNear(eb, etmp, TOL_HIGH);
	etmp = eb;
	EXPECT_FALSE(ValidateEuler(etmp, ROT_CONV_DEFAULT_TOL*10.0, true));
	ExpectNear(ebu, etmp, TOL_HIGH);
}

// Test checking and validation: Fused angles
TEST(RotConvTest, testCheckValidateFused)
{
	// Define rotations
	FusedAngles ftmp, fout;
	FusedAngles fa(12.8212600967838668, 7.1925287664956974, 10.2649509332849362, true);
	FusedAngles fao(0.2548894824246943, 0.4448765926455277, -1.1259197341493687, true);
	FusedAngles fb(-0.1923242743652673, -1.0225495882399438, -0.5482467388549528, false);
	FusedAngles fbo(-0.1923242743652671, -1.0225495880446509, -0.5482467387502455, false);
	FusedAngles fbu(-0.1923242743652671, -1.0225495880446509, -0.5482467387502455, true);

	// Test some invalid fused angles
	ftmp = fa;
	EXPECT_FALSE(CheckFused(ftmp));
	EXPECT_FALSE(CheckFused(ftmp, fout));
	ExpectNear(fao, fout, TOL_HIGH);
	EXPECT_FALSE(ValidateFused(ftmp));
	ExpectNear(fao, ftmp, TOL_HIGH);

	// Test tolerances and uniqueness
	ftmp = fb;
	EXPECT_FALSE(CheckFused(ftmp, ROT_CONV_DEFAULT_TOL));
	EXPECT_TRUE(CheckFused(ftmp, ROT_CONV_DEFAULT_TOL*10.0));
	EXPECT_FALSE(CheckFused(ftmp, fout, ROT_CONV_DEFAULT_TOL));
	ExpectNear(fbo, fout, TOL_HIGH);
	EXPECT_TRUE(CheckFused(ftmp, fout, ROT_CONV_DEFAULT_TOL*10.0));
	ExpectNear(fbo, fout, TOL_HIGH);
	EXPECT_FALSE(ValidateFused(ftmp, ROT_CONV_DEFAULT_TOL));
	ExpectNear(fbo, ftmp, TOL_HIGH);
	ftmp = fb;
	EXPECT_TRUE(ValidateFused(ftmp, ROT_CONV_DEFAULT_TOL*10.0));
	ExpectNear(fbo, ftmp, TOL_HIGH);
	ftmp = fb;
	EXPECT_FALSE(CheckFused(ftmp, ROT_CONV_DEFAULT_TOL, true));
	EXPECT_FALSE(CheckFused(ftmp, ROT_CONV_DEFAULT_TOL*10.0, true));
	EXPECT_FALSE(CheckFused(ftmp, fout, ROT_CONV_DEFAULT_TOL, true));
	ExpectNear(fbu, fout, TOL_HIGH);
	EXPECT_FALSE(CheckFused(ftmp, fout, ROT_CONV_DEFAULT_TOL*10.0, true));
	ExpectNear(fbu, fout, TOL_HIGH);
	EXPECT_FALSE(ValidateFused(ftmp, ROT_CONV_DEFAULT_TOL, true));
	ExpectNear(fbu, ftmp, TOL_HIGH);
	ftmp = fb;
	EXPECT_FALSE(ValidateFused(ftmp, ROT_CONV_DEFAULT_TOL*10.0, true));
	ExpectNear(fbu, ftmp, TOL_HIGH);
}

// Test checking and validation: Tilt angles
TEST(RotConvTest, testCheckValidateTilt)
{
	// Define rotations
	TiltAngles ttmp, tout;
	TiltAngles ta(4.9606854735443724, -2.4002054406729227, -6.7145991346431311);
	TiltAngles tao(-1.3224998336352138, 0.7413872129168704, 0.4314138274635440);
	TiltAngles tb(-1.4171825008680596, -2.1267661176916981, 0.0000200000000000);
	TiltAngles tbu(-1.4171825008680594, 0.0000000000000000, 0.0000200000000001);

	// Test some invalid tilt angles
	ttmp = ta;
	EXPECT_FALSE(CheckTilt(ttmp));
	EXPECT_FALSE(CheckTilt(ttmp, tout));
	ExpectNear(tao, tout, TOL_HIGH);
	EXPECT_FALSE(ValidateTilt(ttmp));
	ExpectNear(tao, tout, TOL_HIGH);

	// Test tolerances and uniqueness
	ttmp = tb;
	EXPECT_TRUE(CheckTilt(ttmp, ROT_CONV_DEFAULT_TOL));
	EXPECT_TRUE(CheckTilt(ttmp, ROT_CONV_DEFAULT_TOL*10.0));
	EXPECT_TRUE(CheckTilt(ttmp, tout, ROT_CONV_DEFAULT_TOL));
	ExpectNear(tb, tout, TOL_HIGH);
	EXPECT_TRUE(CheckTilt(ttmp, tout, ROT_CONV_DEFAULT_TOL*10.0));
	ExpectNear(tb, tout, TOL_HIGH);
	EXPECT_TRUE(ValidateTilt(ttmp, ROT_CONV_DEFAULT_TOL));
	ExpectNear(tb, ttmp, TOL_HIGH);
	ttmp = tb;
	EXPECT_TRUE(ValidateTilt(ttmp, ROT_CONV_DEFAULT_TOL*10.0));
	ExpectNear(tb, ttmp, TOL_HIGH);
	ttmp = tb;
	EXPECT_TRUE(CheckTilt(ttmp, ROT_CONV_DEFAULT_TOL, true));
	EXPECT_FALSE(CheckTilt(ttmp, ROT_CONV_DEFAULT_TOL*10.0, true));
	EXPECT_TRUE(CheckTilt(ttmp, tout, ROT_CONV_DEFAULT_TOL, true));
	ExpectNear(tb, tout, TOL_HIGH);
	EXPECT_FALSE(CheckTilt(ttmp, tout, ROT_CONV_DEFAULT_TOL*10.0, true));
	ExpectNear(tbu, tout, TOL_HIGH);
	EXPECT_TRUE(ValidateTilt(ttmp, ROT_CONV_DEFAULT_TOL, true));
	ExpectNear(tb, ttmp, TOL_HIGH);
	ttmp = tb;
	EXPECT_FALSE(ValidateTilt(ttmp, ROT_CONV_DEFAULT_TOL*10.0, true));
	ExpectNear(tbu, ttmp, TOL_HIGH);
}

// ###########################
// #### Rotation equality ####
// ###########################

// Test the equality of rotation matrices
TEST(RotConvTest, testEqualRotmat)
{
	// Define rotations
	Rotmat Ra, Ran;
	Ra  << 0.9363335936741253, 0.3427190709696175, -0.0763088445137274, 0.0813118145888689, -0.0002299890325057, 0.9966886855549781, 0.3415666701620100, -0.9394379093366241, -0.0280824560120627;
	Ran << 0.9363335934610981, 0.3427190708801889, -0.0763088447681474, 0.0813118148006876, -0.0002299890245560, 0.9966886853989279, 0.3415666702352431, -0.9394379093955393, -0.0280824562380714;

	// Test rotation equality
	EXPECT_FALSE(RotmatEqual(Ra, Ran, ROT_CONV_DEFAULT_TOL));
	EXPECT_TRUE(RotmatEqual(Ra, Ran, ROT_CONV_DEFAULT_TOL*10.0));
}

// Test the equality of quaternions
TEST(RotConvTest, testEqualQuat)
{
	// Define rotations
	Quat qa(-0.5146724492228377, -0.4234184379393630, -0.1347086660388057, -0.7332684854253596);
	Quat qan(-0.5146724489812080, -0.4234184376724907, -0.1347086660442872, -0.7332684854318080);
	Quat qann(0.5146724491254694, 0.4234184381793953, 0.1347086659603537, 0.7332684851920812);

	// Test rotation equality
	EXPECT_FALSE(QuatEqual(qa, qan, ROT_CONV_DEFAULT_TOL));
	EXPECT_TRUE(QuatEqual(qa, qan, ROT_CONV_DEFAULT_TOL*10.0));
	EXPECT_FALSE(QuatEqual(qa, qann, ROT_CONV_DEFAULT_TOL));
	EXPECT_TRUE(QuatEqual(qa, qann, ROT_CONV_DEFAULT_TOL*10.0));
}

// Test the equality of Euler angles
TEST(RotConvTest, testEqualEuler)
{
	// Define rotations
	EulerAngles eatmp, etmp;
	EulerAngles ea(2.7774857769905843, 1.4329889206345048, 0.4725495395706337);

	// Test rotation equality
	double eps = ROT_CONV_DEFAULT_TOL*3.0;
	double tol = ROT_CONV_DEFAULT_TOL*10.0;
	EXPECT_TRUE(EulerEqual(ea, ea));
	etmp = EulerAngles(ea.yaw + eps, ea.pitch - eps, ea.roll + eps);
	EXPECT_FALSE(EulerEqual(ea, etmp));
	EXPECT_TRUE(EulerEqual(ea, etmp, tol));
	etmp = EulerAngles(ea.yaw + M_2PI + eps, ea.pitch - M_2PI - eps, ea.roll - 3.0*M_2PI + eps);
	EXPECT_FALSE(EulerEqual(ea, etmp));
	EXPECT_TRUE(EulerEqual(ea, etmp, tol));
	etmp = EulerAngles(M_PI + ea.yaw + eps, M_PI - ea.pitch - eps, M_PI + ea.roll + eps);
	EXPECT_FALSE(EulerEqual(ea, etmp));
	EXPECT_TRUE(EulerEqual(ea, etmp, tol));
	eatmp = ea; eatmp.pitch = M_PI_2;
	etmp = EulerAngles(eps, M_PI_2 - eps, ea.roll - ea.yaw + M_2PI - eps);
	EXPECT_FALSE(EulerEqual(eatmp, etmp));
	EXPECT_TRUE(EulerEqual(eatmp, etmp, tol));
	etmp = EulerAngles(ea.yaw - eps, asin(1.0 - eps), ea.roll + eps);
	EXPECT_FALSE(EulerEqual(eatmp, etmp));
	EXPECT_TRUE(EulerEqual(eatmp, etmp, tol));
	eatmp = ea; eatmp.pitch = -M_PI_2;
	etmp = EulerAngles(eps, -M_PI_2 - eps, ea.roll + ea.yaw - M_2PI + eps);
	EXPECT_FALSE(EulerEqual(eatmp, etmp));
	EXPECT_TRUE(EulerEqual(eatmp, etmp, tol));
	etmp = EulerAngles(ea.yaw - eps, asin(-1.0 + eps), ea.roll + eps);
	EXPECT_FALSE(EulerEqual(eatmp, etmp));
	EXPECT_TRUE(EulerEqual(eatmp, etmp, tol));
}

// Test the equality of fused angles
TEST(RotConvTest, testEqualFused)
{
	// Define rotations
	FusedAngles fatmp, ftmp;
	FusedAngles fa(-2.7659867076743128, -0.6472647148125336, -0.1859487609211632, true);

	// Test rotation equality
	double eps = ROT_CONV_DEFAULT_TOL*3.0;
	double tol = ROT_CONV_DEFAULT_TOL*10.0;
	EXPECT_TRUE(FusedEqual(fa, fa));
	ftmp = FusedAngles(fa.fusedYaw + eps, fa.fusedPitch + eps, fa.fusedRoll - eps, fa.hemi);
	EXPECT_FALSE(FusedEqual(fa, ftmp));
	EXPECT_TRUE(FusedEqual(fa, ftmp, tol));
	ftmp = FusedAngles(fa.fusedYaw + M_2PI + eps, fa.fusedPitch - M_2PI + eps, fa.fusedRoll + M_2PI - eps, fa.hemi);
	EXPECT_FALSE(FusedEqual(fa, ftmp));
	EXPECT_TRUE(FusedEqual(fa, ftmp, tol));
	double scale = M_PI_2 / (fabs(fa.fusedPitch) + fabs(fa.fusedRoll));
	fatmp = FusedAngles(fa.fusedYaw, fa.fusedPitch*scale, fa.fusedRoll*scale, true);
	ftmp = FusedAngles(fa.fusedYaw + eps, fa.fusedPitch*scale*1.5 + eps, fa.fusedRoll*scale*1.5 - eps, false);
	EXPECT_FALSE(FusedEqual(fatmp, ftmp));
	EXPECT_TRUE(FusedEqual(fatmp, ftmp, tol));
	fatmp = FusedAngles(fa.fusedYaw, eps, -eps, false);
	ftmp = FusedAngles(fa.fusedYaw + 1.9, -eps, eps, false);
	EXPECT_FALSE(FusedEqual(fatmp, ftmp));
	EXPECT_TRUE(FusedEqual(fatmp, ftmp, tol));
	fatmp = FusedAngles(fa.fusedYaw, M_PI_2, 0.0, fa.hemi);
	ftmp = FusedAngles(fa.fusedYaw + eps, asin(1.0 - eps), eps, fa.hemi);
	EXPECT_FALSE(FusedEqual(fatmp, ftmp));
	EXPECT_TRUE(FusedEqual(fatmp, ftmp, tol));
	fatmp = FusedAngles(fa.fusedYaw, -M_PI_2, 0.0, fa.hemi);
	ftmp = FusedAngles(fa.fusedYaw + eps, asin(-1.0 + eps), -eps, fa.hemi);
	EXPECT_FALSE(FusedEqual(fatmp, ftmp));
	EXPECT_TRUE(FusedEqual(fatmp, ftmp, tol));
	fatmp = FusedAngles(fa.fusedYaw, 0.0, M_PI_2, fa.hemi);
	ftmp = FusedAngles(fa.fusedYaw + eps, -eps, asin(1.0 - eps), fa.hemi);
	EXPECT_FALSE(FusedEqual(fatmp, ftmp));
	EXPECT_TRUE(FusedEqual(fatmp, ftmp, tol));
	fatmp = FusedAngles(fa.fusedYaw, 0.0, -M_PI_2, fa.hemi);
	ftmp = FusedAngles(fa.fusedYaw - eps, eps, asin(-1.0 + eps), fa.hemi);
	EXPECT_FALSE(FusedEqual(fatmp, ftmp));
	EXPECT_TRUE(FusedEqual(fatmp, ftmp, tol));
}

// Test the equality of tilt angles
TEST(RotConvTest, testEqualTilt)
{
	// Define rotations
	TiltAngles tatmp, ttmp;
	TiltAngles ta(1.5374502501808673, -1.9543532794194114, 2.1575686561264202);

	// Test rotation equality
	double eps = ROT_CONV_DEFAULT_TOL*3.0;
	double tol = ROT_CONV_DEFAULT_TOL*10.0;
	EXPECT_TRUE(TiltEqual(ta, ta));
	ttmp = TiltAngles(ta.fusedYaw + eps, ta.tiltAxisAngle + eps, ta.tiltAngle - eps);
	EXPECT_FALSE(TiltEqual(ta, ttmp));
	EXPECT_TRUE(TiltEqual(ta, ttmp, tol));
	ttmp = TiltAngles(ta.fusedYaw - M_2PI + eps, ta.tiltAxisAngle + 2.0*M_2PI + eps, ta.tiltAngle + M_2PI - eps);
	EXPECT_FALSE(TiltEqual(ta, ttmp));
	EXPECT_TRUE(TiltEqual(ta, ttmp, tol));
	ttmp = TiltAngles(ta.fusedYaw + eps, M_PI + ta.tiltAxisAngle + eps, -ta.tiltAngle - eps);
	EXPECT_FALSE(TiltEqual(ta, ttmp));
	EXPECT_TRUE(TiltEqual(ta, ttmp, tol));
	tatmp = TiltAngles(ta.fusedYaw, ta.tiltAxisAngle, 0.0);
	ttmp = TiltAngles(ta.fusedYaw - eps, ta.tiltAxisAngle - eps, acos(1.0 - eps));
	EXPECT_FALSE(TiltEqual(tatmp, ttmp));
	EXPECT_TRUE(TiltEqual(tatmp, ttmp, tol));
	tatmp = TiltAngles(ta.fusedYaw, ta.tiltAxisAngle, M_PI);
	ttmp = TiltAngles(ta.fusedYaw + eps, ta.tiltAxisAngle - eps, acos(-1.0 + eps));
	EXPECT_FALSE(TiltEqual(tatmp, ttmp));
	EXPECT_TRUE(TiltEqual(tatmp, ttmp, tol));
}

// #########################
// #### Yaw of rotation ####
// #########################

// Test the yaw of rotation matrices
TEST(RotConvTest, testYawOfRotmat)
{
	// Define rotations
	Rotmat Ra, Rb, Rc, Rd;
	Ra << 0.1093363974494114, 0.6815738458110217, -0.7235348263208605, -0.9936267206075498, 0.0950162743415330, -0.0606452611911313, 0.0274133596531030, 0.7255542711035667, 0.6876187224006495;
	Rb << -0.6332447541604538, -0.7194975493137308, 0.2851742587608521, 0.7081710268113921, -0.6873089366379439, -0.1615556325310782, 0.3122416982293756, 0.0996478908414711, 0.9447620968993632;
	Rc << -0.6879495150449249, 0.4874821414400084, 0.5376677659359139, 0.4384462698803744, 0.8695224234362546, -0.2273667160547662, -0.5783513924747126, 0.0793216043622822, -0.8119222560713277;
	Rd << -0.0323338135304458, 0.8822912645615979, 0.4695920026798533, 0.9120777516719440, -0.1660952535202127, 0.3748686992309555, 0.4087403814252154, 0.4404253526267004, -0.7993477399455240;

	// Check the yaws
	EXPECT_NEAR(-1.4611995524260222, EYawOfRotmat(Ra), TOL_HIGH);
	EXPECT_NEAR(-1.4494090818111385, FYawOfRotmat(Ra), TOL_HIGH);
	EXPECT_NEAR(2.3003963563924894, EYawOfRotmat(Rb), TOL_HIGH);
	EXPECT_NEAR(2.3172381454320607, FYawOfRotmat(Rb), TOL_HIGH);
	EXPECT_NEAR(2.5741806630268611, EYawOfRotmat(Rc), TOL_HIGH);
	EXPECT_NEAR(-0.2637692552570381, FYawOfRotmat(Rc), TOL_HIGH);
	EXPECT_NEAR(1.6062322076326374, EYawOfRotmat(Rd), TOL_HIGH);
	EXPECT_NEAR(2.9925936504088253, FYawOfRotmat(Rd), TOL_HIGH);
}

// Test the yaw of quaternions
TEST(RotConvTest, testYawOfQuat)
{
	// Define rotations
	Quat qa(0.6877447553765121, 0.2857889958987350, -0.2729748864325581, -0.6089470524211659);
	Quat qb(0.3950343042385575, 0.1653043296303268, -0.0171298031450059, 0.9035092400880248);
	Quat qc(0.3039945115951940, 0.2522153433031724, 0.9177954830124884, -0.0403262803186158);
	Quat qd(0.0235753738242827, 0.6951814834874579, 0.6452879783390818, 0.3158644199277336);

	// Check the yaws
	EXPECT_NEAR(-1.4611995524260222, EYawOfQuat(qa), TOL_HIGH);
	EXPECT_NEAR(-1.4494090818111385, FYawOfQuat(qa), TOL_HIGH);
	EXPECT_NEAR(2.3003963563924894, EYawOfQuat(qb), TOL_HIGH);
	EXPECT_NEAR(2.3172381454320607, FYawOfQuat(qb), TOL_HIGH);
	EXPECT_NEAR(2.5741806630268611, EYawOfQuat(qc), TOL_HIGH);
	EXPECT_NEAR(-0.2637692552570381, FYawOfQuat(qc), TOL_HIGH);
	EXPECT_NEAR(1.6062322076326374, EYawOfQuat(qd), TOL_HIGH);
	EXPECT_NEAR(2.9925936504088253, FYawOfQuat(qd), TOL_HIGH);
}

// Test the yaw of Euler angles
TEST(RotConvTest, testYawOfEuler)
{
	// Define rotations
	EulerAngles ea(-1.4611995524260222, -0.0274167943027894, 0.8122359536270399);
	EulerAngles eb(2.3003963563924894, -0.3175517959453028, 0.1050855215916976);
	EulerAngles ec(2.5741806630268611, 0.6167063615707554, 3.0442056476459807);
	EulerAngles ed(1.6062322076326374, -0.4210734589421157, 2.6379966495582572);

	// Check the yaws
	EXPECT_NEAR(-1.4611995524260222, EYawOfEuler(ea), TOL_HIGH);
	EXPECT_NEAR(-1.4494090818111385, FYawOfEuler(ea), TOL_HIGH);
	EXPECT_NEAR(2.3003963563924894, EYawOfEuler(eb), TOL_HIGH);
	EXPECT_NEAR(2.3172381454320607, FYawOfEuler(eb), TOL_HIGH);
	EXPECT_NEAR(2.5741806630268611, EYawOfEuler(ec), TOL_HIGH);
	EXPECT_NEAR(-0.2637692552570381, FYawOfEuler(ec), TOL_HIGH);
	EXPECT_NEAR(1.6062322076326374, EYawOfEuler(ed), TOL_HIGH);
	EXPECT_NEAR(2.9925936504088253, FYawOfEuler(ed), TOL_HIGH);
}

// Test the yaw of fused angles
TEST(RotConvTest, testYawOfFused)
{
	// Define rotations
	FusedAngles fa(-1.4494090818111385, -0.0274167943027894, 0.8118394861777727, true);
	FusedAngles fb(2.3172381454320607, -0.3175517959453028, 0.0998135444277342, true);
	FusedAngles fc(-0.2637692552570381, 0.6167063615707554, 0.0794050215872836, false);
	FusedAngles fd(2.9925936504088253, -0.4210734589421157, 0.4560723961893606, false);

	// Check the yaws
	EXPECT_NEAR(-1.4611995524260222, EYawOfFused(fa), TOL_HIGH);
	EXPECT_NEAR(-1.4494090818111385, FYawOfFused(fa), TOL_HIGH);
	EXPECT_NEAR(2.3003963563924894, EYawOfFused(fb), TOL_HIGH);
	EXPECT_NEAR(2.3172381454320607, FYawOfFused(fb), TOL_HIGH);
	EXPECT_NEAR(2.5741806630268611, EYawOfFused(fc), TOL_HIGH);
	EXPECT_NEAR(-0.2637692552570381, FYawOfFused(fc), TOL_HIGH);
	EXPECT_NEAR(1.6062322076326374, EYawOfFused(fd), TOL_HIGH);
	EXPECT_NEAR(2.9925936504088253, FYawOfFused(fd), TOL_HIGH);
}

// Test the yaw of tilt angles
TEST(RotConvTest, testYawOfTilt)
{
	// Define rotations
	TiltAngles ta(-1.4494090818111385, -0.0377646820603568, 0.8125920607562114);
	TiltAngles tb(2.3172381454320607, -1.2618763808332645, 0.3339283373395991);
	TiltAngles tc(-0.2637692552570381, 1.4344954796502363, 2.5182337979721927);
	TiltAngles td(2.9925936504088253, -0.7481024064161945, 2.4970052312101210);

	// Check the yaws
	EXPECT_NEAR(-1.4611995524260222, EYawOfTilt(ta), TOL_HIGH);
	EXPECT_NEAR(-1.4494090818111385, FYawOfTilt(ta), TOL_HIGH);
	EXPECT_NEAR(2.3003963563924894, EYawOfTilt(tb), TOL_HIGH);
	EXPECT_NEAR(2.3172381454320607, FYawOfTilt(tb), TOL_HIGH);
	EXPECT_NEAR(2.5741806630268611, EYawOfTilt(tc), TOL_HIGH);
	EXPECT_NEAR(-0.2637692552570381, FYawOfTilt(tc), TOL_HIGH);
	EXPECT_NEAR(1.6062322076326374, EYawOfTilt(td), TOL_HIGH);
	EXPECT_NEAR(2.9925936504088253, FYawOfTilt(td), TOL_HIGH);
}

// ##################################
// #### Remove yaw from rotation ####
// ##################################

// Test the removal of yaw from rotation matrices
TEST(RotConvTest, testRotmatNoYaw)
{
	// Define rotations
	Rotmat Ra, Rb, Rc, Rd, Rae, Rbe, Rce, Rde, Raf, Rbf, Rcf, Rdf, Rout;
	Ra << -0.4675712316207492, -0.8828736928043754, -0.0437171123774387, 0.8828656408290165, -0.4639683350301569, -0.0726749223110316, 0.0438794211929251, -0.0725770393672128, 0.9963970944118935;
	Rb << 0.7916396759958976, -0.5207794028337154, 0.3195237658974197, -0.5434657449973512, -0.3612150783989314, 0.7577391709234306, -0.2791981507882605, -0.7735066132370251, -0.5689779537688637;
	Rc << -0.4357592980111133, -0.1012576489733479, 0.8943493292451512, 0.8906247419495379, 0.0950142362333455, 0.4447019945315208, -0.1300053969040533, 0.9903126695329028, 0.0487792306052002;
	Rd << -0.6461600901150271, -0.7283239544253354, -0.2280819049218548, 0.0939742874872020, -0.3725013406338937, 0.9232613847211548, -0.7573941980460457, 0.5751408250473911, 0.3091392245089476;
	Rae << 0.9990368343538562, 0.0031877087709076, -0.0437634792608860, -0.0000000000000001, 0.9973577150999948, 0.0726470104719945, 0.0438794211929251, -0.0725770393672128, 0.9963970944118935;
	Rbe << 0.9602335094113388, -0.2249053109703082, -0.1654364183030241, 0.0000000000000001, -0.5925412393883960, 0.8055401167068366, -0.2791981507882605, -0.7735066132370251, -0.5689779537688637;
	Rce << 0.9915132862326250, 0.1298479742524919, 0.0063958429236979, 0.0000000000000001, 0.0491967493350924, -0.9987891067962544, -0.1300053969040533, 0.9903126695329028, 0.0487792306052002;
	Rde << 0.6529579073463979, 0.6671307890589909, 0.3585839950741541, -0.0000000000000001, 0.4734443384953868, -0.8808237385235853, -0.7573941980460457, 0.5751408250473911, 0.3091392245089476;
	Raf << 0.9990355608062068, 0.0015951929043801, -0.0438794211929250, 0.0015951929043787, 0.9973615336056871, 0.0725770393672129, 0.0438794211929251, -0.0725770393672128, 0.9963970944118935;
	Rbf << 0.8191470527199378, -0.5010454057434867, 0.2791981507882605, -0.5010454057434867, -0.3881250064888015, 0.7735066132370251, -0.2791981507882605, -0.7735066132370251, -0.5689779537688637;
	Rcf << 0.9838846892358579, 0.1227579531561132, 0.1300053969040534, 0.1227579531561133, 0.0648945413693421, -0.9903126695329028, -0.1300053969040533, 0.9903126695329028, 0.0487792306052002;
	Rdf << 0.5618143888026997, 0.3327440777841675, 0.7573941980460457, 0.3327440777841675, 0.7473248357062476, -0.5751408250473911, -0.7573941980460457, 0.5751408250473911, 0.3091392245089476;

	// Check removal of Euler yaw
	ExpectNear(Rae, RotmatNoEYaw(Ra), TOL_HIGH);
	ExpectNear(Rbe, RotmatNoEYaw(Rb), TOL_HIGH);
	ExpectNear(Rce, RotmatNoEYaw(Rc), TOL_HIGH);
	ExpectNear(Rde, RotmatNoEYaw(Rd), TOL_HIGH);
	RotmatNoEYaw(Ra, Rout);
	ExpectNear(Rae, Rout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfRotmat(Rout), TOL_HIGH);
	RotmatNoEYaw(Rb, Rout);
	ExpectNear(Rbe, Rout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfRotmat(Rout), TOL_HIGH);
	RotmatNoEYaw(Rc, Rout);
	ExpectNear(Rce, Rout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfRotmat(Rout), TOL_HIGH);
	RotmatNoEYaw(Rd, Rout);
	ExpectNear(Rde, Rout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfRotmat(Rout), TOL_HIGH);

	// Check removal of fused yaw
	ExpectNear(Raf, RotmatNoFYaw(Ra), TOL_HIGH);
	ExpectNear(Rbf, RotmatNoFYaw(Rb), TOL_HIGH);
	ExpectNear(Rcf, RotmatNoFYaw(Rc), TOL_HIGH);
	ExpectNear(Rdf, RotmatNoFYaw(Rd), TOL_HIGH);
	RotmatNoFYaw(Ra, Rout);
	ExpectNear(Raf, Rout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfRotmat(Rout), TOL_HIGH);
	RotmatNoFYaw(Rb, Rout);
	ExpectNear(Rbf, Rout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfRotmat(Rout), TOL_HIGH);
	RotmatNoFYaw(Rc, Rout);
	ExpectNear(Rcf, Rout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfRotmat(Rout), TOL_HIGH);
	RotmatNoFYaw(Rd, Rout);
	ExpectNear(Rdf, Rout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfRotmat(Rout), TOL_HIGH);
}

// Test the removal of yaw from quaternions
TEST(RotConvTest, testQuatNoYaw)
{
	// Define rotations
	Quat qa(0.5159596708467115, 0.0000474276137019, -0.0424434982615086, 0.8555607314888292);
	Quat qb(0.4640707499477054, -0.8248988889802938, 0.3225380595271024, -0.0122213812043704);
	Quat qc(0.4207238312799242, 0.3242095137215829, 0.6086859419354242, 0.5893904250119287);
	Quat qd(0.2694799592548704, -0.3229558894067110, 0.4910497747103101, 0.7628565814191204);
	Quat qae(0.9990985491764246, -0.0363387700740128, -0.0219304943756691, -0.0007976462315796);
	Quat qbe(0.4468540914700456, -0.8834241199119195, 0.0636459051493670, 0.1258270402260514);
	Quat qce(0.7227532888498187, 0.6880293064783528, 0.0471811204224399, -0.0449143491478021);
	Quat qde(0.7803110710400585, 0.4664692766790925, 0.3575427270411333, -0.2137387299176043);
	Quat qaf(0.9990988675831571, -0.0363212499393470, -0.0219594990128807, 0.0000000000000000);
	Quat qbf(0.4642316481193072, -0.8331041370947576, 0.3007099493532449, 0.0000000000000000);
	Quat qcf(0.7241475093533085, 0.6837782749658905, 0.0897644438632076, 0.0000000000000000);
	Quat qdf(0.8090547646818933, 0.3554399838888080, 0.4680735044826294, 0.0000000000000000);
	Quat qout;

	// Check removal of Euler yaw
	ExpectNear(qae, QuatNoEYaw(qa), TOL_HIGH);
	ExpectNear(qbe, QuatNoEYaw(qb), TOL_HIGH);
	ExpectNear(qce, QuatNoEYaw(qc), TOL_HIGH);
	ExpectNear(qde, QuatNoEYaw(qd), TOL_HIGH);
	QuatNoEYaw(qa, qout);
	ExpectNear(qae, qout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfQuat(qout), TOL_HIGH);
	QuatNoEYaw(qb, qout);
	ExpectNear(qbe, qout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfQuat(qout), TOL_HIGH);
	QuatNoEYaw(qc, qout);
	ExpectNear(qce, qout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfQuat(qout), TOL_HIGH);
	QuatNoEYaw(qd, qout);
	ExpectNear(qde, qout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfQuat(qout), TOL_HIGH);

	// Check removal of fused yaw
	ExpectNear(qaf, QuatNoFYaw(qa), TOL_HIGH);
	ExpectNear(qbf, QuatNoFYaw(qb), TOL_HIGH);
	ExpectNear(qcf, QuatNoFYaw(qc), TOL_HIGH);
	ExpectNear(qdf, QuatNoFYaw(qd), TOL_HIGH);
	QuatNoFYaw(qa, qout);
	ExpectNear(qaf, qout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfQuat(qout), TOL_HIGH);
	QuatNoFYaw(qb, qout);
	ExpectNear(qbf, qout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfQuat(qout), TOL_HIGH);
	QuatNoFYaw(qc, qout);
	ExpectNear(qcf, qout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfQuat(qout), TOL_HIGH);
	QuatNoFYaw(qd, qout);
	ExpectNear(qdf, qout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfQuat(qout), TOL_HIGH);
}

// Test the removal of yaw from Euler angles
TEST(RotConvTest, testEulerNoYaw)
{
	// Define rotations
	EulerAngles ea(2.0578475173823771, -0.0438935143397104, -0.0727110628745876);
	EulerAngles eb(-0.6016123690679743, 0.2829589512327975, -2.2050062233955425);
	EulerAngles ec(2.0258261652654053, 0.1303744227992718, 1.5215797105002926);
	EulerAngles ed(2.9971702236761355, 0.8593130589551469, 1.0775992790361570);
	EulerAngles eae(0.0, ea.pitch, ea.roll);
	EulerAngles ebe(0.0, eb.pitch, eb.roll);
	EulerAngles ece(0.0, ec.pitch, ec.roll);
	EulerAngles ede(0.0, ed.pitch, ed.roll);
	EulerAngles eaf(0.0015967314991432, ea.pitch, ea.roll);
	EulerAngles ebf(-0.5489542041758281, eb.pitch, eb.roll);
	EulerAngles ecf(0.1241271860106765, ec.pitch, ec.roll);
	EulerAngles edf(0.5347140087302225, ed.pitch, ed.roll);
	EulerAngles eout;

	// Check removal of Euler yaw
	ExpectNear(eae, EulerNoEYaw(ea), TOL_HIGH);
	ExpectNear(ebe, EulerNoEYaw(eb), TOL_HIGH);
	ExpectNear(ece, EulerNoEYaw(ec), TOL_HIGH);
	ExpectNear(ede, EulerNoEYaw(ed), TOL_HIGH);
	EulerNoEYaw(ea, eout);
	ExpectNear(eae, eout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfEuler(eout), TOL_HIGH);
	EulerNoEYaw(eb, eout);
	ExpectNear(ebe, eout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfEuler(eout), TOL_HIGH);
	EulerNoEYaw(ec, eout);
	ExpectNear(ece, eout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfEuler(eout), TOL_HIGH);
	EulerNoEYaw(ed, eout);
	ExpectNear(ede, eout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfEuler(eout), TOL_HIGH);

	// Check removal of fused yaw
	ExpectNear(eaf, EulerNoFYaw(ea), TOL_HIGH);
	ExpectNear(ebf, EulerNoFYaw(eb), TOL_HIGH);
	ExpectNear(ecf, EulerNoFYaw(ec), TOL_HIGH);
	ExpectNear(edf, EulerNoFYaw(ed), TOL_HIGH);
	EulerNoFYaw(ea, eout);
	ExpectNear(eaf, eout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfEuler(eout), TOL_HIGH);
	EulerNoFYaw(eb, eout);
	ExpectNear(ebf, eout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfEuler(eout), TOL_HIGH);
	EulerNoFYaw(ec, eout);
	ExpectNear(ecf, eout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfEuler(eout), TOL_HIGH);
	EulerNoFYaw(ed, eout);
	ExpectNear(edf, eout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfEuler(eout), TOL_HIGH);
}

// Test the removal of yaw from fused angles
TEST(RotConvTest, testFusedNoYaw)
{
	// Define rotations
	FusedAngles fa(2.0562507858832344, -0.0438935143397104, -0.0726409065755999, true);
	FusedAngles fb(-0.0526581648921463, 0.2829589512327975, -0.8843554083240905, false);
	FusedAngles fc(1.9016989792547285, 0.1303744227992718, 1.4314908227654577, true);
	FusedAngles fd(2.4624562149459126, 0.8593130589551469, 0.6127762839408055, true);
	FusedAngles fae(-0.0015967314991432, fa.fusedPitch, fa.fusedRoll, fa.hemi);
	FusedAngles fbe(0.5489542041758281, fb.fusedPitch, fb.fusedRoll, fb.hemi);
	FusedAngles fce(-0.1241271860106767, fc.fusedPitch, fc.fusedRoll, fc.hemi);
	FusedAngles fde(-0.5347140087302225, fd.fusedPitch, fd.fusedRoll, fd.hemi);
	FusedAngles faf(0.0, fa.fusedPitch, fa.fusedRoll, fa.hemi);
	FusedAngles fbf(0.0, fb.fusedPitch, fb.fusedRoll, fb.hemi);
	FusedAngles fcf(0.0, fc.fusedPitch, fc.fusedRoll, fc.hemi);
	FusedAngles fdf(0.0, fd.fusedPitch, fd.fusedRoll, fd.hemi);
	FusedAngles fout;

	// Check removal of Euler yaw
	ExpectNear(fae, FusedNoEYaw(fa), TOL_HIGH);
	ExpectNear(fbe, FusedNoEYaw(fb), TOL_HIGH);
	ExpectNear(fce, FusedNoEYaw(fc), TOL_HIGH);
	ExpectNear(fde, FusedNoEYaw(fd), TOL_HIGH);
	FusedNoEYaw(fa, fout);
	ExpectNear(fae, fout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfFused(fout), TOL_HIGH);
	FusedNoEYaw(fb, fout);
	ExpectNear(fbe, fout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfFused(fout), TOL_HIGH);
	FusedNoEYaw(fc, fout);
	ExpectNear(fce, fout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfFused(fout), TOL_HIGH);
	FusedNoEYaw(fd, fout);
	ExpectNear(fde, fout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfFused(fout), TOL_HIGH);

	// Check removal of fused yaw
	ExpectNear(faf, FusedNoFYaw(fa), TOL_HIGH);
	ExpectNear(fbf, FusedNoFYaw(fb), TOL_HIGH);
	ExpectNear(fcf, FusedNoFYaw(fc), TOL_HIGH);
	ExpectNear(fdf, FusedNoFYaw(fd), TOL_HIGH);
	FusedNoFYaw(fa, fout);
	ExpectNear(faf, fout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfFused(fout), TOL_HIGH);
	FusedNoFYaw(fb, fout);
	ExpectNear(fbf, fout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfFused(fout), TOL_HIGH);
	FusedNoFYaw(fc, fout);
	ExpectNear(fcf, fout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfFused(fout), TOL_HIGH);
	FusedNoFYaw(fd, fout);
	ExpectNear(fdf, fout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfFused(fout), TOL_HIGH);
}

// Test the removal of yaw from tilt angles
TEST(RotConvTest, testTiltNoYaw)
{
	// Define rotations
	TiltAngles ta(2.0562507858832344, -2.5978042907802665, 0.0849125568718138);
	TiltAngles tb(-0.0526581648921463, 2.7951952483622455, 2.1760588150085263);
	TiltAngles tc(1.9016989792547285, 0.1305306954798606, 1.5219977311223241);
	TiltAngles td(2.4624562149459126, 0.9213266852112820, 1.2565085383834584);
	TiltAngles tae(-0.0015967314991432, ta.tiltAxisAngle, ta.tiltAngle);
	TiltAngles tbe(0.5489542041758281, tb.tiltAxisAngle, tb.tiltAngle);
	TiltAngles tce(-0.1241271860106767, tc.tiltAxisAngle, tc.tiltAngle);
	TiltAngles tde(-0.5347140087302229, td.tiltAxisAngle, td.tiltAngle);
	TiltAngles taf(0.0, ta.tiltAxisAngle, ta.tiltAngle);
	TiltAngles tbf(0.0, tb.tiltAxisAngle, tb.tiltAngle);
	TiltAngles tcf(0.0, tc.tiltAxisAngle, tc.tiltAngle);
	TiltAngles tdf(0.0, td.tiltAxisAngle, td.tiltAngle);
	TiltAngles tout;

	// Check removal of Euler yaw
	ExpectNear(tae, TiltNoEYaw(ta), TOL_HIGH);
	ExpectNear(tbe, TiltNoEYaw(tb), TOL_HIGH);
	ExpectNear(tce, TiltNoEYaw(tc), TOL_HIGH);
	ExpectNear(tde, TiltNoEYaw(td), TOL_HIGH);
	TiltNoEYaw(ta, tout);
	ExpectNear(tae, tout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfTilt(tout), TOL_HIGH);
	TiltNoEYaw(tb, tout);
	ExpectNear(tbe, tout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfTilt(tout), TOL_HIGH);
	TiltNoEYaw(tc, tout);
	ExpectNear(tce, tout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfTilt(tout), TOL_HIGH);
	TiltNoEYaw(td, tout);
	ExpectNear(tde, tout, TOL_HIGH);
	EXPECT_NEAR(0.0, EYawOfTilt(tout), TOL_HIGH);

	// Check removal of fused yaw
	ExpectNear(taf, TiltNoFYaw(ta), TOL_HIGH);
	ExpectNear(tbf, TiltNoFYaw(tb), TOL_HIGH);
	ExpectNear(tcf, TiltNoFYaw(tc), TOL_HIGH);
	ExpectNear(tdf, TiltNoFYaw(td), TOL_HIGH);
	TiltNoFYaw(ta, tout);
	ExpectNear(taf, tout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfTilt(tout), TOL_HIGH);
	TiltNoFYaw(tb, tout);
	ExpectNear(tbf, tout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfTilt(tout), TOL_HIGH);
	TiltNoFYaw(tc, tout);
	ExpectNear(tcf, tout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfTilt(tout), TOL_HIGH);
	TiltNoFYaw(td, tout);
	ExpectNear(tdf, tout, TOL_HIGH);
	EXPECT_NEAR(0.0, FYawOfTilt(tout), TOL_HIGH);
}

// ###########################
// #### Rotation inverses ####
// ###########################

// Test the inverse of a rotation matrix
TEST(RotConvTest, testRotmatInv)
{
	// Define rotations
	Rotmat Ra, Rb, Rai, Rbi, Rinv;
	Ra << -0.7913685098447312, 0.0702884213014767, 0.6072852867121651, 0.6106729927710146, 0.1372583855396122, 0.7798965517933463, -0.0285373006120527, 0.9880382955280658, -0.1515452046214996;
	Rb << -0.3419448972247667, 0.3265280019371199, -0.8811657909910574, 0.6932452401897411, -0.5453932245855906, -0.4711234100852978, -0.6344168379040801, -0.7719622364643776, -0.0398695530035709;
	Rai << -0.7913685098447312, 0.6106729927710146, -0.0285373006120527, 0.0702884213014767, 0.1372583855396122, 0.9880382955280658, 0.6072852867121651, 0.7798965517933463, -0.1515452046214996;
	Rbi << -0.3419448972247667, 0.6932452401897411, -0.6344168379040801, 0.3265280019371199, -0.5453932245855906, -0.7719622364643776, -0.8811657909910574, -0.4711234100852978, -0.0398695530035709;

	// Test the inverse rotations
	Rotmat ident = Rotmat::Identity();
	ExpectNear(Rai, RotmatInv(Ra), TOL_HIGH);
	ExpectNear(Rbi, RotmatInv(Rb), TOL_HIGH);
	RotmatInv(Ra, Rinv);
	ExpectNear(Rai, Rinv, TOL_HIGH);
	ExpectNear(ident, Rinv*Ra, TOL_HIGH);
	RotmatInv(Rb, Rinv);
	ExpectNear(Rbi, Rinv, TOL_HIGH);
	ExpectNear(ident, Rinv*Rb, TOL_HIGH);
}

// Test the inverse of a quaternion
TEST(RotConvTest, testQuatInv)
{
	// Define rotations
	Quat qa(-0.6266303755482459, 0.4943624879864148, 0.4934278926895020, 0.3456429047849006);
	Quat qb(-0.5739766195619466, 0.2616413648997640, 0.0304827444588237, 0.7753485917056193);
	Quat qai(-0.6266303755482459, -0.4943624879864148, -0.4934278926895020, -0.3456429047849006);
	Quat qbi(-0.5739766195619466, -0.2616413648997640, -0.0304827444588237, -0.7753485917056193);
	Quat qinv;

	// Test the inverse rotations
	Quat ident = Quat::Identity();
	ExpectNear(qai, QuatInv(qa), TOL_HIGH);
	ExpectNear(qbi, QuatInv(qb), TOL_HIGH);
	QuatInv(qa, qinv);
	ExpectNear(qai, qinv, TOL_HIGH);
	ExpectNear(ident, qinv*qa, TOL_HIGH);
	QuatInv(qb, qinv);
	ExpectNear(qbi, qinv, TOL_HIGH);
	ExpectNear(ident, qinv*qb, TOL_HIGH);
}

// Test the inverse of Euler angles
TEST(RotConvTest, testEulerInv)
{
	// Define rotations
	EulerAngles ea(0.9361411579050304, 0.9435163291755003, -0.2902975576850511);
	EulerAngles eb(-0.4247967100682601, 1.0220034297464795, -2.6171363392752642);
	EulerAngles eai(-1.2051662555132063, -0.2315050199457343, 0.9547648446386308);
	EulerAngles ebi(-1.0035170045482087, 0.4853059347612895, 2.1065510697680798);
	EulerAngles einv;

	// Test the inverse rotations
	Rotmat ident = Rotmat::Identity();
	ExpectNear(eai, EulerInv(ea), TOL_HIGH);
	ExpectNear(ebi, EulerInv(eb), TOL_HIGH);
	EulerInv(ea, einv);
	ExpectNear(eai, einv, TOL_HIGH);
	ExpectNear(ident, RotmatFromEuler(einv) * RotmatFromEuler(ea), TOL_HIGH);
	EulerInv(eb, einv);
	ExpectNear(ebi, einv, TOL_HIGH);
	ExpectNear(ident, RotmatFromEuler(einv) * RotmatFromEuler(eb), TOL_HIGH);
}

// Test the inverse of fused angles
TEST(RotConvTest, testFusedInv)
{
	// Define rotations
	FusedAngles fa(-2.3048545352501737, -0.6843544663041080, -0.3417254672999978, true);
	FusedAngles fb(1.9060946880379266, -0.8486557811537475, -0.5321647334070991, false);
	FusedAngles fai(2.3048545352501737, -0.7373033750636966, 0.2473917461632297, true);
	FusedAngles fbi(-1.9060946880379266, 0.2343661741468948, -1.0666046177278012, false);
	FusedAngles finv;

	// Test the inverse rotations
	Rotmat ident = Rotmat::Identity();
	ExpectNear(fai, FusedInv(fa), TOL_HIGH);
	ExpectNear(fbi, FusedInv(fb), TOL_HIGH);
	FusedInv(fa, finv);
	ExpectNear(fai, finv, TOL_HIGH);
	ExpectNear(ident, RotmatFromFused(finv) * RotmatFromFused(fa), TOL_HIGH);
	FusedInv(fb, finv);
	ExpectNear(fbi, finv, TOL_HIGH);
	ExpectNear(ident, RotmatFromFused(finv) * RotmatFromFused(fb), TOL_HIGH);
}

// Test the inverse of tilt angles
TEST(RotConvTest, testTiltInv)
{
	// Define rotations
	TiltAngles ta(-0.5227643606468035, 0.9855797622517890, 1.9728364918817358);
	TiltAngles tb(-1.3070025724439964, -0.4294483628561599, 0.0486542401233392);
	TiltAngles tai(0.5227643606468035, -2.6787772519848074, 1.9728364918817358);
	TiltAngles tbi(1.3070025724439964, 1.4051417182896362, 0.0486542401233392);
	TiltAngles tinv;

	// Test the inverse rotations
	Rotmat ident = Rotmat::Identity();
	ExpectNear(tai, TiltInv(ta), TOL_HIGH);
	ExpectNear(tbi, TiltInv(tb), TOL_HIGH);
	TiltInv(ta, tinv);
	ExpectNear(tai, tinv, TOL_HIGH);
	ExpectNear(ident, RotmatFromTilt(tinv) * RotmatFromTilt(ta), TOL_HIGH);
	TiltInv(tb, tinv);
	ExpectNear(tbi, tinv, TOL_HIGH);
	ExpectNear(ident, RotmatFromTilt(tinv) * RotmatFromTilt(tb), TOL_HIGH);
}

// ##########################
// #### Vector rotations ####
// ##########################

// Test vector rotations by a rotation matrix
TEST(RotConvTest, testRotmatRotVec)
{
	// Define rotations and vectors
	Rotmat Ra, Rb;
	Ra << -0.5261472043537714, 0.5297129577173956, -0.6652618294904640, 0.2370219307688934, -0.6599506543167390, -0.7129416092510618, -0.8166943881803623, -0.5327938778676012, 0.2216776037887165;
	Rb << 0.0851884417222457, -0.9466832821901454, -0.3106987167959164, 0.0272483903469391, -0.3095020891756364, 0.9505082756185858, -0.9959921960640237, -0.0894383587595945, -0.0005704050786135;
	Vec3 va(-0.6166368999817718, -0.7295627565412773, -0.2363864029248184);
	Vec3 vb(-0.5952124824635381, 0.6028681598642963, 0.4986624453213788);
	Vec3 vao(0.0952417862952838, 0.5038486524338052, 0.8399088945968446);
	Vec3 vbo(-0.7763642140586299, 0.2711752439811628, 0.5386230091755737);

	// Test the vector rotation
	ExpectNear(vao, RotmatRotVec(Ra, va), TOL_HIGH);
	ExpectNear(vbo, RotmatRotVec(Rb, vb), TOL_HIGH);
}

// Test vector rotations by a quaternion
TEST(RotConvTest, testQuatRotVec)
{
	// Define rotations and vectors
	Quat qa(0.0943129698374057, 0.4775263987923211, 0.4014096866819223, -0.7758504144581008);
	Quat qb(0.4402033471783229, -0.5906058194719418, 0.3891914291774458, 0.5531146450725151);
	Vec3 va(-0.6166368999817718, -0.7295627565412773, -0.2363864029248184);
	Vec3 vb(-0.5952124824635381, 0.6028681598642963, 0.4986624453213788);
	Vec3 vao(0.0952417862952838, 0.5038486524338052, 0.8399088945968446);
	Vec3 vbo(-0.7763642140586299, 0.2711752439811628, 0.5386230091755737);

	// Test the vector rotation
	ExpectNear(vao, QuatRotVec(qa, va), TOL_HIGH);
	ExpectNear(vbo, QuatRotVec(qb, vb), TOL_HIGH);
}

// Test vector rotations by Euler angles
TEST(RotConvTest, testEulerRotVec)
{
	// Define rotations and vectors
	EulerAngles ea(2.7183346725614341, 0.9556593133855379, -1.1765168289632895);
	EulerAngles eb(0.3095761074953327, 1.4812364710239367, -1.5771738738912546);
	Vec3 va(-0.6166368999817718, -0.7295627565412773, -0.2363864029248184);
	Vec3 vb(-0.5952124824635381, 0.6028681598642963, 0.4986624453213788);
	Vec3 vao(0.0952417862952838, 0.5038486524338052, 0.8399088945968446);
	Vec3 vbo(-0.7763642140586299, 0.2711752439811628, 0.5386230091755737);

	// Test the vector rotation
	ExpectNear(vao, EulerRotVec(ea, va), TOL_HIGH);
	ExpectNear(vbo, EulerRotVec(eb, vb), TOL_HIGH);
}

// Test vector rotations by fused angles
TEST(RotConvTest, testFusedRotVec)
{
	// Define rotations and vectors
	FusedAngles fa(-2.8996581541484874, 0.9556593133855379, -0.5618986469577061, true);
	FusedAngles fb(1.7971663779554685, 1.4812364710239367, -0.0895580295531960, false);
	Vec3 va(-0.6166368999817718, -0.7295627565412773, -0.2363864029248184);
	Vec3 vb(-0.5952124824635381, 0.6028681598642963, 0.4986624453213788);
	Vec3 vao(0.0952417862952838, 0.5038486524338052, 0.8399088945968446);
	Vec3 vbo(-0.7763642140585979, 0.2711752439810655, 0.5386230091756689);

	// Test the vector rotation
	ExpectNear(vao, FusedRotVec(fa, va), TOL_HIGH);
	ExpectNear(vbo, FusedRotVec(fb, vb), TOL_HIGH);
}

// Test vector rotations by tilt angles
TEST(RotConvTest, testTiltRotVec)
{
	// Define rotations and vectors
	TiltAngles ta(-2.8996581541484874, 2.1488418022086417, 1.3472617842450922);
	TiltAngles tb(1.7971663779554685, 1.6603543709565614, 1.5713667319044415);
	Vec3 va(-0.6166368999817718, -0.7295627565412773, -0.2363864029248184);
	Vec3 vb(-0.5952124824635381, 0.6028681598642963, 0.4986624453213788);
	Vec3 vao(0.0952417862952838, 0.5038486524338052, 0.8399088945968446);
	Vec3 vbo(-0.7763642140586299, 0.2711752439811628, 0.5386230091755737);

	// Test the vector rotation
	ExpectNear(vao, TiltRotVec(ta, va), TOL_HIGH);
	ExpectNear(vbo, TiltRotVec(tb, vb), TOL_HIGH);
}

// ############################################
// #### Conversions from rotation matrices ####
// ############################################

// Helper function: Rotation matrix --> Quaternion
void checkQuatFromRotmat(const Rotmat& R, const Quat& qtrue)
{
	// Declare variables
	Quat qout;

	// Test the various function overloads
	ExpectNear(qtrue, QuatFromRotmat(R), TOL_HIGH);
	QuatFromRotmat(R, qout);
	ExpectNear(qtrue, qout, TOL_HIGH);
}

// Test conversion: Rotation matrix --> Quaternion
TEST(RotConvTest, testQuatFromRotmat)
{
	// Declare variables
	Rotmat R;
	Quat qtrue;

	// Test the conversions
	R.setIdentity();
	qtrue.setIdentity();
	checkQuatFromRotmat(R, qtrue);
	R << 0.1414327180943838, 0.7124388355699762, -0.6873337557723549, -0.9899305838779120, 0.0976803964897218, -0.1024498865043154, -0.0058502440607973, 0.6949024720874919, 0.7190801964531646;
	qtrue = Quat(0.6996773025897850, 0.2849000373602545, -0.2434992207082876, -0.6082694883579686);
	checkQuatFromRotmat(R, qtrue);
	R << 0.1566008414562970, 0.7287389955948977, 0.6666450725494935, 0.9519507232286177, 0.0684925409730932, -0.2984938732630218, -0.2631843403390321, 0.6813576506728231, -0.6829976243515929;
	qtrue = Quat(0.3681357623478726, 0.6654145183332711, 0.6314446380856340, 0.1515824802038619);
	checkQuatFromRotmat(R, qtrue);
	R << -0.8984411235873682, -0.3468697916607746, -0.2692301897638494, 0.1416850441552862, 0.3513360525272200, -0.9254665452934023, 0.4156066598477244, -0.8696230941205514, -0.2665084960417143;
	qtrue = Quat(-0.2158624752580549, -0.0646748017529359, 0.7931402259620146, -0.5658172352929950);
	checkQuatFromRotmat(R, qtrue);
	R << -0.4711772747166694, 0.3715968214679360, 0.7999423592143993, -0.8760231242927109, -0.0914114756145323, -0.4735265861917483, -0.1028370628470578, -0.9238829711607962, 0.3685984184774668;
	qtrue = Quat(-0.4488902059931428, 0.2508165576773173, -0.5027840939768954, 0.6948358023318654);
	checkQuatFromRotmat(R, qtrue);
}

// Helper function: Rotation matrix --> Euler angles
void checkEulerFromRotmat(const Rotmat& R, const EulerAngles& etrue)
{
	// Declare variables
	double yaw, pitch, roll;
	EulerAngles eout;

	// Test the various function overloads
	yaw = pitch = roll = -100.0;
	EulerFromRotmat(R, yaw);
	EXPECT_NEAR(etrue.yaw, yaw, TOL_HIGH);
	yaw = pitch = roll = -100.0;
	EulerFromRotmat(R, yaw, pitch, roll);
	EXPECT_NEAR(etrue.yaw, yaw, TOL_HIGH);
	EXPECT_NEAR(etrue.pitch, pitch, TOL_HIGH);
	EXPECT_NEAR(etrue.roll, roll, TOL_HIGH);
	ExpectNear(etrue, EulerFromRotmat(R), TOL_HIGH);
	EulerFromRotmat(R, eout);
	ExpectNear(etrue, eout, TOL_HIGH);
}

// Test conversion: Rotation matrix --> Euler angles
TEST(RotConvTest, testEulerFromRotmat)
{
	// Declare variables
	Rotmat R;
	EulerAngles etrue;

	// Test the conversions
	R.setIdentity();
	etrue.setIdentity();
	checkEulerFromRotmat(R, etrue);
	R << 0.1414327180943838, 0.7124388355699762, -0.6873337557723549, -0.9899305838779120, 0.0976803964897218, -0.1024498865043154, -0.0058502440607973, 0.6949024720874919, 0.7190801964531646;
	etrue.set(-1.4288853500406331, 0.0058502774324251, 0.7683008050048191);
	checkEulerFromRotmat(R, etrue);
	R << 0.1566008414562970, 0.7287389955948977, 0.6666450725494935, 0.9519507232286177, 0.0684925409730932, -0.2984938732630218, -0.2631843403390321, 0.6813576506728231, -0.6829976243515929;
	etrue.set(1.4077514350384410, 0.2663214290123691, 2.3573965032742450);
	checkEulerFromRotmat(R, etrue);
	R << -0.8984411235873682, -0.3468697916607746, -0.2692301897638494, 0.1416850441552862, 0.3513360525272200, -0.9254665452934023, 0.4156066598477244, -0.8696230941205514, -0.2665084960417143;
	etrue.set(2.9851798277226744, -0.4286096944862452, -1.8681731363605480);
	checkEulerFromRotmat(R, etrue);
	R << -0.4711772747166694, 0.3715968214679360, 0.7999423592143993, -0.8760231242927109, -0.0914114756145323, -0.4735265861917483, -0.1028370628470578, -0.9238829711607962, 0.3685984184774668;
	etrue.set(-2.0642707853655820, 0.1030191891497684, -1.1911811564647661);
	checkEulerFromRotmat(R, etrue);

}

// Helper function: Rotation matrix --> Fused angles
void checkFusedFromRotmat(const Rotmat& R, const FusedAngles& ftrue)
{
	// Declare variables
	double fusedYaw, fusedPitch, fusedRoll;
	bool hemi;

	// Test the various function overloads
	fusedYaw = fusedPitch = fusedRoll = -100.0; hemi = false;
	FusedFromRotmat(R, fusedYaw);
	EXPECT_NEAR(ftrue.fusedYaw, fusedYaw, TOL_HIGH);
	fusedYaw = fusedPitch = fusedRoll = -100.0; hemi = false;
	FusedFromRotmat(R, fusedPitch, fusedRoll);
	EXPECT_NEAR(ftrue.fusedPitch, fusedPitch, TOL_HIGH);
	EXPECT_NEAR(ftrue.fusedRoll, fusedRoll, TOL_HIGH);
	fusedYaw = fusedPitch = fusedRoll = -100.0; hemi = false;
	FusedFromRotmat(R, fusedYaw, fusedPitch, fusedRoll);
	EXPECT_NEAR(ftrue.fusedYaw, fusedYaw, TOL_HIGH);
	EXPECT_NEAR(ftrue.fusedPitch, fusedPitch, TOL_HIGH);
	EXPECT_NEAR(ftrue.fusedRoll, fusedRoll, TOL_HIGH);
	fusedYaw = fusedPitch = fusedRoll = -100.0; hemi = false;
	FusedFromRotmat(R, fusedYaw, fusedPitch, fusedRoll, hemi);
	EXPECT_NEAR(ftrue.fusedYaw, fusedYaw, TOL_HIGH);
	EXPECT_NEAR(ftrue.fusedPitch, fusedPitch, TOL_HIGH);
	EXPECT_NEAR(ftrue.fusedRoll, fusedRoll, TOL_HIGH);
	EXPECT_TRUE(ftrue.hemi == hemi);
	FusedAngles f = FusedFromRotmat(R);
	ExpectNear(ftrue, f, TOL_HIGH);
	f.fusedYaw = f.fusedPitch = f.fusedRoll = -100.0; f.hemi = false;
	FusedFromRotmat(R, f);
	ExpectNear(ftrue, f, TOL_HIGH);
}

// Test conversion: Rotation matrix --> Fused angles
TEST(RotConvTest, testFusedFromRotmat)
{
	// Declare variables
	Rotmat R;
	FusedAngles ftrue;

	// Test the conversions
	R.setIdentity();
	ftrue.setIdentity();
	checkFusedFromRotmat(R, ftrue);
	R << 0.1783659175469134, 0.8565242905862485, 0.4843054192277552, 0.1514540974716472, 0.4624339622672285, -0.8736226227043806, -0.9722382711129494, 0.2291745408581753, -0.0472416554086341;
	ftrue.set(-0.8331156414245902, 1.3346135980822995, 0.2312295689401531, false);
	checkFusedFromRotmat(R, ftrue);
	R << 0.8650848400897053, -0.4638362214106089, 0.1910083222126657, -0.2111356361227564, -0.6820929015748582, -0.7001221441864488, 0.4550274306083184, 0.5653363895316842, -0.6880005843495141;
	ftrue.set(0.9440485983371594, -0.4724030182703050, 0.6008410104827017, false);
	checkFusedFromRotmat(R, ftrue);
	R << -0.6675585829283448, 0.5098346479055411, 0.5426178859508163, -0.5068702586599423, 0.2226451269321638, -0.8327734916166363, -0.5453880079766177, -0.8309619601692930, 0.1097913544269455;
	ftrue.set(-1.9832934972315632, 0.5768519602564895, -0.9808345783548854, true);
	checkFusedFromRotmat(R, ftrue);
	R << -0.5325551619535422, -0.5998075586842398, 0.5971732512612138, -0.5909022738852802, -0.2416669623589607, -0.7696957723811390, 0.6059863877958331, -0.7627764887891112, -0.2257266620421992;
	ftrue.set(3.1300909258255607, -0.6510052810944137, -0.8675958792259617, false);
	checkFusedFromRotmat(R, ftrue);
}

// Helper function: Rotation matrix --> Tilt angles
void checkTiltFromRotmat(const Rotmat& R, const TiltAngles& ttrue)
{
	// Declare variables
	double fusedYaw, tiltAxisAngle, tiltAngle;

	// Test the various function overloads
	fusedYaw = tiltAxisAngle = tiltAngle = -100.0;
	TiltFromRotmat(R, fusedYaw);
	EXPECT_NEAR(ttrue.fusedYaw, fusedYaw, TOL_HIGH);
	fusedYaw = tiltAxisAngle = tiltAngle = -100.0;
	TiltFromRotmat(R, tiltAxisAngle, tiltAngle);
	EXPECT_NEAR(ttrue.tiltAxisAngle, tiltAxisAngle, TOL_HIGH);
	EXPECT_NEAR(ttrue.tiltAngle, tiltAngle, TOL_HIGH);
	fusedYaw = tiltAxisAngle = tiltAngle = -100.0;
	TiltFromRotmat(R, fusedYaw, tiltAxisAngle, tiltAngle);
	EXPECT_NEAR(ttrue.fusedYaw, fusedYaw, TOL_HIGH);
	EXPECT_NEAR(ttrue.tiltAxisAngle, tiltAxisAngle, TOL_HIGH);
	EXPECT_NEAR(ttrue.tiltAngle, tiltAngle, TOL_HIGH);
	TiltAngles t = TiltFromRotmat(R);
	ExpectNear(ttrue, t, TOL_HIGH);
	t.fusedYaw = t.tiltAxisAngle = t.tiltAngle = -100.0;
	TiltFromRotmat(R, t);
	ExpectNear(ttrue, t, TOL_HIGH);
}

// Test conversion: Rotation matrix --> Tilt angles
TEST(RotConvTest, testTiltFromRotmat)
{
	// Declare variables
	Rotmat R;
	TiltAngles ttrue;

	// Test the conversions
	R.setIdentity();
	ttrue.setIdentity();
	checkTiltFromRotmat(R, ttrue);
	R << -0.3156072068346027, -0.3408609739669622, -0.8855540002847676, 0.9042693503036098, 0.1747767554159254, -0.3895510593847349, 0.2875570084271079, -0.9237244622681462, 0.2530693278765259;
	ttrue.set(1.6834226766356468, -2.8397998341990722, 1.3149447797732721);
	checkTiltFromRotmat(R, ttrue);
	R << 0.7450384997308501, 0.2660892881068550, 0.6116486938378852, -0.1215335122038756, -0.8474824997798944, 0.5167233476225078, 0.6558561117579026, -0.4593146016856069, -0.5990683244407078;
	ttrue.set(-1.8291765277467400, -2.1817425988018928, 2.2131333490376615);
	checkTiltFromRotmat(R, ttrue);
	R << -0.5592056290071654, 0.7007546382688931, 0.4429808138411117, -0.3611178926635866, 0.2750848525346579, -0.8910231150223860, -0.7462458925078992, -0.6582334394807199, 0.0992260402539099;
	ttrue.set(-1.8322379731555882, 2.2936107879584866, 1.4714067341208246);
	checkTiltFromRotmat(R, ttrue);
	R << -0.3110156395790578, -0.7430003273048269, -0.5926379886255604, -0.0161620700874716, -0.6193388262223538, 0.7849574547859297, -0.9502673620764408, 0.2537123015512597, 0.1806156377887733;
	ttrue.set(2.4783904412818125, 1.3098916809871868, 1.3891839794574117);
	checkTiltFromRotmat(R, ttrue);
}

// Helper function: Rotation matrix --> Z vector
void checkZVecFromRotmat(const Rotmat& R, const ZVec& ztrue)
{
	// Declare variables
	ZVec zout;

	// Test the various function overloads
	ExpectNear(ztrue, ZVecFromRotmat(R), TOL_HIGH);
	ZVecFromRotmat(R, zout);
	ExpectNear(ztrue, zout, TOL_HIGH);
}

// Test conversion: Rotation matrix --> Z vector
TEST(RotConvTest, testZVecFromRotmat)
{
	// Declare variables
	Rotmat R;
	ZVec ztrue;

	// Test the conversions
	R.setIdentity();
	ztrue << 0.0, 0.0, 1.0;
	checkZVecFromRotmat(R, ztrue);
	R << 0.0999336102319131, 0.9938753635299167, 0.0471702799895468, 0.1759525137059373, 0.0290082901464270, -0.9839711540605968, -0.9793130176131521, 0.1066315191257109, -0.1719759653625037;
	ztrue << -0.9793130176131521, 0.1066315191257109, -0.1719759653625037;
	checkZVecFromRotmat(R, ztrue);
	R << -0.9462415419501677, -0.0789123384173683, -0.3136874035298052, 0.2061065938711324, 0.6003241125225014, -0.7727425391984779, 0.2492930328962448, -0.7958541341034635, -0.5517872606175476;
	ztrue << 0.2492930328962448, -0.7958541341034635, -0.5517872606175476;
	checkZVecFromRotmat(R, ztrue);
	R << -0.8017286274718856, 0.1982542542121769, -0.5638496772889212, -0.2260514152559287, -0.9738888002752906, -0.0210086258272107, -0.5532919351959134, 0.1106158007932393, 0.8256101859001206;
	ztrue << -0.5532919351959134, 0.1106158007932393, 0.8256101859001206;
	checkZVecFromRotmat(R, ztrue);
}

// ######################################
// #### Conversions from quaternions ####
// ######################################

// Helper function: Quaternion --> Rotation matrix
void checkRotmatFromQuat(const Quat& q, const Rotmat& Rtrue)
{
	// Declare variables
	Rotmat Rout;

	// Test the various function overloads
	ExpectNear(Rtrue, RotmatFromQuat(q), TOL_HIGH);
	RotmatFromQuat(q, Rout);
	ExpectNear(Rtrue, Rout, TOL_HIGH);
}

// Test conversion: Quaternion --> Rotation matrix
TEST(RotConvTest, testRotmatFromQuat)
{
	// Declare variables
	Quat q;
	Rotmat Rtrue;

	// Test the conversions
	q.setIdentity();
	Rtrue.setIdentity();
	checkRotmatFromQuat(q, Rtrue);
	q = Quat(-0.6963650210050275, 0.3430383715579999, -0.3095946706841617, 0.5491371167964049);
	Rtrue << 0.2051991336809942, 0.5523940563960773, 0.8079320033238295, -0.9772054630941976, 0.1615462051907344, 0.1377399959609681, -0.0544315940922537, -0.8177796953011001, 0.5729516310456021;
	checkRotmatFromQuat(q, Rtrue);
	q = Quat(0.3538368966498407, 0.4975179488949202, 0.1108456621231138, 0.7842120760898099);
	Rtrue << -0.2545506821932026, -0.4446709217111123, 0.8587617373751375, 0.6652617475646762, -0.7250253795153967, -0.1782274004909119, 0.7018765970041798, 0.5259334277673787, 0.4803782594317600;
	checkRotmatFromQuat(q, Rtrue);
	q = Quat(-0.0427835998400327, -0.6862801891011874, 0.2272596511760396, 0.6895956181555327);
	Rtrue << -0.0543781312639329, -0.2529208268111121, -0.9659575944136606, -0.3709343587255373, -0.8930452290641455, 0.2547114452918091, -0.9270656505108562, 0.3721575932463979, -0.0452548940108333;
	checkRotmatFromQuat(q, Rtrue);
	q = Quat(0.4052136149127184, -0.8916879388893326, -0.1910389910776326, 0.0647969892564565);
	Rtrue << 0.9186109081426778, 0.2881810838988851, -0.2703805879203400, 0.3932075729071624, -0.5986120603548151, 0.6978906832779919, 0.0392658127350443, -0.7474056890876786, -0.6632065529453319;
	checkRotmatFromQuat(q, Rtrue);
}

// Helper function: Quaternion --> Euler angles
void checkEulerFromQuat(const Quat& q, const EulerAngles& etrue)
{
	// Declare variables
	double yaw, pitch, roll;
	EulerAngles eout;

	// Test the various function overloads
	yaw = pitch = roll = -100.0;
	EulerFromQuat(q, yaw);
	EXPECT_NEAR(etrue.yaw, yaw, TOL_HIGH);
	yaw = pitch = roll = -100.0;
	EulerFromQuat(q, yaw, pitch, roll);
	EXPECT_NEAR(etrue.yaw, yaw, TOL_HIGH);
	EXPECT_NEAR(etrue.pitch, pitch, TOL_HIGH);
	EXPECT_NEAR(etrue.roll, roll, TOL_HIGH);
	ExpectNear(etrue, EulerFromQuat(q), TOL_HIGH);
	EulerFromQuat(q, eout);
	ExpectNear(etrue, eout, TOL_HIGH);
}

// Test conversion: Quaternion --> Euler angles
TEST(RotConvTest, testEulerFromQuat)
{
	// Declare variables
	Quat q;
	EulerAngles etrue;

	// Test the conversions
	q.setIdentity();
	etrue.setIdentity();
	checkEulerFromQuat(q, etrue);
	q = Quat(-0.6963650210050275, 0.3430383715579999, -0.3095946706841617, 0.5491371167964049);
	etrue.set(-1.3638178673649697, 0.0544585082983020, -0.9596553433028447);
	checkEulerFromQuat(q, etrue);
	q = Quat(0.3538368966498407, 0.4975179488949202, 0.1108456621231138, 0.7842120760898099);
	etrue.set(1.9362415463259968, -0.7780286539987713, 0.8306367192520472);
	checkEulerFromQuat(q, etrue);
	q = Quat(-0.0427835998400327, -0.6862801891011874, 0.2272596511760396, 0.6895956181555327);
	etrue.set(-1.7163572119321702, 1.1865084726270507, 1.6918036452446508);
	checkEulerFromQuat(q, etrue);
	q = Quat(0.4052136149127184, -0.8916879388893326, -0.1910389910776326, 0.0647969892564565);
	etrue.set(0.4044476655652079, -0.0392759097735742, -2.2965754467720090);
	checkEulerFromQuat(q, etrue);
}

// Helper function: Quaternion --> Fused angles
void checkFusedFromQuat(const Quat& q, const FusedAngles& ftrue)
{
	// Declare variables
	double fusedYaw, fusedPitch, fusedRoll;
	bool hemi;

	// Test the various function overloads
	fusedYaw = fusedPitch = fusedRoll = -100.0; hemi = false;
	FusedFromQuat(q, fusedYaw);
	EXPECT_NEAR(ftrue.fusedYaw, fusedYaw, TOL_HIGH);
	fusedYaw = fusedPitch = fusedRoll = -100.0; hemi = false;
	FusedFromQuat(q, fusedPitch, fusedRoll);
	EXPECT_NEAR(ftrue.fusedPitch, fusedPitch, TOL_HIGH);
	EXPECT_NEAR(ftrue.fusedRoll, fusedRoll, TOL_HIGH);
	fusedYaw = fusedPitch = fusedRoll = -100.0; hemi = false;
	FusedFromQuat(q, fusedYaw, fusedPitch, fusedRoll);
	EXPECT_NEAR(ftrue.fusedYaw, fusedYaw, TOL_HIGH);
	EXPECT_NEAR(ftrue.fusedPitch, fusedPitch, TOL_HIGH);
	EXPECT_NEAR(ftrue.fusedRoll, fusedRoll, TOL_HIGH);
	fusedYaw = fusedPitch = fusedRoll = -100.0; hemi = false;
	FusedFromQuat(q, fusedYaw, fusedPitch, fusedRoll, hemi);
	EXPECT_NEAR(ftrue.fusedYaw, fusedYaw, TOL_HIGH);
	EXPECT_NEAR(ftrue.fusedPitch, fusedPitch, TOL_HIGH);
	EXPECT_NEAR(ftrue.fusedRoll, fusedRoll, TOL_HIGH);
	EXPECT_TRUE(ftrue.hemi == hemi);
	FusedAngles f = FusedFromQuat(q);
	ExpectNear(ftrue, f, TOL_HIGH);
	f.fusedYaw = f.fusedPitch = f.fusedRoll = -100.0; f.hemi = false;
	FusedFromQuat(q, f);
	ExpectNear(ftrue, f, TOL_HIGH);
}

// Test conversion: Quaternion --> Fused angles
TEST(RotConvTest, testFusedFromQuat)
{
	// Declare variables
	Quat q;
	FusedAngles ftrue;

	// Test the conversions
	q.setIdentity();
	ftrue.setIdentity();
	checkFusedFromQuat(q, ftrue);
	q = Quat(0.7222856332669739, 0.5029423807110762, 0.3807601824433657, 0.2835032788637483);
	ftrue.set(0.7480631709823684, 0.2680624583862969, 1.2298230154711272, true);
	checkFusedFromQuat(q, ftrue);
	q = Quat(-0.1177398416985938, 0.6616531189965961, -0.3860555283890073, -0.6319126591540564);
	ftrue.set( 2.7731713536579932, 1.1866593328830430,  0.3385300011717289, false);
	checkFusedFromQuat(q, ftrue);
}

// Helper function: Quaternion --> Tilt angles
void checkTiltFromQuat(const Quat& q, const TiltAngles& ttrue)
{
	// Declare variables
	double fusedYaw, tiltAxisAngle, tiltAngle;

	// Test the various function overloads
	fusedYaw = tiltAxisAngle = tiltAngle = -100.0;
	TiltFromQuat(q, fusedYaw);
	EXPECT_NEAR(ttrue.fusedYaw, fusedYaw, TOL_HIGH);
	fusedYaw = tiltAxisAngle = tiltAngle = -100.0;
	TiltFromQuat(q, tiltAxisAngle, tiltAngle);
	EXPECT_NEAR(ttrue.tiltAxisAngle, tiltAxisAngle, TOL_HIGH);
	EXPECT_NEAR(ttrue.tiltAngle, tiltAngle, TOL_HIGH);
	fusedYaw = tiltAxisAngle = tiltAngle = -100.0;
	TiltFromQuat(q, fusedYaw, tiltAxisAngle, tiltAngle);
	EXPECT_NEAR(ttrue.fusedYaw, fusedYaw, TOL_HIGH);
	EXPECT_NEAR(ttrue.tiltAxisAngle, tiltAxisAngle, TOL_HIGH);
	EXPECT_NEAR(ttrue.tiltAngle, tiltAngle, TOL_HIGH);
	TiltAngles t = TiltFromQuat(q);
	ExpectNear(ttrue, t, TOL_HIGH);
	t.fusedYaw = t.tiltAxisAngle = t.tiltAngle = -100.0;
	TiltFromQuat(q, t);
	ExpectNear(ttrue, t, TOL_HIGH);
}

// Test conversion: Quaternion --> Tilt angles
TEST(RotConvTest, testTiltFromQuat)
{
	// Declare variables
	Quat q;
	TiltAngles ttrue;

	// Test the conversions
	q.setIdentity();
	ttrue.setIdentity();
	checkTiltFromQuat(q, ttrue);
	q = Quat(0.7961214638267942, 0.2664622853518403, 0.4718794447199803, 0.2692921368516353);
	ttrue.set(0.6523471503703271, 0.7305765598723097, 1.1454291345775982);
	checkTiltFromQuat(q, ttrue);
	q = Quat(0.3366969741955911, -0.0255487574246867, 0.9188176663497852, 0.2043440837536595);
	ttrue.set(1.0909668560335302, 1.0531118591298443, 2.3319501044970479);
	checkTiltFromQuat(q, ttrue);
}

// Helper function: Quaternion --> Z vector
void checkZVecFromQuat(const Quat& q, const ZVec& ztrue)
{
	// Declare variables
	ZVec zout;

	// Test the various function overloads
	ExpectNear(ztrue, ZVecFromQuat(q), TOL_HIGH);
	ZVecFromQuat(q, zout);
	ExpectNear(ztrue, zout, TOL_HIGH);
}

// Test conversion: Quaternion --> Z vector
TEST(RotConvTest, testZVecFromQuat)
{
	// Declare variables
	Quat q;
	ZVec ztrue;

	// Test the conversions
	q.setIdentity();
	ztrue << 0.0, 0.0, 1.0;
	checkZVecFromQuat(q, ztrue);
	q = Quat(0.4720522657429341, 0.0362643261355121, 0.4757794728749056, -0.7412728581620781);
	ztrue << -0.5029490776967417, -0.6711275047898380, 0.5446375836816284;
	checkZVecFromQuat(q, ztrue);
	q = Quat(0.7117956329130622, 0.1258779217353490, -0.3086564320171634, -0.6182498950747317);
	ztrue << 0.2837525769504792, 0.5608523233602919, 0.7777719115880471;
	checkZVecFromQuat(q, ztrue);
	q = Quat(0.4501615898560536, -0.6666740148091347, -0.4767187144515561, -0.3544567227295818);
	ztrue << 0.9018150816597303, -0.2622697624676808, -0.3434299494601492;
	checkZVecFromQuat(q, ztrue);
}

// #######################################
// #### Conversions from Euler angles ####
// #######################################

// Helper function: Euler angles --> Rotation matrix
void checkRotmatFromEuler(const EulerAngles& e, const Rotmat& R0, const Rotmat& R1)
{
	// Test the various function overloads
	ExpectNear(R0, RotmatFromEuler(e.yaw), TOL_HIGH);
	ExpectNear(R1, RotmatFromEuler(e.yaw, e.pitch, e.roll), TOL_HIGH);
	ExpectNear(R1, RotmatFromEuler(e), TOL_HIGH);
}

// Test conversion: Euler angles --> Rotation matrix
TEST(RotConvTest, testRotmatFromEuler)
{
	// Declare variables
	EulerAngles e;
	Rotmat R0, R1;

	// Test the conversions
	e.setIdentity();
	R0.setIdentity();
	R1.setIdentity();
	checkRotmatFromEuler(e, R0, R1);
	e.set(-1.7339076236950002, 0.8812784802083941, -0.7114375669352448);
	R0 << -0.1623889877322682, 0.9867268196736568, -0.0000000000000000, -0.9867268196736568, -0.1623889877322682, 0.0000000000000000, -0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
	R1 << -0.1033062308722450, 0.8291764708846633, 0.5493579914720689, -0.6277213132771279, 0.3740809330145921, -0.6826634664405077, -0.7715528339139247, -0.4153671095382824, 0.4818468520113048;
	checkRotmatFromEuler(e, R0, R1);
	e.set(1.0545241244364663, 0.5508219296894964, 2.6137500730419276);
	R0 << 0.4936416386077898, -0.8696654141867528, 0.0000000000000000, 0.8696654141867528, 0.4936416386077898, 0.0000000000000000, -0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
	R1 << 0.4206293854464470, 0.8814316089866747, 0.2148237388607373, 0.7410372223564511, -0.1971979595388462, -0.6418541889214024, -0.5233877675119855, 0.4291751197738255, -0.7361208877527811;
	checkRotmatFromEuler(e, R0, R1);
	e.set(2.1638797377996237, -1.5496995446654214, -3.1343603483484621);
	R0 << -0.5589205057093048, -0.8292212420685176, 0.0000000000000000, 0.8292212420685176, -0.5589205057093048, 0.0000000000000000, -0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
	R1 << -0.0117905494796340, 0.8251582064941098, -0.5647786444248689, 0.0174926022293006, 0.5649016825777077, 0.8249727861500173, 0.9997774711455624, -0.0001525657200745, -0.0210946655220055;
	checkRotmatFromEuler(e, R0, R1);
	e.set(-0.9772714920274336, 0.3209780532006551, -0.2359388909208409);
	R0 << 0.5592864892040225, 0.8289744404949039, 0.0000000000000000, -0.8289744404949039, 0.5592864892040225, -0.0000000000000000, -0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
	R1 << 0.5307222188463538, 0.7647612463441738, 0.3653411590731451, -0.7866364786542851, 0.6049274902694710, -0.1235547731440017, -0.3154948127516355, -0.2218174195287314, 0.9226375537124127;
	checkRotmatFromEuler(e, R0, R1);
}

// Helper function: Euler angles --> Quaternion
void checkQuatFromEuler(const EulerAngles& e, const Quat& q0, const Quat& q1)
{
	// Test the various function overloads
	ExpectNear(q0, QuatFromEuler(e.yaw), TOL_HIGH);
	ExpectNear(q1, QuatFromEuler(e.yaw, e.pitch, e.roll), TOL_HIGH);
	ExpectNear(q1, QuatFromEuler(e), TOL_HIGH);
}

// Test conversion: Euler angles --> Quaternion
TEST(RotConvTest, testQuatFromEuler)
{
	// Declare variables
	EulerAngles e;
	Quat q0, q1;

	// Test the conversions
	e.setIdentity();
	q0.setIdentity();
	q1.setIdentity();
	checkQuatFromEuler(e, q0, q1);
	e.set(-1.7339076236950002, 0.8812784802083941, -0.7114375669352448);
	q0 = Quat(0.6471518416367722, 0.0000000000000000, 0.0000000000000000, -0.7623611308731145);
	q1 = Quat(0.6619330695307593, 0.1009529396573699, 0.4988838321381873, -0.5502436164711404);
	checkQuatFromEuler(e, q0, q1);
	e.set(1.0545241244364663, 0.5508219296894964, 2.6137500730419276);
	q0 = Quat(0.8641879536905701, 0.0000000000000000, 0.0000000000000000, 0.5031691372651000);
	q1 = Quat(0.3490381562790879, 0.7671291013802817, 0.5287469959175862, -0.1005580508209291);
	checkQuatFromEuler(e, q0, q1);
	e.set(2.1638797377996237, -1.5496995446654214, -3.1343603483484621);
	q0 = Quat(0.4696165959006853, 0.0000000000000000, 0.0000000000000000, 0.8828704621033895);
	q1 = Quat(0.6188732640000028, -0.3333175788436097, -0.6320179779047720, -0.3262645404345685);
	checkQuatFromEuler(e, q0, q1);
	e.set(-0.9772714920274336, 0.3209780532006551, -0.2359388909208409);
	q0 = Quat(0.8829740905609922, 0.0000000000000000, 0.0000000000000000, -0.4694217244631833);
	q1 = Quat(0.8743979732976623, -0.0280943716092304, 0.1946584943630155, -0.4435616768265119);
	checkQuatFromEuler(e, q0, q1);
}

// Helper function: Euler angles --> Fused angles
void checkFusedFromEuler(const EulerAngles& e, const FusedAngles& f0, const FusedAngles& f1)
{
	// Test the various function overloads
	ExpectNear(f0, FusedFromEuler(e.yaw), TOL_HIGH);
	ExpectNear(f1, FusedFromEuler(e.yaw, e.pitch, e.roll), TOL_HIGH);
	ExpectNear(f1, FusedFromEuler(e), TOL_HIGH);
}

// Test conversion: Euler angles --> Fused angles
TEST(RotConvTest, testFusedFromEuler)
{
	// Declare variables
	EulerAngles e;
	FusedAngles f0, f1;

	// Test the conversions
	e.setIdentity();
	f0.setIdentity();
	f1.setIdentity();
	checkFusedFromEuler(e, f0, f1);
	e.set(-1.7339076236950002, 0.8812784802083941, -0.7114375669352448);
	f0.set(-1.7339076236950000, 0.0000000000000000, 0.0000000000000000, true);
	f1.set(-1.3870360143374842, 0.8812784802083941, -0.4283463363844862, true);
	checkFusedFromEuler(e, f0, f1);
	e.set(1.0545241244364663, 0.5508219296894964, 2.6137500730419276);
	f0.set(1.0545241244364663, 0.0000000000000000, 0.0000000000000000, true);
	f1.set(-0.5610086893540762, 0.5508219296894964, 0.4435793137612023, false);
	checkFusedFromEuler(e, f0, f1);
	e.set(2.1638797377996237, -1.5496995446654214, -3.1343603483484621);
	f0.set(2.1638797377996237, 0.0000000000000000, 0.0000000000000000, true);
	f1.set(-0.9703264011734518, -1.5496995446654214, -0.0001525657206664, false);
	checkFusedFromEuler(e, f0, f1);
	e.set(-0.9772714920274336, 0.3209780532006551, -0.2359388909208409);
	f0.set(-0.9772714920274339, 0.0000000000000000, 0.0000000000000000, true);
	f1.set(-0.9389039421535887, 0.3209780532006551, -0.2236779279971249, true);
	checkFusedFromEuler(e, f0, f1);
}

// Helper function: Euler angles --> Tilt angles
void checkTiltFromEuler(const EulerAngles& e, const TiltAngles& t0, const TiltAngles& t1)
{
	// Test the various function overloads
	ExpectNear(t0, TiltFromEuler(e.yaw), TOL_HIGH);
	ExpectNear(t1, TiltFromEuler(e.yaw, e.pitch, e.roll), TOL_HIGH);
	ExpectNear(t1, TiltFromEuler(e), TOL_HIGH);
}

// Test conversion: Euler angles --> Tilt angles
TEST(RotConvTest, testTiltFromEuler)
{
	// Declare variables
	EulerAngles e;
	TiltAngles t0, t1;

	// Test the conversions
	e.setIdentity();
	t0.setIdentity();
	t1.setIdentity();
	checkTiltFromEuler(e, t0, t1);
	e.set(-1.7339076236950002, 0.8812784802083941, -0.7114375669352448);
	t0.set(-1.7339076236950000, 0.0000000000000000, 0.0000000000000000);
	t1.set(-1.3870360143374842, 2.0646528731729470, 1.0680351688609544);
	checkTiltFromEuler(e, t0, t1);
	e.set(1.0545241244364663, 0.5508219296894964, 2.6137500730419276);
	t0.set(1.0545241244364663, 0.0000000000000000, 0.0000000000000000);
	t1.set(-0.5610086893540762, 0.8839819325475061, 2.3981175553855856);
	checkTiltFromEuler(e, t0, t1);
	e.set(2.1638797377996237, -1.5496995446654214, -3.1343603483484621);
	t0.set(2.1638797377996237, 0.0000000000000000, 0.0000000000000000);
	t1.set(-0.9703264011734518, -1.5709489264716183, 1.5918925570982445);
	checkTiltFromEuler(e, t0, t1);
	e.set(-0.9772714920274336, 0.3209780532006551, -0.2359388909208409);
	t0.set(-0.9772714920274339, 0.0000000000000000, 0.0000000000000000);
	t1.set(-0.9389039421535887, 2.1835849871932962, 0.3959319185885756);
	checkTiltFromEuler(e, t0, t1);
}

// Helper function: Euler angles --> Z vector
void checkZVecFromEuler(const EulerAngles& e, const ZVec& ztrue)
{
	// Test the various function overloads
	ExpectNear(ztrue, ZVecFromEuler(e.pitch, e.roll), TOL_HIGH);
	ExpectNear(ztrue, ZVecFromEuler(e), TOL_HIGH);
}

// Test conversion: Euler angles --> Z vector
TEST(RotConvTest, testZVecFromEuler)
{
	// Declare variables
	EulerAngles e;
	ZVec ztrue;

	// Test the conversions
	e.setIdentity();
	ztrue << 0.0, 0.0, 1.0;
	checkZVecFromEuler(e, ztrue);
	e.set(-1.7339076236950002, 0.8812784802083941, -0.7114375669352448);
	ztrue << -0.7715528339139247, -0.4153671095382824, 0.4818468520113048;
	checkZVecFromEuler(e, ztrue);
	e.set(1.0545241244364663, 0.5508219296894964, 2.6137500730419276);
	ztrue << -0.5233877675119855, 0.4291751197738255, -0.7361208877527811;
	checkZVecFromEuler(e, ztrue);
	e.set(2.1638797377996237, -1.5496995446654214, -3.1343603483484621);
	ztrue << 0.9997774711455624, -0.0001525657200745, -0.0210946655220055;
	checkZVecFromEuler(e, ztrue);
	e.set(-0.9772714920274336, 0.3209780532006551, -0.2359388909208409);
	ztrue << -0.3154948127516355, -0.2218174195287314, 0.9226375537124127;
	checkZVecFromEuler(e, ztrue);
	
}

// #######################################
// #### Conversions from fused angles ####
// #######################################

// Helper function: Fused angles --> Rotation matrix
void checkRotmatFromFused(const FusedAngles& f, const Rotmat& R0, const Rotmat& R1, const Rotmat& R2, const Rotmat& R3)
{
	// Test the various function overloads
	ExpectNear(R0, RotmatFromFused(f.fusedYaw), TOL_HIGH);
	ExpectNear(R1, RotmatFromFused(f.fusedPitch, f.fusedRoll), TOL_HIGH);
	ExpectNear(R2, RotmatFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll), TOL_HIGH);
	ExpectNear(R3, RotmatFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi), TOL_HIGH);
	ExpectNear(R3, RotmatFromFused(f), TOL_HIGH);
}

// Test conversion: Fused angles --> Rotation matrix
TEST(RotConvTest, testRotmatFromFused)
{
	// Declare variables
	FusedAngles f;
	Rotmat R0, R1, R2, R3;

	// Test the conversions
	f.setIdentity();
	R0.setIdentity();
	checkRotmatFromFused(f, R0, R0, R0, R0);
	f.set(-0.4753290015060624, 0.7956041425923459, 0.0989307340163384, true);
	R0 << 0.8891422084585736, 0.4576310010668099, -0.0000000000000000, -0.4576310010668099, 0.8891422084585736, -0.0000000000000000, -0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
	R1 << 0.6986114181545878, 0.0416751236501361, 0.7142865464873194, 0.0416751236501361, 0.9942372868918273, -0.0987694356707789, -0.7142865464873194, 0.0987694356707789, 0.6928487050464152;
	R2 << 0.6402367277479410, 0.4920489163783222, 0.5899023616951603, -0.2826511311667216, 0.8649465084432648, -0.4146997414780698, -0.7142865464873194, 0.0987694356707789, 0.6928487050464152;
	checkRotmatFromFused(f, R0, R1, R2, R2);
	f.set(-0.2455697147179982, -0.0866401825990110, -0.0033439728816333, false);
	R0 << 0.9699989796308462, 0.2431089869073480, -0.0000000000000000, -0.2431089869073480, 0.9699989796308462, -0.0000000000000000, -0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
	R1 << 0.9962490760746641, 0.0001449520329514, -0.0865318288798178, 0.0001449520329514, 0.9999943984222887, 0.0033439666494998, 0.0865318288798178, -0.0033439666494998, 0.9962434744969529;
	R2 << 0.9663958263924786, 0.2432482284375239, -0.0831228373746024, -0.2420565002678348, 0.9699583069643011, 0.0242803094921469, 0.0865318288798178, -0.0033439666494998, 0.9962434744969529;
	R3 << -0.9447414191418304, 0.3171028931824429, -0.0831228373746024, 0.3161896480731055, 0.9483852450467433, 0.0242803094921469, 0.0865318288798178, -0.0033439666494998, -0.9962434744969529;
	checkRotmatFromFused(f, R0, R1, R2, R3);
	f.set(1.6974636373852383, -1.2747264560675389, -0.1837179306168055, true);
	R0 << -0.1263288610039000, -0.9919884167052846, 0.0000000000000000, 0.9919884167052846, -0.1263288610039000, 0.0000000000000000, -0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
	R1 << 0.2546788028982193, 0.1423536189777170, -0.9564905407364346, 0.1423536189777170, 0.9728109801319852, 0.1826861902240934, 0.9564905407364346, -0.1826861902240934, 0.2274897830302042;
	R2 << -0.1733864241939416, -0.9830005944798821, -0.0603902240220807, 0.2346550519101636, 0.0183190380098116, -0.9719060754308687, 0.9564905407364346, -0.1826861902240934, 0.2274897830302042;
	checkRotmatFromFused(f, R0, R1, R2, R2);
	f.set(-1.1154425326941773, -0.7586961609299141, -0.2595743935992043, false);
	R0 << 0.4397800857480921, 0.8981054927899065, -0.0000000000000000, -0.8981054927899065, 0.4397800857480921, -0.0000000000000000, -0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
	R1 << 0.7180711105851463, 0.1051817100215022, -0.6879757902862331, 0.1051817100215022, 0.9607589270258570, 0.2566692268603243, 0.6879757902862332, -0.2566692268603243, 0.6788300376110034;
	R2 << 0.4102576460977096, 0.9091196910612462, -0.0720420095712951, -0.5986467871778811, 0.3280583717993301, 0.7307528507600727, 0.6879757902862332, -0.2566692268603243, 0.6788300376110034;
	R3 << 0.2854594322848958, 0.9556792670014949, -0.0720420095712951, 0.6672347596608093, -0.1442152786857513, 0.7307528507600727, 0.6879757902862332, -0.2566692268603243, -0.6788300376110034;
	checkRotmatFromFused(f, R0, R1, R2, R3);
}

// Helper function: Fused angles --> Quaternion
void checkQuatFromFused(const FusedAngles& f, const Quat& q0, const Quat& q1, const Quat& q2, const Quat& q3)
{
	// Declare variables
	Quat qout;

	// Test the various function overloads
	qout.setIdentity();
	qout = QuatFromFused(f.fusedYaw);
	ExpectNear(q0, qout, TOL_HIGH);
	qout.setIdentity();
	qout = QuatFromFused(f.fusedPitch, f.fusedRoll);
	ExpectNear(q1, qout, TOL_HIGH);
	qout.setIdentity();
	qout = QuatFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll);
	ExpectNear(q2, qout, TOL_HIGH);
	qout.setIdentity();
	qout = QuatFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi);
	ExpectNear(q3, qout, TOL_HIGH);
	qout.setIdentity();
	qout = QuatFromFused(f);
	ExpectNear(q3, qout, TOL_HIGH);
}

// Test conversion: Fused angles --> Quaternion
TEST(RotConvTest, testQuatFromFused)
{
	// Declare variables
	FusedAngles f;
	Quat q0, q1, q2, q3;

	// Test the conversions
	f.setIdentity();
	q0.setIdentity();
	checkQuatFromFused(f, q0, q0, q0, q0);
	f.set(2.4534701287862291, -0.2650491423322411, 0.2746203053006582, true);
	q0 = Quat(0.3373131054531923, 0.0000000000000000, 0.0000000000000000, 0.9413925158452895);
	q1 = Quat(0.9813756008800009, 0.1381639656628278, -0.1334640348173924, 0.0000000000000000);
	q2 = Quat(0.3310308515488257, 0.1722465598310650,  0.0850473551839241, 0.9238596459016067);
	checkQuatFromFused(f, q0, q1, q2, q2);
	f.set(2.3862836331701032, -0.3599342692652525, -0.7344627162396046, false);
	q0 = Quat(0.3687412777128309, 0.0000000000000000, 0.0000000000000000, 0.9295320705122061);
	q1 = Quat(0.9092014129497605, -0.3685588669926899, -0.1936934491696190, 0.0000000000000000);
	q2 = Quat(0.3352600907094059,  0.0441414053240089, -0.4140100566727584, 0.8451318718918143);
	q3 = Quat(0.1535278524138601,  0.0963919661500578, -0.9040773186910327, 0.3870167815784569);
	checkQuatFromFused(f, q0, q1, q2, q3);
	f.set(0.4, 1.2, -1.4, true);
	q0 = Quat(0.9800665778412416, 0.0000000000000000, 0.0000000000000000, 0.1986693307950612);
	q1 = Quat(0.7071067811865476, -0.5137279825527999, 0.4858843071578151, 0.0000000000000000);
	q2 = Quat(0.6930117232058354, -0.6000179359486728, 0.3741369756384608, 0.1404804310189812);
	checkQuatFromFused(f, q0, q1, q2, q2);
}

// Helper function: Fused angles --> Euler angles
void checkEulerFromFused(const FusedAngles& f, const EulerAngles& e0, const EulerAngles& e1, const EulerAngles& e2, const EulerAngles& e3)
{
	// Test the various function overloads
	ExpectNear(e0, EulerFromFused(f.fusedYaw), TOL_HIGH);
	ExpectNear(e1, EulerFromFused(f.fusedPitch, f.fusedRoll), TOL_HIGH);
	ExpectNear(e2, EulerFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll), TOL_HIGH);
	ExpectNear(e3, EulerFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi), TOL_HIGH);
	ExpectNear(e3, EulerFromFused(f), TOL_HIGH);
}

// Test conversion: Fused angles --> Euler angles
TEST(RotConvTest, testEulerFromFused)
{
	// Declare variables
	FusedAngles f;
	EulerAngles e0, e1, e2, e3;

	// Test the conversions
	f.setIdentity();
	e0.setIdentity();
	checkEulerFromFused(f, e0, e0, e0, e0);
	f.set(-0.4753290015060624, 0.7956041425923459, 0.0989307340163384, true);
	e0.set(-0.4753290015060624, 0.0000000000000000, 0.0000000000000000);
	e1.set(0.0595836146551716, 0.7956041425923459, 0.1416014916799435);
	e2.set(-0.4157453868508909, 0.7956041425923459, 0.1416014916799435);
	checkEulerFromFused(f, e0, e1, e2, e2);
	f.set(-0.2455697147179982, -0.0866401825990110, -0.0033439728816333, false);
	e0.set(-0.2455697147179982, 0.0000000000000000, 0.0000000000000000);
	e1.set(0.0001454977830441, -0.0866401825990110, -0.0033565631061166);
	e2.set(-0.2454242169349542, -0.0866401825990110, -0.0033565631061166);
	e3.set(2.8186271721044909, -0.0866401825990110, -3.1382360904836766);
	checkEulerFromFused(f, e0, e1, e2, e3);
	f.set(1.6974636373852383, -1.2747264560675389, -0.1837179306168055, true);
	e0.set(1.6974636373852385, 0.0000000000000000, 0.0000000000000000);
	e1.set(0.5096913329344481, -1.2747264560675389, -0.6765993228868794);
	e2.set(2.2071549703196864, -1.2747264560675389, -0.6765993228868794);
	checkEulerFromFused(f, e0, e1, e2, e2);
	f.set(-1.1154425326941773, -0.7586961609299141, -0.2595743935992043, false);
	e0.set(-1.1154425326941773, 0.0000000000000000, 0.0000000000000000);
	e1.set(0.1454438044163448, -0.7586961609299141, -0.3614903312676470);
	e2.set(-0.9699987282778324, -0.7586961609299141, -0.3614903312676470);
	e3.set(1.1665356675015857, -0.7586961609299141, -2.7801023223221462);
	checkEulerFromFused(f, e0, e1, e2, e3);
}

// Helper function: Fused angles --> Tilt angles
void checkTiltFromFused(const FusedAngles& f, const TiltAngles& t0, const TiltAngles& t1, const TiltAngles& t2, const TiltAngles& t3)
{
	// Test the various function overloads
	ExpectNear(t0, TiltFromFused(f.fusedYaw), TOL_HIGH);
	ExpectNear(t1, TiltFromFused(f.fusedPitch, f.fusedRoll), TOL_HIGH);
	ExpectNear(t2, TiltFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll), TOL_HIGH);
	ExpectNear(t3, TiltFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi), TOL_HIGH);
	ExpectNear(t3, TiltFromFused(f), TOL_HIGH);
}

// Test conversion: Fused angles --> Tilt angles
TEST(RotConvTest, testTiltFromFused)
{
	// Declare variables
	FusedAngles f;
	TiltAngles t0, t1, t2, t3;

	// Test the conversions
	f.setIdentity();
	t0.setIdentity();
	checkTiltFromFused(f, t0, t0, t0, t0);
	f.set(-0.4753290015060624, 0.7956041425923459, 0.0989307340163384, true);
	t0.set(-0.4753290015060624, 0.0000000000000000, 0.0000000000000000);
	t1.set(0.0000000000000000, 1.4333906140591133, 0.8053641410516513);
	t2.set(-0.4753290015060624, 1.4333906140591133, 0.8053641410516513);
	checkTiltFromFused(f, t0, t1, t2, t2);
	f.set(-0.2455697147179982, -0.0866401825990110, -0.0033439728816333, false);
	t0.set(-0.2455697147179982, 0.0000000000000000, 0.0000000000000000);
	t1.set(0.0000000000000000, -1.6094214612870268, 0.0867050145709935);
	t2.set(-0.2455697147179982, -1.6094214612870268, 0.0867050145709935);
	t3.set(-0.2455697147179982, -1.6094214612870268, 3.0548876390187996);
	checkTiltFromFused(f, t0, t1, t2, t3);
	f.set(1.6974636373852383, -1.2747264560675389, -0.1837179306168055, true);
	t0.set(1.6974636373852383, 0.0000000000000000, 0.0000000000000000);
	t1.set(0.0000000000000000, -1.7595197203514674, 1.3412972292885268);
	t2.set(1.6974636373852383, -1.7595197203514674, 1.3412972292885268);
	checkTiltFromFused(f, t0, t1, t2, t2);
	f.set(-1.1154425326941773, -0.7586961609299141, -0.2595743935992043, false);
	t0.set(-1.1154425326941773, 0.0000000000000000, 0.0000000000000000);
	t1.set(0.0000000000000000, -1.9278816512837393, 0.8246281808956211);
	t2.set(-1.1154425326941773, -1.9278816512837393, 0.8246281808956211);
	t3.set(-1.1154425326941773, -1.9278816512837393, 2.3169644726941723);
	checkTiltFromFused(f, t0, t1, t2, t3);
}

// Helper function: Fused angles --> Z vector
void checkZVecFromFused(const FusedAngles& f, const ZVec& ztrue)
{
	// Test the various function overloads
	ZVec ztruh(ztrue.x(), ztrue.y(), fabs(ztrue.z()));
	ExpectNear(ztruh, ZVecFromFused(f.fusedPitch, f.fusedRoll), TOL_HIGH);
	ExpectNear(ztrue, ZVecFromFused(f.fusedPitch, f.fusedRoll, f.hemi), TOL_HIGH);
	ExpectNear(ztrue, ZVecFromFused(f), TOL_HIGH);
}

// Test conversion: Fused angles --> Z vector
TEST(RotConvTest, testZVecFromFused)
{
	// Declare variables
	FusedAngles f;
	ZVec ztrue;

	// Test the conversions
	f.setIdentity();
	ztrue << 0.0, 0.0, 1.0;
	checkZVecFromFused(f, ztrue);
	f.set(2.8152398353463099, 0.1132399708595934, 0.2041261900052033, false);
	ztrue << -0.1129981078025050, 0.2027115690236323, -0.9726969967143060;
	checkZVecFromFused(f, ztrue);
	f.set(-0.4961590942268063, -1.0250480269221482, 0.4195227513843564, true);
	ztrue << 0.8547391192101734, 0.4073246362063569, 0.3217260928665500;
	checkZVecFromFused(f, ztrue);
	f.set(1.0237175660269251, 0.8936173187001297, 0.6543857119757933, true);
	ztrue << -0.7793434424710965, 0.6086719685821497, 0.1488026657678926;
	checkZVecFromFused(f, ztrue);
}

// ######################################
// #### Conversions from tilt angles ####
// ######################################

// Helper function: Tilt angles --> Rotation matrix
void checkRotmatFromTilt(const TiltAngles& t, const Rotmat& R0, const Rotmat& R1, const Rotmat& R2)
{
	// Test the various function overloads
	ExpectNear(R0, RotmatFromTilt(t.fusedYaw), TOL_HIGH);
	ExpectNear(R1, RotmatFromTilt(t.tiltAxisAngle, t.tiltAngle), TOL_HIGH);
	ExpectNear(R2, RotmatFromTilt(t.fusedYaw, t.tiltAxisAngle, t.tiltAngle), TOL_HIGH);
	ExpectNear(R2, RotmatFromTilt(t), TOL_HIGH);
}

// Test conversion: Tilt angles --> Rotation matrix
TEST(RotConvTest, testRotmatFromTilt)
{
	// Declare variables
	TiltAngles t;
	Rotmat R0, R1, R2;

	// Test the conversions
	t.setIdentity();
	R0.setIdentity();
	checkRotmatFromTilt(t, R0, R0, R0);
	t.set(-1.1402457949183806, -2.2852413126226203, 1.5551066632734325);
	R0 << 0.4173711538500757, 0.9087361112742557, -0.0000000000000000, -0.9087361112742557, 0.4173711538500757, -0.0000000000000000, -0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
	R1 << 0.4382386817197934, 0.4872084308666204, -0.7553640200174901, 0.4872084308666204, 0.5774503381005078, 0.6551175863309540, 0.7553640200174901, -0.6551175863309540, 0.0156890198203011;
	R2 << 0.6256520790968908, 0.7280967196557457, 0.2800618552182361, -0.1948965704797151, -0.2017327809416394, 0.9598537450616098, 0.7553640200174901, -0.6551175863309540, 0.0156890198203011;
	checkRotmatFromTilt(t, R0, R1, R2);
	t.set(0.2140312245976421, 1.1225055366143117, 0.4637249057714166);
	R0 << 0.9771826214564960, -0.2124008576334152, 0.0000000000000000, 0.2124008576334152, 0.9771826214564960, -0.0000000000000000, -0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
	R1 << 0.9142318075628348, 0.0412501059738903, 0.4030863813099307, 0.0412501059738903, 0.9801608125984028, -0.1938638960806825, -0.4030863813099307, 0.1938638960806825, 0.8943926201612374;
	R2 << 0.8846098764468381, -0.1678781105236416, 0.4350658645535431, 0.2344925066930179, 0.9665576701901601, -0.1038245370873119, -0.4030863813099307, 0.1938638960806825, 0.8943926201612374;
	checkRotmatFromTilt(t, R0, R1, R2);
	t.set(-2.5764158703676476, -0.0303036827926521, 0.1727063760578469);
	R0 << -0.8444939346930328, 0.5355651167380860, -0.0000000000000000, -0.5355651167380860, -0.8444939346930328, 0.0000000000000000, -0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
	R1 << 0.9999863426995148, -0.0004505432479280, -0.0052068632813270, -0.0004505432479280, 0.9851369442685655, -0.1717701896371292, 0.0052068632813270, 0.1717701896371292, 0.9851232869680800;
	R2 << -0.8447236964327808, 0.5279854636003876, -0.0875969572052758, -0.5351773213241651, -0.8317008790296596, 0.1478474976507308, 0.0052068632813270, 0.1717701896371292, 0.9851232869680800;
	checkRotmatFromTilt(t, R0, R1, R2);
	t.set(-2.4397247629474959, -1.9496070178810596, 2.6725926878620720);
	R0 << -0.7636375255219162, 0.6456452041289897, -0.0000000000000000, -0.6456452041289897, -0.7636375255219162, 0.0000000000000000, -0.0000000000000000, 0.0000000000000000, 1.0000000000000000;
	R1 << -0.6332610240672344, 0.6500939665522715, -0.4199503661739702, 0.6500939665522715, 0.7412402799552336, 0.1671546649789376, 0.4199503661739702, -0.1671546649789376, -0.8920207441120007;
	R2 << 0.9033119331658435, -0.0178579161143781, 0.4286124662585524, -0.0875742048238818, -0.9857689449398530, 0.1434933651484759, 0.4199503661739702, -0.1671546649789376, -0.8920207441120007;
	checkRotmatFromTilt(t, R0, R1, R2);
}

// Helper function: Tilt angles --> Quaternion
void checkQuatFromTilt(const TiltAngles& t, const Quat& q0, const Quat& q1, const Quat& q2)
{
	// Declare variables
	Quat qout;

	// Test the various function overloads
	qout.setIdentity();
	qout = QuatFromTilt(t.fusedYaw);
	ExpectNear(q0, qout, TOL_HIGH);
	qout.setIdentity();
	qout = QuatFromTilt(t.tiltAxisAngle, t.tiltAngle);
	ExpectNear(q1, qout, TOL_HIGH);
	qout.setIdentity();
	qout = QuatFromTilt(t.fusedYaw, t.tiltAxisAngle, t.tiltAngle);
	ExpectNear(q2, qout, TOL_HIGH);
	qout.setIdentity();
	qout = QuatFromTilt(t);
	ExpectNear(q2, qout, TOL_HIGH);
}

// Test conversion: Tilt angles --> Quaternion
TEST(RotConvTest, testQuatFromTilt)
{
	// Declare variables
	TiltAngles t;
	Quat q0, q1, q2;

	// Test the conversions
	t.setIdentity();
	q0.setIdentity();
	checkQuatFromTilt(t, q0, q0, q0);
	t.set(-2.1372119601868680, 2.0216320967452766, 1.0261162316549439);
	q0 = Quat(0.4813465970868976, 0.0000000000000000, 0.0000000000000000, -0.8765303494305624);
	q1 = Quat(0.8712475264256645, -0.2138695032645000, 0.4418003884907267, 0.0000000000000000);
	q2 = Quat(0.4193720320653705, 0.2843060912853027, 0.4001222240206537, -0.7636748987784009);
	checkQuatFromTilt(t, q0, q1, q2);
	t.set(-1.8050256853639468, 1.3723370232880385, 2.6992429997769527);
	q0 = Quat(0.6196396305104982, 0.0000000000000000, 0.0000000000000000, -0.7848864429335070);
	q1 = Quat(0.2193759827962727, 0.1923563921842024, 0.9564900399680317, 0.0000000000000000);
	q2 = Quat(0.1359340529227598, 0.8699277089511879, 0.4417012105157816, -0.1721852348020087);
	checkQuatFromTilt(t, q0, q1, q2);
	t.set(10.4, -6.7, 3.7);
	q0 = Quat(0.4685166713003771, 0.0000000000000000, 0.0000000000000000, -0.8834546557201531);
	q1 = Quat(-0.2755902468245129, 0.8789738464171004, -0.3891721896152546, -0.0000000000000000);
	q2 = Quat(-0.1291186250850701, 0.0679979178910258, -0.9588671957146565, 0.2434714866281821);
	checkQuatFromTilt(t, q0, q1, q2);
}

// Helper function: Tilt angles --> Euler angles
void checkEulerFromTilt(const TiltAngles& t, const EulerAngles& e0, const EulerAngles& e1, const EulerAngles& e2)
{
	// Test the various function overloads
	ExpectNear(e0, EulerFromTilt(t.fusedYaw), TOL_HIGH);
	ExpectNear(e1, EulerFromTilt(t.tiltAxisAngle, t.tiltAngle), TOL_HIGH);
	ExpectNear(e2, EulerFromTilt(t.fusedYaw, t.tiltAxisAngle, t.tiltAngle), TOL_HIGH);
	ExpectNear(e2, EulerFromTilt(t), TOL_HIGH);
}

// Test conversion: Tilt angles --> Euler angles
TEST(RotConvTest, testEulerFromTilt)
{
	// Declare variables
	TiltAngles t;
	EulerAngles e0, e1, e2;

	// Test the conversions
	t.setIdentity();
	e0.setIdentity();
	checkEulerFromTilt(t, e0, e0, e0);
	t.set(-1.1402457949183806, -2.2852413126226203, 1.5551066632734325);
	e0.set(-1.1402457949183806, 0.0000000000000000, 0.0000000000000000);
	e1.set(0.8382635520172667, -0.8562094385264346, -1.5468525007921716);
	e2.set(-0.3019822429011140, -0.8562094385264346, -1.5468525007921716);
	checkEulerFromTilt(t, e0, e1, e2);
	t.set(0.2140312245976421, 1.1225055366143117, 0.4637249057714166);
	e0.set(0.2140312245976421, 0.0000000000000000, 0.0000000000000000);
	e1.set(0.0450893824747747, 0.4148868488163316, 0.2134528248478124);
	e2.set(0.2591206070724169, 0.4148868488163316, 0.2134528248478124);
	checkEulerFromTilt(t, e0, e1, e2);
	t.set(-2.5764158703676476, -0.0303036827926521, 0.1727063760578469);
	e0.set(-2.5764158703676476, 0.0000000000000000, 0.0000000000000000);
	e1.set(-0.0004505493707302, -0.0052068868091948, 0.1726286491663114);
	e2.set(-2.5768664197383782, -0.0052068868091948, 0.1726286491663114);
	checkEulerFromTilt(t, e0, e1, e2);
	t.set(-2.4397247629474959, -1.9496070178810596, 2.6725926878620720);
	e0.set(-2.4397247629474959, 0.0000000000000000, 0.0000000000000000);
	e1.set(2.3430788850782953, -0.4333906292934219, -2.9563521636319625);
	e2.set(-0.0966458778692007, -0.4333906292934219, -2.9563521636319625);
	checkEulerFromTilt(t, e0, e1, e2);
}

// Helper function: Tilt angles --> Fused angles
void checkFusedFromTilt(const TiltAngles& t, const FusedAngles& f0, const FusedAngles& f1, const FusedAngles& f2)
{
	// Test the various function overloads
	ExpectNear(f0, FusedFromTilt(t.fusedYaw), TOL_HIGH);
	ExpectNear(f1, FusedFromTilt(t.tiltAxisAngle, t.tiltAngle), TOL_HIGH);
	ExpectNear(f2, FusedFromTilt(t.fusedYaw, t.tiltAxisAngle, t.tiltAngle), TOL_HIGH);
	ExpectNear(f2, FusedFromTilt(t), TOL_HIGH);
}

// Test conversion: Tilt angles --> Fused angles
TEST(RotConvTest, testFusedFromTilt)
{
	// Declare variables
	TiltAngles t;
	FusedAngles f0, f1, f2;

	// Test the conversions
	t.setIdentity();
	f0.setIdentity();
	checkFusedFromTilt(t, f0, f0, f0);
	t.set(-1.1402457949183806, -2.2852413126226203, 1.5551066632734325);
	f0.set(-1.1402457949183806, 0.0000000000000000, 0.0000000000000000, true);
	f1.set(0.0000000000000000, -0.8562094385264346, -0.7143382448399271, true);
	f2.set(-1.1402457949183806, -0.8562094385264346, -0.7143382448399271, true);
	checkFusedFromTilt(t, f0, f1, f2);
	t.set(0.2140312245976421, 1.1225055366143117, 0.4637249057714166);
	f0.set(0.2140312245976421, 0.0000000000000000, 0.0000000000000000, true);
	f1.set(0.0000000000000000, 0.4148868488163316, 0.1950992429579740, true);
	f2.set(0.2140312245976421, 0.4148868488163316, 0.1950992429579740, true);
	checkFusedFromTilt(t, f0, f1, f2);
	t.set(-2.5764158703676476, -0.0303036827926521, 0.1727063760578469);
	f0.set(-2.5764158703676476, 0.0000000000000000, 0.0000000000000000, true);
	f1.set(0.0000000000000000, -0.0052068868091948, 0.1726262855204005, true);
	f2.set(-2.5764158703676476, -0.0052068868091948, 0.1726262855204005, true);
	checkFusedFromTilt(t, f0, f1, f2);
	t.set(-2.4397247629474959, -1.9496070178810596, 2.6725926878620720);
	f0.set(-2.4397247629474959, 0.0000000000000000, 0.0000000000000000, true);
	f1.set(0.0000000000000000, -0.4333906292934219, -0.1679430205632803, false);
	f2.set(-2.4397247629474959, -0.4333906292934219, -0.1679430205632803, false);
	checkFusedFromTilt(t, f0, f1, f2);
}

// Helper function: Tilt angles --> Z vector
void checkZVecFromTilt(const TiltAngles& t, const ZVec& ztrue)
{
	// Test the various function overloads
	ExpectNear(ztrue, ZVecFromTilt(t.tiltAxisAngle, t.tiltAngle), TOL_HIGH);
	ExpectNear(ztrue, ZVecFromTilt(t), TOL_HIGH);
}

// Test conversion: Tilt angles --> Z vector
TEST(RotConvTest, testZVecFromTilt)
{
	// Declare variables
	TiltAngles t;
	ZVec ztrue;

	// Test the conversions
	t.setIdentity();
	ztrue << 0.0, 0.0, 1.0;
	checkZVecFromTilt(t, ztrue);
	t.set(2.2051471220187433, 2.4049534628388769, 2.2760585195566234);
	ztrue << -0.5115381955296429, -0.5640230569716014, -0.6482334962948818;
	checkZVecFromTilt(t, ztrue);
	t.set(2.3363825576018220, -1.0172392397171559, 1.3860021820538988);
	ztrue << 0.8361766350987291, 0.5167656225462592, 0.1837441870355194;
	checkZVecFromTilt(t, ztrue);
	t.set(-0.2409452572424218, -1.8014313205971391, 2.4449493985778110);
	ztrue << 0.6246567986735801, -0.1466777043891246, -0.7670003487000583;
	checkZVecFromTilt(t, ztrue);
}

// ####################################
// #### Conversions from Z vectors ####
// ####################################

// Helper function: Z vector --> Fused angles
void checkFusedFromZVec(const ZVec& z, const FusedAngles& ftrue)
{
	// Declare variables
	double fusedPitch, fusedRoll;
	bool hemi;

	// Test the various function overloads
	fusedPitch = fusedRoll = -100.0; hemi = false;
	FusedFromZVec(z, fusedPitch, fusedRoll);
	EXPECT_NEAR(ftrue.fusedPitch, fusedPitch, TOL_HIGH);
	EXPECT_NEAR(ftrue.fusedRoll, fusedRoll, TOL_HIGH);
	fusedPitch = fusedRoll = -100.0; hemi = false;
	FusedFromZVec(z, fusedPitch, fusedRoll, hemi);
	EXPECT_NEAR(ftrue.fusedPitch, fusedPitch, TOL_HIGH);
	EXPECT_NEAR(ftrue.fusedRoll, fusedRoll, TOL_HIGH);
	EXPECT_TRUE(ftrue.hemi == hemi);
	FusedAngles f = FusedFromZVec(z);
	ExpectNear(ftrue, f, TOL_HIGH);
	f.fusedPitch = f.fusedRoll = -100.0; f.hemi = false;
	FusedFromZVec(z, f);
	ExpectNear(ftrue, f, TOL_HIGH);
}

// Test conversion: Z vector --> Fused angles
TEST(RotConvTest, testFusedFromZVec)
{
	// Declare variables
	ZVec z;
	FusedAngles ftrue;

	// Test the conversions
	z << 0.0, 0.0, 1.0;
	ftrue.setIdentity();
	checkFusedFromZVec(z, ftrue);
	z << -0.9722382711129494, 0.2291745408581753, -0.0472416554086341;
	ftrue.set(0.0, 1.3346135980822995, 0.2312295689401531, false);
	checkFusedFromZVec(z, ftrue);
	z << 0.4550274306083184, 0.5653363895316842, -0.6880005843495141;
	ftrue.set(0.0, -0.4724030182703050, 0.6008410104827017, false);
	checkFusedFromZVec(z, ftrue);
	z << -0.5453880079766177, -0.8309619601692930, 0.1097913544269455;
	ftrue.set(0.0, 0.5768519602564895, -0.9808345783548854, true);
	checkFusedFromZVec(z, ftrue);
	z << 0.6059863877958331, -0.7627764887891112, -0.2257266620421992;
	ftrue.set(0.0, -0.6510052810944137, -0.8675958792259617, false);
	checkFusedFromZVec(z, ftrue);
}

// Helper function: Z vector --> Tilt angles
void checkTiltFromZVec(const ZVec& z, const TiltAngles& ttrue)
{
	// Declare variables
	double tiltAxisAngle, tiltAngle;

	// Test the various function overloads
	tiltAxisAngle = tiltAngle = -100.0;
	TiltFromZVec(z, tiltAxisAngle, tiltAngle);
	EXPECT_NEAR(ttrue.tiltAxisAngle, tiltAxisAngle, TOL_HIGH);
	EXPECT_NEAR(ttrue.tiltAngle, tiltAngle, TOL_HIGH);
	TiltAngles t = TiltFromZVec(z);
	ExpectNear(ttrue, t, TOL_HIGH);
	t.tiltAxisAngle = t.tiltAngle = -100.0;
	TiltFromZVec(z, t);
	ExpectNear(ttrue, t, TOL_HIGH);
}

// Test conversion: Z vector --> Tilt angles
TEST(RotConvTest, testTiltFromZVec)
{
	// Declare variables
	ZVec z;
	TiltAngles ttrue;

	// Test the conversions
	z << 0.0, 0.0, 1.0;
	ttrue.setIdentity();
	checkTiltFromZVec(z, ttrue);
	z << 0.2875570084271079, -0.9237244622681462, 0.2530693278765259;
	ttrue.set(0.0, -2.8397998341990722, 1.3149447797732721);
	checkTiltFromZVec(z, ttrue);
	z << 0.6558561117579026, -0.4593146016856069, -0.5990683244407078;
	ttrue.set(0.0, -2.1817425988018928, 2.2131333490376615);
	checkTiltFromZVec(z, ttrue);
	z << -0.7462458925078992, -0.6582334394807199, 0.0992260402539099;
	ttrue.set(0.0, 2.2936107879584866, 1.4714067341208246);
	checkTiltFromZVec(z, ttrue);
	z << -0.9502673620764408, 0.2537123015512597, 0.1806156377887733;
	ttrue.set(0.0, 1.3098916809871868, 1.3891839794574117);
	checkTiltFromZVec(z, ttrue);
}

// #######################
// #### Main function ####
// #######################

// Main function
int main(int argc, char **argv)
{
	// Run all unit tests
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// EOF