// Rotations conversion library
// File: rot_conv.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ROT_CONV_H
#define ROT_CONV_H

// Includes
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>

// Defines
#define ROT_CONV_DEFAULT_TOL 1e-10

// Note: The inline keywords below for the functions that are defined (as well as declared) in this header are
//       required to avoid multiple definition linker errors.
// Note: Where quaternions and rotation matrices are used as inputs, it is implicitly assumed that they are valid
//       (i.e. unit norm q, orthogonal R, det(R) = 1, etc...).
// Note: Where unit vectors (i.e. ZVec, BzG, etc...) are used as inputs, it is implicitly assumed they have unit norm.

// Rotations conversion namespace
namespace rot_conv
{
	// ########################
	// #### Rotation types ####
	// ########################

	// Typedefs
	typedef Eigen::Matrix3d Rotmat;
	typedef Eigen::Quaterniond Quat; // Format: (w,x,y,z)
	typedef Eigen::Vector3d ZVec; // Equivalent to BzG, i.e. the bottom row in RGB
	typedef Eigen::Vector2d Vec2;
	typedef Eigen::Vector3d Vec3;

	// Unit axis enumeration
	enum UnitAxis
	{
		X_AXIS = 0,
		Y_AXIS,
		Z_AXIS,
		AXIS_COUNT
	};

	// Unit axis functions
	inline Vec3 VecUnitX() { return Vec3::UnitX(); }
	inline Vec3 VecUnitY() { return Vec3::UnitY(); }
	inline Vec3 VecUnitZ() { return Vec3::UnitZ(); }
	inline Vec3 VecUnit(UnitAxis axis) { return (axis == X_AXIS ? Vec3::UnitX() : (axis == Y_AXIS ? Vec3::UnitY() : Vec3::UnitZ())); }

	// Helper functions
	namespace internal
	{
		// Picut functions
		inline double picut(double var) { return var + (2.0*M_PI)*std::floor((M_PI - var) / (2.0*M_PI)); }
		inline void picutVar(double& var) { var += (2.0*M_PI)*std::floor((M_PI - var) / (2.0*M_PI)); }
	}

	// Stream insertion operator for the vector types
	inline std::ostream& operator<<(std::ostream& os, const Vec2& v) { return os << "(" << v.x() << ", " << v.y() << ")"; }
	inline std::ostream& operator<<(std::ostream& os, const Vec3& v) { return os << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")"; }

	// Additional quaternion operators
	inline Quat operator+(const Quat& q) { return q; }
	inline Quat operator-(const Quat& q) { return Quat(-q.w(), -q.x(), -q.y(), -q.z()); }
	inline Quat operator+(const Quat& qa, const Quat& qb) { return Quat(qa.w() + qb.w(), qa.x() + qb.x(), qa.y() + qb.y(), qa.z() + qb.z()); }
	inline Quat operator-(const Quat& qa, const Quat& qb) { return Quat(qa.w() - qb.w(), qa.x() - qb.x(), qa.y() - qb.y(), qa.z() - qb.z()); }
	inline Quat& operator+=(Quat& qa, const Quat& qb) { qa.w() += qb.w(); qa.x() += qb.x(); qa.y() += qb.y(); qa.z() += qb.z(); return qa; }
	inline Quat& operator-=(Quat& qa, const Quat& qb) { qa.w() -= qb.w(); qa.x() -= qb.x(); qa.y() -= qb.y(); qa.z() -= qb.z(); return qa; }
	inline Quat operator*(const Quat& q, double s) { return Quat(q.w()*s, q.x()*s, q.y()*s, q.z()*s); }
	inline Quat operator*(double s, const Quat& q) { return Quat(s*q.w(), s*q.x(), s*q.y(), s*q.z()); }
	inline Quat operator/(const Quat& q, double s) { return Quat(q.w()/s, q.x()/s, q.y()/s, q.z()/s); }
	inline Quat& operator*=(Quat& q, double s) { q.w() *= s; q.x() *= s; q.y() *= s; q.z() *= s; return q; }
	inline Quat& operator/=(Quat& q, double s) { q.w() /= s; q.x() /= s; q.y() /= s; q.z() /= s; return q; }

	// Stream insertion operator for the Quat type
	inline std::ostream& operator<<(std::ostream& os, const Quat& q) { return os << "(" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << ")"; }

	// Euler angles struct with format: ZYX Euler (yaw, pitch, roll)
	struct EulerAngles // Note: POD type
	{
		// Constants
		static inline EulerAngles Identity() { return EulerAngles(0.0, 0.0, 0.0); }

		// Constructors
		EulerAngles() = default;
		explicit EulerAngles(double yaw) : yaw(yaw), pitch(0.0), roll(0.0) {}
		EulerAngles(double yaw, double pitch, double roll) : yaw(yaw), pitch(pitch), roll(roll) {}

		// Set functions
		void set(double yaw, double pitch, double roll) { this->yaw = yaw; this->pitch = pitch; this->roll = roll; }
		void setIdentity() { yaw = pitch = roll = 0.0; }

		// Data members
		double yaw;   // Yaw:    psi  is in (-pi,pi]
		double pitch; // Pitch: theta is in [-pi/2,pi/2]
		double roll;  // Roll:   phi  is in (-pi,pi]
	};

	// Stream insertion operator for the EulerAngles struct
	inline std::ostream& operator<<(std::ostream& os, const EulerAngles& e) { return os << "E(" << e.yaw << ", " << e.pitch << ", " << e.roll << ")"; }

	// Fused angles struct with format: (fusedYaw, fusedPitch, fusedRoll, hemi)
	struct FusedAngles // Note: POD type
	{
		// Constants
		static inline FusedAngles Identity() { return FusedAngles(0.0, 0.0, 0.0, true); }

		// Constructors
		FusedAngles() = default;
		explicit FusedAngles(double fusedYaw) : fusedYaw(fusedYaw), fusedPitch(0.0), fusedRoll(0.0), hemi(true) {}
		explicit FusedAngles(double fusedPitch, double fusedRoll, bool hemi = true) : fusedYaw(0.0), fusedPitch(fusedPitch), fusedRoll(fusedRoll), hemi(hemi) {}
		FusedAngles(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi = true) : fusedYaw(fusedYaw), fusedPitch(fusedPitch), fusedRoll(fusedRoll), hemi(hemi) {}

		// Set functions
		void set(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi = true) { this->fusedYaw = fusedYaw; this->fusedPitch = fusedPitch; this->fusedRoll = fusedRoll; this->hemi = hemi; }
		void setIdentity() { fusedYaw = fusedPitch = fusedRoll = 0.0; hemi = true; }

		// Get functions
		int hemiSign() const { return (hemi ? 1 : -1); }

		// Data members
		double fusedYaw;   // Fused yaw:    psi  = yaw   is in (-pi,pi]
		double fusedPitch; // Fused pitch: theta = pitch is in [-pi/2,pi/2]
		double fusedRoll;  // Fused roll:   phi  = roll  is in [-pi/2,pi/2]
		bool hemi;         // Hemisphere:    h   = hemi  is in {-1,1} (stored as {false,true} respectively)
	};

	// Stream insertion operator for the FusedAngles struct
	inline std::ostream& operator<<(std::ostream& os, const FusedAngles& f) { return os << "F(" << f.fusedYaw << ", " << f.fusedPitch << ", " << f.fusedRoll << ", " << f.hemiSign() << ")"; }

	// Tilt angles struct with format: (fusedYaw, tiltAxisAngle, tiltAngle)
	struct TiltAngles // Note: POD type
	{
		// Constants
		static inline TiltAngles Identity() { return TiltAngles(0.0, 0.0, 0.0); }

		// Constructors
		TiltAngles() = default;
		explicit TiltAngles(double fusedYaw) : fusedYaw(fusedYaw), tiltAxisAngle(0.0), tiltAngle(0.0) {}
		explicit TiltAngles(double tiltAxisAngle, double tiltAngle) : fusedYaw(0.0), tiltAxisAngle(tiltAxisAngle), tiltAngle(tiltAngle) {}
		TiltAngles(double fusedYaw, double tiltAxisAngle, double tiltAngle) : fusedYaw(fusedYaw), tiltAxisAngle(tiltAxisAngle), tiltAngle(tiltAngle) {}

		// Set functions
		void set(double fusedYaw, double tiltAxisAngle, double tiltAngle) { this->fusedYaw = fusedYaw; this->tiltAxisAngle = tiltAxisAngle; this->tiltAngle = tiltAngle; }
		void setIdentity() { fusedYaw = tiltAxisAngle = tiltAngle = 0.0; }

		// Data members
		double fusedYaw;      // Fused yaw:        psi  is in (-pi,pi]
		double tiltAxisAngle; // Tilt axis angle: gamma is in (-pi,pi]
		double tiltAngle;     // Tilt angle:      alpha is in [0,pi]
	};

	// Stream insertion operator for the TiltAngles struct
	inline std::ostream& operator<<(std::ostream& os, const TiltAngles& t) { return os << "T(" << t.fusedYaw << ", " << t.tiltAxisAngle << ", " << t.tiltAngle << ")"; }

	// Forward declarations of tilt phase structs
	struct TiltPhase2D;
	struct TiltPhase3D;

	// Tilt phase 2D struct with format: (px, py)
	struct TiltPhase2D // Note: POD type
	{
		// Constants
		static inline TiltPhase2D Identity() { return TiltPhase2D(0.0, 0.0); }

		// Constructors
		TiltPhase2D() = default;
		TiltPhase2D(double px, double py) : px(px), py(py) {}
		TiltPhase2D(const TiltAngles& t) { setTilt(t); }
		TiltPhase2D(const TiltPhase3D& p);

		// Set functions
		void set(double px, double py) { this->px = px; this->py = py; }
		void setTilt(double tiltAxisAngle, double tiltAngle) { px = tiltAngle*cos(tiltAxisAngle); py = tiltAngle*sin(tiltAxisAngle); }
		void setTilt(const TiltAngles& t) { setTilt(t.tiltAxisAngle, t.tiltAngle); }
		void setIdentity() { px = py = 0.0; }

		// Get functions
		void get(double& px, double& py) const { px = this->px; py = this->py; }
		void getTilt(double& tiltAxisAngle, double& tiltAngle) const { tiltAxisAngle = atan2(py, px); tiltAngle = sqrt(px*px + py*py); }
		void getTilt(TiltAngles& t) const { t.fusedYaw = 0.0; getTilt(t.tiltAxisAngle, t.tiltAngle); }
		TiltAngles getTilt() const { TiltAngles t; getTilt(t); return t; }

		// Array subscript operator
		double& operator[](std::size_t idx) { return (idx == 0 ? px : py); }
		const double& operator[](std::size_t idx) const { return (idx == 0 ? px : py); }

		// Assignment operator
		TiltPhase2D& operator=(const TiltPhase3D& p); // Note: Does not touch pz!

		// Data members
		double px; // Phase x: tiltAngle*cos(tiltAxisAngle) in R
		double py; // Phase y: tiltAngle*sin(tiltAxisAngle) in R
	};

	// Stream insertion operator for the TiltPhase2D struct
	inline std::ostream& operator<<(std::ostream& os, const TiltPhase2D& p) { return os << "P(" << p.px << ", " << p.py << ")"; }

	// Additional tilt phase 2D operators
	inline TiltPhase2D operator+(const TiltPhase2D& p) { return p; }
	inline TiltPhase2D operator-(const TiltPhase2D& p) { return TiltPhase2D(-p.px, -p.py); }
	inline TiltPhase2D operator+(const TiltPhase2D& pa, const TiltPhase2D& pb) { return TiltPhase2D(pa.px + pb.px, pa.py + pb.py); }
	inline TiltPhase2D operator-(const TiltPhase2D& pa, const TiltPhase2D& pb) { return TiltPhase2D(pa.px - pb.px, pa.py - pb.py); }
	inline TiltPhase2D& operator+=(TiltPhase2D& pa, const TiltPhase2D& pb) { pa.px += pb.px; pa.py += pb.py; return pa; }
	inline TiltPhase2D& operator-=(TiltPhase2D& pa, const TiltPhase2D& pb) { pa.px -= pb.px; pa.py -= pb.py; return pa; }
	inline TiltPhase2D operator*(const TiltPhase2D& p, double s) { return TiltPhase2D(p.px*s, p.py*s); }
	inline TiltPhase2D operator*(double s, const TiltPhase2D& p) { return TiltPhase2D(s*p.px, s*p.py); }
	inline TiltPhase2D operator/(const TiltPhase2D& p, double s) { return TiltPhase2D(p.px/s, p.py/s); }
	inline TiltPhase2D& operator*=(TiltPhase2D& p, double s) { p.px *= s; p.py *= s; return p; }
	inline TiltPhase2D& operator/=(TiltPhase2D& p, double s) { p.px /= s; p.py /= s; return p; }

	// Tilt phase 3D struct with format: (px, py, pz)
	struct TiltPhase3D // Note: POD type
	{
		// Constants
		static inline TiltPhase3D Identity() { return TiltPhase3D(0.0, 0.0, 0.0); }

		// Constructors
		TiltPhase3D() = default;
		explicit TiltPhase3D(double pz) : px(0.0), py(0.0), pz(pz) {}
		TiltPhase3D(double px, double py, double pz = 0.0) : px(px), py(py), pz(pz) {}
		TiltPhase3D(const TiltAngles& t) { setTilt(t); }
		TiltPhase3D(const TiltPhase2D& p);

		// Set functions
		void set(double px, double py, double pz = 0.0) { this->px = px; this->py = py; this->pz = pz; }
		void setTilt(double tiltAxisAngle, double tiltAngle) { px = tiltAngle*cos(tiltAxisAngle); py = tiltAngle*sin(tiltAxisAngle); } // Note: Does not touch pz!
		void setTilt(double tiltAxisAngle, double tiltAngle, double fusedYaw) { setTilt(tiltAxisAngle, tiltAngle); pz = fusedYaw; }
		void setTilt(const TiltAngles& t) { setTilt(t.tiltAxisAngle, t.tiltAngle, t.fusedYaw); }
		void setIdentity() { px = py = pz = 0.0; }

		// Get functions
		void get(double& px, double& py) const { px = this->px; py = this->py; }
		void get(double& px, double& py, double& pz) const { px = this->px; py = this->py; pz = this->pz; }
		void getTilt(double& tiltAxisAngle, double& tiltAngle) const { tiltAxisAngle = atan2(py, px); tiltAngle = sqrt(px*px + py*py); }
		void getTilt(double& tiltAxisAngle, double& tiltAngle, double& fusedYaw) const { fusedYaw = pz; getTilt(tiltAxisAngle, tiltAngle); }
		void getTilt(TiltAngles& t) const { t.fusedYaw = pz; getTilt(t.tiltAxisAngle, t.tiltAngle); }
		TiltAngles getTilt() const { TiltAngles t; getTilt(t); return t; }

		// Array subscript operator
		double& operator[](std::size_t idx) { return (idx == 0 ? px : (idx == 1 ? py : pz)); }
		const double& operator[](std::size_t idx) const { return (idx == 0 ? px : (idx == 1 ? py : pz)); }

		// Assignment operator
		TiltPhase3D& operator=(const TiltPhase2D& p); // Note: Does not touch pz!

		// Data members
		double px; // Phase x: tiltAngle*cos(tiltAxisAngle) in R
		double py; // Phase y: tiltAngle*sin(tiltAxisAngle) in R
		double pz; // Phase z: fusedYaw in (-pi,pi]
	};

	// Stream insertion operator for the TiltPhase3D struct
	inline std::ostream& operator<<(std::ostream& os, const TiltPhase3D& p) { return os << "P(" << p.px << ", " << p.py << ", " << p.pz << ")"; }

	// Additional tilt phase 3D operators
	inline TiltPhase3D operator+(const TiltPhase3D& p) { return p; }
	inline TiltPhase3D operator-(const TiltPhase3D& p) { return TiltPhase3D(-p.px, -p.py, -p.pz); }
	inline TiltPhase3D operator+(const TiltPhase3D& pa, const TiltPhase3D& pb) { return TiltPhase3D(pa.px + pb.px, pa.py + pb.py, pa.pz + pb.pz); }
	inline TiltPhase3D operator-(const TiltPhase3D& pa, const TiltPhase3D& pb) { return TiltPhase3D(pa.px - pb.px, pa.py - pb.py, pa.pz - pb.pz); }
	inline TiltPhase3D& operator+=(TiltPhase3D& pa, const TiltPhase3D& pb) { pa.px += pb.px; pa.py += pb.py; pa.pz += pb.pz; return pa; }
	inline TiltPhase3D& operator-=(TiltPhase3D& pa, const TiltPhase3D& pb) { pa.px -= pb.px; pa.py -= pb.py; pa.pz -= pb.pz; return pa; }
	inline TiltPhase3D operator*(const TiltPhase3D& p, double s) { return TiltPhase3D(p.px*s, p.py*s, p.pz*s); }
	inline TiltPhase3D operator*(double s, const TiltPhase3D& p) { return TiltPhase3D(s*p.px, s*p.py, s*p.pz); }
	inline TiltPhase3D operator/(const TiltPhase3D& p, double s) { return TiltPhase3D(p.px/s, p.py/s, p.pz/s); }
	inline TiltPhase3D& operator*=(TiltPhase3D& p, double s) { p.px *= s; p.py *= s; p.pz *= s; return p; }
	inline TiltPhase3D& operator/=(TiltPhase3D& p, double s) { p.px /= s; p.py /= s; p.pz /= s; return p; }

	// Tilt phase constructors
	inline TiltPhase2D::TiltPhase2D(const TiltPhase3D& p) : px(p.px), py(p.py) {}
	inline TiltPhase3D::TiltPhase3D(const TiltPhase2D& p) : px(p.px), py(p.py), pz(0.0) {}

	// Tilt phase assignment operators
	inline TiltPhase2D& TiltPhase2D::operator=(const TiltPhase3D& p) { px = p.px; py = p.py; return *this; }
	inline TiltPhase3D& TiltPhase3D::operator=(const TiltPhase2D& p) { px = p.px; py = p.py; return *this; }

	// #################################
	// #### Rotation velocity types ####
	// #################################

	// Typedefs
	typedef Eigen::Vector3d AngVel;

	// Forward declarations of tilt phase velocity structs
	struct TiltPhaseVel2D;
	struct TiltPhaseVel3D;

	// Tilt phase velocity 2D struct with format: <pxVel, pyVel>
	struct TiltPhaseVel2D // Note: POD type
	{
		// Constants
		static inline TiltPhaseVel2D Zero() { return TiltPhaseVel2D(0.0, 0.0); }

		// Constructors
		TiltPhaseVel2D() = default;
		TiltPhaseVel2D(double pxVel, double pyVel) : pxVel(pxVel), pyVel(pyVel) {}
		TiltPhaseVel2D(const TiltPhaseVel3D& pdot);

		// Set functions
		void set(double pxVel, double pyVel) { this->pxVel = pxVel; this->pyVel = pyVel; }
		void setZero() { pxVel = pyVel = 0.0; }

		// Get functions
		void get(double& pxVel, double& pyVel) const { pxVel = this->pxVel; pyVel = this->pyVel; }

		// Array subscript operator
		double& operator[](std::size_t idx) { return (idx == 0 ? pxVel : pyVel); }
		const double& operator[](std::size_t idx) const { return (idx == 0 ? pxVel : pyVel); }

		// Assignment operator
		TiltPhaseVel2D& operator=(const TiltPhaseVel3D& pdot); // Note: Does not touch pzVel!

		// Data members
		double pxVel; // Phase x velocity
		double pyVel; // Phase y velocity
	};

	// Stream insertion operator for the TiltPhaseVel2D struct
	inline std::ostream& operator<<(std::ostream& os, const TiltPhaseVel2D& pdot) { return os << "P<" << pdot.pxVel << ", " << pdot.pyVel << ">"; }

	// Additional tilt phase velocity 2D operators
	inline TiltPhaseVel2D operator+(const TiltPhaseVel2D& pdot) { return pdot; }
	inline TiltPhaseVel2D operator-(const TiltPhaseVel2D& pdot) { return TiltPhaseVel2D(-pdot.pxVel, -pdot.pyVel); }
	inline TiltPhaseVel2D operator+(const TiltPhaseVel2D& pdota, const TiltPhaseVel2D& pdotb) { return TiltPhaseVel2D(pdota.pxVel + pdotb.pxVel, pdota.pyVel + pdotb.pyVel); }
	inline TiltPhaseVel2D operator-(const TiltPhaseVel2D& pdota, const TiltPhaseVel2D& pdotb) { return TiltPhaseVel2D(pdota.pxVel - pdotb.pxVel, pdota.pyVel - pdotb.pyVel); }
	inline TiltPhaseVel2D& operator+=(TiltPhaseVel2D& pdota, const TiltPhaseVel2D& pdotb) { pdota.pxVel += pdotb.pxVel; pdota.pyVel += pdotb.pyVel; return pdota; }
	inline TiltPhaseVel2D& operator-=(TiltPhaseVel2D& pdota, const TiltPhaseVel2D& pdotb) { pdota.pxVel -= pdotb.pxVel; pdota.pyVel -= pdotb.pyVel; return pdota; }
	inline TiltPhaseVel2D operator*(const TiltPhaseVel2D& pdot, double s) { return TiltPhaseVel2D(pdot.pxVel*s, pdot.pyVel*s); }
	inline TiltPhaseVel2D operator*(double s, const TiltPhaseVel2D& pdot) { return TiltPhaseVel2D(s*pdot.pxVel, s*pdot.pyVel); }
	inline TiltPhaseVel2D operator/(const TiltPhaseVel2D& pdot, double s) { return TiltPhaseVel2D(pdot.pxVel/s, pdot.pyVel/s); }
	inline TiltPhaseVel2D& operator*=(TiltPhaseVel2D& pdot, double s) { pdot.pxVel *= s; pdot.pyVel *= s; return pdot; }
	inline TiltPhaseVel2D& operator/=(TiltPhaseVel2D& pdot, double s) { pdot.pxVel /= s; pdot.pyVel /= s; return pdot; }

	// Finite difference operations
	inline void TiltPhaseDiff(const TiltPhase2D& pa, const TiltPhase2D& pb, double dt, TiltPhaseVel2D& pdot) { pdot.pxVel = (pb.px - pa.px)/dt; pdot.pyVel = (pb.py - pa.py)/dt; } // Calculates (pb - pa)/dt
	inline TiltPhaseVel2D TiltPhaseDiff(const TiltPhase2D& pa, const TiltPhase2D& pb, double dt) { TiltPhaseVel2D pdot; TiltPhaseDiff(pa, pb, dt, pdot); return pdot; }

	// Tilt phase velocity 3D struct with format: <pxVel, pyVel, pzVel>
	struct TiltPhaseVel3D // Note: POD type
	{
		// Constants
		static inline TiltPhaseVel3D Zero() { return TiltPhaseVel3D(0.0, 0.0, 0.0); }

		// Constructors
		TiltPhaseVel3D() = default;
		explicit TiltPhaseVel3D(double pzVel) : pxVel(0.0), pyVel(0.0), pzVel(pzVel) {}
		TiltPhaseVel3D(double pxVel, double pyVel, double pzVel = 0.0) : pxVel(pxVel), pyVel(pyVel), pzVel(pzVel) {}
		TiltPhaseVel3D(const TiltPhaseVel2D& pdot);

		// Set functions
		void set(double pxVel, double pyVel, double pzVel = 0.0) { this->pxVel = pxVel; this->pyVel = pyVel; this->pzVel = pzVel; }
		void setZero() { pxVel = pyVel = pzVel = 0.0; }

		// Get functions
		void get(double& pxVel, double& pyVel) const { pxVel = this->pxVel; pyVel = this->pyVel; }
		void get(double& pxVel, double& pyVel, double& pzVel) const { pxVel = this->pxVel; pyVel = this->pyVel; pzVel = this->pzVel; }

		// Array subscript operator
		double& operator[](std::size_t idx) { return (idx == 0 ? pxVel : (idx == 1 ? pyVel : pzVel)); }
		const double& operator[](std::size_t idx) const { return (idx == 0 ? pxVel : (idx == 1 ? pyVel : pzVel)); }

		// Assignment operator
		TiltPhaseVel3D& operator=(const TiltPhaseVel2D& pdot); // Note: Does not touch pzVel!

		// Data members
		double pxVel; // Phase x velocity
		double pyVel; // Phase y velocity
		double pzVel; // Phase z velocity
	};

	// Stream insertion operator for the TiltPhaseVel3D struct
	inline std::ostream& operator<<(std::ostream& os, const TiltPhaseVel3D& pdot) { return os << "P<" << pdot.pxVel << ", " << pdot.pyVel << ", " << pdot.pzVel << ">"; }

	// Additional tilt phase velocity 3D operators
	inline TiltPhaseVel3D operator+(const TiltPhaseVel3D& pdot) { return pdot; }
	inline TiltPhaseVel3D operator-(const TiltPhaseVel3D& pdot) { return TiltPhaseVel3D(-pdot.pxVel, -pdot.pyVel, -pdot.pzVel); }
	inline TiltPhaseVel3D operator+(const TiltPhaseVel3D& pdota, const TiltPhaseVel3D& pdotb) { return TiltPhaseVel3D(pdota.pxVel + pdotb.pxVel, pdota.pyVel + pdotb.pyVel, pdota.pzVel + pdotb.pzVel); }
	inline TiltPhaseVel3D operator-(const TiltPhaseVel3D& pdota, const TiltPhaseVel3D& pdotb) { return TiltPhaseVel3D(pdota.pxVel - pdotb.pxVel, pdota.pyVel - pdotb.pyVel, pdota.pzVel - pdotb.pzVel); }
	inline TiltPhaseVel3D& operator+=(TiltPhaseVel3D& pdota, const TiltPhaseVel3D& pdotb) { pdota.pxVel += pdotb.pxVel; pdota.pyVel += pdotb.pyVel; pdota.pzVel += pdotb.pzVel; return pdota; }
	inline TiltPhaseVel3D& operator-=(TiltPhaseVel3D& pdota, const TiltPhaseVel3D& pdotb) { pdota.pxVel -= pdotb.pxVel; pdota.pyVel -= pdotb.pyVel; pdota.pzVel -= pdotb.pzVel; return pdota; }
	inline TiltPhaseVel3D operator*(const TiltPhaseVel3D& pdot, double s) { return TiltPhaseVel3D(pdot.pxVel*s, pdot.pyVel*s, pdot.pzVel*s); }
	inline TiltPhaseVel3D operator*(double s, const TiltPhaseVel3D& pdot) { return TiltPhaseVel3D(s*pdot.pxVel, s*pdot.pyVel, s*pdot.pzVel); }
	inline TiltPhaseVel3D operator/(const TiltPhaseVel3D& pdot, double s) { return TiltPhaseVel3D(pdot.pxVel/s, pdot.pyVel/s, pdot.pzVel/s); }
	inline TiltPhaseVel3D& operator*=(TiltPhaseVel3D& pdot, double s) { pdot.pxVel *= s; pdot.pyVel *= s; pdot.pzVel *= s; return pdot; }
	inline TiltPhaseVel3D& operator/=(TiltPhaseVel3D& pdot, double s) { pdot.pxVel /= s; pdot.pyVel /= s; pdot.pzVel /= s; return pdot; }

	// Finite difference operations
	inline void TiltPhaseDiff(const TiltPhase3D& pa, const TiltPhase3D& pb, double dt, TiltPhaseVel3D& pdot) { pdot.pxVel = (pb.px - pa.px)/dt; pdot.pyVel = (pb.py - pa.py)/dt; pdot.pzVel = (pb.pz - pa.pz)/dt; } // Calculates (pb - pa)/dt
	inline TiltPhaseVel3D TiltPhaseDiff(const TiltPhase3D& pa, const TiltPhase3D& pb, double dt) { TiltPhaseVel3D pdot; TiltPhaseDiff(pa, pb, dt, pdot); return pdot; }

	// Tilt phase velocity constructors
	inline TiltPhaseVel2D::TiltPhaseVel2D(const TiltPhaseVel3D& pdot) : pxVel(pdot.pxVel), pyVel(pdot.pyVel) {}
	inline TiltPhaseVel3D::TiltPhaseVel3D(const TiltPhaseVel2D& pdot) : pxVel(pdot.pxVel), pyVel(pdot.pyVel), pzVel(0.0) {}

	// Tilt phase velocity assignment operators
	inline TiltPhaseVel2D& TiltPhaseVel2D::operator=(const TiltPhaseVel3D& pdot) { pxVel = pdot.pxVel; pyVel = pdot.pyVel; return *this; }
	inline TiltPhaseVel3D& TiltPhaseVel3D::operator=(const TiltPhaseVel2D& pdot) { pxVel = pdot.pxVel; pyVel = pdot.pyVel; return *this; }

	// ################################
	// #### Rotation normalisation ####
	// ################################

	// Normalise: Rotation matrix
	void NormaliseRotmat(Rotmat& R);
	inline Rotmat NormalisedRotmat(const Rotmat& R) { Rotmat Rout = R; NormaliseRotmat(Rout); return Rout; }

	// Normalise: Quaternion
	inline double QuatNorm(const Quat& q) { return sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z()); }
	inline double QuatNormSq(const Quat& q) { return q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z(); }
	void NormaliseQuat(Quat& q, double normTol = 0.0);
	inline Quat NormalisedQuat(const Quat& q, double normTol = 0.0) { Quat qout = q; NormaliseQuat(qout, normTol); return qout; }

	// Normalise: Vector
	inline double VecNorm(const Vec3& v) { return sqrt(v.x()*v.x() + v.y()*v.y() + v.z()*v.z()); }
	inline double VecNormSq(const Vec3& v) { return v.x()*v.x() + v.y()*v.y() + v.z()*v.z(); }
	void NormaliseVec(Vec3& v, double normTol = 0.0, const Vec3& vdef = VecUnitZ());
	inline Vec3 NormalisedVec(const Vec3& v, double normTol = 0.0, const Vec3& vdef = VecUnitZ()) { Vec3 vout = v; NormaliseVec(vout, normTol, vdef); return vout; }

	// ##########################
	// #### Random rotations ####
	// ##########################

	// Random: Angle
	inline double RandAng(double maxAbs = M_PI) { return maxAbs*((2.0*std::rand()) / RAND_MAX - 1.0); } // Range [-maxAbs,maxAbs]
	inline double RandAng(double min, double max) { return min + ((max - min) * std::rand()) / RAND_MAX; } // Range [min,max]

	// Random: Vector
	Vec3 RandVec(double maxNorm);
	Vec3 RandUnitVec();

	// Random: Rotation matrix
	void RandRotmat(Rotmat& R);
	inline Rotmat RandRotmat() { Rotmat R; RandRotmat(R); return R; }

	// Random: Quaternion
	void RandQuat(Quat& q);
	inline Quat RandQuat() { Quat q; RandQuat(q); return q; }

	// Random: Euler angles
	void RandEuler(EulerAngles& e);
	inline EulerAngles RandEuler() { EulerAngles e; RandEuler(e); return e; }

	// Random: Fused angles
	void RandFused(FusedAngles& f);
	inline FusedAngles RandFused() { FusedAngles f; RandFused(f); return f; }

	// Random: Tilt angles
	void RandTilt(TiltAngles& t);
	inline TiltAngles RandTilt() { TiltAngles t; RandTilt(t); return t; }

	// ##########################################
	// #### Rotation checking and validation ####
	// ##########################################

	// Check and validate: Rotation matrix
	bool ValidateRotmat(Rotmat& R, double tol = ROT_CONV_DEFAULT_TOL);
	inline bool CheckRotmat(const Rotmat& R, double tol = ROT_CONV_DEFAULT_TOL) { Rotmat RR = R; return ValidateRotmat(RR, tol); }
	inline bool CheckRotmat(const Rotmat& R, Rotmat& Rout, double tol = ROT_CONV_DEFAULT_TOL) { Rout = R; return ValidateRotmat(Rout, tol); }

	// Check and validate: Quaternion
	bool ValidateQuat(Quat& q, double tol = ROT_CONV_DEFAULT_TOL, bool unique = false);
	inline bool CheckQuat(const Quat& q, double tol = ROT_CONV_DEFAULT_TOL, bool unique = false) { Quat qq = q; return ValidateQuat(qq, tol, unique); }
	inline bool CheckQuat(const Quat& q, Quat& qout, double tol = ROT_CONV_DEFAULT_TOL, bool unique = false) { qout = q; return ValidateQuat(qout, tol, unique); }

	// Check and validate: Euler angles
	bool ValidateEuler(EulerAngles& e, double tol = ROT_CONV_DEFAULT_TOL, bool unique = false);
	inline bool CheckEuler(const EulerAngles& e, double tol = ROT_CONV_DEFAULT_TOL, bool unique = false) { EulerAngles ee = e; return ValidateEuler(ee, tol, unique); }
	inline bool CheckEuler(const EulerAngles& e, EulerAngles& eout, double tol = ROT_CONV_DEFAULT_TOL, bool unique = false) { eout = e; return ValidateEuler(eout, tol, unique); }

	// Check and validate: Fused angles
	bool ValidateFused(FusedAngles& f, double tol = ROT_CONV_DEFAULT_TOL, bool unique = false);
	inline bool CheckFused(const FusedAngles& f, double tol = ROT_CONV_DEFAULT_TOL, bool unique = false) { FusedAngles ff = f; return ValidateFused(ff, tol, unique); }
	inline bool CheckFused(const FusedAngles& f, FusedAngles& fout, double tol = ROT_CONV_DEFAULT_TOL, bool unique = false) { fout = f; return ValidateFused(fout, tol, unique); }

	// Check and validate: Tilt angles
	bool ValidateTilt(TiltAngles& t, double tol = ROT_CONV_DEFAULT_TOL, bool unique = false);
	inline bool CheckTilt(const TiltAngles& t, double tol = ROT_CONV_DEFAULT_TOL, bool unique = false) { TiltAngles tt = t; return ValidateTilt(tt, tol, unique); }
	inline bool CheckTilt(const TiltAngles& t, TiltAngles& tout, double tol = ROT_CONV_DEFAULT_TOL, bool unique = false) { tout = t; return ValidateTilt(tout, tol, unique); }

	// ###########################
	// #### Rotation equality ####
	// ###########################

	// Check equality: Rotation matrix
	bool RotmatEqual(const Rotmat& Ra, const Rotmat& Rb, double tol = ROT_CONV_DEFAULT_TOL);
	bool RotmatEqualExact(const Rotmat& Ra, const Rotmat& Rb, double tol = ROT_CONV_DEFAULT_TOL);

	// Check equality: Quaternion
	bool QuatEqual(const Quat& qa, const Quat& qb, double tol = ROT_CONV_DEFAULT_TOL);
	bool QuatEqualExact(const Quat& qa, const Quat& qb, double tol = ROT_CONV_DEFAULT_TOL);

	// Check equality: Euler angles
	bool EulerEqual(const EulerAngles& ea, const EulerAngles& eb, double tol = ROT_CONV_DEFAULT_TOL);
	bool EulerEqualExact(const EulerAngles& ea, const EulerAngles& eb, double tol = ROT_CONV_DEFAULT_TOL);

	// Check equality: Fused angles
	bool FusedEqual(const FusedAngles& fa, const FusedAngles& fb, double tol = ROT_CONV_DEFAULT_TOL);
	bool FusedEqualExact(const FusedAngles& fa, const FusedAngles& fb, double tol = ROT_CONV_DEFAULT_TOL);

	// Check equality: Tilt angles
	bool TiltEqual(const TiltAngles& ta, const TiltAngles& tb, double tol = ROT_CONV_DEFAULT_TOL);
	bool TiltEqualExact(const TiltAngles& ta, const TiltAngles& tb, double tol = ROT_CONV_DEFAULT_TOL);

	// ##############################
	// #### Rotation composition ####
	// ##############################

	// Helper functions
	namespace internal
	{
		// Compose: Rotation matrix
		inline void ComposeRotmatHelper(Rotmat& Rout) {}
		template<typename... Types> void ComposeRotmatHelper(Rotmat& Rout, const Rotmat& R, Types&&... args);

		// Compose: Quaternion
		inline void ComposeQuatHelper(Quat& qout) {}
		template<typename... Types> void ComposeQuatHelper(Quat& qout, const Quat& q, Types&&... args);

		// Compose: Euler angles
		inline void ComposeEulerHelper(Rotmat& Rout) {}
		template<typename... Types> void ComposeEulerHelper(Rotmat& Rout, const EulerAngles& e, Types&&... args);

		// Compose: Fused angles
		inline void ComposeFusedHelper(Rotmat& Rout) {}
		template<typename... Types> void ComposeFusedHelper(Rotmat& Rout, const FusedAngles& f, Types&&... args);

		// Compose: Tilt angles
		inline void ComposeTiltHelper(Rotmat& Rout) {}
		template<typename... Types> void ComposeTiltHelper(Rotmat& Rout, const TiltAngles& t, Types&&... args);
	}

	// Compose: Rotation matrix (alternative to operator*)
	template<typename... Types> Rotmat ComposeRotmat(const Rotmat& R, Types&&... args);
	template<typename... Types> inline Rotmat RotmatMult(Types&&... args) { return ComposeRotmat(std::forward<Types>(args)...); } // Function alias for convenience

	// Compose: Quaternion (alternative to operator*)
	template<typename... Types> Quat ComposeQuat(const Quat& q, Types&&... args);
	template<typename... Types> inline Quat QuatMult(Types&&... args) { return ComposeQuat(std::forward<Types>(args)...); } // Function alias for convenience

	// Compose: Euler angles
	template<typename... Types> EulerAngles ComposeEuler(const EulerAngles& e, Types&&... args);

	// Compose: Fused angles
	template<typename... Types> FusedAngles ComposeFused(const FusedAngles& f, Types&&... args);

	// Compose: Tilt angles
	template<typename... Types> TiltAngles ComposeTilt(const TiltAngles& t, Types&&... args);

	// #########################
	// #### Yaw of rotation ####
	// #########################

	// Yaw of: Rotation matrix
	double EYawOfRotmat(const Rotmat& R);
	double FYawOfRotmat(const Rotmat& R);

	// Yaw of: Quaternion
	double EYawOfQuat(const Quat& q);
	double FYawOfQuat(const Quat& q);

	// Yaw of: Euler angles
	inline double EYawOfEuler(const EulerAngles& e) { return e.yaw; }
	double FYawOfEuler(const EulerAngles& e);

	// Yaw of: Fused angles
	double EYawOfFused(const FusedAngles& f);
	inline double FYawOfFused(const FusedAngles& f) { return f.fusedYaw; }

	// Yaw of: Tilt angles
	double EYawOfTilt(const TiltAngles& t);
	inline double FYawOfTilt(const TiltAngles& t) { return t.fusedYaw; }

	// ##################################
	// #### Remove yaw from rotation ####
	// ##################################

	// Remove yaw from: Rotation matrix
	void RotmatNoEYaw(const Rotmat& R, Rotmat& Rout);
	void RotmatNoFYaw(const Rotmat& R, Rotmat& Rout);
	inline Rotmat RotmatNoEYaw(const Rotmat& R) { Rotmat Rout; RotmatNoEYaw(R, Rout); return Rout; }
	inline Rotmat RotmatNoFYaw(const Rotmat& R) { Rotmat Rout; RotmatNoFYaw(R, Rout); return Rout; }

	// Remove yaw from: Quaternion
	void QuatNoEYaw(const Quat& q, Quat& qout);
	void QuatNoFYaw(const Quat& q, Quat& qout);
	inline Quat QuatNoEYaw(const Quat& q) { Quat qout; QuatNoEYaw(q, qout); return qout; }
	inline Quat QuatNoFYaw(const Quat& q) { Quat qout; QuatNoFYaw(q, qout); return qout; }

	// Remove yaw from: Euler angles
	inline void EulerNoEYaw(const EulerAngles& e, EulerAngles& eout) { eout.set(0.0, e.pitch, e.roll); }
	void EulerNoFYaw(const EulerAngles& e, EulerAngles& eout);
	inline EulerAngles EulerNoEYaw(const EulerAngles& e) { return EulerAngles(0.0, e.pitch, e.roll); }
	inline EulerAngles EulerNoFYaw(const EulerAngles& e) { EulerAngles eout; EulerNoFYaw(e, eout); return eout; }

	// Remove yaw from: Fused angles
	void FusedNoEYaw(const FusedAngles& f, FusedAngles& fout);
	inline void FusedNoFYaw(const FusedAngles& f, FusedAngles& fout) { fout.set(0.0, f.fusedPitch, f.fusedRoll, f.hemi); }
	inline FusedAngles FusedNoEYaw(const FusedAngles& f) { FusedAngles fout; FusedNoEYaw(f, fout); return fout; }
	inline FusedAngles FusedNoFYaw(const FusedAngles& f) { return FusedAngles(f.fusedPitch, f.fusedRoll, f.hemi); }

	// Remove yaw from: Tilt angles
	void TiltNoEYaw(const TiltAngles& t, TiltAngles& tout);
	inline void TiltNoFYaw(const TiltAngles& t, TiltAngles& tout) { tout.set(0.0, t.tiltAxisAngle, t.tiltAngle); }
	inline TiltAngles TiltNoEYaw(const TiltAngles& t) { TiltAngles tout; TiltNoEYaw(t, tout); return tout; }
	inline TiltAngles TiltNoFYaw(const TiltAngles& t) { return TiltAngles(t.tiltAxisAngle, t.tiltAngle); }

	// #################################
	// #### Rotation with given yaw ####
	// #################################

	// Rotation with given yaw: Rotation matrix
	void RotmatWithEYaw(const Rotmat& R, double eulerYaw, Rotmat& Rout);
	void RotmatWithFYaw(const Rotmat& R, double fusedYaw, Rotmat& Rout);
	inline Rotmat RotmatWithEYaw(const Rotmat& R, double eulerYaw) { Rotmat Rout; RotmatWithEYaw(R, eulerYaw, Rout); return Rout; }
	inline Rotmat RotmatWithFYaw(const Rotmat& R, double fusedYaw) { Rotmat Rout; RotmatWithFYaw(R, fusedYaw, Rout); return Rout; }

	// Rotation with given yaw: Quaternion
	void QuatWithEYaw(const Quat& q, double eulerYaw, Quat& qout);
	void QuatWithFYaw(const Quat& q, double fusedYaw, Quat& qout);
	inline Quat QuatWithEYaw(const Quat& q, double eulerYaw) { Quat qout; QuatWithEYaw(q, eulerYaw, qout); return qout; }
	inline Quat QuatWithFYaw(const Quat& q, double fusedYaw) { Quat qout; QuatWithFYaw(q, fusedYaw, qout); return qout; }

	// Rotation with given yaw: Euler angles
	inline void EulerWithEYaw(const EulerAngles& e, double eulerYaw, EulerAngles& eout) { eout.set(eulerYaw, e.pitch, e.roll); }
	void EulerWithFYaw(const EulerAngles& e, double fusedYaw, EulerAngles& eout);
	inline EulerAngles EulerWithEYaw(const EulerAngles& e, double eulerYaw) { return EulerAngles(eulerYaw, e.pitch, e.roll); }
	inline EulerAngles EulerWithFYaw(const EulerAngles& e, double fusedYaw) { EulerAngles eout; EulerWithFYaw(e, fusedYaw, eout); return eout; }

	// Rotation with given yaw: Fused angles
	void FusedWithEYaw(const FusedAngles& f, double eulerYaw, FusedAngles& fout);
	inline void FusedWithFYaw(const FusedAngles& f, double fusedYaw, FusedAngles& fout) { fout.set(fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); }
	inline FusedAngles FusedWithEYaw(const FusedAngles& f, double eulerYaw) { FusedAngles fout; FusedWithEYaw(f, eulerYaw, fout); return fout; }
	inline FusedAngles FusedWithFYaw(const FusedAngles& f, double fusedYaw) { return FusedAngles(fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); }

	// Rotation with given yaw: Tilt angles
	void TiltWithEYaw(const TiltAngles& t, double eulerYaw, TiltAngles& tout);
	inline void TiltWithFYaw(const TiltAngles& t, double fusedYaw, TiltAngles& tout) { tout.set(fusedYaw, t.tiltAxisAngle, t.tiltAngle); }
	inline TiltAngles TiltWithEYaw(const TiltAngles& t, double eulerYaw) { TiltAngles tout; TiltWithEYaw(t, eulerYaw, tout); return tout; }
	inline TiltAngles TiltWithFYaw(const TiltAngles& t, double fusedYaw) { return TiltAngles(fusedYaw, t.tiltAxisAngle, t.tiltAngle); }

	// ###########################
	// #### Rotation inverses ####
	// ###########################

	// Inverse: Rotation matrix
	void RotmatInv(const Rotmat& R, Rotmat& Rinv);
	inline Rotmat RotmatInv(const Rotmat& R) { Rotmat Rinv; RotmatInv(R, Rinv); return Rinv; }

	// Inverse: Quaternion
	void QuatInv(const Quat& q, Quat& qinv);
	inline Quat QuatInv(const Quat& q) { Quat qinv; QuatInv(q, qinv); return qinv; }

	// Inverse: Euler angles
	void EulerInv(const EulerAngles& e, EulerAngles& einv);
	inline EulerAngles EulerInv(const EulerAngles& e) { EulerAngles einv; EulerInv(e, einv); return einv; }

	// Inverse: Fused angles
	void FusedInv(const FusedAngles& f, FusedAngles& finv);
	inline FusedAngles FusedInv(const FusedAngles& f) { FusedAngles finv; FusedInv(f, finv); return finv; }

	// Inverse: Tilt angles
	void TiltInv(const TiltAngles& t, TiltAngles& tinv);
	inline TiltAngles TiltInv(const TiltAngles& t) { TiltAngles tinv; TiltInv(t, tinv); return tinv; }

	// ##########################
	// #### Vector rotations ####
	// ##########################

	// Rotate vector by: Rotation matrix
	Vec3 RotmatRotVec(const Rotmat& R, const Vec3& v);
	void RotmatRotVecInPlace(const Rotmat& R, Vec3& v);
	Vec3 RotmatRotVecPureZ(const Rotmat& R, double vz);

	// Rotate vector by: Quaternion
	Vec3 QuatRotVec(const Quat& q, const Vec3& v);
	void QuatRotVecInPlace(const Quat& q, Vec3& v);
	Vec3 QuatRotVecPureZ(const Quat& q, double vz);

	// Rotate vector by: Euler angles
	Vec3 EulerRotVec(const EulerAngles& e, const Vec3& v);
	void EulerRotVecInPlace(const EulerAngles& e, Vec3& v);
	Vec3 EulerRotVecPureZ(const EulerAngles& e, double vz);

	// Rotate vector by: Fused angles
	Vec3 FusedRotVec(const FusedAngles& f, const Vec3& v);
	void FusedRotVecInPlace(const FusedAngles& f, Vec3& v);
	Vec3 FusedRotVecPureZ(const FusedAngles& f, double vz);

	// Rotate vector by: Tilt angles
	Vec3 TiltRotVec(const TiltAngles& t, const Vec3& v);
	void TiltRotVecInPlace(const TiltAngles& t, Vec3& v);
	Vec3 TiltRotVecPureZ(const TiltAngles& t, double vz);

	// #########################################
	// #### Rotation about global unit axis ####
	// #########################################

	// Rotate about global unit axis: Rotation matrix
	void RotmatRotGlobalX(const Rotmat& R, double angle, Rotmat& Rout);
	void RotmatRotGlobalY(const Rotmat& R, double angle, Rotmat& Rout);
	void RotmatRotGlobalZ(const Rotmat& R, double angle, Rotmat& Rout);
	inline Rotmat RotmatRotGlobalX(const Rotmat& R, double angle) { Rotmat Rout; RotmatRotGlobalX(R, angle, Rout); return Rout; }
	inline Rotmat RotmatRotGlobalY(const Rotmat& R, double angle) { Rotmat Rout; RotmatRotGlobalY(R, angle, Rout); return Rout; }
	inline Rotmat RotmatRotGlobalZ(const Rotmat& R, double angle) { Rotmat Rout; RotmatRotGlobalZ(R, angle, Rout); return Rout; }

	// Rotate about global unit axis: Quaternion
	void QuatRotGlobalX(const Quat& q, double angle, Quat& qout);
	void QuatRotGlobalY(const Quat& q, double angle, Quat& qout);
	void QuatRotGlobalZ(const Quat& q, double angle, Quat& qout);
	inline Quat QuatRotGlobalX(const Quat& q, double angle) { Quat qout; QuatRotGlobalX(q, angle, qout); return qout; }
	inline Quat QuatRotGlobalY(const Quat& q, double angle) { Quat qout; QuatRotGlobalY(q, angle, qout); return qout; }
	inline Quat QuatRotGlobalZ(const Quat& q, double angle) { Quat qout; QuatRotGlobalZ(q, angle, qout); return qout; }

	// Rotate about global unit axis: Euler angles
	void EulerRotGlobalX(const EulerAngles& e, double angle, EulerAngles& eout);
	void EulerRotGlobalY(const EulerAngles& e, double angle, EulerAngles& eout);
	inline void EulerRotGlobalZ(const EulerAngles& e, double angle, EulerAngles& eout) { eout.set(internal::picut(e.yaw + angle), e.pitch, e.roll); }
	inline EulerAngles EulerRotGlobalX(const EulerAngles& e, double angle) { EulerAngles eout; EulerRotGlobalX(e, angle, eout); return eout; }
	inline EulerAngles EulerRotGlobalY(const EulerAngles& e, double angle) { EulerAngles eout; EulerRotGlobalY(e, angle, eout); return eout; }
	inline EulerAngles EulerRotGlobalZ(const EulerAngles& e, double angle) { EulerAngles eout; EulerRotGlobalZ(e, angle, eout); return eout; }

	// Rotate about global unit axis: Fused angles
	void FusedRotGlobalX(const FusedAngles& f, double angle, FusedAngles& fout);
	void FusedRotGlobalY(const FusedAngles& f, double angle, FusedAngles& fout);
	inline void FusedRotGlobalZ(const FusedAngles& f, double angle, FusedAngles& fout) { fout.set(internal::picut(f.fusedYaw + angle), f.fusedPitch, f.fusedRoll, f.hemi); }
	inline FusedAngles FusedRotGlobalX(const FusedAngles& f, double angle) { FusedAngles fout; FusedRotGlobalX(f, angle, fout); return fout; }
	inline FusedAngles FusedRotGlobalY(const FusedAngles& f, double angle) { FusedAngles fout; FusedRotGlobalY(f, angle, fout); return fout; }
	inline FusedAngles FusedRotGlobalZ(const FusedAngles& f, double angle) { FusedAngles fout; FusedRotGlobalZ(f, angle, fout); return fout; }

	// Rotate about global unit axis: Tilt angles
	void TiltRotGlobalX(const TiltAngles& t, double angle, TiltAngles& tout);
	void TiltRotGlobalY(const TiltAngles& t, double angle, TiltAngles& tout);
	inline void TiltRotGlobalZ(const TiltAngles& t, double angle, TiltAngles& tout) { tout.set(internal::picut(t.fusedYaw + angle), t.tiltAxisAngle, t.tiltAngle); }
	inline TiltAngles TiltRotGlobalX(const TiltAngles& t, double angle) { TiltAngles tout; TiltRotGlobalX(t, angle, tout); return tout; }
	inline TiltAngles TiltRotGlobalY(const TiltAngles& t, double angle) { TiltAngles tout; TiltRotGlobalY(t, angle, tout); return tout; }
	inline TiltAngles TiltRotGlobalZ(const TiltAngles& t, double angle) { TiltAngles tout; TiltRotGlobalZ(t, angle, tout); return tout; }

	// #####################################
	// #### Conversions from axis angle ####
	// #####################################

	// Conversion: Axis angle --> Rotation matrix
	void RotmatFromAxis(UnitAxis axis, double angle, Rotmat& R);
	void RotmatFromAxis(const Vec3& axis, double angle, Rotmat& R);
	inline Rotmat RotmatFromAxis(UnitAxis axis, double angle) { Rotmat R; RotmatFromAxis(axis, angle, R); return R; }
	inline Rotmat RotmatFromAxis(const Vec3& axis, double angle) { Rotmat R; RotmatFromAxis(axis, angle, R); return R; }

	// Conversion: Axis angle --> Quaternion
	void QuatFromAxis(UnitAxis axis, double angle, Quat& q);
	void QuatFromAxis(const Vec3& axis, double angle, Quat& q);
	inline Quat QuatFromAxis(UnitAxis axis, double angle) { Quat q; QuatFromAxis(axis, angle, q); return q; }
	inline Quat QuatFromAxis(const Vec3& axis, double angle) { Quat q; QuatFromAxis(axis, angle, q); return q; }

	// Conversion: Axis angle --> Euler angles
	void EulerFromAxis(UnitAxis axis, double angle, EulerAngles& e);
	void EulerFromAxis(const Vec3& axis, double angle, EulerAngles& e);
	inline EulerAngles EulerFromAxis(UnitAxis axis, double angle) { EulerAngles e; EulerFromAxis(axis, angle, e); return e; }
	inline EulerAngles EulerFromAxis(const Vec3& axis, double angle) { EulerAngles e; EulerFromAxis(axis, angle, e); return e; }

	// Conversion: Axis angle --> Fused angles
	void FusedFromAxis(UnitAxis axis, double angle, FusedAngles& f);
	void FusedFromAxis(const Vec3& axis, double angle, FusedAngles& f);
	inline FusedAngles FusedFromAxis(UnitAxis axis, double angle) { FusedAngles f; FusedFromAxis(axis, angle, f); return f; }
	inline FusedAngles FusedFromAxis(const Vec3& axis, double angle) { FusedAngles f; FusedFromAxis(axis, angle, f); return f; }

	// Conversion: Axis angle --> Tilt angles
	void TiltFromAxis(UnitAxis axis, double angle, TiltAngles& t);
	void TiltFromAxis(const Vec3& axis, double angle, TiltAngles& t);
	inline TiltAngles TiltFromAxis(UnitAxis axis, double angle) { TiltAngles t; TiltFromAxis(axis, angle, t); return t; }
	inline TiltAngles TiltFromAxis(const Vec3& axis, double angle) { TiltAngles t; TiltFromAxis(axis, angle, t); return t; }

	// Conversion: Axis angle --> Z vector
	void ZVecFromAxis(UnitAxis axis, double angle, ZVec& z);
	void ZVecFromAxis(const Vec3& axis, double angle, ZVec& z);
	inline ZVec ZVecFromAxis(UnitAxis axis, double angle) { ZVec z; ZVecFromAxis(axis, angle, z); return z; }
	inline ZVec ZVecFromAxis(const Vec3& axis, double angle) { ZVec z; ZVecFromAxis(axis, angle, z); return z; }

	// ############################################
	// #### Conversions from rotation matrices ####
	// ############################################

	// Conversion: Rotation matrix --> Axes
	inline void AxisXFromRotmat(const Rotmat& R, Vec3& axis) { axis = R.col(0); }
	inline void AxisYFromRotmat(const Rotmat& R, Vec3& axis) { axis = R.col(1); }
	inline void AxisZFromRotmat(const Rotmat& R, Vec3& axis) { axis = R.col(2); }
	inline Vec3 AxisXFromRotmat(const Rotmat& R) { return R.col(0); }
	inline Vec3 AxisYFromRotmat(const Rotmat& R) { return R.col(1); }
	inline Vec3 AxisZFromRotmat(const Rotmat& R) { return R.col(2); }

	// Conversion: Rotation matrix --> Quaternion
	void QuatFromRotmat(const Rotmat& R, Quat& q);
	inline Quat QuatFromRotmat(const Rotmat& R) { Quat q; QuatFromRotmat(R, q); return q; }

	// Conversion: Rotation matrix --> Euler angles
	inline void EulerFromRotmat(const Rotmat& R, double& yaw) { yaw = EYawOfRotmat(R); }
	void EulerFromRotmat(const Rotmat& R, double& yaw, double& pitch, double& roll);
	inline void EulerFromRotmat(const Rotmat& R, EulerAngles& e) { EulerFromRotmat(R, e.yaw, e.pitch, e.roll); }
	inline EulerAngles EulerFromRotmat(const Rotmat& R) { EulerAngles e; EulerFromRotmat(R, e.yaw, e.pitch, e.roll); return e; }

	// Conversion: Rotation matrix --> Fused angles
	inline void FusedFromRotmat(const Rotmat& R, double& fusedYaw) { fusedYaw = FYawOfRotmat(R); }
	void FusedFromRotmat(const Rotmat& R, double& fusedPitch, double& fusedRoll);
	void FusedFromRotmat(const Rotmat& R, double& fusedYaw, double& fusedPitch, double& fusedRoll);
	void FusedFromRotmat(const Rotmat& R, double& fusedYaw, double& fusedPitch, double& fusedRoll, bool& hemi);
	inline void FusedFromRotmat(const Rotmat& R, FusedAngles& f) { FusedFromRotmat(R, f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); }
	inline FusedAngles FusedFromRotmat(const Rotmat& R) { FusedAngles f; FusedFromRotmat(R, f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); return f; }

	// Conversion: Rotation matrix --> Tilt angles
	inline void TiltFromRotmat(const Rotmat& R, double& fusedYaw) { fusedYaw = FYawOfRotmat(R); }
	void TiltFromRotmat(const Rotmat& R, double& tiltAxisAngle, double& tiltAngle);
	void TiltFromRotmat(const Rotmat& R, double& fusedYaw, double& tiltAxisAngle, double& tiltAngle);
	inline void TiltFromRotmat(const Rotmat& R, TiltAngles& t) { TiltFromRotmat(R, t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); }
	inline TiltAngles TiltFromRotmat(const Rotmat& R) { TiltAngles t; TiltFromRotmat(R, t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); return t; }

	// Conversion: Rotation matrix --> Z vector
	inline void ZVecFromRotmat(const Rotmat& R, ZVec& z) { z = R.row(2); }
	inline ZVec ZVecFromRotmat(const Rotmat& R) { return R.row(2); }

	// ######################################
	// #### Conversions from quaternions ####
	// ######################################

	// Conversion: Quaternion --> Axes
	void AxisXFromQuat(const Quat& q, Vec3& axis);
	void AxisYFromQuat(const Quat& q, Vec3& axis);
	void AxisZFromQuat(const Quat& q, Vec3& axis);
	inline Vec3 AxisXFromQuat(const Quat& q) { Vec3 axis; AxisXFromQuat(q, axis); return axis; }
	inline Vec3 AxisYFromQuat(const Quat& q) { Vec3 axis; AxisYFromQuat(q, axis); return axis; }
	inline Vec3 AxisZFromQuat(const Quat& q) { Vec3 axis; AxisZFromQuat(q, axis); return axis; }

	// Conversion: Quaternion --> Rotation matrix
	void RotmatFromQuat(const Quat& q, Rotmat& R);
	inline Rotmat RotmatFromQuat(const Quat& q) { Rotmat R; RotmatFromQuat(q, R); return R; }

	// Conversion: Quaternion --> Euler angles
	inline void EulerFromQuat(const Quat& q, double& yaw) { yaw = EYawOfQuat(q); }
	void EulerFromQuat(const Quat& q, double& yaw, double& pitch, double& roll);
	inline void EulerFromQuat(const Quat& q, EulerAngles& e) { EulerFromQuat(q, e.yaw, e.pitch, e.roll); }
	inline EulerAngles EulerFromQuat(const Quat& q) { EulerAngles e; EulerFromQuat(q, e.yaw, e.pitch, e.roll); return e; }

	// Conversion: Quaternion --> Fused angles
	inline void FusedFromQuat(const Quat& q, double& fusedYaw) { fusedYaw = FYawOfQuat(q); }
	void FusedFromQuat(const Quat& q, double& fusedPitch, double& fusedRoll);
	void FusedFromQuat(const Quat& q, double& fusedYaw, double& fusedPitch, double& fusedRoll);
	void FusedFromQuat(const Quat& q, double& fusedYaw, double& fusedPitch, double& fusedRoll, bool& hemi);
	inline void FusedFromQuat(const Quat& q, FusedAngles& f) { FusedFromQuat(q, f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); }
	inline FusedAngles FusedFromQuat(const Quat& q) { FusedAngles f; FusedFromQuat(q, f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); return f; }

	// Conversion: Quaternion --> Tilt angles
	inline void TiltFromQuat(const Quat& q, double& fusedYaw) { fusedYaw = FYawOfQuat(q); }
	void TiltFromQuat(const Quat& q, double& tiltAxisAngle, double& tiltAngle);
	void TiltFromQuat(const Quat& q, double& fusedYaw, double& tiltAxisAngle, double& tiltAngle);
	inline void TiltFromQuat(const Quat& q, TiltAngles& t) { TiltFromQuat(q, t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); }
	inline TiltAngles TiltFromQuat(const Quat& q) { TiltAngles t; TiltFromQuat(q, t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); return t; }

	// Conversion: Quaternion --> Z vector
	void ZVecFromQuat(const Quat& q, ZVec& z);
	inline ZVec ZVecFromQuat(const Quat& q) { ZVec z; ZVecFromQuat(q, z); return z; }

	// #######################################
	// #### Conversions from Euler angles ####
	// #######################################

	// Conversion: Euler angles --> Rotation matrix
	inline Rotmat RotmatFromEuler(double yaw) { return RotmatFromAxis(Z_AXIS, yaw); }
	Rotmat RotmatFromEuler(double yaw, double pitch, double roll);
	inline Rotmat RotmatFromEuler(const EulerAngles& e) { return RotmatFromEuler(e.yaw, e.pitch, e.roll); }

	// Conversion: Euler angles --> Quaternion
	inline Quat QuatFromEuler(double yaw) { return QuatFromAxis(Z_AXIS, yaw); }
	Quat QuatFromEuler(double yaw, double pitch, double roll);
	inline Quat QuatFromEuler(const EulerAngles& e) { return QuatFromEuler(e.yaw, e.pitch, e.roll); }

	// Conversion: Euler angles --> Fused angles
	inline FusedAngles FusedFromEuler(double yaw) { return FusedAngles(yaw); }
	FusedAngles FusedFromEuler(double yaw, double pitch, double roll);
	inline FusedAngles FusedFromEuler(const EulerAngles& e) { return FusedFromEuler(e.yaw, e.pitch, e.roll); }

	// Conversion: Euler angles --> Tilt angles
	inline TiltAngles TiltFromEuler(double yaw) { return TiltAngles(yaw); }
	TiltAngles TiltFromEuler(double yaw, double pitch, double roll);
	inline TiltAngles TiltFromEuler(const EulerAngles& e) { return TiltFromEuler(e.yaw, e.pitch, e.roll); }

	// Conversion: Euler angles --> Z vector
	ZVec ZVecFromEuler(double pitch, double roll);
	inline ZVec ZVecFromEuler(const EulerAngles& e) { return ZVecFromEuler(e.pitch, e.roll); }

	// #######################################
	// #### Conversions from fused angles ####
	// #######################################

	// Conversion: Fused angles --> Rotation matrix
	inline Rotmat RotmatFromFused(double fusedYaw) { return RotmatFromAxis(Z_AXIS, fusedYaw); }
	Rotmat RotmatFromFused(double fusedPitch, double fusedRoll);
	Rotmat RotmatFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi = true);
	inline Rotmat RotmatFromFused(const FusedAngles& f) { return RotmatFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); }

	// Conversion: Fused angles --> Quaternion
	inline Quat QuatFromFused(double fusedYaw) { return QuatFromAxis(Z_AXIS, fusedYaw); }
	Quat QuatFromFused(double fusedPitch, double fusedRoll);
	Quat QuatFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi = true);
	inline Quat QuatFromFused(const FusedAngles& f) { return QuatFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); }

	// Conversion: Fused angles --> Euler angles
	inline EulerAngles EulerFromFused(double fusedYaw) { return EulerAngles(fusedYaw); }
	EulerAngles EulerFromFused(double fusedPitch, double fusedRoll);
	EulerAngles EulerFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi = true);
	inline EulerAngles EulerFromFused(const FusedAngles& f) { return EulerFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); }

	// Conversion: Fused angles --> Tilt angles
	inline TiltAngles TiltFromFused(double fusedYaw) { return TiltAngles(fusedYaw); }
	TiltAngles TiltFromFused(double fusedPitch, double fusedRoll);
	TiltAngles TiltFromFused(double fusedYaw, double fusedPitch, double fusedRoll, bool hemi = true);
	inline TiltAngles TiltFromFused(const FusedAngles& f) { return TiltFromFused(f.fusedYaw, f.fusedPitch, f.fusedRoll, f.hemi); }
	double TiltAngleFromFused(double fusedPitch, double fusedRoll);

	// Conversion: Fused angles --> Z vector
	ZVec ZVecFromFused(double fusedPitch, double fusedRoll, bool hemi = true);
	inline ZVec ZVecFromFused(const FusedAngles& f) { return ZVecFromFused(f.fusedPitch, f.fusedRoll, f.hemi); }

	// ######################################
	// #### Conversions from tilt angles ####
	// ######################################

	// Conversion: Tilt angles --> Rotation matrix
	inline Rotmat RotmatFromTilt(double fusedYaw) { return RotmatFromAxis(Z_AXIS, fusedYaw); }
	Rotmat RotmatFromTilt(double tiltAxisAngle, double tiltAngle);
	Rotmat RotmatFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle);
	inline Rotmat RotmatFromTilt(const TiltAngles& t) { return RotmatFromTilt(t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); }

	// Conversion: Tilt angles --> Quaternion
	inline Quat QuatFromTilt(double fusedYaw) { return QuatFromAxis(Z_AXIS, fusedYaw); }
	Quat QuatFromTilt(double tiltAxisAngle, double tiltAngle);
	Quat QuatFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle);
	inline Quat QuatFromTilt(const TiltAngles& t) { return QuatFromTilt(t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); }

	// Conversion: Tilt angles --> Euler angles
	inline EulerAngles EulerFromTilt(double fusedYaw) { return EulerAngles(fusedYaw); }
	EulerAngles EulerFromTilt(double tiltAxisAngle, double tiltAngle);
	EulerAngles EulerFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle);
	inline EulerAngles EulerFromTilt(const TiltAngles& t) { return EulerFromTilt(t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); }

	// Conversion: Tilt angles --> Fused angles
	inline FusedAngles FusedFromTilt(double fusedYaw) { return FusedAngles(fusedYaw); }
	FusedAngles FusedFromTilt(double tiltAxisAngle, double tiltAngle);
	FusedAngles FusedFromTilt(double fusedYaw, double tiltAxisAngle, double tiltAngle);
	inline FusedAngles FusedFromTilt(const TiltAngles& t) { return FusedFromTilt(t.fusedYaw, t.tiltAxisAngle, t.tiltAngle); }

	// Conversion: Tilt angles --> Z vector
	ZVec ZVecFromTilt(double tiltAxisAngle, double tiltAngle);
	inline ZVec ZVecFromTilt(const TiltAngles& t) { return ZVecFromTilt(t.tiltAxisAngle, t.tiltAngle); }

	// ####################################
	// #### Conversions from Z vectors ####
	// ####################################

	// Conversion: Z vector --> Rotation matrix (zero fused yaw)
	void RotmatFromZVec(const ZVec& z, Rotmat& R);
	inline Rotmat RotmatFromZVec(const ZVec& z) { Rotmat R; RotmatFromZVec(z, R); return R; }

	// Conversion: Z vector --> Quaternion (zero fused yaw)
	void QuatFromZVec(const ZVec& z, Quat& q);
	inline Quat QuatFromZVec(const ZVec& z) { Quat q; QuatFromZVec(z, q); return q; }

	// Conversion: Z vector --> Euler angles (zero Euler yaw)
	void EulerFromZVec(const ZVec& z, double& pitch, double& roll);
	inline void EulerFromZVec(const ZVec& z, EulerAngles& e) { EulerFromZVec(z, e.pitch, e.roll); e.yaw = 0.0; }
	inline EulerAngles EulerFromZVec(const ZVec& z) { EulerAngles e; EulerFromZVec(z, e.pitch, e.roll); e.yaw = 0.0; return e; }

	// Conversion: Z vector --> Fused angles (zero fused yaw)
	void FusedFromZVec(const ZVec& z, double& fusedPitch, double& fusedRoll);
	void FusedFromZVec(const ZVec& z, double& fusedPitch, double& fusedRoll, bool& hemi);
	inline void FusedFromZVec(const ZVec& z, FusedAngles& f) { FusedFromZVec(z, f.fusedPitch, f.fusedRoll, f.hemi); f.fusedYaw = 0.0; }
	inline FusedAngles FusedFromZVec(const ZVec& z) { FusedAngles f; FusedFromZVec(z, f.fusedPitch, f.fusedRoll, f.hemi); f.fusedYaw = 0.0; return f; }

	// Conversion: Z vector --> Tilt angles (zero fused yaw)
	void TiltFromZVec(const ZVec& z, double& tiltAxisAngle, double& tiltAngle);
	inline void TiltFromZVec(const ZVec& z, TiltAngles& t) { TiltFromZVec(z, t.tiltAxisAngle, t.tiltAngle); t.fusedYaw = 0.0; }
	inline TiltAngles TiltFromZVec(const ZVec& z) { TiltAngles t; TiltFromZVec(z, t.tiltAxisAngle, t.tiltAngle); t.fusedYaw = 0.0; return t; }

	// #########################################
	// #### Conversions from yaw and z-axis ####
	// #########################################

	// Conversion: Yaw and z-axis --> Rotation matrix
	void RotmatFromFYawBzG(double fusedYaw, const Vec3& BzG, Rotmat& RGB);
	void RotmatFromFYawGzB(double fusedYaw, const Vec3& GzB, Rotmat& RGB);
	inline Rotmat RotmatFromFYawBzG(double fusedYaw, const Vec3& BzG) { Rotmat RGB; RotmatFromFYawBzG(fusedYaw, BzG, RGB); return RGB; }
	inline Rotmat RotmatFromFYawGzB(double fusedYaw, const Vec3& GzB) { Rotmat RGB; RotmatFromFYawGzB(fusedYaw, GzB, RGB); return RGB; }

	// Conversion: Yaw and z-axis --> Quaternion
	void QuatFromFYawBzG(double fusedYaw, const Vec3& BzG, Quat& qGB);
	void QuatFromFYawGzB(double fusedYaw, const Vec3& GzB, Quat& qGB);
	inline Quat QuatFromFYawBzG(double fusedYaw, const Vec3& BzG) { Quat qGB; QuatFromFYawBzG(fusedYaw, BzG, qGB); return qGB; }
	inline Quat QuatFromFYawGzB(double fusedYaw, const Vec3& GzB) { Quat qGB; QuatFromFYawGzB(fusedYaw, GzB, qGB); return qGB; }

	// Conversion: Yaw and z-axis --> Euler angles
	void EulerFromFYawBzG(double fusedYaw, const Vec3& BzG, EulerAngles& eGB);
	void EulerFromFYawGzB(double fusedYaw, const Vec3& GzB, EulerAngles& eGB);
	inline EulerAngles EulerFromFYawBzG(double fusedYaw, const Vec3& BzG) { EulerAngles eGB; EulerFromFYawBzG(fusedYaw, BzG, eGB); return eGB; }
	inline EulerAngles EulerFromFYawGzB(double fusedYaw, const Vec3& GzB) { EulerAngles eGB; EulerFromFYawGzB(fusedYaw, GzB, eGB); return eGB; }

	// Conversion: Yaw and z-axis --> Fused angles
	void FusedFromFYawBzG(double fusedYaw, const Vec3& BzG, FusedAngles& fGB);
	void FusedFromFYawGzB(double fusedYaw, const Vec3& GzB, FusedAngles& fGB);
	inline FusedAngles FusedFromFYawBzG(double fusedYaw, const Vec3& BzG) { FusedAngles fGB; FusedFromFYawBzG(fusedYaw, BzG, fGB); return fGB; }
	inline FusedAngles FusedFromFYawGzB(double fusedYaw, const Vec3& GzB) { FusedAngles fGB; FusedFromFYawGzB(fusedYaw, GzB, fGB); return fGB; }

	// Conversion: Yaw and z-axis --> Tilt angles
	void TiltFromFYawBzG(double fusedYaw, const Vec3& BzG, TiltAngles& tGB);
	void TiltFromFYawGzB(double fusedYaw, const Vec3& GzB, TiltAngles& tGB);
	inline TiltAngles TiltFromFYawBzG(double fusedYaw, const Vec3& BzG) { TiltAngles tGB; TiltFromFYawBzG(fusedYaw, BzG, tGB); return tGB; }
	inline TiltAngles TiltFromFYawGzB(double fusedYaw, const Vec3& GzB) { TiltAngles tGB; TiltFromFYawGzB(fusedYaw, GzB, tGB); return tGB; }

	// ########################################
	// #### Spherical Linear Interpolation ####
	// ########################################

	// Slerp: Quaternion
	Quat QuatSlerp(const Quat& q0, const Quat& q1, double u);
	Quat QuatSlerp(const Quat& q, double u);

	// Slerp: Unit vector
	Vec3 VecSlerp(const Vec3& v0, const Vec3& v1, double u);

	// ##############################
	// #### Tilt phase functions ####
	// ##############################

	// Helper functions
	namespace internal
	{
		// Sum of tilt phases
		inline void TiltPhaseSumHelper(TiltPhase2D& pout) {}
		inline void TiltPhaseSumHelper(TiltPhase3D& pout) {}
		template<typename... Types> void TiltPhaseSumHelper(TiltPhase2D& pout, const TiltPhase2D& p, Types&&... args);
		template<typename... Types> void TiltPhaseSumHelper(TiltPhase3D& pout, const TiltPhase3D& p, Types&&... args);

		// Sum of tilt phase velocities
		inline void TiltPhaseVelSumHelper(TiltPhaseVel2D& pdotout) {}
		inline void TiltPhaseVelSumHelper(TiltPhaseVel3D& pdotout) {}
		template<typename... Types> void TiltPhaseVelSumHelper(TiltPhaseVel2D& pdotout, const TiltPhaseVel2D& pdot, Types&&... args);
		template<typename... Types> void TiltPhaseVelSumHelper(TiltPhaseVel3D& pdotout, const TiltPhaseVel3D& pdot, Types&&... args);
	}

	// Sum of tilt phases (alternative to operator+)
	template<typename... Types> TiltPhase2D TiltPhaseSum(const TiltPhase2D& p, Types&&... args);
	template<typename... Types> TiltPhase3D TiltPhaseSum(const TiltPhase3D& p, Types&&... args);

	// Mean of tilt phases
	template<typename... Types> TiltPhase2D TiltPhaseMean(const TiltPhase2D& p, Types&&... args);
	template<typename... Types> TiltPhase3D TiltPhaseMean(const TiltPhase3D& p, Types&&... args);

	// Sum of tilt phase velocities (alternative to operator+)
	template<typename... Types> TiltPhaseVel2D TiltPhaseVelSum(const TiltPhaseVel2D& pdot, Types&&... args);
	template<typename... Types> TiltPhaseVel3D TiltPhaseVelSum(const TiltPhaseVel3D& pdot, Types&&... args);

	// Conversion: Tilt phase velocity --> Angular velocity
	void AngVelFromTiltPhaseVel(const TiltPhaseVel2D& pdot, const TiltAngles& t, AngVel& angVel);
	void AngVelFromTiltPhaseVel(const TiltPhaseVel3D& pdot, const TiltAngles& t, AngVel& angVel);
	inline AngVel AngVelFromTiltPhaseVel(const TiltPhaseVel2D& pdot, const TiltAngles& t) { AngVel angVel; AngVelFromTiltPhaseVel(pdot, t, angVel); return angVel; }
	inline AngVel AngVelFromTiltPhaseVel(const TiltPhaseVel3D& pdot, const TiltAngles& t) { AngVel angVel; AngVelFromTiltPhaseVel(pdot, t, angVel); return angVel; }

	// #######################
	// #### Miscellaneous ####
	// #######################

	// Conversion: Split yaw and tilt --> Quaternion
	Quat QuatHFromFYawGTiltH(const Quat& qGH, double fusedYawG, const Quat& qH);
	inline Quat QuatHFromFYawGTiltH(const Quat& qGH, double fusedYawG, const TiltAngles& tH) { Quat qH = QuatFromTilt(tH.tiltAxisAngle, tH.tiltAngle); return QuatHFromFYawGTiltH(qGH, fusedYawG, qH); }
	inline Quat QuatHFromFYawGTiltH(const Quat& qGH, double fusedYawG, double tiltAxisAngleH, double tiltAngleH) { Quat qH = QuatFromTilt(tiltAxisAngleH, tiltAngleH); return QuatHFromFYawGTiltH(qGH, fusedYawG, qH); }
	inline Quat QuatGFromFYawGTiltH(const Quat& qGH, double fusedYawG, const Quat& qH) { return qGH * QuatHFromFYawGTiltH(qGH, fusedYawG, qH); }
	inline Quat QuatGFromFYawGTiltH(const Quat& qGH, double fusedYawG, const TiltAngles& tH) { Quat qH = QuatFromTilt(tH.tiltAxisAngle, tH.tiltAngle); return qGH * QuatHFromFYawGTiltH(qGH, fusedYawG, qH); }
	inline Quat QuatGFromFYawGTiltH(const Quat& qGH, double fusedYawG, double tiltAxisAngleH, double tiltAngleH) { Quat qH = QuatFromTilt(tiltAxisAngleH, tiltAngleH); return qGH * QuatHFromFYawGTiltH(qGH, fusedYawG, qH); }
}

// Include implementations that should occur in the header
#include <rot_conv/rot_conv_impl.h>

#endif
// EOF