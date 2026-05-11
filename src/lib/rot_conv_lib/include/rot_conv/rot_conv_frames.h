// Rotations conversion library
// File: rot_conv_frames.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ROT_CONV_FRAMES_H
#define ROT_CONV_FRAMES_H

// Includes
#include <rot_conv/rot_conv.h>

// Rotations conversion namespace
namespace rot_conv
{
	// #####################
	// #### Frame types ####
	// #####################

	// Typedefs
	typedef Eigen::Matrix4d Transform;

	// Quaternion frame struct
	struct QuatFrame
	{
		// Constants
		static inline QuatFrame Identity() { return QuatFrame(Vec3::Zero(), Quat::Identity()); }

		// Constructors
		QuatFrame() = default;
		QuatFrame(const Vec3& pos, const Quat& rot) : pos(pos), rot(rot) {}

		// Set functions
		void set(const Vec3& pos, const Quat& rot) { this->pos = pos; this->rot = rot; }
		void setIdentity() { pos.setZero(); rot.setIdentity(); }

		// Data members
		Vec3 pos;
		Quat rot;
	};

	// Rotation matrix frame struct
	struct RotmatFrame
	{
		// Constants
		static inline RotmatFrame Identity() { return RotmatFrame(Vec3::Zero(), Rotmat::Identity()); }

		// Constructors
		RotmatFrame() = default;
		RotmatFrame(const Vec3& pos, const Rotmat& rot) : pos(pos), rot(rot) {}

		// Set functions
		void set(const Vec3& pos, const Rotmat& rot) { this->pos = pos; this->rot = rot; }
		void setIdentity() { pos.setZero(); rot.setIdentity(); }

		// Data members
		Vec3 pos;
		Rotmat rot;
	};

	// Default frame type
	typedef QuatFrame Frame;
}

#endif
// EOF