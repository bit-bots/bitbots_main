// Rotations conversion library
// File: rot_conv_impl.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef ROT_CONV_IMPL_H
#define ROT_CONV_IMPL_H

// Includes
#include <rot_conv/rot_conv.h>

// Rotations conversion namespace
namespace rot_conv
{
	// ##############################
	// #### Rotation composition ####
	// ##############################

	// Helper functions
	namespace internal
	{
		// Compose: Rotation matrix
		template<typename... Types> void ComposeRotmatHelper(Rotmat& Rout, const Rotmat& R, Types&&... args)
		{
			// Recursively compose the rotations
			Rout *= R;
			ComposeRotmatHelper(Rout, std::forward<Types>(args)...);
		}

		// Compose: Quaternion
		template<typename... Types> void ComposeQuatHelper(Quat& qout, const Quat& q, Types&&... args)
		{
			// Recursively compose the rotations
			qout *= q;
			ComposeQuatHelper(qout, std::forward<Types>(args)...);
		}

		// Compose: Euler angles
		template<typename... Types> void ComposeEulerHelper(Rotmat& Rout, const EulerAngles& e, Types&&... args)
		{
			// Recursively compose the rotations
			Rout *= RotmatFromEuler(e);
			ComposeEulerHelper(Rout, std::forward<Types>(args)...);
		}

		// Compose: Fused angles
		template<typename... Types> void ComposeFusedHelper(Rotmat& Rout, const FusedAngles& f, Types&&... args)
		{
			// Recursively compose the rotations
			Rout *= RotmatFromFused(f);
			ComposeFusedHelper(Rout, std::forward<Types>(args)...);
		}

		// Compose: Tilt angles
		template<typename... Types> void ComposeTiltHelper(Rotmat& Rout, const TiltAngles& t, Types&&... args)
		{
			// Recursively compose the rotations
			Rout *= RotmatFromTilt(t);
			ComposeTiltHelper(Rout, std::forward<Types>(args)...);
		}
	}

	// Compose: Rotation matrix (alternative to operator*)
	template<typename... Types> Rotmat ComposeRotmat(const Rotmat& R, Types&&... args)
	{
		// Recursively compose the rotations
		Rotmat Rout = R;
		internal::ComposeRotmatHelper(Rout, std::forward<Types>(args)...);
		return Rout;
	}

	// Compose: Quaternion (alternative to operator*)
	template<typename... Types> Quat ComposeQuat(const Quat& q, Types&&... args)
	{
		// Recursively compose the rotations
		Quat qout = q;
		internal::ComposeQuatHelper(qout, std::forward<Types>(args)...);
		return qout;
	}

	// Compose: Euler angles
	template<typename... Types> EulerAngles ComposeEuler(const EulerAngles& e, Types&&... args)
	{
		// Recursively compose the rotations
		Rotmat Rout = RotmatFromEuler(e);
		internal::ComposeEulerHelper(Rout, std::forward<Types>(args)...);
		return EulerFromRotmat(Rout);
	}

	// Compose: Fused angles
	template<typename... Types> FusedAngles ComposeFused(const FusedAngles& f, Types&&... args)
	{
		// Recursively compose the rotations
		Rotmat Rout = RotmatFromFused(f);
		internal::ComposeFusedHelper(Rout, std::forward<Types>(args)...);
		return FusedFromRotmat(Rout);
	}

	// Compose: Tilt angles
	template<typename... Types> TiltAngles ComposeTilt(const TiltAngles& t, Types&&... args)
	{
		// Recursively compose the rotations
		Rotmat Rout = RotmatFromTilt(t);
		internal::ComposeTiltHelper(Rout, std::forward<Types>(args)...);
		return TiltFromRotmat(Rout);
	}

	// ##############################
	// #### Tilt phase functions ####
	// ##############################

	// Helper functions
	namespace internal
	{
		// Sum of tilt phases 2D
		template<typename... Types> void TiltPhaseSumHelper(TiltPhase2D& pout, const TiltPhase2D& p, Types&&... args)
		{
			// Recursively sum the tilt phases
			pout += p;
			TiltPhaseSumHelper(pout, std::forward<Types>(args)...);
		}

		// Sum of tilt phases 3D
		template<typename... Types> void TiltPhaseSumHelper(TiltPhase3D& pout, const TiltPhase3D& p, Types&&... args)
		{
			// Recursively sum the tilt phases
			pout += p;
			TiltPhaseSumHelper(pout, std::forward<Types>(args)...);
		}

		// Sum of tilt phase velocities 2D
		template<typename... Types> void TiltPhaseVelSumHelper(TiltPhaseVel2D& pdotout, const TiltPhaseVel2D& pdot, Types&&... args)
		{
			// Recursively sum the tilt phase velocities
			pdotout += pdot;
			TiltPhaseVelSumHelper(pdotout, std::forward<Types>(args)...);
		}

		// Sum of tilt phase velocities 3D
		template<typename... Types> void TiltPhaseVelSumHelper(TiltPhaseVel3D& pdotout, const TiltPhaseVel3D& pdot, Types&&... args)
		{
			// Recursively sum the tilt phase velocities
			pdotout += pdot;
			TiltPhaseVelSumHelper(pdotout, std::forward<Types>(args)...);
		}
	}

	// Sum of tilt phases 2D (alternative to operator+)
	template<typename... Types> TiltPhase2D TiltPhaseSum(const TiltPhase2D& p, Types&&... args)
	{
		// Recursively sum the tilt phases
		TiltPhase2D pout = p;
		internal::TiltPhaseSumHelper(pout, std::forward<Types>(args)...);
		return pout;
	}

	// Sum of tilt phases 3D (alternative to operator+)
	template<typename... Types> TiltPhase3D TiltPhaseSum(const TiltPhase3D& p, Types&&... args)
	{
		// Recursively sum the tilt phases
		TiltPhase3D pout = p;
		internal::TiltPhaseSumHelper(pout, std::forward<Types>(args)...);
		return pout;
	}

	// Mean of tilt phases 2D
	template<typename... Types> TiltPhase2D TiltPhaseMean(const TiltPhase2D& p, Types&&... args)
	{
		// Recursively calculate the mean of the tilt phases
		TiltPhase2D pout = p;
		internal::TiltPhaseSumHelper(pout, std::forward<Types>(args)...);
		pout /= sizeof...(args) + 1.0;
		return pout;
	}

	// Mean of tilt phases 3D
	template<typename... Types> TiltPhase3D TiltPhaseMean(const TiltPhase3D& p, Types&&... args)
	{
		// Recursively sum the tilt phases
		TiltPhase3D pout = p;
		internal::TiltPhaseSumHelper(pout, std::forward<Types>(args)...);
		pout /= sizeof...(args) + 1.0;
		return pout;
	}

	// Sum of tilt phase velocities 2D (alternative to operator+)
	template<typename... Types> TiltPhaseVel2D TiltPhaseVelSum(const TiltPhaseVel2D& pdot, Types&&... args)
	{
		// Recursively sum the tilt phase velocities
		TiltPhaseVel2D pdotout = pdot;
		internal::TiltPhaseVelSumHelper(pdotout, std::forward<Types>(args)...);
		return pdotout;
	}

	// Sum of tilt phase velocities 3D (alternative to operator+)
	template<typename... Types> TiltPhaseVel3D TiltPhaseVelSum(const TiltPhaseVel3D& pdot, Types&&... args)
	{
		// Recursively sum the tilt phase velocities
		TiltPhaseVel3D pdotout = pdot;
		internal::TiltPhaseVelSumHelper(pdotout, std::forward<Types>(args)...);
		return pdotout;
	}
}

#endif
// EOF