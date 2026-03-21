// Rotations conversion library
// File: rot_conv_extras.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <rot_conv/rot_conv_extras.h>

// Rotations conversion namespace
namespace rot_conv
{
	// ######################################
	// #### Absolute rotation velocities ####
	// ######################################

	// Conversion: Tilt phase velocity 2D --> Absolute tilt phase velocity 2D (assumes zero pzVel)
	void AbsTiltPhaseVelFromTiltPhaseVel(const TiltPhaseVel2D& pdot, double fusedYaw, AbsTiltPhaseVel2D& apdot)
	{
		// Precalculate trigonometric values
		double cpsi = cos(fusedYaw);
		double spsi = sin(fusedYaw);

		// Perform the required conversion
		apdot.pxVel = cpsi*pdot.pxVel - spsi*pdot.pyVel;
		apdot.pyVel = cpsi*pdot.pyVel + spsi*pdot.pxVel;
	}

	// Conversion: Tilt phase velocity 3D --> Absolute tilt phase velocity 3D
	void AbsTiltPhaseVelFromTiltPhaseVel(const TiltPhaseVel3D& pdot, const TiltAngles& t, AbsTiltPhaseVel3D& apdot)
	{
		// Precalculate trigonometric values
		double cpsi = cos(t.fusedYaw);
		double spsi = sin(t.fusedYaw);
		double psigam = t.tiltAxisAngle + t.fusedYaw;
		double cpsigam = cos(psigam);
		double spsigam = sin(psigam);

		// Perform the required conversion
		apdot.pxVel = cpsi*pdot.pxVel - spsi*pdot.pyVel - spsigam*t.tiltAngle*pdot.pzVel;
		apdot.pyVel = cpsi*pdot.pyVel + spsi*pdot.pxVel + cpsigam*t.tiltAngle*pdot.pzVel;
		apdot.pzVel = pdot.pzVel;
	}

	// Conversion: Absolute tilt phase velocity 2D --> Tilt phase velocity 2D (assumes zero pzVel)
	void TiltPhaseVelFromAbsTiltPhaseVel(const AbsTiltPhaseVel2D& apdot, double fusedYaw, TiltPhaseVel2D& pdot)
	{
		// Precalculate trigonometric values
		double cpsi = cos(fusedYaw);
		double spsi = sin(fusedYaw);

		// Perform the required conversion
		pdot.pxVel = cpsi*apdot.pxVel + spsi*apdot.pyVel;
		pdot.pyVel = cpsi*apdot.pyVel - spsi*apdot.pxVel;
	}

	// Conversion: Absolute tilt phase velocity 3D --> Tilt phase velocity 3D
	void TiltPhaseVelFromAbsTiltPhaseVel(const AbsTiltPhaseVel3D& apdot, const TiltAngles& t, TiltPhaseVel3D& pdot)
	{
		// Precalculate trigonometric values
		double cpsi = cos(t.fusedYaw);
		double spsi = sin(t.fusedYaw);
		double cgam = cos(t.tiltAxisAngle);
		double sgam = sin(t.tiltAxisAngle);

		// Perform the required conversion
		pdot.pxVel = cpsi*apdot.pxVel + spsi*apdot.pyVel + sgam*t.tiltAngle*apdot.pzVel;
		pdot.pyVel = cpsi*apdot.pyVel - spsi*apdot.pxVel - cgam*t.tiltAngle*apdot.pzVel;
		pdot.pzVel = apdot.pzVel;
	}

	// Conversion: Absolute tilt phase velocity 3D --> Angular velocity
	void AngVelFromAbsTiltPhaseVel(const AbsTiltPhaseVel3D& apdot, double absTiltAxisAngle, double tiltAngle, AngVel& angVel)
	{
		// Precalculate trigonometric values
		double cpsigam = cos(absTiltAxisAngle);
		double spsigam = sin(absTiltAxisAngle);

		// Precalculate additional terms
		double S, C;
		if(tiltAngle == 0.0)
		{
			S = 1.0;
			C = 0.0;
		}
		else
		{
			S = sin(tiltAngle) / tiltAngle;
			C = (1.0 - cos(tiltAngle)) / tiltAngle;
		}

		// Calculate the tilt velocity parameters
		double alphadot = cpsigam*apdot.pxVel + spsigam*apdot.pyVel;
		double agammadot = cpsigam*apdot.pyVel - spsigam*apdot.pxVel - tiltAngle*apdot.pzVel; // This is alpha*gammadot
		double psidot = apdot.pzVel;

		// Calculate the required angular velocity
		angVel.x() = cpsigam*alphadot - S*agammadot*spsigam;
		angVel.y() = spsigam*alphadot + S*agammadot*cpsigam;
		angVel.z() = psidot + C*agammadot;
	}
}