/** @file KalmanFilter.hpp
	@brief KalmanFilter classes using the Eigen math template library,

	https://github.com/slightperturbation/kalman
*/


#pragma once
#ifndef INCLUDED_KalmanFilter_h_GUID_75AD5E97_70AC_4563_A322_C2A627FD07EB
#define INCLUDED_KalmanFilter_h_GUID_75AD5E97_70AC_4563_A322_C2A627FD07EB

#include <Eigen/Core>
#include <Eigen/LU>

#include <stdexcept>
namespace Util {
namespace Kalman {


/// KalmanFilter tracks the discrete movement of a single point in state_dim_t dimensions,
/// as it moves according to a control signal of control_dim_t dimensions and with noisy
/// input of measurement_dim_t dimensions.
/// Usage is very simple. Create an instance by providing the full system model in the constructor.
/// Each timestep, available measurements and the known control signal are combined to update
/// the state of the system in the update() method.
/// The output of the system is available either through currentState() or predictNext().
template< typename scalar, size_t state_dim_t, size_t measurement_dim_t, size_t control_dim_t>
class KalmanFilter
{
public:
    typedef Eigen::Matrix< scalar, state_dim_t, 1 > state_vector_t;
    typedef Eigen::Matrix< scalar, measurement_dim_t, 1 > measurement_vector_t;
    typedef Eigen::Matrix< scalar, control_dim_t, 1 > control_vector_t;
private:
    // old state to new state
    Eigen::Matrix< scalar, state_dim_t, state_dim_t> m_stateTransitionMatrix;
    Eigen::Matrix< scalar, state_dim_t, state_dim_t>& A = m_stateTransitionMatrix;
    // control vector to state vector
    Eigen::Matrix< scalar, state_dim_t, control_dim_t> m_controlMatrix;
    Eigen::Matrix< scalar, state_dim_t, control_dim_t>& B = m_controlMatrix;
    // state vector to measurement vector
    Eigen::Matrix< scalar, measurement_dim_t, state_dim_t> m_observationMatrix;
    Eigen::Matrix< scalar, measurement_dim_t, state_dim_t>& H = m_observationMatrix;
    /// Estimated process noise covariance, how much noise is expected when updating
    /// the state to the next time step.
    Eigen::Matrix< scalar, state_dim_t, state_dim_t> m_processNoiseCovariance;
    Eigen::Matrix< scalar, state_dim_t, state_dim_t>& Q = m_processNoiseCovariance;
    /// Estimated measurement noise covariance, how much noise is expected in the measurements.
    Eigen::Matrix< scalar, measurement_dim_t, measurement_dim_t> m_measurementNoiseCovariance;
    Eigen::Matrix< scalar, measurement_dim_t, measurement_dim_t>& R = m_measurementNoiseCovariance;
    //////////////////////////////
    // State variables
    state_vector_t m_stateEstimate;
    state_vector_t& x = m_stateEstimate;
    Eigen::Matrix< scalar, state_dim_t, state_dim_t > m_predictedCovarianceEstimate;
    Eigen::Matrix< scalar, state_dim_t, state_dim_t >& P = m_predictedCovarianceEstimate;
    /// y stores the last measurement error from the previous update for use by predictors.
    Eigen::Matrix< scalar, measurement_dim_t, 1 > y;
public:
    /// Create a KF to track a state using an online set of measurements.
    ///
    /// ProcessNoiseCovariance (Q) indicates how much uncertainty to expect in the state transitions.
    /// If an element of Q is large then that state element will be tracked more closely. If Q is small
    /// the state will be tracked loosely. Think of Q as the gain in a PD controller.
    ///
    /// MeasurementNoiseCovariance (R) is how much noise is expected in the measurement input. If R is
    /// large, the measurements are considered to be inaccurate, smaller R indicates the measurements
    /// are likely correct. Think of R as the damping in a PD controller.
    ///
    /// Note that R and Q are covariance matrices, so if state/measurements are independent, then they
    /// are diagonal.
    KalmanFilter( const Eigen::Matrix< scalar, state_dim_t, state_dim_t >& argStateTransitionMatrix
    , const Eigen::Matrix< scalar, state_dim_t, control_dim_t >& argControlMatrix
    , const Eigen::Matrix< scalar, measurement_dim_t, state_dim_t >& argObservationMatrix
    , const state_vector_t& argInitialState
    , const Eigen::Matrix< scalar, state_dim_t, state_dim_t >& argInitialCovarianceEstimate
    , const Eigen::Matrix< scalar, state_dim_t, state_dim_t>& argProcessNoiseCovariance
    , const Eigen::Matrix< scalar, measurement_dim_t, measurement_dim_t>& argMeasurementNoiseCovariance )
    {
        A = argStateTransitionMatrix;
        B = argControlMatrix;
        H = argObservationMatrix;
        x = argInitialState;
        P = argInitialCovarianceEstimate;
        Q = argProcessNoiseCovariance;
        R = argMeasurementNoiseCovariance;
    }
    Eigen::Matrix< scalar, state_dim_t, 1 > getCurrentState()
    {
        return x;
    }
    void update( const control_vector_t& control
    , const measurement_vector_t& measurement )
    {
        ///////////
        // Prediction
        // 1) Project current state (x) into the future using state transition and control.
        state_vector_t x_p = A * x + B * control;
        // 2) Project the covariance into the future using state transition and estimated error.
        Eigen::Matrix< scalar, state_dim_t, state_dim_t > P_p = A * P * A.transpose() + Q;
        // Correction
        // 1) Compute the Kalman gain
        // 1a) Estimate the "innovation" covariance from projected covariance plus the measurement noise R.
        Eigen::Matrix< scalar, measurement_dim_t, measurement_dim_t > S = H * P_p * H.transpose() + R;
        // 1b) Final Kalmna gain, K = PH^T(HPH^T+R)^-1
        Eigen::Matrix< scalar, state_dim_t, measurement_dim_t > K = P_p * H.transpose() * S.inverse();
        // 2) Update our state estimate
        // 2a) Compare measurement to what our prediction should have measured
        const measurement_vector_t& z = measurement;
        Eigen::Matrix< scalar, measurement_dim_t, 1 > y = z - H * x_p;
        // 2b) Update using Kalman gain times error
        x = x_p + K * y;
        // 3) Update our estimate of the noise covariance
        P = ( Eigen::Matrix< scalar, state_dim_t, state_dim_t >::Identity() - K * H ) * P_p;
    }
    // Provide a prediction without a corresponding measurement
    Eigen::Matrix< scalar, state_dim_t, 1 > predictNext() const
    {
        // Project the covariance into the future using state transition and estimated error.
        Eigen::Matrix< scalar, state_dim_t, state_dim_t > P_p = A * P * A.transpose() + Q;
        // Estimate the "innovation" covariance from projected covariance plus the measurement noise R.
        Eigen::Matrix< scalar, measurement_dim_t, measurement_dim_t > S = H * P_p * H.transpose() + R;
        // Final Kalmna gain, K = PH^T(HPH^T+R)^-1
        Eigen::Matrix< scalar, state_dim_t, measurement_dim_t > K = P_p * H.transpose() * S.inverse();
        state_vector_t out = A * x + K * y;
    }
};


} // end of namespace Kalman

#endif // INCLUDED_KalmanFilter_h_GUID_75AD5E97_70AC_4563_A322_C2A627FD07EB

/// Neue Coolere Implementation gefunden die Vollständiger und einfacher ist

/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

Contact information
-------------------

Kristian Lauszus, TKJ Electronics
Web : http://www.tkjelectronics.com
e-mail : kristianl@tkjelectronics.com
*/

#ifndef _Kalman_h
#define _Kalman_h

class SimpleKalman {
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SimpleKalman() {
        /* We will set the variables like so, these can also be tuned by the user */
        Q_angle = 0.001;
        Q_bias = 0.003;
        R_measure = 0.03;

        angle = 0; // Reset the angle
        bias = 0; // Reset bias

        P = Eigen::Matrix2d::Zero(); // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    };

    SimpleKalman(double Q_angle, double Q_bias, double R_measure)
    :Q_angle(Q_angle), Q_bias(Q_bias), R_measure(R_measure)
    {
        angle = 0; // Reset the angle
        bias = 0; // Reset bias

        P = Eigen::Matrix2d::Zero(); // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    };
    double getAngle(double newRate, double dt) {
        #if 1
        return getAngle(angle + (newRate * dt), newRate, dt);
        #else
        return getAngle(angle, newRate, dt);
        #endif
    }
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    double getAngle(double newAngle, double newRate, double dt) {
        // KasBot V2 - Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
        rate = newRate - bias;
        angle += dt * rate;

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        P(0,0) += dt * (dt*P(1,1) - P(0,1) - P(1,0) + Q_angle);
        P(0,1) -= dt * P(1,1);
        P(1,0) -= dt * P(1,1);
        P(1,1) += Q_bias * dt;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        S = P(0,0) + R_measure;
        /* Step 5 */
        K(0) = P(0,0) / S;
        K(1) = P(1,0) / S;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        y = newAngle - angle;
        /* Step 6 */
        angle += K(0) * y;
        bias += K(1) * y;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        P(0,0) -= K(0) * P(0,0);
        P(0,1) -= K(0) * P(0,1);
        P(1,0) -= K(1) * P(0,0);
        P(1,1) -= K(1) * P(0,1);

        return angle;
    };
    void setAngle(double newAngle) { angle = newAngle; }; // Used to set angle, this should be set as the starting angle
    double getRate() { return rate; }; // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void setQangle(double newQ_angle) { Q_angle = newQ_angle; };
    void setQbias(double newQ_bias) { Q_bias = newQ_bias; };
    void setRmeasure(double newR_measure) { R_measure = newR_measure; };

    double getQangle() { return Q_angle; };
    double getQbias() { return Q_bias; };
    double getRmeasure() { return R_measure; };

private:
    /* Kalman filter variables */
    double Q_angle; // Process noise variance for the accelerometer
    double Q_bias; // Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    double angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    Eigen::Matrix2d P; // Error covariance matrix - This is a 2x2 matrix
    Eigen::Vector2d K; // Kalman gain - This is a 2x1 vector
    double y; // Angle difference
    double S; // Estimate error

    typedef Kalman::KalmanFilter<double, 2, 1, 1 > EigenKalman;
    EigenKalman* kalman;
};

} // namespace Util
#endif
