#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>

#include "../eigen_util.hpp"
#include "../KalmanFilter.hpp"

namespace Test {

using namespace Eigen;
using namespace Util;
using namespace Kalman;

TEST( KalmanFilterTest, Stationary1D )
{
    using namespace Eigen;
    // A 1d position-only KF to track a fixed position
    Matrix< double, 1, 1 > A;
    A << 1;
    Matrix< double, 1, 1 > B;
    B << 0;
    Matrix< double, 1, 1 > H;
    H << 1;
    Matrix< double, 1, 1 > initialGuess;
    initialGuess << 10;
    Matrix< double, 1, 1 > initialCovariance;
    initialCovariance << 1;
    Matrix< double, 1, 1 > Q;
    Q << 0.1;
    Matrix< double, 1, 1 > R;
    R << 0.1;
    Kalman::KalmanFilter< double, 1, 1, 1 > filter( A, B, H, initialGuess, initialCovariance, Q, R );
    Matrix< double, 1, 1 > u;
    u << 0; // no control
    Matrix< double, 1, 1 > z;
    z << 0; // measuring zero
    for( size_t i=0; i<10; ++i )
    {
        filter.update( u, z );
    }
    EXPECT_NEAR( filter.getCurrentState()(0,0), z(0,0), 0.001 );
    SUCCEED();
}
/// Pos/Vel 1D Tests
const int state_dims = 2;
const int control_dims = 1;
const int measurement_dims = 2;
double runKalmanFilterTest_PositionVelocity1D( const Eigen::Matrix< double, state_dims, state_dims >& Q
, const Eigen::Matrix< double, measurement_dims, measurement_dims >& R
, bool shouldSaveToFile );
TEST( KalmanFilterTest, PositionVelocity1D )
{
    using namespace Eigen;
    // try cross-product of ranges of Q and R diag matrices.
    // remember and report the best
    double bestQ = 0;
    double bestR = 0;
    double leastMeanSquaredError = std::numeric_limits<double>::infinity();
    for( double q = 0.01; q < 1; q += 0.01 )
    {
        for( double r = 0.01; r < 1; r += 0.01 )
        {
            // stiffness
            Matrix< double, state_dims, state_dims > Q;
            Q << q, 0
            , 0, q ;
            // damping
            Matrix< double, measurement_dims, measurement_dims > R;
            R << r, 0.0
            , 0.0, r;
            double error = runKalmanFilterTest_PositionVelocity1D( Q, R, false );
            if( error < leastMeanSquaredError )
            {
                std::cout << "New best: q=" << q << ", r=" << r << ", err= " << error << "\n";
                leastMeanSquaredError = error;
                bestQ = q;
                bestR = r;
            }
        }
    }
    std::cout << "Final best: q=" << bestQ << ", r=" << bestR << ", err=" << leastMeanSquaredError << "\n";
    // stiffness
    Matrix< double, state_dims, state_dims > Q;
    Q << bestQ, 0 // 0.025
    , 0, bestQ ;
    // damping
    Matrix< double, measurement_dims, measurement_dims > R;
    R << bestR, 0 //0.5
    , 0, bestR;
    runKalmanFilterTest_PositionVelocity1D( Q, R, false );
    SUCCEED();
}
double runKalmanFilterTest_PositionVelocity1D( const Eigen::Matrix< double, state_dims, state_dims >& Q
, const Eigen::Matrix< double, measurement_dims, measurement_dims >& R
, bool shouldSaveToFile )
{
    using namespace Eigen;
    const double dt = 0.1; // timestep
    // State Transition
    Matrix< double, state_dims, state_dims > A;
    A << 1, dt
    , 0, 1;
    // Control
    Matrix< double, state_dims, control_dims > B;
    B << 0
    , 0;
    // Observation
    Matrix< double, measurement_dims, state_dims > H;
    H << 1, 0
    , 0, 1;
    Matrix< double, state_dims, 1 > initialGuess;
    initialGuess << 10
    , 0;
    Matrix< double, state_dims, state_dims > initialCovariance;
    initialCovariance << 10, 0
    , 0 , 10;
    Kalman::KalmanFilter< double, state_dims, measurement_dims, control_dims > filter( A, B, H, initialGuess, initialCovariance, Q, R );
    Matrix< double, control_dims, 1 > u;
    u << 0; // no control
    Matrix< double, measurement_dims, 1 > z;
    z << 0, 0; // measuring zero
    double t = 0;
    std::random_device rd;
    std::default_random_engine e1(rd());
    std::normal_distribution<double> posUncertainty(0, 1);
    std::normal_distribution<double> velUncertainty(0, 1);
    std::stringstream ss;
    ss << "/Users/vector/Desktop/tmp/KF_PV1D_" << Q(0,0) << "_" << R(0,0) << ".csv";
    std::ofstream fileOut;
    if( shouldSaveToFile )
    {
        fileOut.open( ss.str().c_str(), std::ios_base::trunc );
    }
    double totalSquaredError = 0;
    const size_t stepCount = 1000;
    for( size_t i=0; i<stepCount; ++i )
    {
        z << std::sin( t ) + posUncertainty(e1)
        , std::cos( t ) + velUncertainty(e1);
        filter.update( u, z );
        double errPos = filter.getCurrentState()(0,0) - std::sin(t);
        double errVel = filter.getCurrentState()(1,0) - std::cos(t);
        totalSquaredError += errPos*errPos + errVel*errVel;
        if( shouldSaveToFile )
        {
            fileOut << t << ", " << std::sin( t ) << ", " << filter.getCurrentState()(0,0) << ", " << z(0,0) << "\n";
        }
        t += dt;
    }
    //EXPECT_NEAR( filter.getCurrentState()(0,0), std::sin(t-dt), 0.1 ); // pos
    return totalSquaredError / stepCount;
}

} //namespace Test

using namespace Test;

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
