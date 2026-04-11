#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>

// Standalone reimplementation of the motor unit-conversion math so tests have
// no ROS / hardware dependency. Mirrors the logic in motor.cc exactly.

static constexpr float my_2pi = 6.28318530717f;

enum pos_vel_convert_type
{
    radian_2pi = 0,
    angle_360,
    turns,
};

static int16_t pos_float2int(float in, uint8_t type)
{
    switch (type)
    {
    case radian_2pi: return (int16_t)(in / my_2pi * 10000.0f);
    case angle_360:  return (int16_t)(in / 360.0f  * 10000.0f);
    case turns:      return (int16_t)(in * 10000.0f);
    default:         return 0;
    }
}

static float pos_int2float(int16_t in, uint8_t type)
{
    switch (type)
    {
    case radian_2pi: return (float)(in * my_2pi / 10000.0f);
    case angle_360:  return (float)(in * 360.0f  / 10000.0f);
    case turns:      return (float)(in / 10000.0f);
    default:         return 0.0f;
    }
}

static int16_t vel_float2int(float in, uint8_t type)
{
    switch (type)
    {
    case radian_2pi: return (int16_t)(in / my_2pi * 4000.0f);
    case angle_360:  return (int16_t)(in / 360.0f  * 4000.0f);
    case turns:      return (int16_t)(in * 4000.0f);
    default:         return 0;
    }
}

static float vel_int2float(int16_t in, uint8_t type)
{
    switch (type)
    {
    case radian_2pi: return (float)(in * my_2pi / 4000.0f);
    case angle_360:  return (float)(in * 360.0f  / 4000.0f);
    case turns:      return (float)(in / 4000.0f);
    default:         return 0.0f;
    }
}

// ---- Position round-trip tests ----

TEST(MotorConversion, PosRoundTripRadian)
{
    const float input = 1.5708f;  // ~π/2
    const int16_t encoded = pos_float2int(input, radian_2pi);
    const float decoded = pos_int2float(encoded, radian_2pi);
    EXPECT_NEAR(input, decoded, 1e-3f);
}

TEST(MotorConversion, PosRoundTripDegree)
{
    const float input = 90.0f;
    const int16_t encoded = pos_float2int(input, angle_360);
    const float decoded = pos_int2float(encoded, angle_360);
    EXPECT_NEAR(input, decoded, 1e-2f);
}

TEST(MotorConversion, PosRoundTripTurns)
{
    const float input = 0.5f;
    const int16_t encoded = pos_float2int(input, turns);
    const float decoded = pos_int2float(encoded, turns);
    EXPECT_NEAR(input, decoded, 1e-4f);
}

TEST(MotorConversion, PosZeroIsZero)
{
    EXPECT_EQ(0, pos_float2int(0.0f, radian_2pi));
    EXPECT_EQ(0, pos_float2int(0.0f, angle_360));
    EXPECT_EQ(0, pos_float2int(0.0f, turns));
}

TEST(MotorConversion, PosNegativeRadian)
{
    const float input = -1.5708f;
    const int16_t encoded = pos_float2int(input, radian_2pi);
    const float decoded = pos_int2float(encoded, radian_2pi);
    EXPECT_NEAR(input, decoded, 1e-3f);
    EXPECT_LT(encoded, 0);
}

// ---- Velocity round-trip tests ----

TEST(MotorConversion, VelRoundTripRadian)
{
    const float input = 3.14159f;
    const int16_t encoded = vel_float2int(input, radian_2pi);
    const float decoded = vel_int2float(encoded, radian_2pi);
    EXPECT_NEAR(input, decoded, 5e-3f);
}

TEST(MotorConversion, VelRoundTripDegree)
{
    const float input = 180.0f;
    const int16_t encoded = vel_float2int(input, angle_360);
    const float decoded = vel_int2float(encoded, angle_360);
    EXPECT_NEAR(input, decoded, 5e-2f);
}

TEST(MotorConversion, VelRoundTripTurns)
{
    const float input = 1.0f;
    const int16_t encoded = vel_float2int(input, turns);
    const float decoded = vel_int2float(encoded, turns);
    EXPECT_NEAR(input, decoded, 3e-4f);
}

TEST(MotorConversion, VelZeroIsZero)
{
    EXPECT_EQ(0, vel_float2int(0.0f, radian_2pi));
    EXPECT_EQ(0, vel_float2int(0.0f, angle_360));
    EXPECT_EQ(0, vel_float2int(0.0f, turns));
}

// ---- Cross-type consistency ----

TEST(MotorConversion, FullRotationEquivalence)
{
    // 2π rad == 360° == 1 turn all encode to the same int16
    const int16_t rad  = pos_float2int(my_2pi, radian_2pi);
    const int16_t deg  = pos_float2int(360.0f,  angle_360);
    const int16_t turn = pos_float2int(1.0f,    turns);
    EXPECT_EQ(rad,  (int16_t)10000);
    EXPECT_EQ(deg,  (int16_t)10000);
    EXPECT_EQ(turn, (int16_t)10000);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
