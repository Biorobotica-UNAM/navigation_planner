#include <gtest/gtest.h>
#include "path_planner/motion_observation.hpp"

#include <cmath>
#include <chrono>
#include <limits>

using path_planner::MotionObservation;

TEST(MotionObservation, StationaryJitterAndIrregularSamples)
{
  MotionObservation motion;
  int64_t stamp = 1000000000;
  for (int i = 0; i < 40; ++i) {
    stamp += i % 2 ? 80000000 : 20000000;
    motion.observe(stamp, i % 2 ? 0.002 : -0.002, 0.0, 0.0);
  }
  EXPECT_TRUE(motion.stationary);
  EXPECT_LT(motion.linear_speed, 0.02);
  EXPECT_LE(motion.position_span, 0.0041);
}

TEST(MotionObservation, RepeatedStampCannotManufactureRest)
{
  MotionObservation motion;
  for (int i = 0; i < 30; ++i) {motion.observe(1000000000, 0.0, 0.0, 0.0);}
  EXPECT_FALSE(motion.stationary);
  EXPECT_EQ(motion.count, 1u);
  EXPECT_EQ(motion.span, 0.0);
}

TEST(MotionObservation, MeasuresSourceTimeInsteadOfControlTime)
{
  MotionObservation motion;
  for (int i = 0; i <= 8; ++i) {
    for (int repeat = 0; repeat < 4; ++repeat) {
      motion.observe(1000000000 + i * 80000000LL, i * 0.008, 0.0, 0.0);
    }
  }
  EXPECT_NEAR(motion.linear_speed, 0.1, 1e-9);
  EXPECT_FALSE(motion.stationary);
}

TEST(MotionObservation, RoundTripIsNotRestEvenWithZeroRegressionSlope)
{
  MotionObservation motion;
  for (int i = 0; i <= 10; ++i) {
    motion.observe(1000000000 + i * 30000000LL, 0.03 * std::sin(i * std::acos(-1.0) / 10), 0.0, 0.0);
  }
  EXPECT_NEAR(motion.linear_speed, 0.0, 1e-9);
  EXPECT_GT(motion.position_span, 0.02);
  EXPECT_FALSE(motion.stationary);
}

TEST(MotionObservation, YawWrapAndAngularMotion)
{
  MotionObservation motion;
  for (int i = 0; i <= 30; ++i) {
    const double yaw = std::remainder(3.13 + i * 0.006, 2.0 * std::acos(-1.0));
    motion.observe(1000000000 + i * 30000000LL, 0.0, 0.0, yaw);
  }
  EXPECT_NEAR(motion.angular_speed, 0.2, 1e-9);
  EXPECT_NEAR(motion.yaw_rate, 0.2, 0.001);
  EXPECT_FALSE(motion.stationary);
}

TEST(MotionObservation, ClockResetAndLongGapDiscardOldWindow)
{
  MotionObservation motion;
  for (int i = 0; i <= 20; ++i) {motion.observe(1000000000 + i * 30000000LL, 0.0, 0.0, 0.0);}
  ASSERT_TRUE(motion.stationary);
  motion.observe(1000000000, 0.0, 0.0, 0.0);
  EXPECT_FALSE(motion.stationary);
  EXPECT_EQ(motion.count, 1u);
  motion.observe(2000000000, 0.0, 0.0, 0.0);
  EXPECT_FALSE(motion.stationary);
  EXPECT_EQ(motion.count, 1u);
}

TEST(MotionObservation, RollingWindowCanSettleAfterMotion)
{
  MotionObservation motion;
  for (int i = 0; i < 1000; ++i) {
    motion.observe(1000000000 + i * 20000000LL, i < 900 ? i * 0.002 : 1.8, 0.0, 0.0);
  }
  EXPECT_TRUE(motion.stationary);
  EXPECT_LT(motion.count, motion.samples.size());
  EXPECT_LE(motion.span, 0.32);
}

TEST(MotionObservation, HighFrequencyDoesNotOverflowObservationWindow)
{
  MotionObservation motion;
  for (int i = 0; i <= 1000; ++i) {motion.observe(1000000000 + i * 1000000LL, 0.0, 0.0, 0.0);}
  EXPECT_TRUE(motion.stationary);
  EXPECT_LT(motion.count, motion.samples.size());
}

TEST(MotionObservation, AngularRoundTripAndInvalidDataCannotConfirmRest)
{
  MotionObservation motion;
  for (int i = 0; i <= 10; ++i) {
    motion.observe(1000000000 + i * 30000000LL, 0.0, 0.0, 0.05 * std::sin(i * std::acos(-1.0) / 10));
  }
  EXPECT_NEAR(motion.angular_speed, 0.0, 1e-9);
  EXPECT_FALSE(motion.stationary);
  motion.observe(1400000000, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  EXPECT_EQ(motion.count, 0u);
  EXPECT_FALSE(motion.stationary);
}

TEST(MotionObservation, BoundedUpdateCost)
{
  MotionObservation motion;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 100000; ++i) {
    motion.observe(1000000000 + i * 20000000LL, i % 2 ? 0.002 : -0.002, 0.0, 0.0);
  }
  RecordProperty("updates_100000_ms", std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count());
  EXPECT_TRUE(motion.stationary);
}
