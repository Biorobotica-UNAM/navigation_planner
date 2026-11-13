#include <simple_move/tracking_reference.hpp>
#include <gtest/gtest.h>

TEST(DirectionReference, AlternatingLocalizationJitterDoesNotReverseTheReference)
{
  simple_move::DirectionReference reference;
  reference.reset(0.0);
  double largest_excursion = 0.0;
  for (int i = 0; i < 200; ++i) {
    const double angle = reference.update(i % 2 ? -0.48 : 0.48, 0.03, 0.15, 1.2);
    largest_excursion = std::max(largest_excursion, std::abs(angle));
    EXPECT_LE(std::abs(reference.rate), 1.2);
  }
  EXPECT_LT(largest_excursion, 0.04);
}

TEST(DirectionReference, TracksSustainedTurnsAndTakesTheShortArcAcrossPi)
{
  simple_move::DirectionReference reference;
  reference.reset(3.13);
  EXPECT_GT(reference.update(-3.13, 0.03, 0.15, 1.2), 3.13);
  reference.reset(0.0);
  for (int i = 0; i < 200; ++i) {reference.update(1.0, 0.03, 0.15, 1.2);}
  EXPECT_NEAR(reference.angle, 1.0, 1e-6);
}

TEST(HeadTracking, SmallMeasuredErrorsRetainMotionDespiteChangingTargets)
{
  // Errors from the supplied trace, including the 0.06 rad one-cycle jump
  // which previously predicted 0.56 rad and cancelled translation.
  for (double error : {0.025, 0.029, 0.06, 0.118}) {
    EXPECT_DOUBLE_EQ(simple_move::head_tracking_scale(error, 0.2, 0.55, 0.0, 1.5), 1.0);
    EXPECT_GT(simple_move::head_tracking_scale(error, 0.2, 0.55, 2.5, 1.5), 0.4);
  }
}

TEST(HeadTracking, ActualLagSlowsContinuouslyAndStopsAtTheLimit)
{
  EXPECT_NEAR(simple_move::head_tracking_scale(0.375, 0.2, 0.55, 0.0, 1.5), 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(simple_move::head_tracking_scale(0.55, 0.2, 0.55, 0.0, 1.5), 0.0);
  EXPECT_DOUBLE_EQ(simple_move::head_tracking_scale(-0.6, 0.2, 0.55, 0.0, 1.5), 0.0);
}
