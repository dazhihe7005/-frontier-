#include "mine_uav_control/motion_continuity.hpp"
#include "mine_uav_control/tracking_speed_governor.hpp"
#include <gtest/gtest.h>
using namespace mine_uav_control;
static const char* check(const MotionSample& a,const MotionSample& b,double dt=.02) { return motionTransitionFailure(a,b,dt,1.1,1.75,6.0,.75); }
TEST(MotionContinuity, SmoothCubicPasses) {
  MotionSample a,b;
  for(int k=1;k<=100;++k) { double t=k*.02;b.p[0]=.02*t*t*t;b.v[0]=.06*t*t;b.a[0]=.12*t;EXPECT_EQ(nullptr,check(a,b));a=b; }
}
TEST(MotionContinuity, ObservedTwentySevenCentimetreJumpRejected) { MotionSample a,b;b.p[0]=.27;EXPECT_NE(nullptr,check(a,b)); }
TEST(MotionContinuity, ShortRosStampJitterDoesNotBecomePositionFault) {
  EXPECT_TRUE(sourcePositionStepAllowed(.0075,.002,2.2));
  EXPECT_FALSE(sourcePositionStepAllowed(.27,.002,2.2));
}
TEST(MotionContinuity, VelocityStepRejected) { MotionSample a,b;b.v[0]=.8;EXPECT_NE(nullptr,check(a,b)); }
TEST(MotionContinuity, AccelerationStepRejected) { MotionSample a,b;b.a[0]=1.;EXPECT_NE(nullptr,check(a,b)); }
TEST(MotionContinuity, StationaryHoldDoesNotMoveReference) { MotionSample a;a.p[0]=5;EXPECT_EQ(nullptr,check(a,a)); }
TEST(MotionContinuity, ExactStationaryHoldMayResumeAfterBoundedStreamGap) {
  MotionSample hold;
  hold.p = {{0.093, -0.033, 0.663}};
  hold.yaw = -1.138;
  EXPECT_EQ(nullptr, check(hold, hold, .46));
  MotionSample changed = hold;
  changed.p[0] += .01;
  EXPECT_STREQ("TIME_DISCONTINUITY", check(hold, changed, .46));
  MotionSample moving = hold;
  moving.v[0] = .1;
  EXPECT_STREQ("TIME_DISCONTINUITY", check(moving, moving, .46));
}
TEST(MotionContinuity, MovingHoldCannotZeroVelocityInstantly) { MotionSample a,b;a.v[0]=.8;EXPECT_NE(nullptr,check(a,b)); }
TEST(MotionContinuity, GovernedFeedforwardMayDifferFromPositionDerivative) {
  // id307 faulted on this safe shape: the position reference remained smooth,
  // while the intentional speed governor reduced feed-forward velocity.  All
  // independent position/velocity/acceleration/yaw transition limits still
  // apply; only the source-trajectory PVA identity is inapplicable afterward.
  MotionSample before, after;
  before.v[0] = 0.34;
  after.p[0] = 0.022;
  after.v[0] = 0.3406;
  EXPECT_STREQ("INCONSISTENT_PVA",
      motionTransitionFailure(before, after, .02, 1.1, 1.75, 6.0, .75));
  EXPECT_EQ(nullptr,
      motionTransitionFailure(before, after, .02, 1.1, 1.75, 6.0, .75,
                              false));
}
TEST(MotionContinuity, YawWrapIsNotAnArtificialTurn) { MotionSample a,b;a.yaw=M_PI-.002;b.yaw=-M_PI+.002;EXPECT_EQ(nullptr,check(a,b)); }
TEST(MotionContinuity, NinetyDegreeYawStepRejected) { MotionSample a,b;b.yaw=M_PI_2;EXPECT_NE(nullptr,check(a,b)); }
TEST(MotionContinuity, BackwardAndDuplicateTimeRejected) { MotionSample a;EXPECT_NE(nullptr,check(a,a,-.1));EXPECT_NE(nullptr,check(a,a,0)); }
TEST(MotionContinuity, FutureAndStaleMessagesNotRelabelledFresh) { EXPECT_FALSE(sourceStampFresh(10,11,2));EXPECT_FALSE(sourceStampFresh(10,7,2));EXPECT_TRUE(sourceStampFresh(10,9.9,2)); }

TEST(MotionContinuity, ManagedHoldAloneMayUseSmoothReacquisitionEnvelope) {
  EXPECT_TRUE(managedHoldReacquireAllowed(
      true, true, true, true, 0.88, 1.5, 2.05, 2.6));
  EXPECT_FALSE(managedHoldReacquireAllowed(
      false, true, true, true, 0.88, 1.5, 2.05, 2.6));
  EXPECT_FALSE(managedHoldReacquireAllowed(
      true, true, true, true, 1.51, 1.5, 2.05, 2.6));
  EXPECT_FALSE(managedHoldReacquireAllowed(
      true, true, true, true, 0.88, 1.5, 2.61, 2.6));
}

TEST(MotionContinuity, ExecutionClockStopsForEveryManagedHold) {
  EXPECT_TRUE(trajectoryExecutionAuthorized(
      true, false, true, true, false, false, false));
  EXPECT_FALSE(trajectoryExecutionAuthorized(
      true, false, true, true, true, false, false));
  EXPECT_FALSE(trajectoryExecutionAuthorized(
      true, false, true, true, false, true, false));
  EXPECT_FALSE(trajectoryExecutionAuthorized(
      true, false, true, true, false, false, true));
}

TEST(MotionContinuity, ExecutionClockRequiresHealthyOwnedOffboard) {
  EXPECT_FALSE(trajectoryExecutionAuthorized(
      false, false, true, true, false, false, false));
  EXPECT_FALSE(trajectoryExecutionAuthorized(
      true, true, true, true, false, false, false));
  EXPECT_FALSE(trajectoryExecutionAuthorized(
      true, false, false, true, false, false, false));
  EXPECT_FALSE(trajectoryExecutionAuthorized(
      true, false, true, false, false, false, false));
}

TEST(MotionContinuity, ReacquireDurationRespectsLargeReturnYaw) {
  const double duration = smoothReacquireDuration(3.5, 0.88, 2.05, 1.1, 0.75);
  EXPECT_GT(duration, 5.0);
  EXPECT_LE(1.875 * 2.05 / duration, 0.90 * 0.75 + 1e-12);
}

TEST(TrackingSpeedGovernor, LeavesRequestBelowLimitUnchanged) {
  const auto result = governTrackingSpeed(
      {{0.2, 0.0, 0.0}}, {{0.1, 0.0, 0.0}}, {{0.9, 0.9, 1.0}}, 0.4);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.active);
  EXPECT_NEAR(result.feedforward_velocity[0], 0.2, 1e-12);
  EXPECT_NEAR(result.closed_loop_velocity[0], 0.29, 1e-12);
}

TEST(TrackingSpeedGovernor, AlongTrackLagIsNotCrossTrackDeviation) {
  const auto error = decomposeTrackingError(
      {{0.655, 0.0, 0.0}}, {{0.9, 0.0, 0.0}});
  ASSERT_TRUE(error.valid);
  EXPECT_TRUE(error.tangent_available);
  EXPECT_NEAR(error.total, 0.655, 1e-12);
  EXPECT_NEAR(error.along_track_lag, 0.655, 1e-12);
  EXPECT_NEAR(error.non_lag_deviation, 0.0, 1e-12);
  EXPECT_TRUE(trackingErrorWithinExecutionEnvelope(error, 0.65, 1.5));
}

TEST(TrackingSpeedGovernor, LateralErrorStillUsesStrictLimit) {
  const auto error = decomposeTrackingError(
      {{0.0, 0.655, 0.0}}, {{0.9, 0.0, 0.0}});
  ASSERT_TRUE(error.valid);
  EXPECT_NEAR(error.along_track_lag, 0.0, 1e-12);
  EXPECT_NEAR(error.non_lag_deviation, 0.655, 1e-12);
  EXPECT_FALSE(trackingErrorWithinExecutionEnvelope(error, 0.65, 1.5));
}

TEST(TrackingSpeedGovernor, AheadOfReferenceIsNotTreatedAsSafeLag) {
  const auto error = decomposeTrackingError(
      {{-0.655, 0.0, 0.0}}, {{0.9, 0.0, 0.0}});
  ASSERT_TRUE(error.valid);
  EXPECT_NEAR(error.along_track_lag, 0.0, 1e-12);
  EXPECT_NEAR(error.non_lag_deviation, 0.655, 1e-12);
  EXPECT_FALSE(trackingErrorWithinExecutionEnvelope(error, 0.65, 1.5));
}

TEST(TrackingSpeedGovernor, StationaryReferenceKeepsTotalErrorStrict) {
  const auto error = decomposeTrackingError(
      {{0.655, 0.0, 0.0}}, {{0.0, 0.0, 0.0}});
  ASSERT_TRUE(error.valid);
  EXPECT_FALSE(error.tangent_available);
  EXPECT_NEAR(error.along_track_lag, 0.0, 1e-12);
  EXPECT_NEAR(error.non_lag_deviation, 0.655, 1e-12);
  EXPECT_FALSE(trackingErrorWithinExecutionEnvelope(error, 0.65, 1.5));
}

TEST(TrackingSpeedGovernor, ExcessiveAlongTrackLagRemainsBounded) {
  const auto error = decomposeTrackingError(
      {{1.51, 0.0, 0.0}}, {{0.9, 0.0, 0.0}});
  ASSERT_TRUE(error.valid);
  EXPECT_FALSE(trackingErrorWithinExecutionEnvelope(error, 0.65, 1.5));
}

TEST(TrackingSpeedGovernor, CompensatesPositionPursuitWithoutAborting) {
  const auto result = governTrackingSpeed(
      {{0.3, 0.0, 0.0}}, {{0.15, 0.0, 0.0}}, {{0.9, 0.9, 1.0}}, 0.4);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.active);
  EXPECT_NEAR(result.feedforward_velocity[0], 0.265, 1e-12);
  EXPECT_NEAR(result.closed_loop_velocity[0], 0.4, 1e-12);
}

TEST(TrackingSpeedGovernor, ProjectsTheCombinedThreeDimensionalVector) {
  const auto result = governTrackingSpeed(
      {{0.3, 0.0, 0.0}}, {{0.0, 0.3, 0.2}}, {{0.9, 0.9, 1.0}}, 0.4);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.active);
  EXPECT_NEAR(norm3(result.closed_loop_velocity), 0.4, 1e-12);
  EXPECT_LT(result.scale, 1.0);
}

TEST(TrackingSpeedGovernor, MeasuredSpeedFeedbackOnlyActsAboveSoftThreshold) {
  EXPECT_NEAR(measuredSpeedRequestLimit(.50, .65, 1.0, .70, .35), .70,
              1e-12);
  EXPECT_NEAR(measuredSpeedRequestLimit(.65, .65, 1.0, .70, .35), .70,
              1e-12);
  EXPECT_NEAR(measuredSpeedRequestLimit(.70, .65, 1.0, .70, .35), .65,
              1e-12);
  EXPECT_NEAR(measuredSpeedRequestLimit(.80, .65, 1.0, .70, .35), .55,
              1e-12);
  EXPECT_NEAR(measuredSpeedRequestLimit(1.00, .65, 1.0, .70, .35), .35,
              1e-12);
}

TEST(TrackingSpeedGovernor, AdaptiveLimitSlewsDownAndRecoversMoreSlowly) {
  EXPECT_NEAR(slewMeasuredSpeedRequestLimit(.70, .10, .02, 1.25, .35),
              .675, 1e-12);
  EXPECT_NEAR(slewMeasuredSpeedRequestLimit(.30, .70, .02, 1.25, .35),
              .307, 1e-12);
}

TEST(TrackingSpeedGovernor, LatestFlightTraceReducesRequestBeforeHardLimit) {
  double limit = .70;
  const double measured[] = {.643, .711, .770, .814};
  for (double speed : measured) {
    const double target = measuredSpeedRequestLimit(
        speed, .65, 1.0, .70, .35);
    limit = slewMeasuredSpeedRequestLimit(limit, target, .10, .80, .50);
  }
  EXPECT_LE(limit, .55);
  EXPECT_GE(limit, .35);
}

TEST(TrackingSpeedGovernor, OverspeedGetsOnePointFiveSecondsToRecover) {
  EXPECT_EQ(OverspeedRecoveryAction::kReduceSpeed,
            overspeedRecoveryAction(.81, .80, 0.0, 1.5));
  EXPECT_EQ(OverspeedRecoveryAction::kReduceSpeed,
            overspeedRecoveryAction(.81, .80, 1.499, 1.5));
  EXPECT_EQ(OverspeedRecoveryAction::kExitOffboard,
            overspeedRecoveryAction(.81, .80, 1.5, 1.5));
}

TEST(TrackingSpeedGovernor, RecoveredSpeedContinuesNormalFlight) {
  EXPECT_EQ(OverspeedRecoveryAction::kNormalFlight,
            overspeedRecoveryAction(.80, .80, 1.4, 1.5));
  EXPECT_EQ(OverspeedRecoveryAction::kNormalFlight,
            overspeedRecoveryAction(.72, .80, 1.4, 1.5));
}

TEST(MotionContinuity, QuinticOffsetMatchesBothBoundaryStates) {
  const auto q=QuinticOffset1D::make(.11,-.08,.25,3.5);
  const auto a=q.eval(0.0), b=q.eval(3.5);
  EXPECT_NEAR(a.p,.11,1e-12);
  EXPECT_NEAR(a.v,-.08,1e-12);
  EXPECT_NEAR(a.a,.25,1e-12);
  EXPECT_NEAR(b.p,0.0,1e-10);
  EXPECT_NEAR(b.v,0.0,1e-10);
  EXPECT_NEAR(b.a,0.0,1e-10);
}
TEST(MotionContinuity, QuinticOffsetDerivativesAreConsistent) {
  const auto q=QuinticOffset1D::make(.12,0.0,0.0,3.5);
  const double h=1e-4,t=1.7;
  const auto m=q.eval(t-h), c=q.eval(t), p=q.eval(t+h);
  EXPECT_NEAR((p.p-m.p)/(2*h),c.v,1e-7);
  EXPECT_NEAR((p.v-m.v)/(2*h),c.a,1e-6);
}
TEST(MotionContinuity, QuinticHandoffRemovesObservedHoldOffsetSmoothly) {
  const auto q=QuinticOffset1D::make(.11,0.0,0.0,3.5);
  double prev=.11;
  for(int i=1;i<=175;++i) {
    const auto x=q.eval(i*.02);
    EXPECT_LT(std::abs(x.p-prev),.003);
    prev=x.p;
  }
  EXPECT_NEAR(q.eval(3.5).p,0.0,1e-10);
}

TEST(MotionContinuity, TimeoutBrakeKeepsTheSetpointStreamContinuous) {
  const double duration = 1.5;
  const double stop_position = 0.5 * 0.8 * duration;
  const auto x = QuinticOffset1D::make(-stop_position, 0.8, 0.0, duration);
  const auto yaw = QuinticOffset1D::make(-0.5 * 0.4 * duration,
                                         0.4, 0.0, duration);
  MotionSample before;
  before.v[0] = 0.8;
  for (int i = 1; i <= 75; ++i) {
    const auto sx = x.eval(i * .02);
    const auto sy = yaw.eval(i * .02);
    MotionSample after;
    after.p[0] = stop_position + sx.p;
    after.v[0] = sx.v;
    after.a[0] = sx.a;
    after.yaw = 0.5 * 0.4 * duration + sy.p;
    EXPECT_EQ(nullptr, check(before, after)) << "brake tick " << i;
    before = after;
  }
  EXPECT_NEAR(before.p[0], stop_position, 1e-10);
  EXPECT_NEAR(before.v[0], 0.0, 1e-10);
  EXPECT_NEAR(before.a[0], 0.0, 1e-10);
}

TEST(MotionContinuity, NearWallBrakeIsShorterThanOrdinarySmoothStop) {
  const double speed = 0.64;
  const double deceleration = 1.75;
  const double duration = 1.5 * speed / deceleration;
  const double stop_position = 0.5 * speed * duration;
  EXPECT_LT(duration, 0.56);
  EXPECT_LT(stop_position, 0.18);
  EXPECT_LT(stop_position, 0.5 * speed * 1.5);

  const auto x = QuinticOffset1D::make(
      -stop_position, speed, 0.0, duration);
  MotionSample before;
  before.v[0] = speed;
  double peak_deceleration = 0.0;
  for (int i = 1; i <= 100; ++i) {
    const auto state = x.eval(duration * i / 100.0);
    peak_deceleration = std::max(peak_deceleration, -state.a);
    MotionSample after;
    after.p[0] = stop_position + state.p;
    after.v[0] = state.v;
    after.a[0] = state.a;
    const double dt = duration / 100.0;
    EXPECT_EQ(nullptr, motionTransitionFailure(
        before, after, dt, 1.1, deceleration, 30.0, 0.75));
    before = after;
  }
  EXPECT_NEAR(peak_deceleration, deceleration, 1e-6);
  EXPECT_NEAR(before.p[0], stop_position, 1e-10);
  EXPECT_NEAR(before.v[0], 0.0, 1e-10);
}
