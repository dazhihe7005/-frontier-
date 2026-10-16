#include "mine_uav_control/fastlio_mount_alignment.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(FastlioMountAlignment, ClockwiseNinetyMapsSensorLeftToBodyForward) {
  const auto alignment = mine_uav_control::makePlanarMountAlignment(
      0.0, -kPi / 2.0, true);

  EXPECT_NEAR(alignment.map_to_output_yaw, -kPi / 2.0, 1e-12);
  EXPECT_NEAR(alignment.sensor_from_body_yaw, kPi / 2.0, 1e-12);

  double x = 0.0;
  double y = 0.0;
  mine_uav_control::rotatePlanar(
      alignment.map_to_output_yaw, 0.0, 1.0, &x, &y);
  EXPECT_NEAR(x, 1.0, 1e-12);
  EXPECT_NEAR(y, 0.0, 1e-12);
}

TEST(FastlioMountAlignment, MountAndWorldRotationsCancelAtInitialPose) {
  const auto alignment = mine_uav_control::makePlanarMountAlignment(
      0.0, -kPi / 2.0, true);
  const double initial_body_yaw = alignment.map_to_output_yaw +
                                  alignment.sensor_from_body_yaw;
  EXPECT_NEAR(initial_body_yaw, 0.0, 1e-12);
}

TEST(FastlioMountAlignment, ZeroMountKeepsLegacyAlignment) {
  const auto alignment = mine_uav_control::makePlanarMountAlignment(
      0.3, 0.0, true);
  EXPECT_NEAR(alignment.map_to_output_yaw, -0.3, 1e-12);
  EXPECT_NEAR(alignment.sensor_from_body_yaw, 0.0, 1e-12);
}

TEST(FastlioMountAlignment, DownwardPitchDoesNotLeakHorizontalMotionIntoZ) {
  const double pitch = 25.0 * kPi / 180.0;
  const double yaw = -kPi / 2.0;
  const auto alignment = mine_uav_control::makeMountAlignment(
      mine_uav_control::Quaternion(), 0.0, pitch, yaw, true, true);

  // Express aircraft-forward horizontal travel in the initial sensor/map
  // frame, then apply the bridge alignment. It must remain level in PX4.
  const auto forward_in_sensor = mine_uav_control::rotateVector(
      alignment.sensor_from_body,
      mine_uav_control::Vector3(1.0, 0.0, 0.0));
  const auto forward_in_px4 = mine_uav_control::rotateVector(
      alignment.map_to_output, forward_in_sensor);
  EXPECT_NEAR(forward_in_px4.x, 1.0, 1e-12);
  EXPECT_NEAR(forward_in_px4.y, 0.0, 1e-12);
  EXPECT_NEAR(forward_in_px4.z, 0.0, 1e-12);
}

TEST(FastlioMountAlignment, ForwardFacingRadarKeepsOnlyPitchCompensation) {
  const double pitch = 25.0 * kPi / 180.0;
  const auto alignment = mine_uav_control::makeMountAlignment(
      mine_uav_control::Quaternion(), 0.0, pitch, 0.0, true, true);

  const auto forward_in_sensor = mine_uav_control::rotateVector(
      alignment.sensor_from_body,
      mine_uav_control::Vector3(1.0, 0.0, 0.0));
  const auto forward_in_px4 = mine_uav_control::rotateVector(
      alignment.map_to_output, forward_in_sensor);
  EXPECT_NEAR(forward_in_px4.x, 1.0, 1e-12);
  EXPECT_NEAR(forward_in_px4.y, 0.0, 1e-12);
  EXPECT_NEAR(forward_in_px4.z, 0.0, 1e-12);
  EXPECT_NEAR(
      mine_uav_control::yawFromQuaternion(alignment.sensor_from_body),
      0.0, 1e-12);
}

TEST(FastlioMountAlignment, InitialBodyAttitudeBecomesIdentity) {
  const auto sensor_in_map = mine_uav_control::quaternionFromRpy(
      0.02, -0.03, 0.17);
  const auto alignment = mine_uav_control::makeMountAlignment(
      sensor_in_map, 0.0, 25.0 * kPi / 180.0, -kPi / 2.0, true, true);
  const auto body_in_output = mine_uav_control::multiply(
      mine_uav_control::multiply(alignment.map_to_output, sensor_in_map),
      alignment.sensor_from_body);

  EXPECT_NEAR(body_in_output.x, 0.0, 1e-12);
  EXPECT_NEAR(body_in_output.y, 0.0, 1e-12);
  EXPECT_NEAR(body_in_output.z, 0.0, 1e-12);
  EXPECT_NEAR(std::abs(body_in_output.w), 1.0, 1e-12);
}

TEST(FastlioMountAlignment, PlannerSeesHorizontalTravelInALevelFrame) {
  const double pitch = 25.0 * kPi / 180.0;
  const auto alignment = mine_uav_control::makeMountAlignment(
      mine_uav_control::Quaternion(), 0.0, pitch, 0.0, true, true);
  const auto raw_forward = mine_uav_control::rotateVector(
      alignment.sensor_from_body,
      mine_uav_control::Vector3(5.0, 0.0, 0.0));
  const auto planning_forward = mine_uav_control::positionInPlanningFrame(
      alignment.map_to_output, raw_forward);
  const auto planning_body =
      mine_uav_control::bodyOrientationInPlanningFrame(
          alignment.map_to_output, mine_uav_control::Quaternion(),
          alignment.sensor_from_body);

  EXPECT_NEAR(planning_forward.x, 5.0, 1e-12);
  EXPECT_NEAR(planning_forward.y, 0.0, 1e-12);
  EXPECT_NEAR(planning_forward.z, 0.0, 1e-12);
  EXPECT_NEAR(planning_body.x, 0.0, 1e-12);
  EXPECT_NEAR(planning_body.y, 0.0, 1e-12);
  EXPECT_NEAR(planning_body.z, 0.0, 1e-12);
  EXPECT_NEAR(std::abs(planning_body.w), 1.0, 1e-12);
}

TEST(FastlioMountAlignment, PitchSignMakesSensorForwardPointDownward) {
  const auto body_from_sensor = mine_uav_control::quaternionFromRpy(
      0.0, 25.0 * kPi / 180.0, 0.0);
  const auto sensor_forward_in_body = mine_uav_control::rotateVector(
      body_from_sensor, mine_uav_control::Vector3(1.0, 0.0, 0.0));
  EXPECT_GT(sensor_forward_in_body.x, 0.0);
  EXPECT_LT(sensor_forward_in_body.z, 0.0);
}

TEST(FastlioMountAlignment, RecordedLandingResidualIsRemovedByBodyPitch) {
  const auto alignment = mine_uav_control::makeMountAlignment(
      mine_uav_control::Quaternion(), 0.0, 25.0 * kPi / 180.0,
      -90.9 * kPi / 180.0, true, true);
  // Median post-flight minus pre-flight Fast-LIO displacement from
  // 20261013_193016. Raw Z falsely changed by -0.533 m.
  const auto corrected = mine_uav_control::rotateVector(
      alignment.map_to_output,
      mine_uav_control::Vector3(-0.1989286, -1.16855066, -0.53340568));
  EXPECT_NEAR(corrected.z, 0.0, 0.02);
  EXPECT_GT(std::abs(corrected.x), 1.0);
}

TEST(FastlioMountAlignment, LeverArmRemovesRotationOnlyTranslation) {
  const mine_uav_control::Vector3 lever(0.133803, -0.011364, 0.173245);
  const auto body_rotation = mine_uav_control::quaternionFromRpy(
      0.12, -0.08, 0.5);
  const auto initial_sensor = lever;
  const auto rotated_sensor =
      mine_uav_control::rotateVector(body_rotation, lever);

  // A body rotating in place makes the off-centre sensor translate by this
  // amount. Subtracting the lever-arm change must recover zero body motion.
  const mine_uav_control::Vector3 corrected(
      rotated_sensor.x - initial_sensor.x -
          (rotated_sensor.x - initial_sensor.x),
      rotated_sensor.y - initial_sensor.y -
          (rotated_sensor.y - initial_sensor.y),
      rotated_sensor.z - initial_sensor.z -
          (rotated_sensor.z - initial_sensor.z));
  EXPECT_NEAR(corrected.x, 0.0, 1e-12);
  EXPECT_NEAR(corrected.y, 0.0, 1e-12);
  EXPECT_NEAR(corrected.z, 0.0, 1e-12);
}

TEST(FastlioMountAlignment, RotationIncrementCanBeBoundedlyExtrapolated) {
  const auto ten_degrees = mine_uav_control::quaternionFromRpy(
      0.0, 0.0, 10.0 * kPi / 180.0);
  const auto fifteen_degrees =
      mine_uav_control::scaledRotation(ten_degrees, 1.5);
  EXPECT_NEAR(mine_uav_control::yawFromQuaternion(fifteen_degrees),
              15.0 * kPi / 180.0, 1e-12);
}

}  // namespace
