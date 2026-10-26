#pragma once

// Estimator-agnostic rigid-body alignment shared by Omni-LIVO production and
// the retained Fast-LIO SITL compatibility path.

#include <algorithm>
#include <cmath>

namespace mine_uav_control {

struct Vector3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  Vector3() = default;
  Vector3(double x_value, double y_value, double z_value)
      : x(x_value), y(y_value), z(z_value) {}
};

struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
  Quaternion() = default;
  Quaternion(double x_value, double y_value, double z_value, double w_value)
      : x(x_value), y(y_value), z(z_value), w(w_value) {}
};

inline Quaternion normalized(Quaternion q) {
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z +
                                q.w * q.w);
  if (!std::isfinite(norm) || norm < 1e-12) {
    return Quaternion();
  }
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  return q;
}

inline Quaternion conjugate(const Quaternion& input) {
  const Quaternion q = normalized(input);
  return Quaternion{-q.x, -q.y, -q.z, q.w};
}

inline Quaternion multiply(const Quaternion& a_input,
                           const Quaternion& b_input) {
  const Quaternion a = normalized(a_input);
  const Quaternion b = normalized(b_input);
  return normalized(Quaternion{
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z});
}

// ROS fixed-axis roll/pitch/yaw convention: Rz(yaw) * Ry(pitch) * Rx(roll).
inline Quaternion quaternionFromRpy(double roll, double pitch, double yaw) {
  const double cr = std::cos(0.5 * roll);
  const double sr = std::sin(0.5 * roll);
  const double cp = std::cos(0.5 * pitch);
  const double sp = std::sin(0.5 * pitch);
  const double cy = std::cos(0.5 * yaw);
  const double sy = std::sin(0.5 * yaw);
  return normalized(Quaternion{
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
      cr * cp * cy + sr * sp * sy});
}

inline Vector3 rotateVector(const Quaternion& rotation,
                            const Vector3& input) {
  const Quaternion q = normalized(rotation);
  // Expanded q * [v, 0] * conjugate(q), avoiding normalization of the
  // intermediate pure-vector quaternion.
  const double tx = 2.0 * (q.y * input.z - q.z * input.y);
  const double ty = 2.0 * (q.z * input.x - q.x * input.z);
  const double tz = 2.0 * (q.x * input.y - q.y * input.x);
  return Vector3{
      input.x + q.w * tx + (q.y * tz - q.z * ty),
      input.y + q.w * ty + (q.z * tx - q.x * tz),
      input.z + q.w * tz + (q.x * ty - q.y * tx)};
}

inline double vectorNorm(const Vector3& input) {
  return std::sqrt(input.x * input.x + input.y * input.y +
                   input.z * input.z);
}

inline Vector3 normalizedVector(const Vector3& input) {
  const double norm = vectorNorm(input);
  if (!std::isfinite(norm) || norm < 1e-12) {
    return Vector3();
  }
  return Vector3(input.x / norm, input.y / norm, input.z / norm);
}

inline double vectorDot(const Vector3& a, const Vector3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vector3 vectorCross(const Vector3& a, const Vector3& b) {
  return Vector3(a.y * b.z - a.z * b.y,
                 a.z * b.x - a.x * b.z,
                 a.x * b.y - a.y * b.x);
}

// Return the minimum-angle rotation that maps one measured gravity direction
// onto another. Gravity observes roll/pitch only; this deliberately adds no
// unobservable twist about the gravity axis.
inline bool gravityAlignmentRotation(const Vector3& sensor_gravity,
                                     const Vector3& body_gravity,
                                     Quaternion* body_from_sensor) {
  if (body_from_sensor == nullptr || vectorNorm(sensor_gravity) < 1e-12 ||
      vectorNorm(body_gravity) < 1e-12) {
    return false;
  }
  const Vector3 from = normalizedVector(sensor_gravity);
  const Vector3 to = normalizedVector(body_gravity);
  const double dot = std::max(-1.0, std::min(1.0, vectorDot(from, to)));
  if (dot > 1.0 - 1e-12) {
    *body_from_sensor = Quaternion();
    return true;
  }
  if (dot < -1.0 + 1e-9) {
    const Vector3 reference = std::abs(from.x) < 0.9
        ? Vector3(1.0, 0.0, 0.0)
        : Vector3(0.0, 1.0, 0.0);
    const Vector3 axis = normalizedVector(vectorCross(from, reference));
    *body_from_sensor = Quaternion(axis.x, axis.y, axis.z, 0.0);
    return true;
  }
  const Vector3 axis = vectorCross(from, to);
  *body_from_sensor = normalized(
      Quaternion(axis.x, axis.y, axis.z, 1.0 + dot));
  return true;
}

inline double yawFromQuaternion(const Quaternion& input) {
  const Quaternion q = normalized(input);
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline double quaternionAngularDistance(const Quaternion& a,
                                        const Quaternion& b) {
  const Quaternion an = normalized(a);
  const Quaternion bn = normalized(b);
  const double dot = std::abs(an.x * bn.x + an.y * bn.y +
                              an.z * bn.z + an.w * bn.w);
  return 2.0 * std::acos(std::min(1.0, std::max(0.0, dot)));
}

inline Quaternion scaledRotation(const Quaternion& input, double scale) {
  Quaternion q = normalized(input);
  // q and -q represent the same rotation. Select the shortest arc so a small
  // measured increment cannot extrapolate through almost 2*pi.
  if (q.w < 0.0) {
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
    q.w = -q.w;
  }
  const double vector_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
  if (vector_norm < 1e-12 || !std::isfinite(scale)) {
    return Quaternion();
  }
  const double angle = 2.0 * std::atan2(vector_norm, q.w);
  const double half_scaled_angle = 0.5 * angle * scale;
  const double axis_scale = std::sin(half_scaled_angle) / vector_norm;
  return normalized(Quaternion(q.x * axis_scale, q.y * axis_scale,
                               q.z * axis_scale,
                               std::cos(half_scaled_angle)));
}

struct MountAlignment {
  // Rotation from Fast-LIO map coordinates into the task-local PX4 axes.
  Quaternion map_to_output;
  // Fixed rotation from aircraft-body coordinates into sensor coordinates.
  // Appending it to a sensor pose produces the aircraft body pose.
  Quaternion sensor_from_body;
};

// The planner must not operate in the physically tilted Fast-LIO map frame.
// Rotate map positions into a gravity-aligned task frame before mapping and
// planning.  Aircraft attitude additionally needs the fixed body-to-sensor
// rotation because Fast-LIO reports the sensor attitude, not the body attitude.
inline Vector3 positionInPlanningFrame(
    const Quaternion& map_to_output, const Vector3& position_in_map) {
  return rotateVector(map_to_output, position_in_map);
}

inline Quaternion bodyOrientationInPlanningFrame(
    const Quaternion& map_to_output,
    const Quaternion& sensor_orientation_in_map,
    const Quaternion& sensor_from_body) {
  return multiply(multiply(map_to_output, sensor_orientation_in_map),
                  sensor_from_body);
}

// The level planning frame deliberately starts with aircraft yaw zero, while
// PX4's local ENU attitude already carries its magnetometer/EKF heading.  The
// external-vision pose must start at that same heading; otherwise enabling EV
// yaw injects a large artificial innovation and Commander can reject POSCTL
// arming until the two yaw estimates slowly converge.
inline Quaternion planningToReferenceHeading(
    const Quaternion& reference_body_orientation,
    const Quaternion& initial_body_in_planning) {
  const double yaw_offset = yawFromQuaternion(reference_body_orientation) -
                            yawFromQuaternion(initial_body_in_planning);
  return quaternionFromRpy(0.0, 0.0, yaw_offset);
}

inline MountAlignment makeMountAlignmentFromBodyFromSensor(
    const Quaternion& sensor_orientation_in_map,
    const Quaternion& body_from_sensor, bool zero_initial_roll_pitch,
    bool zero_initial_yaw) {
  MountAlignment result;
  result.sensor_from_body = conjugate(body_from_sensor);
  const Quaternion initial_body_in_map =
      multiply(sensor_orientation_in_map, result.sensor_from_body);
  if (zero_initial_roll_pitch) {
    // Alignment is captured only while disarmed. Treat that pose as level and
    // remove all three initial body angles so horizontal motion cannot leak
    // into the PX4 height channel.
    result.map_to_output = conjugate(initial_body_in_map);
  } else if (zero_initial_yaw) {
    result.map_to_output = quaternionFromRpy(
        0.0, 0.0, -yawFromQuaternion(initial_body_in_map));
  }
  return result;
}

inline MountAlignment makeMountAlignment(
    const Quaternion& sensor_orientation_in_map,
    double sensor_roll_in_body, double sensor_pitch_in_body,
    double sensor_yaw_in_body, bool zero_initial_roll_pitch,
    bool zero_initial_yaw) {
  // These are physical mounting corrections about the aircraft body axes.
  // Apply yaw first, then the body-frame down-tilt. This is Rx * Ry * Rz,
  // intentionally different from a conventional sensor-local RPY Euler
  // composition. The 2026-10-13 flight log distinguishes the two orders:
  // body-frame pitch removes the 0.53 m landing residual, sensor-local pitch
  // leaves about 0.40 m behind.
  const Quaternion body_from_sensor = multiply(
      multiply(quaternionFromRpy(sensor_roll_in_body, 0.0, 0.0),
               quaternionFromRpy(0.0, sensor_pitch_in_body, 0.0)),
      quaternionFromRpy(0.0, 0.0, sensor_yaw_in_body));
  return makeMountAlignmentFromBodyFromSensor(
      sensor_orientation_in_map, body_from_sensor,
      zero_initial_roll_pitch, zero_initial_yaw);
}

// Kept for compatibility with the original yaw-only tests and callers.
struct PlanarMountAlignment {
  double map_to_output_yaw{0.0};
  double sensor_from_body_yaw{0.0};
};

inline PlanarMountAlignment makePlanarMountAlignment(
    double sensor_yaw_in_map, double sensor_yaw_in_body,
    bool zero_initial_body_yaw) {
  PlanarMountAlignment result;
  result.sensor_from_body_yaw = -sensor_yaw_in_body;
  const double body_yaw_in_map =
      sensor_yaw_in_map + result.sensor_from_body_yaw;
  result.map_to_output_yaw =
      zero_initial_body_yaw ? -body_yaw_in_map : 0.0;
  return result;
}

inline void rotatePlanar(double yaw, double x, double y,
                         double* output_x, double* output_y) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  *output_x = c * x - s * y;
  *output_y = s * x + c * y;
}

}  // namespace mine_uav_control
