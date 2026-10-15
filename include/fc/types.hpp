#pragma once

#include <cstdint>

namespace fc {

// World frame is NED (north, east, down), meters and radians.
// Body frame is FRD (forward, right, down).
// A quaternion rotates a body-frame vector into the world frame.
// Gyro is body angular rate (rad/s). Positive yaw is clockwise from above.
// The accelerometer reads specific force (m/s^2). In an unaccelerated,
// level hover it reads (0, 0, -g), because +z is down.

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  constexpr Vec3() = default;
  constexpr Vec3(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in) {}

  constexpr Vec3 operator+(Vec3 rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
  constexpr Vec3 operator-(Vec3 rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
  constexpr Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  constexpr Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
  constexpr Vec3 operator-() const { return {-x, -y, -z}; }

  constexpr Vec3& operator+=(Vec3 rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }

  constexpr Vec3& operator-=(Vec3 rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }

  constexpr double dot(Vec3 rhs) const { return x * rhs.x + y * rhs.y + z * rhs.z; }

  constexpr Vec3 cross(Vec3 rhs) const {
    return {y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x};
  }
};

constexpr Vec3 operator*(double s, Vec3 v) {
  return v * s;
}

struct EulerAngles {
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

struct ImuSample {
  double time_s{0.0};
  Vec3 gyro_rad_s{};
  Vec3 accel_m_s2{};
  Vec3 mag_body{};
  double baro_altitude_m{0.0};
  bool mag_valid{false};
  bool baro_valid{false};
};

struct Setpoint {
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double yaw_rad{0.0};
  double altitude_m{0.0};
};

struct MotorThrusts {
  double newton[4]{0.0, 0.0, 0.0, 0.0};
};

// Quad-X motor order, looking down (from -z toward +z), nose along +x:
//   0 front-right (CW prop, reaction torque is -yaw)
//   1 rear-right  (CCW prop, reaction torque is +yaw)
//   2 rear-left   (CW prop)
//   3 front-left  (CCW prop)
constexpr int kMotorCount = 4;

struct VehicleParams {
  double mass_kg{1.05};
  double gravity_m_s2{9.80665};
  double arm_m{0.17};
  double inertia_xx{0.0075};
  double inertia_yy{0.0075};
  double inertia_zz{0.014};
  // Reaction yaw torque (N·m) per newton of thrust. Sign comes from the prop direction.
  double yaw_torque_ratio_m{0.018};
  double max_thrust_per_motor_n{6.5};
  // Isotropic quadratic drag: F = -c |v| v in the world frame.
  double quadratic_drag{0.55};
};

struct PidGains {
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  // Back-calculation gain. Zero disables the extra unwind term (output is still clamped).
  double k_aw{0.0};
  // First-order cutoff for the derivative term, in Hz. Zero disables the filter.
  double derivative_filter_hz{0.0};
  double out_min{-1.0e9};
  double out_max{1.0e9};
};

struct AttitudeGains {
  double roll_angle_p{5.0};
  double pitch_angle_p{5.0};
  double yaw_angle_p{4.0};
  double max_rate_rad_s{3.0};
  PidGains roll_rate{};
  PidGains pitch_rate{};
  PidGains yaw_rate{};
};

struct AltitudeGains {
  double hover_thrust_n{0.0};
  PidGains vertical{};
};

inline Vec3 defaultMagNed() {
  // Unit field at 60 degrees inclination (positive down), pointing north.
  return {0.5, 0.0, 0.8660254037844386};
}

struct EstimatorConfig {
  double gravity_m_s2{9.80665};
  // World magnetic field, NED. Direction is what the filter uses.
  Vec3 mag_ned{defaultMagNed()};
  double comp_kp{1.2};
  double comp_ki{0.2};
  double comp_mag_kp{0.8};
  // Reject the accelerometer correction when it disagrees with the prediction by more than this
  // many m/s^2. Disagreement is expected while the vehicle is linearly accelerating.
  double accel_gate_m_s2{0.45};
  double gyro_noise{0.02};
  double bias_random_walk{0.0008};
  double accel_noise{0.7};
  double mag_noise{0.08};
  double initial_attitude_std_rad{0.2};
  double initial_bias_std{0.05};
  double baro_noise_m{0.08};
  double alt_accel_noise{0.35};
};

inline AttitudeGains demoAttitudeGains() {
  AttitudeGains g;
  g.roll_angle_p = 4.4;
  g.pitch_angle_p = 4.4;
  g.yaw_angle_p = 3.2;
  g.max_rate_rad_s = 2.4;
  const PidGains rate{
      .kp = 0.24,
      .ki = 0.12,
      .kd = 0.001,
      .k_aw = 5.0,
      .derivative_filter_hz = 30.0,
      .out_min = -0.55,
      .out_max = 0.55,
  };
  g.roll_rate = rate;
  g.pitch_rate = rate;
  PidGains yaw = rate;
  yaw.kp = 0.36;
  yaw.ki = 0.10;
  yaw.out_min = -0.40;
  yaw.out_max = 0.40;
  g.yaw_rate = yaw;
  return g;
}

inline AltitudeGains demoAltitudeGains(const VehicleParams& vehicle) {
  AltitudeGains g;
  g.hover_thrust_n = vehicle.mass_kg * vehicle.gravity_m_s2;
  g.vertical = PidGains{
      .kp = 5.5,
      .ki = 2.2,
      .kd = 4.2,
      .k_aw = 1.5,
      .derivative_filter_hz = 8.0,
      .out_min = -0.55 * g.hover_thrust_n,
      .out_max = 0.90 * g.hover_thrust_n,
  };
  return g;
}

inline EstimatorConfig demoEstimatorConfig(const VehicleParams& vehicle) {
  EstimatorConfig cfg;
  cfg.gravity_m_s2 = vehicle.gravity_m_s2;
  return cfg;
}

}  // namespace fc
