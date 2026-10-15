#include "fc/control/mixer.hpp"

#include <algorithm>
#include <cmath>

namespace fc {

QuadXMixer::QuadXMixer(double arm_m, double yaw_torque_ratio_m, double max_thrust_per_motor_n)
    : arm_m_(arm_m), yaw_ratio_m_(yaw_torque_ratio_m), max_thrust_n_(max_thrust_per_motor_n) {}

MotorThrusts QuadXMixer::allocate(double total_thrust_n, Vec3 torque_nm) const {
  const double s = arm_m_ / std::sqrt(2.0);
  const double quarter = 0.25 * total_thrust_n;
  const double roll = torque_nm.x / (4.0 * s);
  const double pitch = torque_nm.y / (4.0 * s);
  const double yaw = (yaw_ratio_m_ > 0.0) ? torque_nm.z / (4.0 * yaw_ratio_m_) : 0.0;
  MotorThrusts out;
  out.newton[0] = quarter - roll + pitch - yaw;
  out.newton[1] = quarter - roll - pitch + yaw;
  out.newton[2] = quarter + roll - pitch - yaw;
  out.newton[3] = quarter + roll + pitch + yaw;
  return out;
}

bool QuadXMixer::inRange(const MotorThrusts& thrusts) const {
  constexpr double kSlack = 1.0e-9;
  for (double thrust : thrusts.newton) {
    if (thrust < -kSlack || thrust > max_thrust_n_ + kSlack) {
      return false;
    }
  }
  return true;
}

MotorThrusts QuadXMixer::mix(double total_thrust_n, Vec3 torque_nm) const {
  const double max_total = max_thrust_n_ * static_cast<double>(kMotorCount);
  const double thrust = std::clamp(total_thrust_n, 0.0, max_total);
  if (inRange(allocate(thrust, torque_nm))) {
    return allocate(thrust, torque_nm);
  }

  double lo = 0.0;
  double hi = 1.0;
  for (int i = 0; i < 24; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (inRange(allocate(thrust, torque_nm * mid))) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  MotorThrusts scaled = allocate(thrust, torque_nm * lo);
  for (double& motor : scaled.newton) {
    motor = std::clamp(motor, 0.0, max_thrust_n_);
  }
  return scaled;
}

}  // namespace fc
