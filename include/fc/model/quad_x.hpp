#pragma once

#include <array>
#include <cmath>

#include "fc/math/vec3.hpp"

namespace fc {

struct MotorLayout {
  Vec3 position_m{};
  double yaw_sign{1.0};
};

inline std::array<MotorLayout, kMotorCount> quadXMotors(double arm_m) {
  const double s = arm_m / std::sqrt(2.0);
  return {{
      {{s, s, 0.0}, -1.0},
      {{-s, s, 0.0}, 1.0},
      {{-s, -s, 0.0}, -1.0},
      {{s, -s, 0.0}, 1.0},
  }};
}

// Torque about the body origin from one motor. Thrust is along -body z (up).
inline Vec3 motorTorque(const MotorLayout& motor, double thrust_n, double yaw_ratio_m) {
  const Vec3 force{0.0, 0.0, -thrust_n};
  Vec3 torque = motor.position_m.cross(force);
  torque.z += motor.yaw_sign * yaw_ratio_m * thrust_n;
  return torque;
}

}  // namespace fc
