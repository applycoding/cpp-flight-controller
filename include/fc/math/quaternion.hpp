#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

#include "fc/math/vec3.hpp"

namespace fc {

struct Quaternion {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};

  constexpr Quaternion() = default;
  constexpr Quaternion(double w_in, double x_in, double y_in, double z_in)
      : w(w_in), x(x_in), y(y_in), z(z_in) {}

  constexpr Quaternion conjugate() const { return {w, -x, -y, -z}; }
  constexpr Quaternion negated() const { return {-w, -x, -y, -z}; }

  constexpr Quaternion operator*(Quaternion rhs) const {
    return {w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w};
  }

  double norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

  Quaternion normalized() const {
    const double n = norm();
    if (n <= 0.0) {
      return {};
    }
    return {w / n, x / n, y / n, z / n};
  }

  // Active rotation of a body vector into the frame this quaternion maps to.
  Vec3 rotate(Vec3 v) const {
    const Vec3 u{x, y, z};
    const Vec3 t = u.cross(v) * 2.0;
    return v + t * w + u.cross(t);
  }

  Vec3 rotateInverse(Vec3 v) const { return conjugate().rotate(v); }

  // Body-to-world quaternion from intrinsic yaw (z), pitch (y), roll (x).
  static Quaternion fromEuler(EulerAngles euler) {
    const double cr = std::cos(euler.roll * 0.5);
    const double sr = std::sin(euler.roll * 0.5);
    const double cp = std::cos(euler.pitch * 0.5);
    const double sp = std::sin(euler.pitch * 0.5);
    const double cy = std::cos(euler.yaw * 0.5);
    const double sy = std::sin(euler.yaw * 0.5);
    return {cy * cp * cr + sy * sp * sr, cy * cp * sr - sy * sp * cr, cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr};
  }

  static Quaternion fromRotationVector(Vec3 rotation) {
    const double angle = fc::norm(rotation);
    if (angle < 1.0e-12) {
      return Quaternion{1.0, rotation.x * 0.5, rotation.y * 0.5, rotation.z * 0.5}.normalized();
    }
    const double half = 0.5 * angle;
    const Vec3 axis = rotation / angle;
    const double s = std::sin(half);
    return {std::cos(half), axis.x * s, axis.y * s, axis.z * s};
  }

  Quaternion integrate(Vec3 omega_body, double dt) const {
    return ((*this) * fromRotationVector(omega_body * dt)).normalized();
  }

  EulerAngles toEuler() const {
    const Quaternion q = normalized();
    EulerAngles e;
    e.roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    const double sin_pitch = std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0);
    e.pitch = std::asin(sin_pitch);
    e.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return e;
  }

  // Smallest rotation, expressed in this body frame, that takes this attitude to `target`.
  // The vector direction is the rotation axis and its magnitude is 2*sin(theta/2) ≈ theta.
  Vec3 errorRotationVectorTo(Quaternion target) const {
    Quaternion err = conjugate() * target;
    if (err.w < 0.0) {
      err = err.negated();
    }
    return {2.0 * err.x, 2.0 * err.y, 2.0 * err.z};
  }
};

inline double wrapPi(double angle) {
  constexpr double pi = std::numbers::pi;
  while (angle > pi) {
    angle -= 2.0 * pi;
  }
  while (angle < -pi) {
    angle += 2.0 * pi;
  }
  return angle;
}

}  // namespace fc
