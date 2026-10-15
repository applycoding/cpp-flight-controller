#pragma once

#include <cmath>

#include "fc/types.hpp"

namespace fc {

inline double norm(Vec3 v) {
  return std::sqrt(v.dot(v));
}

inline Vec3 normalized(Vec3 v) {
  const double n = norm(v);
  if (n <= 0.0) {
    return {};
  }
  return v / n;
}

inline Vec3 clampVec(Vec3 v, double limit) {
  const double n = norm(v);
  if (n <= limit || n <= 0.0) {
    return v;
  }
  return v * (limit / n);
}

}  // namespace fc
