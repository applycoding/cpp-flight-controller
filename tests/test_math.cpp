#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "fc/math/matrix.hpp"
#include "fc/math/quaternion.hpp"
#include "fc/math/vec3.hpp"

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

}  // namespace

TEST(Vec3, DotAndCross) {
  const fc::Vec3 x{1.0, 0.0, 0.0};
  const fc::Vec3 y{0.0, 1.0, 0.0};
  EXPECT_NEAR(x.dot(y), 0.0, 1.0e-12);
  const fc::Vec3 z = x.cross(y);
  EXPECT_NEAR(z.x, 0.0, 1.0e-12);
  EXPECT_NEAR(z.y, 0.0, 1.0e-12);
  EXPECT_NEAR(z.z, 1.0, 1.0e-12);
  EXPECT_NEAR(fc::norm(fc::normalized(fc::Vec3{3.0, 0.0, 4.0})), 1.0, 1.0e-12);
}

TEST(Quaternion, YawRotatesForwardToEast) {
  const fc::Quaternion q = fc::Quaternion::fromEuler({0.0, 0.0, 90.0 * kDeg});
  const fc::Vec3 east = q.rotate({1.0, 0.0, 0.0});
  EXPECT_NEAR(east.x, 0.0, 1.0e-9);
  EXPECT_NEAR(east.y, 1.0, 1.0e-9);
  EXPECT_NEAR(east.z, 0.0, 1.0e-9);
}

TEST(Quaternion, EulerRoundTrip) {
  const fc::EulerAngles in{12.0 * kDeg, -25.0 * kDeg, 140.0 * kDeg};
  const fc::EulerAngles out = fc::Quaternion::fromEuler(in).toEuler();
  EXPECT_NEAR(out.roll, in.roll, 1.0e-9);
  EXPECT_NEAR(out.pitch, in.pitch, 1.0e-9);
  EXPECT_NEAR(out.yaw, in.yaw, 1.0e-9);
}

TEST(Quaternion, IntegratesAQuarterTurn) {
  const fc::Quaternion q = fc::Quaternion{}.integrate({0.0, 0.0, std::numbers::pi}, 0.5);
  const fc::EulerAngles euler = q.toEuler();
  EXPECT_NEAR(euler.yaw, std::numbers::pi / 2.0, 1.0e-9);
  EXPECT_NEAR(euler.roll, 0.0, 1.0e-9);
  EXPECT_NEAR(euler.pitch, 0.0, 1.0e-9);
}

TEST(Quaternion, ShortestYawError) {
  const fc::Quaternion from = fc::Quaternion::fromEuler({0.0, 0.0, -170.0 * kDeg});
  const fc::Quaternion to = fc::Quaternion::fromEuler({0.0, 0.0, 170.0 * kDeg});
  const fc::Vec3 error = from.errorRotationVectorTo(to);
  EXPECT_NEAR(error.x, 0.0, 1.0e-6);
  EXPECT_NEAR(error.y, 0.0, 1.0e-6);
  EXPECT_NEAR(error.z, -20.0 * kDeg, 0.01);
}

TEST(Matrix, InverseAndSkew) {
  fc::Matrix<3, 3> m = fc::Matrix<3, 3>::identity();
  m(0, 1) = 2.0;
  m(1, 2) = -0.5;
  m(2, 0) = 0.25;
  fc::Matrix<3, 3> inv{};
  ASSERT_TRUE(fc::invert3(m, inv));
  const fc::Matrix<3, 3> product = m.times(inv);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      EXPECT_NEAR(product(r, c), r == c ? 1.0 : 0.0, 1.0e-9);
    }
  }

  const fc::Vec3 v{0.2, -0.4, 0.7};
  const fc::Vec3 w{1.0, 2.0, -0.5};
  const fc::Matrix<3, 1> column = [&] {
    fc::Matrix<3, 1> c = fc::Matrix<3, 1>::zero();
    c(0, 0) = w.x;
    c(1, 0) = w.y;
    c(2, 0) = w.z;
    return c;
  }();
  const fc::Matrix<3, 1> skewed = fc::skew(v).times(column);
  const fc::Vec3 crossed = v.cross(w);
  EXPECT_NEAR(skewed(0, 0), crossed.x, 1.0e-12);
  EXPECT_NEAR(skewed(1, 0), crossed.y, 1.0e-12);
  EXPECT_NEAR(skewed(2, 0), crossed.z, 1.0e-12);
}
