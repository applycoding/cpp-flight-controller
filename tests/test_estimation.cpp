#include <gtest/gtest.h>

#include <cmath>

#include "fc/estimation/altitude_kf.hpp"
#include "fc/estimation/attitude_ekf.hpp"
#include "fc/estimation/complementary_filter.hpp"
#include "fc/math/quaternion.hpp"

namespace {

fc::Vec3 benchAccel(fc::Quaternion q, double gravity) {
  return q.rotateInverse({0.0, 0.0, -gravity});
}

fc::Vec3 benchMag(fc::Quaternion q) {
  return q.rotateInverse(fc::normalized(fc::defaultMagNed()));
}

void expectEulerNear(fc::EulerAngles actual, fc::EulerAngles expected, double tol) {
  EXPECT_NEAR(actual.roll, expected.roll, tol);
  EXPECT_NEAR(actual.pitch, expected.pitch, tol);
  EXPECT_NEAR(fc::wrapPi(actual.yaw - expected.yaw), 0.0, tol);
}

}  // namespace

TEST(ComplementaryFilter, StaticTiltAndYawConverge) {
  const fc::EulerAngles truth{0.22, -0.16, 0.45};
  const fc::Quaternion q_true = fc::Quaternion::fromEuler(truth);
  const fc::Vec3 bias{0.01, -0.015, 0.008};
  const fc::Vec3 accel = benchAccel(q_true, 9.80665);
  const fc::Vec3 mag = benchMag(q_true);

  fc::ComplementaryFilter filter(fc::demoEstimatorConfig({}));
  filter.reset({}, false);
  const double dt = 0.005;
  for (int i = 0; i < 3000; ++i) {
    filter.update(bias, accel, dt, &mag);
  }
  expectEulerNear(filter.euler(), truth, 0.03);
  EXPECT_NEAR(filter.gyroBias().x, bias.x, 0.008);
  EXPECT_NEAR(filter.gyroBias().y, bias.y, 0.008);
  EXPECT_NEAR(filter.gyroBias().z, bias.z, 0.008);
}

TEST(AttitudeEkf, StaticAttitudeAndBiasConverge) {
  const fc::EulerAngles truth{0.18, -0.12, -0.35};
  const fc::Quaternion q_true = fc::Quaternion::fromEuler(truth);
  const fc::Vec3 bias{0.012, -0.02, 0.007};
  const fc::Vec3 accel = benchAccel(q_true, 9.80665);
  const fc::Vec3 mag = benchMag(q_true);

  fc::AttitudeEkf ekf(fc::demoEstimatorConfig({}));
  ekf.reset({}, false);
  const double dt = 0.005;
  for (int i = 0; i < 4000; ++i) {
    ekf.propagate(bias, dt);
    ekf.updateAccel(accel);
    ekf.updateMag(mag);
  }
  expectEulerNear(ekf.euler(), truth, 0.02);
  EXPECT_NEAR(ekf.gyroBias().x, bias.x, 0.006);
  EXPECT_NEAR(ekf.gyroBias().y, bias.y, 0.006);
  EXPECT_NEAR(ekf.gyroBias().z, bias.z, 0.006);
}

TEST(AttitudeEkf, AlignedFilterRejectsInconsistentAccel) {
  fc::AttitudeEkf ekf(fc::demoEstimatorConfig({}));
  ekf.reset({}, true);
  ekf.propagate({}, 0.01);
  const fc::Vec3 bogus{6.0, -4.0, -2.0};
  EXPECT_FALSE(ekf.updateAccel(bogus));
  const fc::EulerAngles euler = ekf.euler();
  EXPECT_NEAR(euler.roll, 0.0, 1.0e-9);
  EXPECT_NEAR(euler.pitch, 0.0, 1.0e-9);
  EXPECT_NEAR(euler.yaw, 0.0, 1.0e-9);
}

TEST(AltitudeKf, BaroCorrectsAClimb) {
  fc::AltitudeKf kf(fc::demoEstimatorConfig({}));
  kf.reset(0.0, 0.0);
  const double dt = 0.01;
  for (int i = 0; i < 300; ++i) {
    const double time_s = static_cast<double>(i) * dt;
    const double altitude = 0.4 * time_s;
    kf.predict(0.0, dt);
    kf.updateBaro(altitude);
  }
  EXPECT_NEAR(kf.altitude(), 1.2, 0.15);
  EXPECT_NEAR(kf.verticalSpeed(), 0.4, 0.1);
}
