#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <future>

#include "fc/scenario.hpp"

namespace {

const fc::TelemetrySample& at(const std::vector<fc::TelemetrySample>& rows, double time_s,
                              double dt) {
  const auto index = static_cast<std::size_t>(std::llround(time_s / dt));
  return rows[std::min(index, rows.size() - 1)];
}

}  // namespace

TEST(ClosedLoop, NoiselessScenarioTracksTheSchedule) {
  fc::ScenarioConfig config;
  config.sensor_noise = false;
  config.duration_s = 16.0;
  config.dt_s = 0.001;
  const std::vector<fc::TelemetrySample> rows = fc::runClosedLoop(config);
  ASSERT_GT(rows.size(), 1000u);

  const auto& recovered = at(rows, 1.0, config.dt_s);
  EXPECT_NEAR(recovered.roll_deg, 0.0, 2.5);
  EXPECT_NEAR(recovered.pitch_deg, 0.0, 2.5);
  EXPECT_NEAR(recovered.yaw_deg, 0.0, 3.5);

  const auto& climb = at(rows, 6.0, config.dt_s);
  EXPECT_NEAR(climb.alt_m, 0.5, 0.2);

  const auto& roll = at(rows, 8.2, config.dt_s);
  EXPECT_NEAR(roll.roll_deg, 12.0, 5.0);
  EXPECT_NEAR(roll.pitch_deg, 0.0, 4.0);

  const auto& pitch = at(rows, 11.2, config.dt_s);
  EXPECT_NEAR(pitch.pitch_deg, 10.0, 3.0);
  EXPECT_NEAR(pitch.roll_deg, 0.0, 5.0);
  EXPECT_NEAR(pitch.pitch_ekf_deg, pitch.pitch_deg, 2.0);

  const auto& end = at(rows, 15.8, config.dt_s);
  EXPECT_NEAR(end.roll_deg, 0.0, 3.0);
  EXPECT_NEAR(end.pitch_deg, 0.0, 3.0);
  EXPECT_NEAR(end.yaw_deg, 20.0, 4.0);
  EXPECT_NEAR(end.alt_m, 0.5, 0.2);
  EXPECT_NEAR(end.yaw_ekf_deg, end.yaw_deg, 4.0);
  EXPECT_NEAR(end.bias_ekf_x, end.bias_x, 0.012);
  EXPECT_NEAR(end.bias_ekf_y, end.bias_y, 0.012);
  EXPECT_NEAR(end.bias_ekf_z, end.bias_z, 0.012);
  // Open-loop gyro integration has no bias estimate, so it walks off the truth.
  EXPECT_GT(std::abs(end.roll_gyro_deg - end.roll_deg), 6.0);

  for (const fc::TelemetrySample& row : rows) {
    for (double motor : row.motor_n) {
      EXPECT_GE(motor, -1.0e-6);
      EXPECT_LE(motor, config.vehicle.max_thrust_per_motor_n + 1.0e-6);
      EXPECT_TRUE(std::isfinite(motor));
    }
    EXPECT_TRUE(std::isfinite(row.roll_deg));
    EXPECT_TRUE(std::isfinite(row.alt_m));
  }
}

TEST(ClosedLoop, RepeatedRunIsDeterministic) {
  fc::ScenarioConfig config;
  config.sensor_noise = false;
  config.duration_s = 1.0;
  const auto a = fc::runClosedLoop(config);
  const auto b = fc::runClosedLoop(config);
  ASSERT_EQ(a.size(), b.size());
  EXPECT_NEAR(a.back().roll_deg, b.back().roll_deg, 0.0);
  EXPECT_NEAR(a.back().alt_m, b.back().alt_m, 0.0);
  EXPECT_NEAR(a.back().motor_n[0], b.back().motor_n[0], 0.0);
}

TEST(ThreadedHover, RingsMoveSamplesAndTheQuadStaysUpright) {
  auto future = std::async(std::launch::async, [] { return fc::runThreadedHover(0.35, 400.0); });
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  const fc::ThreadedSimStats stats = future.get();
  EXPECT_GT(stats.imu_pushed, 50u);
  EXPECT_EQ(stats.imu_dropped, 0u);
  EXPECT_GT(stats.control_steps, 20u);
  EXPECT_LT(std::abs(stats.final_roll_rad), 0.35);
  EXPECT_LT(std::abs(stats.final_pitch_rad), 0.35);
  EXPECT_LT(std::abs(stats.final_alt_m), 1.0);
  EXPECT_TRUE(std::isfinite(stats.final_alt_m));
}
