#include <gtest/gtest.h>

#include <cstdint>
#include <thread>
#include <vector>

#include "fc/rt/fixed_rate_loop.hpp"
#include "fc/rt/spsc_ring_buffer.hpp"
#include "fc/sim/quad_sim.hpp"

TEST(SpscRingBuffer, ReportsFullAndWraps) {
  fc::SpscRingBuffer<int, 8> ring;
  for (int i = 0; i < 7; ++i) {
    ASSERT_TRUE(ring.push(i));
  }
  EXPECT_FALSE(ring.push(99));
  int value = -1;
  ASSERT_TRUE(ring.pop(value));
  EXPECT_EQ(value, 0);
  ASSERT_TRUE(ring.push(7));
  std::vector<int> got;
  while (ring.pop(value)) {
    got.push_back(value);
  }
  ASSERT_EQ(got.size(), 7u);
  EXPECT_EQ(got.front(), 1);
  EXPECT_EQ(got.back(), 7);
}

TEST(SpscRingBuffer, PopLatestKeepsTheNewest) {
  fc::SpscRingBuffer<int, 8> ring;
  ASSERT_TRUE(ring.push(1));
  ASSERT_TRUE(ring.push(2));
  ASSERT_TRUE(ring.push(3));
  int value = 0;
  ASSERT_TRUE(ring.popLatest(value));
  EXPECT_EQ(value, 3);
  EXPECT_FALSE(ring.pop(value));
}

TEST(SpscRingBuffer, ThreadedTransferPreservesEverySample) {
  fc::SpscRingBuffer<std::uint64_t, 1024> ring;
  constexpr std::uint64_t kCount = 200000;
  std::uint64_t received = 0;
  std::thread producer([&] {
    for (std::uint64_t i = 0; i < kCount; ++i) {
      while (!ring.push(i)) {
        std::this_thread::yield();
      }
    }
  });
  std::thread consumer([&] {
    std::uint64_t expect = 0;
    std::uint64_t value = 0;
    while (expect < kCount) {
      if (ring.pop(value)) {
        ASSERT_EQ(value, expect);
        ++expect;
      } else {
        std::this_thread::yield();
      }
    }
    received = expect;
  });
  producer.join();
  consumer.join();
  EXPECT_EQ(received, kCount);
}

TEST(FixedRateLoop, PeriodIsPositive) {
  fc::FixedRateLoop loop(200.0);
  EXPECT_NEAR(loop.periodSeconds(), 0.005, 1.0e-9);
  double sum = 0.0;
  for (int i = 0; i < 4; ++i) {
    sum += loop.wait();
  }
  EXPECT_GT(sum, 0.01);
  EXPECT_LT(sum, 0.2);
}

TEST(QuadSim, HoverThrustHoldsAltitude) {
  const fc::VehicleParams vehicle;
  fc::QuadSimulator sim(vehicle);
  fc::MotorThrusts motors;
  const double each = vehicle.mass_kg * vehicle.gravity_m_s2 / 4.0;
  for (double& motor : motors.newton) {
    motor = each;
  }
  sim.setMotorThrusts(motors);
  for (int i = 0; i < 1000; ++i) {
    sim.step(0.001);
  }
  EXPECT_NEAR(-sim.state().position_ned_m.z, 0.0, 0.05);
  EXPECT_NEAR(sim.state().velocity_ned_m_s.z, 0.0, 0.05);
  EXPECT_NEAR(fc::norm(sim.state().omega_body_rad_s), 0.0, 1.0e-6);
}

TEST(QuadSim, FrontRightMotorHasTheExpectedTorqueSign) {
  fc::QuadSimulator sim;
  fc::MotorThrusts motors{};
  motors.newton[0] = 2.0;
  sim.setMotorThrusts(motors);
  sim.step(0.002);
  EXPECT_LT(sim.state().omega_body_rad_s.x, 0.0);
  EXPECT_GT(sim.state().omega_body_rad_s.y, 0.0);
  EXPECT_LT(sim.state().omega_body_rad_s.z, 0.0);
}

TEST(QuadSim, TiltedHoverAcceleratesEastWhileAccelReadsBodyUp) {
  const fc::VehicleParams vehicle;
  fc::QuadSimulator sim(vehicle);
  fc::SimState state;
  state.attitude = fc::Quaternion::fromEuler({0.25, 0.0, 0.0});
  sim.setState(state);
  fc::MotorThrusts motors;
  const double each = vehicle.mass_kg * vehicle.gravity_m_s2 / 4.0;
  for (double& motor : motors.newton) {
    motor = each;
  }
  sim.setMotorThrusts(motors);

  const fc::Vec3 specific = sim.specificForceBody();
  EXPECT_NEAR(specific.x, 0.0, 1.0e-8);
  EXPECT_NEAR(specific.y, 0.0, 1.0e-8);
  EXPECT_NEAR(specific.z, -vehicle.gravity_m_s2, 1.0e-8);

  const fc::Vec3 accel = sim.kinematicAccelerationWorld();
  EXPECT_GT(accel.y, 1.0);
  EXPECT_NEAR(accel.x, 0.0, 1.0e-8);
}

TEST(QuadSim, FreeFallSpecificForceIsZero) {
  fc::QuadSimulator sim;
  const fc::Vec3 specific = sim.specificForceBody();
  EXPECT_NEAR(specific.x, 0.0, 1.0e-12);
  EXPECT_NEAR(specific.y, 0.0, 1.0e-12);
  EXPECT_NEAR(specific.z, 0.0, 1.0e-12);
  const fc::Vec3 accel = sim.kinematicAccelerationWorld();
  EXPECT_NEAR(accel.z, sim.params().gravity_m_s2, 1.0e-12);
}
