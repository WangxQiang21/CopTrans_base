#include "fwis_planning_core/collision.hpp"
#include "fwis_planning_core/trajectory.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
using namespace fwis_planning_core;
const double pi = std::acos(-1.0);

TEST(Geometry, RigidOffsetsAreRotatedAboutFormationReference)
{
  const auto center = formation_center({2.0, 3.0, pi / 2.0}, {1.0, -0.5});
  EXPECT_NEAR(center.x, 2.5, 1e-15);
  EXPECT_NEAR(center.y, 4.0, 1e-15);
}

TEST(Geometry, PrimitiveModesKeepDirectionAndBodyHeadingContracts)
{
  const auto straight = advance({3.0, 4.0, pi / 2.0}, {Mode::ackermann, -2.0, 0.0});
  EXPECT_NEAR(straight.x, 3.0, 1e-15);
  EXPECT_DOUBLE_EQ(straight.y, 2.0);
  EXPECT_DOUBLE_EQ(straight.yaw, pi / 2.0);

  const auto arc = advance({0.0, 0.0, 0.0}, {Mode::ackermann, pi, 0.5});
  EXPECT_NEAR(arc.x, 2.0, 1e-15);
  EXPECT_NEAR(arc.y, 2.0, 1e-15);
  EXPECT_DOUBLE_EQ(arc.yaw, pi / 2.0);
  const auto back = advance(arc, {Mode::ackermann, -pi, 0.5});
  EXPECT_NEAR(back.x, 0.0, 1e-15);
  EXPECT_NEAR(back.y, 0.0, 1e-15);
  EXPECT_DOUBLE_EQ(back.yaw, 0.0);

  const auto oblique = advance({0.0, 0.0, 0.0}, {Mode::oblique, -2.0, pi / 2.0});
  EXPECT_NEAR(oblique.x, 0.0, 1e-15);
  EXPECT_DOUBLE_EQ(oblique.y, -2.0);
  EXPECT_DOUBLE_EQ(oblique.yaw, 0.0);

  const auto spin = advance({2.0, 3.0, pi}, {Mode::spin, pi / 2.0, 0.0});
  EXPECT_DOUBLE_EQ(spin.x, 2.0);
  EXPECT_DOUBLE_EQ(spin.y, 3.0);
  EXPECT_NEAR(spin.yaw, -pi / 2.0, 1e-15);
}

TEST(Timing, QuinticKeepsStableEndpoints)
{
  const auto start = quintic_progress(0.0);
  const auto mid = quintic_progress(0.5);
  const auto end = quintic_progress(1.0);
  EXPECT_DOUBLE_EQ(start.position, 0.0);
  EXPECT_DOUBLE_EQ(start.first, 0.0);
  EXPECT_DOUBLE_EQ(start.second, 0.0);
  EXPECT_DOUBLE_EQ(mid.position, 0.5);
  EXPECT_DOUBLE_EQ(mid.first, 1.875);
  EXPECT_DOUBLE_EQ(end.position, 1.0);
  EXPECT_DOUBLE_EQ(end.first, 0.0);
  EXPECT_DOUBLE_EQ(end.second, 0.0);
  EXPECT_LE(quintic_progress(std::nextafter(1.0, 0.0)).position, 1.0);
}

TEST(Timing, ExactTimedPrimitiveKeepsBodyTwistAndStops)
{
  const TimedPrimitive primitive({0.0, 0.0, 0.0}, {Mode::ackermann, -2.0, 0.5}, 4.0);
  const auto sample = primitive.sample(2.0);
  EXPECT_DOUBLE_EQ(sample.twist.vx, -0.9375);
  EXPECT_DOUBLE_EQ(sample.twist.vy, 0.0);
  EXPECT_DOUBLE_EQ(sample.twist.wz, -0.46875);
  EXPECT_DOUBLE_EQ(sample.pose.yaw, -0.5);
  const auto end = primitive.sample(4.0);
  EXPECT_DOUBLE_EQ(end.pose.yaw, -1.0);
  EXPECT_DOUBLE_EQ(end.twist.vx, 0.0);
  EXPECT_THROW(primitive.sample(-1.0), std::invalid_argument);
  EXPECT_THROW(primitive.sample(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(Collision, OccupancyUnknownAndMapContactAreClosedBounds)
{
  std::vector<std::uint8_t> cells(32 * 32);
  cells[4 * 32 + 8] = 255;
  const GridSnapshot grid(32, 32, 0.125, {0.0, 0.0}, cells, 37, "fixture");
  EXPECT_EQ(grid.version(), 37U);
  EXPECT_EQ(grid.hash(), "fixture");
  EXPECT_TRUE(grid.occupied(8, 4));
  EXPECT_TRUE(grid.occupied(32, 4));
  const CollisionKernel kernel(grid, {{0.0, 0.0, 0.5, 0.25}});
  EXPECT_EQ(kernel.check({0.25, 0.5625, 0.0}, {0.0}).reason, CollisionReason::vehicle_map);
  EXPECT_EQ(kernel.check({std::nextafter(0.25, 1.0), 0.5625, 0.0}, {0.0}).reason,
    CollisionReason::clear);
  EXPECT_EQ(kernel.check({0.75, 0.5625, 0.0}, {0.0}).reason, CollisionReason::vehicle_map);
  EXPECT_EQ(kernel.check({0.75, 0.375, 0.0}, {0.0}).reason, CollisionReason::vehicle_map);
  EXPECT_EQ(kernel.check({std::nextafter(0.75, 0.0), 0.5625, 0.0}, {0.0}).reason,
    CollisionReason::clear);
}

TEST(Collision, PairAndCargoWitnessesFollowOldDiagnosticOrder)
{
  const std::vector<Footprint> vehicles{{0.0, 0.0, 0.5, 0.25}, {0.5, 0.0, 0.5, 0.25}};
  std::vector<std::uint8_t> cells(32 * 32);
  const CollisionKernel no_obstacles(GridSnapshot(32, 32, 0.125, {0.0, 0.0}, cells), vehicles);
  const auto pair_result = no_obstacles.check({2.0, 2.0, 0.0}, {0.0, 0.0});
  EXPECT_EQ(pair_result.reason, CollisionReason::vehicle_pair);
  EXPECT_EQ(pair_result.first, 0U);
  EXPECT_EQ(pair_result.second, 1U);
  EXPECT_EQ(pair_result.map_queries, 2U);
  EXPECT_EQ(pair_result.pair_queries, 1U);

  cells[16 * 32 + 16] = 1;
  const CollisionKernel cargo_kernel(GridSnapshot(32, 32, 0.125, {0.0, 0.0}, cells),
    vehicles, Footprint{0.0, 0.0, 0.75, 0.5});
  const auto cargo_result = cargo_kernel.check({2.0, 2.0, 0.0}, {0.0, 0.0});
  EXPECT_EQ(cargo_result.reason, CollisionReason::cargo_map);
  EXPECT_EQ(cargo_result.map_queries, 1U);
  EXPECT_EQ(cargo_result.pair_queries, 0U);
}

TEST(Collision, ConservativeSweepingFindsInteriorCargoContact)
{
  std::vector<std::uint8_t> cells(32 * 32);
  cells[16 * 32 + 16] = 1;
  const CollisionKernel kernel(GridSnapshot(32, 32, 0.125, {0.0, 0.0}, cells),
    {{0.0, 0.0, 0.25, 0.25}}, Footprint{0.5, 0.0, 0.5, 0.5});
  const auto plan = kernel.sweep_discretization(2.0, 0.0, {0.0}, 0.125, 32);
  EXPECT_DOUBLE_EQ(plan.corner_bound_m, 2.0);
  EXPECT_DOUBLE_EQ(plan.padding_m, 0.125);
  EXPECT_EQ(plan.intervals, 16U);
  EXPECT_THROW(kernel.sweep_discretization(2.0, 0.0, {0.0}, 0.125, 1), std::length_error);
  EXPECT_EQ(kernel.check({1.0, 2.0, 0.0}, {0.0}, plan.padding_m).reason, CollisionReason::clear);
  EXPECT_EQ(kernel.check({3.0, 2.0, 0.0}, {0.0}, plan.padding_m).reason, CollisionReason::clear);
  std::size_t first_hit = 0;
  for (std::size_t i = 1; i < plan.intervals; ++i) {
    const double fraction = static_cast<double>(i) / plan.intervals;
    const auto hit = kernel.check({1.0 + fraction * 2.0, 2.0, 0.0}, {0.0}, plan.padding_m);
    if (hit.reason != CollisionReason::clear) {
      EXPECT_EQ(hit.reason, CollisionReason::cargo_map);
      first_hit = i;
      break;
    }
  }
  EXPECT_GT(first_hit, 0U);
}

TEST(Collision, LargeFormationsAndRejectingInvalidInputs)
{
  const std::vector<std::uint8_t> cells(64 * 64);
  const GridSnapshot grid(64, 64, 0.125, {0.0, 0.0}, cells);
  const CollisionKernel kernel(grid, {
    {-1.0, 0.0, 0.5, 0.3}, {0.0, -1.0, 0.5, 0.3},
    {0.0, 1.0, 0.5, 0.3}, {1.0, 0.0, 0.5, 0.3}});
  const auto clear = kernel.check({4.0, 4.0, 0.0}, {0.0, 0.0, 0.0, 0.0});
  EXPECT_EQ(clear.reason, CollisionReason::clear);
  EXPECT_EQ(clear.pair_queries, 6U);
  EXPECT_THROW(kernel.check({4.0, 4.0, 0.0}, {0.0}), std::invalid_argument);
  EXPECT_THROW(
    kernel.check({4.0, 4.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, -1.0), std::invalid_argument);
  EXPECT_THROW(GridSnapshot(1, 1, 1.0, {0.0, 0.0}, {}), std::invalid_argument);
}

TEST(Collision, StationarySpinBatchPreservesInteriorContact)
{
  std::vector<std::uint8_t> cells(32 * 32);
  cells[16 * 32 + 20] = 1;
  const CollisionKernel kernel(GridSnapshot(32, 32, 0.125, {0.0, 0.0}, cells),
    {{0.0, 0.0, 1.2, 0.15}});
  const Pose2d center{2.0, 2.0, 0.0};
  const auto plan = kernel.sweep_discretization(0.0, 0.0, {pi / 2.0}, 0.01, 100);
  EXPECT_EQ(kernel.check(center, {-pi / 4.0}, plan.padding_m).reason, CollisionReason::clear);
  EXPECT_EQ(kernel.check(center, {pi / 4.0}, plan.padding_m).reason, CollisionReason::clear);
  const auto result = kernel.check_spin_samples(
    center, {-pi / 4.0}, {pi / 4.0}, pi / 2.0, plan.intervals, plan.padding_m);
  EXPECT_EQ(result.collision.reason, CollisionReason::vehicle_map);
  EXPECT_LT(result.samples_checked, plan.intervals);
  EXPECT_EQ(result.collision.map_queries, result.samples_checked);
}
}  // namespace
