#include "fwis_planning_core/collision.hpp"
#include "fwis_planning_core/trajectory.hpp"

#include <benchmark/benchmark.h>

#include <array>
#include <cstddef>
#include <vector>

namespace
{
using namespace fwis_planning_core;

void advance_primitive(benchmark::State & state)
{
  const std::array<MotionPrimitive, 4> motions{{
    {Mode::ackermann, 1.0, 0.4}, {Mode::ackermann, -0.7, 0.0},
    {Mode::oblique, 0.8, 0.9}, {Mode::spin, 1.3, 0.0}}};
  for (auto _ : state) {
    for (const auto & motion : motions) {
      benchmark::DoNotOptimize(advance({1.0, -2.0, 0.45}, motion));
    }
  }
  state.SetItemsProcessed(state.iterations() * motions.size());
}
BENCHMARK(advance_primitive);

void joint_collision(benchmark::State & state)
{
  std::vector<std::uint8_t> cells(256 * 256);
  for (std::size_t row = 12; row < 128; ++row) {
    cells[row * 256 + 90] = 1;
  }
  const GridSnapshot grid(256, 256, 0.05, {0.0, 0.0}, cells);
  const CollisionKernel kernel(grid, {
    {1.0, 0.0, 0.788, 0.55}, {0.0, 1.0, 0.788, 0.55}, {0.0, -1.0, 0.788, 0.55}},
    Footprint{0.5, 0.0, 2.0, 2.75});
  const std::array<double, 3> headings{0.1, 0.5, -0.4};
  for (auto _ : state) {
    for (std::size_t i = 0; i < 100; ++i) {
      const Pose2d reference{2.0 + 0.05 * i, 3.0, 0.02 * i};
      benchmark::DoNotOptimize(kernel.check(reference, headings.data(), headings.size(), 0.01));
    }
  }
  state.SetItemsProcessed(state.iterations() * 100);
}
BENCHMARK(joint_collision);

void sample_trajectory(benchmark::State & state)
{
  const TimedPrimitive trajectory({1.0, -2.0, 0.45}, {Mode::ackermann, 2.5, 0.35}, 8.0);
  for (auto _ : state) {
    for (std::size_t i = 0; i < 100; ++i) {
      benchmark::DoNotOptimize(trajectory.sample(0.08 * i));
    }
  }
  state.SetItemsProcessed(state.iterations() * 100);
}
BENCHMARK(sample_trajectory);
}  // namespace

BENCHMARK_MAIN();
