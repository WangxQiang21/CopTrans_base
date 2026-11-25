#pragma once

#include "fwis_planning_core/geometry.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace fwis_planning_core
{
class GridSnapshot
{
public:
  // Axis-aligned, row-major occupancy. Cell (0,0) starts at origin in metres.
  // Every nonzero cell, including an unknown (-1 cast to uint8_t), is occupied.
  GridSnapshot(
    std::size_t width, std::size_t height, double resolution, Point2d origin,
    std::vector<std::uint8_t> occupancy, std::uint64_t version = 0, std::string hash = {});

  std::size_t width() const noexcept {return width_;}
  std::size_t height() const noexcept {return height_;}
  double resolution() const noexcept {return resolution_;}
  Point2d origin() const noexcept {return origin_;}
  std::uint64_t version() const noexcept {return version_;}
  const std::string & hash() const noexcept {return hash_;}
  bool occupied(std::size_t column, std::size_t row) const noexcept;

private:
  std::size_t width_, height_;
  double resolution_;
  Point2d origin_;
  std::vector<std::uint8_t> occupancy_;
  std::uint64_t version_;
  std::string hash_;
};

// Offset x/y from the formation reference, then length and width including
// the fixed margin. All values are in metres.
using Footprint = std::array<double, 4>;

enum class CollisionReason
{
  clear = 0,
  nonfinite_center = 1,
  cargo_map = 2,
  vehicle_map = 3,
  vehicle_pair = 4,
};

const char * reason_name(CollisionReason reason) noexcept;

struct CollisionResult
{
  CollisionReason reason = CollisionReason::clear;
  std::size_t first = 0;  // Vehicle indices refer to the constructor order.
  std::size_t second = 0;
  std::size_t map_queries = 0;
  std::size_t pair_queries = 0;
};

struct SweepDiscretization
{
  double corner_bound_m;
  double padding_m;
  std::size_t intervals;  // Evaluate interval endpoints i/N, i=0...N.
};

enum class SweepGeometry {translation, rotation, stationary_spin};

struct AnalyticSweepEdge
{
  SweepGeometry geometry;
  Pose2d start;
  Pose2d end;
  Point2d pivot;  // local F coordinates, used for a fixed-pivot rotation
  double length;  // metres for translations, radians for rotations and spin
};

struct SweepResult
{
  // Accumulates query counts through the first failure, if any.
  CollisionResult collision;
  std::size_t samples_checked;
};

using SpinSweepResult = SweepResult;

class CollisionKernel
{
public:
  // Owns the row index rather than referencing the input map. Const queries
  // are thread safe. No geometry payload is modified after construction.
  CollisionKernel(
    const GridSnapshot & grid, std::vector<Footprint> vehicles,
    std::optional<Footprint> cargo = std::nullopt);

  std::size_t vehicle_count() const noexcept {return vehicles_.size();}

  // Tests cargo vs map, then vehicle vs map, then unordered vehicle pairs.
  // Contact counts as collision. Body headings are independent of reference yaw.
  CollisionResult check(
    Pose2d reference, const double * headings, std::size_t count, double padding_m = 0.0) const;
  CollisionResult check(
    Pose2d reference, const std::vector<double> & headings, double padding_m = 0.0) const;

  // Exact samples 1..intervals of a stationary reference spin. Body yaw is
  // interpolated over the unwrapped edge length (in rad). The caller checks
  // sample zero and supplies certified subdivision and padding as above.
  SpinSweepResult check_spin_samples(
    Pose2d reference, const std::vector<double> & start_headings,
    const std::vector<double> & end_headings, double edge_length_rad,
    std::size_t intervals, double padding_m) const;

  // The caller validates an authoritative edge and checks sample zero. Test
  // exact states 1..intervals with its existing conservative subdivision and
  // padding. Return the first witness and accumulated query count.
  SweepResult check_edge_samples(
    const AnalyticSweepEdge & edge, const std::vector<double> & start_headings,
    const std::vector<double> & end_headings, std::size_t intervals, double padding_m) const;

  // Inputs are upper bounds per normalized segment fraction. Each caller
  // must justify them and evaluate the exact continuous joint model rather
  // than interpolating isolated states. Exceeding the budget throws.
  SweepDiscretization sweep_discretization(
    double center_bound_m, double formation_yaw_bound_rad,
    const std::vector<double> & body_yaw_bounds_rad, double max_corner_step_m,
    std::size_t max_intervals) const;

private:
  struct Rectangle
  {
    double x, y, c, s, half_length, half_width;
  };

  static Rectangle place(
    const Footprint & footprint, const Transform2d & transform, double heading);
  static bool overlap(const Rectangle & first, const Rectangle & second, double padding);
  bool map_clear(const Rectangle & pose, const Footprint & shape, double padding) const;

  double resolution_, origin_x_, origin_y_, right_, top_;
  std::vector<Footprint> vehicles_;
  std::optional<Footprint> cargo_;
  std::vector<std::vector<std::size_t>> occupied_columns_;
  std::vector<double> centers_x_, centers_y_;
};
}  // namespace fwis_planning_core
