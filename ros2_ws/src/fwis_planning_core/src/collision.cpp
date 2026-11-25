#include "fwis_planning_core/collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace fwis_planning_core
{
namespace
{
bool finite_nonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

void validate_shape(const Footprint & footprint)
{
  if (!std::all_of(footprint.begin(), footprint.end(), [](double x) {return std::isfinite(x);}) ||
    footprint[2] <= 0.0 || footprint[3] <= 0.0)
  {
    throw std::invalid_argument("footprint geometry must be finite with positive dimensions");
  }
}

std::vector<double> cell_centers(double origin, double resolution, std::size_t count)
{
  std::vector<double> centers;
  centers.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const double center = origin + (static_cast<double>(index) + 0.5) * resolution;
    if (!std::isfinite(center) || (!centers.empty() && center <= centers.back())) {
      throw std::invalid_argument("occupancy grid requires distinct cell centers");
    }
    centers.push_back(center);
  }
  return centers;
}
}  // namespace

GridSnapshot::GridSnapshot(
  std::size_t width, std::size_t height, double resolution, Point2d origin,
  std::vector<std::uint8_t> occupancy, std::uint64_t version, std::string hash)
: width_(width), height_(height), resolution_(resolution), origin_(origin),
  occupancy_(std::move(occupancy)), version_(version), hash_(std::move(hash))
{
  if (width_ == 0 || height_ == 0 || height_ > std::numeric_limits<std::size_t>::max() / width_ ||
    occupancy_.size() != width_ * height_ || !std::isfinite(resolution_) || resolution_ <= 0.0 ||
    !std::isfinite(origin_.x) || !std::isfinite(origin_.y))
  {
    throw std::invalid_argument("expected a non-empty occupancy grid with finite geometry");
  }
  if (!std::isfinite(origin_.x + width_ * resolution_) ||
    !std::isfinite(origin_.y + height_ * resolution_))
  {
    throw std::invalid_argument("occupancy grid extents must be finite");
  }
}

bool GridSnapshot::occupied(std::size_t column, std::size_t row) const noexcept
{
  return row >= height_ || column >= width_ || occupancy_[row * width_ + column] != 0;
}

const char * reason_name(CollisionReason reason) noexcept
{
  switch (reason) {
    case CollisionReason::clear:
      return "";
    case CollisionReason::nonfinite_center:
      return "NONFINITE_CENTER";
    case CollisionReason::cargo_map:
      return "CARGO_OBB_COLLISION";
    case CollisionReason::vehicle_map:
      return "MAP_OBB_COLLISION";
    case CollisionReason::vehicle_pair:
      return "VEHICLE_OBB_COLLISION";
  }
  return "INVALID_COLLISION_REASON";
}

CollisionKernel::CollisionKernel(
  const GridSnapshot & grid, std::vector<Footprint> vehicles, std::optional<Footprint> cargo)
: resolution_(grid.resolution()), origin_x_(grid.origin().x), origin_y_(grid.origin().y),
  right_(origin_x_ + grid.width() * resolution_), top_(origin_y_ + grid.height() * resolution_),
  vehicles_(std::move(vehicles)), cargo_(cargo), occupied_columns_(grid.height()),
  centers_x_(cell_centers(origin_x_, resolution_, grid.width())),
  centers_y_(cell_centers(origin_y_, resolution_, grid.height()))
{
  if (vehicles_.empty()) {
    throw std::invalid_argument("collision kernel requires at least one vehicle");
  }
  for (const auto & vehicle : vehicles_) {
    validate_shape(vehicle);
  }
  if (cargo_) {
    validate_shape(*cargo_);
  }
  for (std::size_t row = 0; row < grid.height(); ++row) {
    auto & columns = occupied_columns_[row];
    for (std::size_t column = 0; column < grid.width(); ++column) {
      if (grid.occupied(column, row)) {
        columns.push_back(column);
      }
    }
  }
}

CollisionKernel::Rectangle CollisionKernel::place(
  const Footprint & footprint, const Transform2d & transform, double heading)
{
  const auto center = transform.apply({footprint[0], footprint[1]});
  return {center.x, center.y, std::cos(heading), std::sin(heading),
    footprint[2] / 2.0, footprint[3] / 2.0};
}

bool CollisionKernel::overlap(const Rectangle & first, const Rectangle & second, double padding)
{
  const double dx = second.x - first.x;
  const double dy = second.y - first.y;
  const std::array<std::array<double, 2>, 4> axes{{
    {first.c, first.s}, {-first.s, first.c},
    {second.c, second.s}, {-second.s, second.c}}};
  for (const auto & [ax, ay] : axes) {
    const double r1 = (first.half_length + padding) * std::abs(first.c * ax + first.s * ay) +
      (first.half_width + padding) * std::abs(-first.s * ax + first.c * ay);
    const double r2 = (second.half_length + padding) * std::abs(second.c * ax + second.s * ay) +
      (second.half_width + padding) * std::abs(-second.s * ax + second.c * ay);
    if (std::abs(dx * ax + dy * ay) > r1 + r2) {
      return false;
    }
  }
  return true;
}

bool CollisionKernel::map_clear(const Rectangle & pose, const Footprint & shape, double padding) const
{
  // Map and vehicle-pair extents use the same operation order as their
  // respective Python checks, including rejection of contact at map edges.
  const double hl = (shape[2] + 2.0 * padding) / 2.0;
  const double hw = (shape[3] + 2.0 * padding) / 2.0;
  const double abs_c = std::abs(pose.c);
  const double abs_s = std::abs(pose.s);
  const double rx = hl * abs_c + hw * abs_s;
  const double ry = hl * abs_s + hw * abs_c;
  if (!std::isfinite(rx) || !std::isfinite(ry) ||
    pose.x - rx - origin_x_ <= 0.0 || pose.y - ry - origin_y_ <= 0.0 ||
    right_ - pose.x - rx <= 0.0 || top_ - pose.y - ry <= 0.0)
  {
    return false;
  }
  // Include a cell beyond each bounding range so an exactly touching cell
  // survives the index conversion. SAT makes the final decision.
  const auto bounded_index = [this](double x, double origin, double extra, std::size_t count) {
      return static_cast<std::size_t>(std::clamp(
          std::floor((x - origin) / resolution_) + extra, 0.0,
          static_cast<double>(count - 1)));
    };
  const auto first_col = bounded_index(pose.x - rx, origin_x_, -1.0, centers_x_.size());
  const auto last_col = bounded_index(pose.x + rx, origin_x_, 1.0, centers_x_.size());
  const auto first_row = bounded_index(pose.y - ry, origin_y_, -1.0, centers_y_.size());
  const auto last_row = bounded_index(pose.y + ry, origin_y_, 1.0, centers_y_.size());
  const double cell_half = resolution_ / 2.0;
  const double pad = cell_half * (abs_c + abs_s);
  for (auto row = first_row; row <= last_row; ++row) {
    const double dy = centers_y_[row] - pose.y;
    if (std::abs(dy) > cell_half + ry) {
      continue;
    }
    const auto & cols = occupied_columns_[row];
    for (auto col = std::lower_bound(cols.begin(), cols.end(), first_col);
      col != cols.end() && *col <= last_col; ++col)
    {
      const double dx = centers_x_[*col] - pose.x;
      if (std::abs(dx) <= cell_half + rx &&
        std::abs(dx * pose.c + dy * pose.s) <= hl + pad &&
        std::abs(-dx * pose.s + dy * pose.c) <= hw + pad)
      {
        return false;
      }
    }
  }
  return true;
}

CollisionResult CollisionKernel::check(
  Pose2d reference, const double * headings, std::size_t count, double padding_m) const
{
  if (headings == nullptr || count != vehicles_.size()) {
    throw std::invalid_argument("one heading is required per vehicle");
  }
  if (!finite_nonnegative(padding_m)) {
    throw std::invalid_argument("collision padding must be finite and nonnegative");
  }

  const Transform2d transform(reference);
  std::array<Rectangle, 3> local_poses;
  std::vector<Rectangle> extra_poses;
  auto * poses = local_poses.data();
  if (vehicles_.size() > local_poses.size()) {
    extra_poses.resize(vehicles_.size());
    poses = extra_poses.data();
  }
  for (std::size_t i = 0; i < vehicles_.size(); ++i) {
    if (!std::isfinite(headings[i])) {
      throw std::invalid_argument("vehicle headings must be finite");
    }
    poses[i] = place(vehicles_[i], transform, headings[i]);
    if (!std::isfinite(poses[i].x) || !std::isfinite(poses[i].y)) {
      return {CollisionReason::nonfinite_center, i};
    }
  }
  CollisionResult result;
  if (cargo_) {
    ++result.map_queries;
    const auto cargo = place(*cargo_, transform, reference.yaw);
    if (!std::isfinite(cargo.x) || !std::isfinite(cargo.y) ||
      !map_clear(cargo, *cargo_, padding_m))
    {
      result.reason = CollisionReason::cargo_map;
      return result;
    }
  }
  for (std::size_t i = 0; i < count; ++i) {
    ++result.map_queries;
    if (!map_clear(poses[i], vehicles_[i], padding_m)) {
      result.reason = CollisionReason::vehicle_map;
      result.first = i;
      return result;
    }
  }
  for (std::size_t i = 0; i < count; ++i) {
    for (std::size_t j = i + 1; j < count; ++j) {
      ++result.pair_queries;
      if (overlap(poses[i], poses[j], padding_m)) {
        result.reason = CollisionReason::vehicle_pair;
        result.first = i;
        result.second = j;
        return result;
      }
    }
  }
  return result;
}

CollisionResult CollisionKernel::check(
  Pose2d reference, const std::vector<double> & headings, double padding_m) const
{
  return check(reference, headings.data(), headings.size(), padding_m);
}

SpinSweepResult CollisionKernel::check_spin_samples(
  Pose2d reference, const std::vector<double> & start_headings,
  const std::vector<double> & end_headings, double edge_length_rad,
  std::size_t intervals, double padding_m) const
{
  return check_edge_samples(
    {SweepGeometry::stationary_spin, reference, reference, {}, edge_length_rad},
    start_headings, end_headings, intervals, padding_m);
}

SweepResult CollisionKernel::check_edge_samples(
  const AnalyticSweepEdge & edge, const std::vector<double> & start_headings,
  const std::vector<double> & end_headings, std::size_t intervals, double padding_m) const
{
  const std::size_t count = vehicle_count();
  if (start_headings.size() != count || end_headings.size() != count || intervals == 0 ||
    !std::isfinite(edge.length) || edge.length <= 0.0)
  {
    throw std::invalid_argument("invalid analytic sweep sample parameters");
  }
  std::vector<double> headings(count);
  std::vector<double> deltas(count);
  for (std::size_t i = 0; i < count; ++i) {
    deltas[i] = end_headings[i] - start_headings[i];
    if (!std::isfinite(deltas[i])) {
      throw std::invalid_argument("body heading change must be finite");
    }
  }

  CollisionResult result;
  const Point2d world_pivot = formation_center(edge.start, edge.pivot);
  const double delta_yaw = edge.end.yaw - edge.start.yaw;
  const double turn_sign = delta_yaw > 0.0 ? 1.0 : -1.0;
  for (std::size_t index = 0; index < intervals; ++index) {
    const double fraction = static_cast<double>(index + 1) / static_cast<double>(intervals);
    const double progress = fraction * edge.length;
    const double model_fraction = progress / edge.length;
    Pose2d reference = edge.start;
    switch (edge.geometry) {
      case SweepGeometry::translation:
        reference.x += model_fraction * (edge.end.x - edge.start.x);
        reference.y += model_fraction * (edge.end.y - edge.start.y);
        headings = start_headings;
        break;
      case SweepGeometry::rotation:
        reference = reference_from_pivot(
          world_pivot, edge.pivot, edge.start.yaw + model_fraction * delta_yaw);
        for (std::size_t i = 0; i < count; ++i) {
          headings[i] = start_headings[i] + turn_sign * progress;
        }
        break;
      case SweepGeometry::stationary_spin:
        for (std::size_t i = 0; i < count; ++i) {
          headings[i] = start_headings[i] + deltas[i] * progress / edge.length;
        }
        break;
      default:
        throw std::invalid_argument("unsupported analytic sweep geometry");
    }
    const auto sample = check(reference, headings, padding_m);
    result.map_queries += sample.map_queries;
    result.pair_queries += sample.pair_queries;
    if (sample.reason != CollisionReason::clear) {
      result.reason = sample.reason;
      result.first = sample.first;
      result.second = sample.second;
      return {result, index + 1};
    }
  }
  return {result, intervals};
}

SweepDiscretization CollisionKernel::sweep_discretization(
  double center_bound_m, double formation_yaw_bound_rad,
  const std::vector<double> & body_yaw_bounds_rad, double max_corner_step_m,
  std::size_t max_intervals) const
{
  if (body_yaw_bounds_rad.size() != vehicles_.size() ||
    !finite_nonnegative(center_bound_m) || !finite_nonnegative(formation_yaw_bound_rad) ||
    !finite_nonnegative(max_corner_step_m) || max_corner_step_m == 0.0 || max_intervals == 0 ||
    !std::all_of(body_yaw_bounds_rad.begin(), body_yaw_bounds_rad.end(), finite_nonnegative))
  {
    throw std::invalid_argument("invalid sweep bounds or sampling policy");
  }
  double bound = 0.0;
  if (cargo_) {
    const double radius = std::hypot(
      std::abs((*cargo_)[0]) + (*cargo_)[2] / 2.0,
      std::abs((*cargo_)[1]) + (*cargo_)[3] / 2.0);
    bound = center_bound_m + radius * formation_yaw_bound_rad;
  }

  for (std::size_t i = 0; i < vehicles_.size(); ++i) {
    const auto & vehicle = vehicles_[i];
    const double center = center_bound_m +
      std::hypot(vehicle[0], vehicle[1]) * formation_yaw_bound_rad;
    const double rotation = std::hypot(vehicle[2] / 2.0, vehicle[3] / 2.0) * body_yaw_bounds_rad[i];
    const double vehicle_bound = center + rotation;
    if (!std::isfinite(vehicle_bound)) {
      throw std::length_error("sweep displacement bound is not finite");
    }
    bound = std::max(bound, vehicle_bound);
  }

  const double intervals = std::max(1.0, std::ceil(bound / max_corner_step_m));
  if (!std::isfinite(intervals) ||
    static_cast<long double>(intervals) > static_cast<long double>(max_intervals))
  {
    throw std::length_error("sweep sample budget exceeded");
  }
  return {bound, max_corner_step_m, static_cast<std::size_t>(intervals)};
}
}  // namespace fwis_planning_core
