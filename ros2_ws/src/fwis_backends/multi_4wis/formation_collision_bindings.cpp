#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace
{
// Layout: offset x/y in F, then length and width including the base margin.
using Footprint = std::array<double, 4>;

struct Rectangle
{
  double x, y, c, s, half_length, half_width;
};

struct Result
{
  const char * reason = "";
  std::size_t first = 0;
  std::size_t second = 0;
  std::size_t map_queries = 0;
  std::size_t pair_queries = 0;
};

Rectangle place(
  const Footprint & footprint, double x, double y, double c, double s, double heading)
{
  return {x + c * footprint[0] - s * footprint[1],
    y + s * footprint[0] + c * footprint[1], std::cos(heading), std::sin(heading),
    footprint[2] / 2.0, footprint[3] / 2.0};
}

bool overlap(const Rectangle & first, const Rectangle & second, double padding)
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

class CollisionKernel
{
public:
  CollisionKernel(
    const py::array_t<bool, py::array::c_style | py::array::forcecast> & occupied,
    double resolution, double origin_x, double origin_y,
    std::vector<Footprint> vehicles, std::optional<Footprint> cargo)
  : resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y),
    vehicles_(std::move(vehicles)), cargo_(cargo)
  {
    if (occupied.ndim() != 2 || occupied.shape(0) == 0 || occupied.shape(1) == 0 ||
      !std::isfinite(resolution_) || resolution_ <= 0.0 ||
      !std::isfinite(origin_x_) || !std::isfinite(origin_y_))
    {
      throw std::invalid_argument("expected a non-empty occupancy grid with finite geometry");
    }
    const auto mask = occupied.unchecked<2>();
    occupied_columns_.resize(mask.shape(0));
    centers_x_.reserve(mask.shape(1));
    centers_y_.reserve(mask.shape(0));
    for (py::ssize_t col = 0; col < mask.shape(1); ++col) {
      centers_x_.push_back(origin_x_ + (col + 0.5) * resolution_);
    }
    for (py::ssize_t row = 0; row < mask.shape(0); ++row) {
      auto & columns = occupied_columns_[row];
      centers_y_.push_back(origin_y_ + (row + 0.5) * resolution_);
      for (py::ssize_t col = 0; col < mask.shape(1); ++col) {
        if (mask(row, col)) {
          columns.push_back(col);
        }
      }
    }
    right_ = origin_x_ + mask.shape(1) * resolution_;
    top_ = origin_y_ + mask.shape(0) * resolution_;
    if (!std::isfinite(right_) || !std::isfinite(top_)) {
      throw std::invalid_argument("occupancy grid extents must be finite");
    }
  }

  std::tuple<const char *, std::size_t, std::size_t, std::size_t, std::size_t> check(
    double x, double y, double yaw, const std::vector<double> & headings, double padding) const
  {
    if (headings.size() != vehicles_.size()) {
      throw std::invalid_argument("one heading is required per vehicle");
    }
    const Result hit = first_collision(x, y, yaw, headings, padding);
    return {hit.reason, hit.first, hit.second, hit.map_queries, hit.pair_queries};
  }

private:
  bool map_clear(const Rectangle & pose, const Footprint & shape, double padding) const
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
        return static_cast<py::ssize_t>(std::clamp(
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

  Result first_collision(
    double x, double y, double yaw, const std::vector<double> & headings, double padding) const
  {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    std::vector<Rectangle> poses;
    poses.reserve(vehicles_.size());
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
      poses.push_back(place(vehicles_[i], x, y, c, s, headings[i]));
      if (!std::isfinite(poses.back().x) || !std::isfinite(poses.back().y)) {
        return {"NONFINITE_CENTER", i};
      }
    }
    Result result;
    if (cargo_) {
      ++result.map_queries;
      const auto cargo = place(*cargo_, x, y, c, s, yaw);
      if (!std::isfinite(cargo.x) || !std::isfinite(cargo.y) ||
        !map_clear(cargo, *cargo_, padding))
      {
        result.reason = "CARGO_OBB_COLLISION";
        return result;
      }
    }
    for (std::size_t i = 0; i < poses.size(); ++i) {
      ++result.map_queries;
      if (!map_clear(poses[i], vehicles_[i], padding)) {
        result.reason = "MAP_OBB_COLLISION";
        result.first = i;
        return result;
      }
    }
    for (std::size_t i = 0; i < poses.size(); ++i) {
      for (std::size_t j = i + 1; j < poses.size(); ++j) {
        ++result.pair_queries;
        if (overlap(poses[i], poses[j], padding)) {
          result.reason = "VEHICLE_OBB_COLLISION";
          result.first = i;
          result.second = j;
          return result;
        }
      }
    }
    return result;
  }

  double resolution_, origin_x_, origin_y_, right_, top_;
  std::vector<Footprint> vehicles_;
  std::optional<Footprint> cargo_;
  std::vector<std::vector<py::ssize_t>> occupied_columns_;
  std::vector<double> centers_x_, centers_y_;
};
}  // namespace

PYBIND11_MODULE(_formation_collision_native, module)
{
  module.doc() = "Native per-state OBB collision kernel for the formation sweep";
  py::class_<CollisionKernel>(module, "CollisionKernel")
  .def(py::init<const py::array_t<bool, py::array::c_style | py::array::forcecast> &,
    double, double, double, std::vector<Footprint>, std::optional<Footprint>>())
  .def("check", &CollisionKernel::check);
}
