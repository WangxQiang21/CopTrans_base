#include "fwis_planning_core/collision.hpp"

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>
#include <utility>

namespace py = pybind11;
namespace core = fwis_planning_core;

namespace
{
using OccupancyMask = py::array_t<bool, py::array::c_style | py::array::forcecast>;

core::CollisionKernel legacy_kernel(
  const OccupancyMask & occupied, double resolution, double origin_x, double origin_y,
  std::vector<core::Footprint> vehicles, std::optional<core::Footprint> cargo)
{
  if (occupied.ndim() != 2 || occupied.size() == 0) {
    throw std::invalid_argument("expected a nonempty two-dimensional occupancy grid");
  }
  const auto * begin = occupied.data();
  const core::GridSnapshot grid(
    static_cast<std::size_t>(occupied.shape(1)), static_cast<std::size_t>(occupied.shape(0)),
    resolution, {origin_x, origin_y}, std::vector<std::uint8_t>(begin, begin + occupied.size()));
  return core::CollisionKernel(grid, std::move(vehicles), cargo);
}
}  // namespace

PYBIND11_MODULE(_formation_collision_native, module)
{
  module.doc() = "Native per-state OBB collision kernel for the formation sweep";
  py::class_<core::CollisionKernel>(module, "CollisionKernel", py::module_local())
  .def(py::init(&legacy_kernel))
  .def("check", [](const core::CollisionKernel & kernel, double x, double y, double yaw,
      const std::vector<double> & headings, double padding) {
      const auto hit = kernel.check({x, y, yaw}, headings, padding);
      return py::make_tuple(
        core::reason_name(hit.reason), hit.first, hit.second, hit.map_queries, hit.pair_queries);
    })
  .def("check_spin_samples", [](const core::CollisionKernel & kernel, double x, double y,
      double yaw, const std::vector<double> & start_headings, const std::vector<double> & end_headings,
      double length, std::size_t intervals, double padding) {
      core::SpinSweepResult scan;
      {
        py::gil_scoped_release release;
        scan = kernel.check_spin_samples({x, y, yaw}, start_headings, end_headings,
            length, intervals, padding);
      }
      const auto & hit = scan.collision;
      return py::make_tuple(core::reason_name(hit.reason), hit.first, hit.second,
        hit.map_queries, hit.pair_queries, scan.samples_checked);
    })
  .def("check_edge_samples", [](const core::CollisionKernel & kernel, const std::string & geometry,
      const std::array<double, 3> & start, const std::array<double, 3> & end,
      const std::array<double, 2> & pivot, double length, const std::vector<double> & start_headings,
      const std::vector<double> & end_headings, std::size_t intervals, double padding) {
      core::SweepGeometry kind;
      if (geometry == "translation") {
        kind = core::SweepGeometry::translation;
      } else if (geometry == "rotation") {
        kind = core::SweepGeometry::rotation;
      } else if (geometry == "body_spin") {
        kind = core::SweepGeometry::stationary_spin;
      } else {
        throw std::invalid_argument("unsupported native analytic sweep geometry");
      }
      core::SweepResult scan;
      {
        py::gil_scoped_release release;
        scan = kernel.check_edge_samples({kind, {start[0], start[1], start[2]},
            {end[0], end[1], end[2]}, {pivot[0], pivot[1]}, length}, start_headings, end_headings,
            intervals, padding);
      }
      const auto & hit = scan.collision;
      return py::make_tuple(core::reason_name(hit.reason), hit.first, hit.second,
        hit.map_queries, hit.pair_queries, scan.samples_checked);
    });
}
