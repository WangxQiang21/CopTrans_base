#include "fwis_planning_core/collision.hpp"
#include "fwis_planning_core/trajectory.hpp"

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <stdexcept>
#include <utility>

namespace py = pybind11;
namespace core = fwis_planning_core;

namespace
{
using Doubles = py::array_t<double, py::array::c_style | py::array::forcecast>;
using OccupancyMask = py::array_t<bool, py::array::c_style | py::array::forcecast>;
using Results = py::array_t<std::size_t>;

core::MotionPrimitive primitive(double mode, double amount, double control)
{
  if (mode != 0.0 && mode != 1.0 && mode != 2.0) {
    throw std::invalid_argument("primitive mode must be 0, 1 or 2");
  }
  return {static_cast<core::Mode>(static_cast<int>(mode)), amount, control};
}

Doubles advance_batch(const Doubles & inputs)
{
  if (inputs.ndim() != 2 || inputs.shape(1) != 6) {
    throw std::invalid_argument("expected Nx6 poses and primitives");
  }
  const auto source = inputs.unchecked<2>();
  Doubles output({source.shape(0), py::ssize_t{3}});
  auto destination = output.mutable_unchecked<2>();

  {
    py::gil_scoped_release release;
    for (py::ssize_t row = 0; row < source.shape(0); ++row) {
      const auto result = core::advance(
        {source(row, 0), source(row, 1), source(row, 2)},
        primitive(source(row, 3), source(row, 4), source(row, 5)));
      destination(row, 0) = result.x;
      destination(row, 1) = result.y;
      destination(row, 2) = result.yaw;
    }
  }
  return output;
}

Doubles trajectory_samples(
  const std::array<double, 3> & pose, const std::array<double, 3> & motion,
  double duration_s, const Doubles & times_s)
{
  if (times_s.ndim() != 1) {
    throw std::invalid_argument("expected one-dimensional sample times");
  }
  const core::TimedPrimitive trajectory(
    {pose[0], pose[1], pose[2]}, primitive(motion[0], motion[1], motion[2]), duration_s);
  const auto times = times_s.unchecked<1>();
  Doubles output({times.shape(0), py::ssize_t{7}});
  auto samples = output.mutable_unchecked<2>();

  {
    py::gil_scoped_release release;
    for (py::ssize_t row = 0; row < times.shape(0); ++row) {
      const auto sample = trajectory.sample(times(row));
      samples(row, 0) = sample.time_s;
      samples(row, 1) = sample.pose.x;
      samples(row, 2) = sample.pose.y;
      samples(row, 3) = sample.pose.yaw;
      samples(row, 4) = sample.twist.vx;
      samples(row, 5) = sample.twist.vy;
      samples(row, 6) = sample.twist.wz;
    }
  }
  return output;
}

Doubles quintic_batch(const Doubles & fractions)
{
  if (fractions.ndim() != 1) {
    throw std::invalid_argument("expected one-dimensional sample fractions");
  }
  const auto source = fractions.unchecked<1>();
  Doubles output({source.shape(0), py::ssize_t{4}});
  auto destination = output.mutable_unchecked<2>();

  {
    py::gil_scoped_release release;
    for (py::ssize_t row = 0; row < source.shape(0); ++row) {
      const auto progress = core::quintic_progress(source(row));
      destination(row, 0) = progress.position;
      destination(row, 1) = progress.first;
      destination(row, 2) = progress.second;
      destination(row, 3) = progress.third;
    }
  }
  return output;
}

core::CollisionKernel collision_kernel(
  const OccupancyMask & mask, double resolution, double origin_x, double origin_y,
  std::vector<core::Footprint> vehicles, std::optional<core::Footprint> cargo)
{
  if (mask.ndim() != 2 || mask.size() == 0) {
    throw std::invalid_argument("expected a nonempty two-dimensional occupancy mask");
  }
  const auto width = static_cast<std::size_t>(mask.shape(1));
  const auto height = static_cast<std::size_t>(mask.shape(0));
  const auto * begin = mask.data();
  return core::CollisionKernel(
    core::GridSnapshot(width, height, resolution, {origin_x, origin_y},
      std::vector<std::uint8_t>(begin, begin + mask.size())),
    std::move(vehicles), cargo);
}

Results collision_batch(const core::CollisionKernel & kernel, const Doubles & states)
{
  if (states.ndim() != 2 ||
    static_cast<std::size_t>(states.shape(1)) != 4 + kernel.vehicle_count())
  {
    throw std::invalid_argument("expected Nx(4+vehicles) formation samples");
  }
  const auto source = states.unchecked<2>();
  Results output({source.shape(0), py::ssize_t{5}});
  auto destination = output.mutable_unchecked<2>();

  {
    py::gil_scoped_release release;
    for (py::ssize_t row = 0; row < source.shape(0); ++row) {
      const auto * sample = source.data(row, 0);
      const auto result = kernel.check(
        {sample[0], sample[1], sample[2]}, sample + 4, kernel.vehicle_count(), sample[3]);
      destination(row, 0) = static_cast<std::size_t>(result.reason);
      destination(row, 1) = result.first;
      destination(row, 2) = result.second;
      destination(row, 3) = result.map_queries;
      destination(row, 4) = result.pair_queries;
    }
  }
  return output;
}
}  // namespace

PYBIND11_MODULE(fwis_planning_core_native, module)
{
  module.doc() = "Batch geometry, timing and joint OBB tests. Do not mutate input arrays during calls.";
  module.def(
    "advance_batch", &advance_batch,
    "Nx6 (x, y, yaw, mode, amount, control) -> Nx3 (x, y, yaw).");
  module.def(
    "trajectory_samples", &trajectory_samples,
    "Pose, (mode, amount, control), duration and times -> (t,x,y,yaw,vx,vy,wz) rows.");
  module.def(
    "quintic_batch", &quintic_batch,
    "Normalized fractions -> (progress, first, second, third) derivative rows.");
  module.def(
    "collision_reason", [](int reason) {return core::reason_name(static_cast<core::CollisionReason>(reason));});
  py::class_<core::CollisionKernel>(module, "CollisionKernel")
  .def(py::init(&collision_kernel), py::arg("occupied"), py::arg("resolution"),
    py::arg("origin_x"), py::arg("origin_y"), py::arg("vehicles"), py::arg("cargo") = std::nullopt)
  .def("check_batch", &collision_batch,
    "(x,y,yaw,padding,*headings) rows -> (reason, first, second, map queries, pair queries).")
  .def(
    "sweep_discretization",
    [](const core::CollisionKernel & kernel, double translation, double rotation,
      const std::vector<double> & body_rates, double corner_step, std::size_t budget) {
      const auto plan = kernel.sweep_discretization(translation, rotation, body_rates, corner_step, budget);
      return py::make_tuple(plan.corner_bound_m, plan.padding_m, plan.intervals);
    });
}
