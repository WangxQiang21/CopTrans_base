#include "fwis_multi_planner_cpp/frontier.hpp"
#include "fwis_multi_planner_cpp/successors.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>

namespace py = pybind11;
namespace search = fwis_multi_planner_cpp;

namespace
{
char mode(py::handle input)
{
  const auto value = input.cast<std::string>();
  if (value.size() != 1 || (value[0] != 'A' && value[0] != 'O' && value[0] != 'S')) {
    throw std::invalid_argument("formation mode must be A, O or S");
  }
  return value[0];
}

search::StateInput read_state(const py::object & input)
{
  const py::object pose = input.attr("qf");
  search::StateInput result{
    {pose.attr("x").cast<double>(), pose.attr("y").cast<double>(), pose.attr("psi").cast<double>()},
    input.attr("headings").cast<std::vector<std::pair<std::uint32_t, double>>>(),
    mode(input.attr("mode")), {}, input.attr("a_labels").cast<std::vector<std::string>>()};
  const py::tuple wheel = input.attr("wheel");
  result.wheel.reserve(wheel.size());
  for (const auto & item : wheel) {
    result.wheel.push_back({
      item.attr("vehicle_id").cast<std::uint32_t>(), mode(item.attr("mode")),
      item.attr("direction").cast<int>(), item.attr("control").cast<double>()});
  }
  return result;
}

search::KeyResolution read_resolution(const py::object & input)
{
  return {
    input.attr("xy_bin_m").cast<double>(), input.attr("psi_bin_rad").cast<double>(),
    input.attr("heading_bin_rad").cast<double>(), input.attr("control_bin").cast<double>()};
}

struct BoundFrontier
{
  explicit BoundFrontier(const py::object & options)
  : resolution(read_resolution(options)) {}

  search::Frontier queue;
  search::KeyResolution resolution;
};

using Point = std::array<double, 2>;
using Translation = std::pair<std::string, Point>;
using Arc = std::tuple<std::string, double, Point>;
using Wheel = std::pair<char, std::array<double, 4>>;

std::vector<search::Point2d> read_points(const std::vector<Point> & inputs)
{
  std::vector<search::Point2d> result;
  result.reserve(inputs.size());
  for (const auto & point : inputs) {
    result.push_back({point[0], point[1]});
  }
  return result;
}

py::list export_candidates(const std::vector<search::Candidate> & candidates)
{
  py::list result;
  for (const auto & item : candidates) {
    result.append(py::make_tuple(item.primitive, item.label,
        std::array<double, 2>{item.control.x, item.control.y}, item.delta_rad,
        item.directions, item.angles, item.refusal, item.failed_vehicle, item.bearing_rad));
  }
  return result;
}

std::vector<search::TranslationSeed> read_translations(const std::vector<Translation> & inputs)
{
  std::vector<search::TranslationSeed> result;
  result.reserve(inputs.size());
  for (const auto & [label, point] : inputs) {
    result.push_back({label, {point[0], point[1]}});
  }
  return result;
}

std::vector<search::WheelSetup> read_wheels(const std::vector<Wheel> & inputs)
{
  std::vector<search::WheelSetup> result;
  result.reserve(inputs.size());
  for (const auto & [mode, angles] : inputs) {
    result.push_back({mode, angles});
  }
  return result;
}
}  // namespace

PYBIND11_MODULE(fwis_multi_frontier_native, module)
{
  module.doc() = "Native FR03 candidate branching/cost and FR04 open/representative storage";
  py::enum_<search::Primitive>(module, "Primitive")
  .value("OBLIQUE", search::Primitive::oblique).value("STRAIGHT", search::Primitive::straight)
  .value("ARC", search::Primitive::arc).value("SPIN", search::Primitive::spin);
  py::enum_<search::Refusal>(module, "Refusal")
  .value("NONE", search::Refusal::none).value("REVERSE_REQUIRED", search::Refusal::reverse_required)
  .value("ALIGNMENT_REQUIRED", search::Refusal::alignment_required)
  .value("ARC_DOMAIN", search::Refusal::arc_domain);
  module.def("generate_candidates", [](double yaw, const std::vector<Point> & offsets,
      const std::vector<double> & headings, const std::vector<Translation> & oblique,
      const std::vector<Translation> & straight, const std::vector<Arc> & arcs,
      double minimum_radius, bool allow_reverse, bool prepare_spins) {
      std::vector<search::ArcSeed> turns;
      turns.reserve(arcs.size());
      for (const auto & [label, delta, pivot] : arcs) {
        turns.push_back({label, {pivot[0], pivot[1]}, delta});
      }
      std::vector<search::Candidate> candidates;
      const auto centers = read_points(offsets);
      const auto oblique_seeds = read_translations(oblique);
      const auto straight_seeds = read_translations(straight);
      {
        py::gil_scoped_release release;
        candidates = search::generate_candidates(yaw, centers, headings, oblique_seeds,
            straight_seeds, turns, minimum_radius, allow_reverse, prepare_spins);
      }
      return export_candidates(candidates);
    });
  module.def("generate_lattice", [](double yaw, const std::vector<Point> & offsets,
      const std::vector<double> & headings, double step_m, const std::optional<Point> & target_step,
      const std::vector<Point> & pivots, double arc_step_rad, std::optional<double> target_yaw,
      double minimum_radius, bool allow_reverse) {
      std::optional<search::Point2d> target;
      if (target_step) {
        target = search::Point2d{(*target_step)[0], (*target_step)[1]};
      }
      std::vector<search::Candidate> candidates;
      const auto centers = read_points(offsets);
      const auto rotations = read_points(pivots);
      {
        py::gil_scoped_release release;
        candidates = search::generate_lattice(yaw, centers, headings, step_m, target, rotations,
            arc_step_rad, target_yaw, minimum_radius, allow_reverse);
      }
      return export_candidates(candidates);
    });
  module.def("spin_increments", &search::spin_increments);
  module.def("goal_arc_branches", [](double yaw, const std::vector<Point> & offsets,
      const std::vector<double> & headings, Point pivot, double delta, double min_radius,
      bool allow_reverse) -> py::object {
      const auto branches = search::goal_arc_branches(yaw, read_points(offsets), headings,
          {pivot[0], pivot[1]}, delta, min_radius, allow_reverse);
      if (!branches) {
        return py::none();
      }
      py::list result;
      for (const auto & branch : *branches) {
        result.append(py::make_tuple(branch.directions, branch.target_headings));
      }
      return result;
    });
  module.def("edge_cost", [](const std::vector<Wheel> & before, const std::vector<Wheel> & after,
      const std::vector<double> & distance, const std::vector<double> & yaw,
      const std::tuple<double, double, double, double, double, double, bool> & limits) {
      const auto & [setup, minimum, steer_rate, steer_accel, speed, yaw_rate, fixed] = limits;
      return search::edge_cost(read_wheels(before), read_wheels(after), distance, yaw,
        {setup, minimum, steer_rate, steer_accel, speed, yaw_rate, fixed});
    });
  py::enum_<search::BudgetStatus>(module, "BudgetStatus")
  .value("RUNNING", search::BudgetStatus::running).value("CANCELLED", search::BudgetStatus::cancelled)
  .value("EXPANSIONS", search::BudgetStatus::expansions).value("DEADLINE", search::BudgetStatus::deadline);
  py::class_<search::SearchBudget>(module, "SearchBudget")
  .def(py::init<std::size_t, double>())
  .def("probe", &search::SearchBudget::probe)
  .def("audit_terminal", [](const search::SearchBudget & budget, double search_start,
      const py::function & clock, const py::function & validator, const py::tuple & edges) {
      const double started = clock().cast<double>();
      const auto verdict = py::tuple(validator(edges));
      const double finished = clock().cast<double>();
      if (verdict.size() != 2) {
        throw std::invalid_argument("terminal validator must return accepted, detail");
      }
      return py::make_tuple(verdict[0], verdict[1], started, finished,
          budget.deadline_reached(finished - search_start));
    });
  module.def("make_key", [](const py::object & state, const py::object & resolution) {
      const auto key = search::make_key(read_state(state), read_resolution(resolution));
      return py::make_tuple(
        key.x, key.y, key.formation_yaw, key.mode, key.wheel, key.headings, key.a_labels);
    });
  py::class_<BoundFrontier>(module, "Frontier")
  .def(py::init<const py::object &>())
  .def("offer", [](BoundFrontier & frontier, const py::object & state, double cost, double rank) {
      return frontier.queue.offer(search::make_key(read_state(state), frontier.resolution), cost, rank);
    })
  .def("pop", [](BoundFrontier & frontier) {
      const auto result = frontier.queue.pop();
      return py::make_tuple(result.serial, result.stale);
    })
  .def("__len__", [](const BoundFrontier & frontier) {return frontier.queue.size();});
}
