#include "fwis_multi_planner_cpp/frontier.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace fwis_multi_planner_cpp
{
namespace
{
std::int64_t bin(double value, double resolution)
{
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    throw std::invalid_argument("bin width must be finite and positive");
  }

  const double scaled = value / resolution;
  if (!std::isfinite(scaled)) {
    throw std::invalid_argument("state bin must be finite");
  }

  // Match Python round(double) for ties, regardless of the floating point
  // environment's rounding mode. The range guard precedes integer conversion.
  double integral;
  const double fraction = std::modf(scaled, &integral);
  const double distance = std::abs(fraction);
  if (distance > 0.5 || (distance == 0.5 && std::fmod(integral, 2.0) != 0.0)) {
    integral += std::copysign(1.0, fraction);
  }

  constexpr double limit = 0x1p63;
  if (integral < -limit || integral >= limit) {
    throw std::out_of_range("state label exceeds the native signed 64 bit range");
  }
  return static_cast<std::int64_t>(integral);
}

template<class T>
void combine(std::size_t & seed, const T & value) noexcept
{
  seed ^= std::hash<T>{}(value) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
}
}  // namespace

bool StateKey::operator==(const StateKey & other) const noexcept
{
  return std::tie(x, y, formation_yaw, mode, wheel, headings, a_labels) ==
         std::tie(other.x, other.y, other.formation_yaw, other.mode,
    other.wheel, other.headings, other.a_labels);
}

StateKey make_key(const StateInput & state, KeyResolution resolution)
{
  StateKey result{
    bin(state.formation.x, resolution.xy_m), bin(state.formation.y, resolution.xy_m),
    bin(fwis_planning_core::wrap_angle(state.formation.yaw), resolution.formation_yaw_rad),
    state.mode, {}, {}, state.a_labels};
  result.wheel.reserve(state.wheel.size());
  result.headings.reserve(state.headings.size());
  for (const auto & wheel : state.wheel) {
    result.wheel.emplace_back(wheel.vehicle_id, wheel.mode, bin(wheel.control, resolution.control));
  }
  // Preserve the Python dictionary's last entry if a malformed signature
  // repeats a vehicle ID. A validated FR03 state always has unique IDs.
  for (const auto & [id, theta] : state.headings) {
    auto found = state.wheel.rbegin();
    while (found != state.wheel.rend() && found->vehicle_id != id) {
      ++found;
    }
    if (found == state.wheel.rend()) {
      throw std::invalid_argument("state heading has no matching wheel signature");
    }
    result.headings.emplace_back(
      id, bin(fwis_planning_core::wrap_angle(theta), resolution.body_yaw_rad), found->direction);
  }
  return result;
}

SearchBudget::SearchBudget(std::size_t max_expansions, double max_time_s)
: max_expansions_(max_expansions), max_time_s_(max_time_s)
{
  if (!std::isfinite(max_time_s_) || max_time_s_ <= 0.0) {
    throw std::invalid_argument("search time budget must be finite and positive");
  }
}

BudgetStatus SearchBudget::probe(
  bool cancelled, std::size_t expansions, double elapsed_s) const noexcept
{
  if (cancelled) {
    return BudgetStatus::cancelled;
  }
  if (expansions >= max_expansions_) {
    return BudgetStatus::expansions;
  }
  return deadline_reached(elapsed_s) ? BudgetStatus::deadline : BudgetStatus::running;
}

bool SearchBudget::deadline_reached(double elapsed_s) const noexcept
{
  return !std::isfinite(elapsed_s) || elapsed_s >= max_time_s_;
}

std::size_t Frontier::KeyHash::operator()(const StateKey & key) const noexcept
{
  std::size_t hash = 0;
  combine(hash, key.x);
  combine(hash, key.y);
  combine(hash, key.formation_yaw);
  combine(hash, key.mode);
  for (const auto & [id, mode, bin] : key.wheel) {
    combine(hash, id);
    combine(hash, mode);
    combine(hash, bin);
  }
  for (const auto & [id, bin, direction] : key.headings) {
    combine(hash, id);
    combine(hash, bin);
    combine(hash, direction);
  }
  for (const auto & label : key.a_labels) {
    combine(hash, label);
  }
  return hash;
}

bool Frontier::Greater::operator()(const Entry & first, const Entry & second) const noexcept
{
  return std::tie(first.rank, first.cost, first.serial) >
         std::tie(second.rank, second.cost, second.serial);
}

std::optional<std::uint64_t> Frontier::offer(StateKey key, double cost, double rank)
{
  if (!std::isfinite(cost) || !std::isfinite(rank)) {
    throw std::invalid_argument("frontier cost and rank must be finite");
  }
  auto incumbent = representatives_.find(key);
  if (incumbent != representatives_.end() && !(cost < incumbent->second.cost)) {
    return std::nullopt;
  }
  if (serial_ == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("frontier insertion sequence exhausted");
  }
  const Representative winner{cost, serial_};
  if (incumbent == representatives_.end()) {
    incumbent = representatives_.emplace(std::move(key), winner).first;
  } else {
    incumbent->second = winner;
  }
  // The node-based unordered_map keeps references stable during rehash.
  open_.push({rank, cost, serial_, &incumbent->second});
  return serial_++;
}

PoppedState Frontier::pop()
{
  if (open_.empty()) {
    throw std::out_of_range("no state in search frontier");
  }
  const auto entry = open_.top();
  open_.pop();
  return {entry.serial, entry.representative->serial != entry.serial};
}
}  // namespace fwis_multi_planner_cpp
