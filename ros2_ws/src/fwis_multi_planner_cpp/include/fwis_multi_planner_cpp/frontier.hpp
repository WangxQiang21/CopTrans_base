#pragma once

#include "fwis_planning_core/geometry.hpp"

#include <cstdint>
#include <optional>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fwis_multi_planner_cpp
{
struct KeyResolution
{
  double xy_m;
  double formation_yaw_rad;
  double body_yaw_rad;
  double control;
};

struct WheelInput
{
  std::uint32_t vehicle_id;
  char mode;  // 'A', 'O' or 'S'
  int direction;
  double control;
};

// Exact pose and labels from one joint FR03 search state. Returning a
// bin key does not modify the pose, headings, or wheel configuration.
struct StateInput
{
  fwis_planning_core::Pose2d formation;
  std::vector<std::pair<std::uint32_t, double>> headings;
  char mode;
  std::vector<WheelInput> wheel;
  std::vector<std::string> a_labels;
};

struct StateKey
{
  std::int64_t x, y, formation_yaw;
  char mode;
  std::vector<std::tuple<std::uint32_t, char, std::int64_t>> wheel;
  std::vector<std::tuple<std::uint32_t, std::int64_t, int>> headings;
  std::vector<std::string> a_labels;

  bool operator==(const StateKey & other) const noexcept;
};

StateKey make_key(const StateInput & state, KeyResolution resolution);

struct PoppedState
{
  std::uint64_t serial;
  bool stale;
};

enum class BudgetStatus {running, cancelled, expansions, deadline};

class SearchBudget
{
public:
  SearchBudget(std::size_t max_expansions, double max_time_s);
  BudgetStatus probe(bool cancelled, std::size_t expansions, double elapsed_s) const noexcept;
  bool deadline_reached(double elapsed_s) const noexcept;

private:
  std::size_t max_expansions_;
  double max_time_s_;
};

// One frontier per request. Call from the owning search thread.
// The caller retains authoritative edge payloads and audits.
class Frontier
{
public:
  Frontier() = default;
  Frontier(const Frontier &) = delete;
  Frontier & operator=(const Frontier &) = delete;

  // The strictly lower-cost exact state wins; equal cost keeps the incumbent.
  // Priorities are ordered by (rank, cost, insertion order).
  std::optional<std::uint64_t> offer(StateKey key, double cost, double rank);
  PoppedState pop();

  std::size_t size() const noexcept {return open_.size();}

private:
  struct KeyHash
  {
    std::size_t operator()(const StateKey & key) const noexcept;
  };

  struct Representative
  {
    double cost;
    std::uint64_t serial;
  };

  struct Entry
  {
    double rank;
    double cost;
    std::uint64_t serial;
    const Representative * representative;
  };

  struct Greater
  {
    bool operator()(const Entry & first, const Entry & second) const noexcept;
  };

  std::unordered_map<StateKey, Representative, KeyHash> representatives_;
  std::priority_queue<Entry, std::vector<Entry>, Greater> open_;
  std::uint64_t serial_ = 0;
};
}  // namespace fwis_multi_planner_cpp
