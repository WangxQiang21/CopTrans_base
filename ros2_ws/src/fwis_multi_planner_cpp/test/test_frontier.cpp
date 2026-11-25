#include "fwis_multi_planner_cpp/frontier.hpp"
#include "fwis_multi_planner_cpp/successors.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace
{
using namespace fwis_multi_planner_cpp;

StateInput formation(double x, double y)
{
  return {{x, y, 0.0}, {{1, 0.0}, {2, 0.0}, {3, 0.0}}, 'A', {
      {1, 'A', 1, 0.0}, {2, 'A', 1, 0.0}, {3, 'A', 1, 0.0}}, {"straight/+++"}};
}

constexpr KeyResolution resolution{1.0, 0.125, 0.125, 0.125};

TEST(StateKey, PythonTiesToEvenAndNoPoseSnap)
{
  const auto first = formation(2.5, -1.5);
  const auto key = make_key(first, resolution);
  EXPECT_EQ(key.x, 2);
  EXPECT_EQ(key.y, -2);
  EXPECT_DOUBLE_EQ(first.formation.x, 2.5);
  EXPECT_DOUBLE_EQ(first.formation.y, -1.5);
  EXPECT_EQ(make_key(formation(-2.5, 0.5), resolution).x, -2);
  EXPECT_EQ(make_key(formation(-2.5, 0.5), resolution).y, 0);
  EXPECT_THROW(make_key(formation(0.0, 0x1p63), resolution), std::out_of_range);
  EXPECT_THROW(make_key(formation(0.0, 0.0), {0.0, 1.0, 1.0, 1.0}), std::invalid_argument);
}

TEST(StateKey, DiscriminatesWheelGearModeAndCapability)
{
  const auto initial = formation(4.0, 5.0);
  const auto key = make_key(initial, resolution);
  auto other = initial;
  other.wheel[1].direction = -1;
  EXPECT_FALSE(make_key(other, resolution) == key);
  other = initial;
  other.wheel[1].control = 0.125;
  EXPECT_FALSE(make_key(other, resolution) == key);
  other = initial;
  other.mode = 'S';
  EXPECT_FALSE(make_key(other, resolution) == key);
  other = initial;
  other.a_labels.clear();
  EXPECT_FALSE(make_key(other, resolution) == key);
}

TEST(Frontier, OrdersPriorityThenCostThenSerialAndRejectsEqualDuplicate)
{
  Frontier frontier;
  EXPECT_EQ(frontier.offer(make_key(formation(1.0, 5.0), resolution), 2.0, 8.0), 0U);
  EXPECT_EQ(frontier.offer(make_key(formation(2.0, 5.0), resolution), 1.0, 8.0), 1U);
  EXPECT_EQ(frontier.offer(make_key(formation(3.0, 5.0), resolution), 1.0, 8.0), 2U);
  EXPECT_FALSE(frontier.offer(make_key(formation(3.0, 5.0), resolution), 1.0, 1.0));
  EXPECT_EQ(frontier.pop().serial, 1U);
  EXPECT_EQ(frontier.pop().serial, 2U);
  EXPECT_EQ(frontier.pop().serial, 0U);
  EXPECT_EQ(frontier.size(), 0U);
  EXPECT_THROW(frontier.pop(), std::out_of_range);
}

TEST(Frontier, SupersededQueueEntriesStayStaleAcrossRehash)
{
  Frontier frontier;
  const auto key = make_key(formation(0.0, 0.0), resolution);
  EXPECT_EQ(frontier.offer(key, 3.0, 3.0), 0U);
  EXPECT_EQ(frontier.offer(key, 1.0, 1.0), 1U);
  for (int i = 1; i < 1000; ++i) {
    ASSERT_TRUE(frontier.offer(make_key(formation(i, i), resolution), 10.0, 10.0));
  }
  const auto current = frontier.pop();
  const auto stale = frontier.pop();
  EXPECT_EQ(current.serial, 1U);
  EXPECT_FALSE(current.stale);
  EXPECT_EQ(stale.serial, 0U);
  EXPECT_TRUE(stale.stale);

  // The lower-cost representative remains even after its expansion.
  EXPECT_FALSE(frontier.offer(key, 2.0, 2.0));
  EXPECT_TRUE(frontier.offer(key, 0.5, 0.5));
  EXPECT_THROW(frontier.offer(key, 0.0, std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

TEST(SearchBudget, PreservesCancellationExpansionAndDeadlinePrecedence)
{
  SearchBudget budget(2, 1.0);
  EXPECT_EQ(budget.probe(true, 2, 1.0), BudgetStatus::cancelled);
  EXPECT_EQ(budget.probe(false, 2, 1.0), BudgetStatus::expansions);
  EXPECT_EQ(budget.probe(false, 1, 1.0), BudgetStatus::deadline);
  EXPECT_EQ(budget.probe(false, 1, 0.999), BudgetStatus::running);
  EXPECT_TRUE(budget.deadline_reached(1.0));
}

TEST(Successors, OrderedBranchesRetainReverseRejectionsAndDoNotInsertZeroSpin)
{
  const auto result = generate_candidates(
    0.0, {{0.0, 0.0}}, {0.0}, {{"O", {-1.0, 0.0}}}, {{"A", {1.0, 0.0}}},
    {{"A_arc", {0.0, 0.0}, 0.4}}, .9, false, true);
  ASSERT_EQ(result.size(), 3U);
  EXPECT_EQ(result[0].label, "O");
  EXPECT_EQ(result[0].refusal, Refusal::reverse_required);
  EXPECT_EQ(result[1].label, "A_gear+");
  EXPECT_EQ(result[1].refusal, Refusal::none);
  EXPECT_EQ(result[2].refusal, Refusal::arc_domain);
}

TEST(Successors, ProxyRetainsMinimumSetupAndSlowestVehicle)
{
  const SearchLimits limits{.15, .70, .8, 1.5, .16, .4, false};
  const auto a = WheelSetup{'A', {0.0, 0.0, 0.0, 0.0}};
  const auto o = WheelSetup{'O', {0.0, 0.0, 0.0, 0.0}};
  const auto cost = edge_cost({a, a}, {a, o}, {1.0, 2.0}, {.2, .4}, limits);
  EXPECT_DOUBLE_EQ(cost[0], .70);
  EXPECT_DOUBLE_EQ(cost[1], 12.5);
  EXPECT_DOUBLE_EQ(edge_cost({a}, {a}, {2.0}, {.4}, limits)[0], 0.0);
}
}  // namespace
