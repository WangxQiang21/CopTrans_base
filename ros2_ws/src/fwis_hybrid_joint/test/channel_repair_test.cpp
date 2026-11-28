#include "../src/channel_repair.hpp"
#include "../src/formation_lattice.hpp"
#include <gtest/gtest.h>

namespace hj = fwis_hybrid_joint;
namespace channel = hj::channel_repair;

TEST(ChannelRepair, RectangleCoverIsTheExactGridInWorldCoordinates)
{
  // Rows with unequal run lengths must neither remove a cell nor bridge a hole.
  const std::vector<std::uint8_t> map = {
    0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0,
    0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0,
    1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0
  };
  const hj::GridSnapshot grid(12, 3, .1, {-1.3, 2.7}, map);
  const auto rectangles = channel::rectangles(grid, []{ return false; });
  ASSERT_EQ(5u, rectangles.size());
  for (int i = 0; i < 36; ++i) {
    const double x = -1.3 + ((i % 12) + .5) * .1;
    const double y = 2.7 + ((i / 12) + .5) * .1;
    int cover = 0;
    for (const auto& box : rectangles) {
      if (std::abs(x - box[0]) < box[2] && std::abs(y - box[1]) < box[3]) ++cover;
    }
    EXPECT_EQ(map[i], cover);
  }
  EXPECT_TRUE(channel::rectangles(grid, []{ return true; }).empty());
}

TEST(ChannelRepair, ForwardJacobianMatchesResidualDifferences)
{
  hj::Request request;
  request.vehicles = {{1., 0., .838, .6}, {0., 1., .838, .6}, {0., -1., .838, .6}};
  request.cargo = hj::Footprint{.5, 0., 2.05, 2.8};
  constexpr int spans = 6, samples = 43;
  channel::Geometry g{request, spans, samples, 1.2, channel::basis(spans, samples), {},
                      channel::Controls(spans + 5, 3), {1., 0.}, {.8, .6}};
  for (int i = 0; i < g.preferred.rows(); ++i) {
    g.preferred.row(i) << .73 * i, .043 * i * i, .0041 * i * i;
  }
  for (int i = 0; i < samples; ++i) {
    g.obstacles.push_back({{3.3, 1.14, .22, .33}, {4.93, 2.0, .4, .64}});
  }
  const auto initial = g.guess();
  const auto current = g.jets(g.controls(initial));
  const auto zero = g.controls(Eigen::VectorXd::Zero(initial.size()));
  Eigen::VectorXd actual(g.rows()), local(g.rows()), plus(g.rows()), minus(g.rows());
  constexpr double step = 1.e-6;
  bool shortened = false;
  for (int variable = 0; variable < initial.size(); ++variable) {
    const auto unit = Eigen::VectorXd::Unit(initial.size(), variable).eval();
    auto derivative = g.jets(g.controls(unit) - zero);
    g.residual(current, &derivative, actual);
    const auto support = g.support(derivative);
    g.residual(current, &derivative, local, &support);
    EXPECT_TRUE((local.array() == actual.array()).all()) << "direction " << variable;
    shortened = shortened || support.end - support.begin < samples;
    g.residual(g.jets(g.controls(initial + step * unit)), nullptr, plus);
    g.residual(g.jets(g.controls(initial - step * unit)), nullptr, minus);
    const auto expected = ((plus - minus) / (2 * step)).eval();
    const double error = ((expected - actual).cwiseAbs().array() /
                          (1. + expected.cwiseAbs().array())).maxCoeff();
    EXPECT_LT(error, 5.e-6) << "direction " << variable;
  }
  EXPECT_TRUE(shortened);
}

TEST(FormationLattice, TurnsTheCargoBeforeAnOffsetGateAndRejectsAClosedGate)
{
  hj::Request r;
  r.vehicles = {{0., -1., .838, .6}, {1., 0., .838, .6}, {0., 1., .838, .6}};
  r.cargo = hj::Footprint{.5, 0., 2.05, 2.8};
  r.start.reference = {3.13, 4.26, 0.};
  r.goal.reference = {18.62, 10.38, 0.};
  const auto stop = []{return false;};
  for (const int gap_cells : {24, 19}) {
    std::vector<std::uint8_t> cells(220 * 160);
    for (int y = 0; y < 160; ++y) {
      if (y >= 65 && y < 65 + gap_cells) continue;
      for (int x = 102; x < 112; ++x) cells[y * 220 + x] = 1;
    }
    r.grid = std::make_shared<hj::GridSnapshot>(220, 160, .1,
      fwis_planning_core::Point2d{0., 0.}, std::move(cells));
    const auto boxes = channel::rectangles(*r.grid, stop);
    const auto path = hj::formation_channel(r, boxes, 4, .20, stop);
    if (gap_cells == 19) {
      EXPECT_TRUE(path.empty());
      continue;
    }
    ASSERT_GE(path.size(), 4u);
    EXPECT_DOUBLE_EQ(path.front().x, r.start.reference.x);
    EXPECT_DOUBLE_EQ(path.back().y, r.goal.reference.y);
    bool turned = false;
    for (size_t i = 1; i < path.size(); ++i) {
      if (std::abs(path[i].yaw - path[i-1].yaw) > 1.e-8) {
        EXPECT_NEAR(path[i].x, path[i-1].x, 1.e-8);
        EXPECT_NEAR(path[i].y, path[i-1].y, 1.e-8);
        turned = true;
      }
    }
    EXPECT_TRUE(turned);
    EXPECT_TRUE(hj::formation_channel(r, boxes, 4, .20, []{return true;}).empty());
  }
}
