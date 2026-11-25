#include "../src/channel_repair.hpp"
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
  Eigen::VectorXd actual(g.rows()), plus(g.rows()), minus(g.rows());
  constexpr double step = 1.e-6;
  for (int variable = 0; variable < initial.size(); ++variable) {
    const auto unit = Eigen::VectorXd::Unit(initial.size(), variable).eval();
    auto derivative = g.jets(g.controls(unit) - zero);
    g.residual(current, &derivative, actual);
    g.residual(g.jets(g.controls(initial + step * unit)), nullptr, plus);
    g.residual(g.jets(g.controls(initial - step * unit)), nullptr, minus);
    const auto expected = ((plus - minus) / (2 * step)).eval();
    const double error = ((expected - actual).cwiseAbs().array() /
                          (1. + expected.cwiseAbs().array())).maxCoeff();
    EXPECT_LT(error, 5.e-6) << "direction " << variable;
  }
}
