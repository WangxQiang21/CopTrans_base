#include "fwis_hybrid_joint/model.hpp"
#include <gtest/gtest.h>
#include <cmath>
#ifdef COPTRANS_TEST_LINKED_IPOPT
extern "C" void casadi_load_nlpsol_ipopt();
class PluginEnvironment : public testing::Environment {
  void SetUp() override {casadi_load_nlpsol_ipopt();}
};
testing::Environment* const plugin_environment=testing::AddGlobalTestEnvironment(new PluginEnvironment);
#endif
namespace hj=fwis_hybrid_joint;
namespace {
hj::Request request(bool formation=false){
  hj::Request r;
  r.grid=std::make_shared<hj::GridSnapshot>(60,60,.25,fwis_planning_core::Point2d{-7.5,-7.5},std::vector<std::uint8_t>(3600));
  if(formation) {
    r.vehicles={{1,0,.838,.6},{0,1,.838,.6},{0,-1,.838,.6}};
    r.cargo=hj::Footprint{.50,0.,2.1,2.8};
  }else r.vehicles={{0,0,.818,.58}};
  r.start.reference={-2.5,0,0};r.goal.reference={2.5,.6,0};
  return r;
}
auto deadline(){return hj::Clock::now()+std::chrono::seconds(8);}
TEST(NativeSolver,RepeatedRequestsRebindEndpointsAndCanonicalInactiveGears){
  auto r=request();
  for(int i=0;i<3;++i){
    r.goal.reference.y=i*.5;
    auto result=hj::solve(r,deadline());ASSERT_TRUE(result.plan) << result.reason;
    auto audit=hj::certify(r,*result.plan,deadline());ASSERT_TRUE(audit.executable)<<audit.reason;
    auto end=hj::evaluate(r,*result.plan,hj::duration(*result.plan));
    EXPECT_NEAR(end.reference.y,r.goal.reference.y,1e-6);
    for(auto& p:result.plan->phases){EXPECT_EQ(1,p.gear[1]);EXPECT_EQ(1,p.gear[2]);}
  }
}
TEST(NativeSolver,OneSharedClockWithOppositeMountHeadings){
  auto r=request(true);r.start.heading[2]=r.goal.heading[2]=std::acos(-1.);
  auto outcome=hj::solve(r,deadline());ASSERT_TRUE(outcome.plan)<<outcome.reason;
  for(auto& p:outcome.plan->phases)if(p.kind==hj::Kind::Motion){
    EXPECT_EQ(hj::Mode::A,p.mode); EXPECT_EQ(-1,p.gear[2]);EXPECT_EQ(1,p.gear[0]);
  }
}
TEST(NativeSolver,CancellationAndStationaryGoal){
  auto r=request();auto stopped=hj::solve(r,deadline(),[]{return true;});
  EXPECT_FALSE(stopped.plan);EXPECT_EQ("CANCELLED",stopped.status);
  r.goal=r.start;auto hold=hj::solve(r,deadline());ASSERT_TRUE(hold.plan)<<hold.reason;
  ASSERT_EQ(1u,hold.plan->phases.size());EXPECT_EQ(hj::Kind::Hold,hold.plan->phases.front().kind);
}
}
