#include "fwis_hybrid_joint/model.hpp"
#include <gtest/gtest.h>
#include <algorithm>
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
  EXPECT_EQ(0.,stopped.first_policy_compliant_s);
  r.goal=r.start;auto hold=hj::solve(r,deadline());ASSERT_TRUE(hold.plan)<<hold.reason;
  ASSERT_EQ(1u,hold.plan->phases.size());EXPECT_EQ(hj::Kind::Hold,hold.plan->phases.front().kind);
  EXPECT_EQ(hold.first_certified_s,hold.first_policy_compliant_s);
}
TEST(NativeSolver,RecoversForwardAckermannAtTheLowSpeedCurvatureWitness){
  auto r=request(true);
  r.grid=std::make_shared<hj::GridSnapshot>(120,96,.25,fwis_planning_core::Point2d{0,0},
                                            std::vector<std::uint8_t>(120*96));
  r.start.reference={4.,5.6,0.};
  r.goal.reference={5.522379369389306,5.948435141765629,.45};
  r.goal.heading.fill(.45);r.goal.constrain_heading.fill(false);
  r.cargo=hj::Footprint{.5,0.,2.05,2.80};
  auto& limits=r.limits;
  limits.speed=.16;limits.acceleration=.22;limits.lateral_acceleration=.6;
  limits.yaw_rate=limits.spin_speed=.4;limits.yaw_acceleration=limits.spin_acceleration=.45;
  limits.yaw_jerk=3.;limits.wheel_speed=1.;
  auto result=hj::solve(r,deadline());ASSERT_TRUE(result.plan)<<result.reason;
  ASSERT_TRUE(hj::certify(r,*result.plan,deadline()).executable);
  bool repaired=false;
  for(const auto& line:result.trace)repaired|=line.find("CONTINUOUS_A:A:1:REFINE_CURVATURE:")==0;
  EXPECT_TRUE(repaired);
  for(const auto& phase:result.plan->phases)if(phase.kind==hj::Kind::Motion){
    EXPECT_EQ(phase.mode,hj::Mode::A);EXPECT_EQ(phase.gear,(hj::Gears{1,1,1}));
  }
  EXPECT_GE(result.first_policy_compliant_s,result.first_certified_s);
}
TEST(NativeSolver,NearRankTerminalAngleAndNecessaryAuxiliaryTrackOriginalContract){
  auto r=request(true);
  r.goal.reference={2.5,0,0};
  r.goal.heading[1]=.004; // less than one degree of permissible terminal error
  auto a=hj::solve(r,deadline());
  ASSERT_TRUE(a.plan)<<a.reason;
  EXPECT_TRUE(hj::certify(r,*a.plan,deadline()).executable);
  for(const auto& p:a.plan->phases)if(p.kind==hj::Kind::Motion)EXPECT_EQ(hj::Mode::A,p.mode);

  // Full rank robust to the entire one-degree neighborhood. Three mounts
  // cannot leave this configuration in pure A, regardless of direction.
  r.start.heading=r.goal.heading={0.,.4,-.4};
  auto auxiliary=hj::solve(r,deadline());
  ASSERT_TRUE(auxiliary.plan)<<auxiliary.reason;
  EXPECT_TRUE(hj::certify(r,*auxiliary.plan,deadline()).executable);
  EXPECT_TRUE(std::any_of(auxiliary.plan->phases.begin(),auxiliary.plan->phases.end(),[](auto& p){
    return p.kind==hj::Kind::Motion && p.mode!=hj::Mode::A;}));
  EXPECT_GT(auxiliary.edit_seconds,0.);
  EXPECT_GT(auxiliary.first_policy_compliant_s,auxiliary.first_certified_s);
}
TEST(NativeSolver,RefinesAnInteriorObstacleAtTheContinuousWitness){
  auto r=request();
  std::vector<std::uint8_t> cells(360*360);cells[65703]=1;
  r.grid=std::make_shared<hj::GridSnapshot>(360,360,.1,fwis_planning_core::Point2d{-18.,-18.},cells);
  r.start.reference={-4.246640372722037,-1.4787562534718381,.2904334947075935};
  r.goal.reference={2.5209852011970657,.5439851451322391,.29043349470759117};
  r.start.heading.fill(r.start.reference.yaw);r.goal.heading.fill(r.goal.reference.yaw);
  r.limits.speed=.6;r.limits.acceleration=.42;r.limits.jerk=1.3;
  r.limits.steer_rate=1.4;r.limits.steer_acceleration=2.8;
  hj::SolveOptions options;options.initial_sequence={hj::Mode::A};options.enable_edits=false;
  auto result=hj::solve(r,deadline(),{},options);
  ASSERT_TRUE(result.plan)<<result.reason;
  EXPECT_EQ(0.,result.edit_seconds);
  ASSERT_TRUE(hj::certify(r,*result.plan,deadline()).executable);
  auto found=[&](const std::string& part){
    return std::find_if(result.trace.begin(),result.trace.end(),[&](auto& line){return line.find(part)!=std::string::npos;});};
  auto collision=found("SEED:A:1:MAP_OBB_COLLISION"),refined=found("SEED:A:1:REFINE_WITNESS");
  ASSERT_NE(refined,result.trace.end());EXPECT_LT(collision,refined);EXPECT_LT(refined,found("SEED:A:1:CERTIFIED"));
}
TEST(NativeSolver,RemovesBlocksAfterAnActualMixedProposal){
  auto r=request();
  hj::SolveOptions options;options.initial_sequence={hj::Mode::A,hj::Mode::S,hj::Mode::O,hj::Mode::S,hj::Mode::A};
  auto result=hj::solve(r,deadline(),{},options);
  ASSERT_TRUE(result.plan)<<result.reason;
  ASSERT_TRUE(hj::certify(r,*result.plan,deadline()).executable);
  ASSERT_GE(result.attempts.size(),2u);
  EXPECT_EQ(0u,result.attempts.front().proposal.find("SEED:ASOSA"));
  EXPECT_NE(result.trace.end(),std::find(result.trace.begin(),result.trace.end(),"REPLACE_SUBSEQUENCE:A:1:CERTIFIED"));
}

TEST(NativeSolver,ArchiveCostUsesAllMountsNormalizesBlocksAndMeasuresComfort){
  auto r=request(true);
  hj::Piece q;q.x.fill(0.);q.y.fill(0.);q.yaw.fill(0.);
  hj::Phase s;s.mode=hj::Mode::S;s.duration=8.;s.pieces={q};
  s.heading1={.2,-.3,.4};
  auto next=s;next.heading0=s.heading1;next.heading1={.3,-.4,.5};
  auto cost=hj::score_policy(r,{{s,next}},deadline());
  EXPECT_EQ(1,cost.auxiliary_blocks);
  EXPECT_NEAR(1.2,cost.spin_angle,1e-12);
  EXPECT_NEAR(.5+.25*1.2,cost.auxiliary_cost,1e-12);
  EXPECT_GT(cost.jerk_effort,0.);
  auto slow=s;slow.duration=16.;
  auto single=hj::score_policy(r,{{s}},deadline()),longer=hj::score_policy(r,{{slow}},deadline());
  EXPECT_LT(longer.jerk_effort,single.jerk_effort);EXPECT_TRUE(single.better_than(longer));

  auto one=request();q.x={0.,.2,.4,.6,.8,1.};hj::Phase drive;drive.duration=10.;drive.pieces={q};
  auto free_cost=hj::score_policy(one,{{drive}},deadline());
  auto near=one;std::vector<std::uint8_t> cells(60*60);
  // Wall starts at y=.50 m; the vehicle's half-width is .29 m. Move the
  // centerline 5 cm toward it so only the optional comfort envelope touches.
  for(int x=0;x<60;++x)cells[32*60+x]=1;
  near.grid=std::make_shared<hj::GridSnapshot>(60,60,.25,fwis_planning_core::Point2d{-7.5,-7.5},cells);
  drive.pieces[0].y.fill(.05);auto close_cost=hj::score_policy(near,{{drive}},deadline());
  EXPECT_NEAR(1.,close_cost.reference_length,1e-12);
  EXPECT_GT(close_cost.clearance_effort,free_cost.clearance_effort);EXPECT_TRUE(free_cost.better_than(close_cost));
  EXPECT_TRUE(free_cost.better_than(single));
}

}
