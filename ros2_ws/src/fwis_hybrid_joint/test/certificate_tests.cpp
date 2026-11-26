#include "fwis_hybrid_joint/model.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
namespace h=fwis_hybrid_joint;
namespace {
constexpr double pi=3.141592653589793;
h::Request request(bool formation=false, std::vector<std::uint8_t> cells={}) {
  if (cells.empty()) cells.resize(100*100);
  h::Request r;
  r.grid=std::make_shared<h::GridSnapshot>(100,100,.125,fwis_planning_core::Point2d{},std::move(cells));
  r.vehicles={{0,0,.5,.3}};
  if (formation) {
    r.vehicles={{1,0,.5,.3},{-.5,.7,.5,.3},{-.5,-.7,.5,.3}};
    r.cargo=h::Footprint{.2,.1,1.8,1.};
  }
  return r;
}
h::Quintic line(double x,double dx) {
  h::Quintic p;
  for(int i=0;i<6;++i) p[i]=x+dx*i/5;
  return p;
}
h::Phase straight(double duration=12.) {
  h::Phase ph; ph.duration=duration;
  ph.pieces.push_back({line(3,2),line(3,0),line(0,0)});
  return ph;
}
h::Phase stopped_at(const h::Request &r,const h::Phase &p) {
  h::Phase b;
  b.duration=8.; b.kind=h::Kind::Prepare; b.mode=p.mode;
  const auto s=h::evaluate_phase(r,p,0);
  b.pieces.push_back({line(s.reference.x,0),line(s.reference.y,0),line(s.reference.yaw,0)});
  for(std::size_t i=0;i<r.vehicles.size();++i) b.heading0[i]=b.heading1[i]=s.bodies[i].pose.yaw;
  return b;
}
void boundary(h::Boundary &b,const h::Sample &s,std::size_t n) {
  b.reference=s.reference;
  for(std::size_t i=0;i<n;++i) b.heading[i]=s.bodies[i].pose.yaw;
}
h::Audit audit(h::Request r,const h::Plan &plan) {
  boundary(r.start,h::evaluate_phase(r,plan.phases.front(),0),r.vehicles.size());
  boundary(r.goal,h::evaluate_phase(r,plan.phases.back(),plan.phases.back().duration),r.vehicles.size());
  return h::certify(r,plan,h::Clock::now()+std::chrono::seconds(2));
}
TEST(HybridCertificate, StraightSingleAndFormation) {
  for(bool f:{false,true}) {
    h::Plan plan{{straight()}};
    const auto res=audit(request(f),plan);
    EXPECT_TRUE(res.executable)<<res.reason;
  }
}
TEST(HybridCertificate,SeparateCargoCenterAndHighSteeringSpeedLimits) {
  auto formation=request(true);
  formation.limits.cargo_center_speed=.15;
  EXPECT_EQ(audit(formation,{{straight()}}).reason,"CARGO_CENTER_SPEED");
  EXPECT_TRUE(audit(formation,{{straight(30.)}}).executable);

  auto vehicle=request();
  auto oblique=straight();
  oblique.mode=h::Mode::O;
  oblique.pieces.front().y=line(3,2);
  h::Plan plan{{stopped_at(vehicle,oblique),oblique}};
  vehicle.limits.high_steer_speed=.20;
  EXPECT_EQ(audit(vehicle,plan).reason,"HIGH_STEER_BODY_SPEED");
  vehicle.limits.high_steer_angle=1.;
  EXPECT_TRUE(audit(vehicle,plan).executable);
}
TEST(HybridCertificate, SampleAtAccumulatedPhaseBoundaryIsAtExactRest) {
  // Four phase durations from Gazebo 20261125_155710_3a0ff9 (S -> A).
  const double times[]={2.893309979854491,2.3121077424637013,2.8933099814015346,9.075133095743006};
  h::Plan plan;
  for(double t:times) plan.phases.push_back(straight(t));
  plan.phases.back().pieces.front()={
    {8.,8.947157691817644,9.295622332951677,9.704344443161169,10.050228764131498,11.},
    {6.,6.047397388889517,6.064835151791835,6.023265256995866,6.012038651933857,6.},
    {0.,0.,0.,0.,0.,0.}};
  for(bool f:{false,true}) {
    const auto r=request(f);
    for(double t:{h::duration(plan),h::duration(plan)+1.}) {
      const auto s=h::evaluate(r,plan,t);
      for(std::size_t i=0;i<r.vehicles.size();++i) {
        EXPECT_EQ(s.bodies[i].vx,0.);
        EXPECT_EQ(s.bodies[i].vy,0.);
        EXPECT_EQ(s.bodies[i].wz,0.);
      }
    }
    EXPECT_THROW(h::evaluate(r,plan,std::numeric_limits<double>::quiet_NaN()),std::invalid_argument);
  }
}
TEST(HybridCertificate, ContinuousCurvatureAndIndependentFormationYaw) {
  for(bool f:{false,true}) {
    auto r=request(f);
    auto ph=straight(20.);
    ph.pieces[0].y={3.,3.,3.,3.12,3.34,3.52};
    ph.pieces[0].yaw={0,0,0,.012,.02,.04};
    const auto res=audit(r,{{ph}});
    EXPECT_TRUE(res.executable)<<res.reason;
  }
}
TEST(HybridCertificate, GearAndSwitchRequireStoppedSteeringPreparation) {
  auto r=request();
  auto o=straight(10.); o.mode=h::Mode::O;
  o.pieces[0].x=line(3,0); o.pieces[0].y=line(3,1);
  auto b=stopped_at(r,o);
  EXPECT_TRUE(audit(r,{{b,o}}).executable);
  EXPECT_EQ(audit(r,{{o}}).reason,"MISSING_WHEEL_PREPARATION");
  b.duration=.1;
  EXPECT_EQ(audit(r,{{b,o}}).reason,"MISSING_WHEEL_PREPARATION");
}
TEST(HybridCertificate, CurvedObliqueChecksWheelSpeedAndSteeringDerivativesInMotion) {
  for(bool formation:{false,true}){
    auto r=request(formation);auto o=straight(18.);o.mode=h::Mode::O;
    // Initial wheels point straight; both subsequent constraints bind in motion.
    o.pieces[0].y={3.,3.,3.35,3.35,3.,3.};
    auto prep=stopped_at(r,o);h::Plan plan{{prep,o}};
    const auto ok=audit(r,plan);ASSERT_TRUE(ok.executable)<<ok.reason;
    auto slow=r;slow.limits.wheel_speed=.02;
    EXPECT_EQ(audit(slow,plan).reason,"WHEEL_SPEED");
    slow=r;slow.limits.steer_rate=.002;
    EXPECT_EQ(audit(slow,plan).reason,"STEER_RATE");
    slow=r;slow.limits.steer_acceleration=.0001;
    EXPECT_EQ(audit(slow,plan).reason,"STEER_ACCELERATION");
    slow=r;slow.limits.steer_angle=.04;
    EXPECT_EQ(audit(slow,plan).reason,"OBLIQUE_ANGLE");
  }
}
TEST(HybridCertificate, RejectsHiddenCollisionAndCargoAndPairContact) {
  std::vector<std::uint8_t> cells(100*100);
  cells[24*100+32]=255;
  EXPECT_EQ(audit(request(false,cells),{{straight()}}).reason,"MAP_OBB_COLLISION");
  cells.assign(100*100,0);
  cells[27*100+31]=100;
  EXPECT_EQ(audit(request(true,cells),{{straight()}}).reason,"CARGO_OBB_COLLISION");
  auto r=request(true); r.vehicles[1]=r.vehicles[0];
  EXPECT_EQ(audit(r,{{straight()}}).reason,"VEHICLE_OBB_COLLISION");
}
TEST(HybridCertificate, RejectsInteriorSpinCornerContact) {
  auto r=request();
  r.vehicles[0]={0,0,1.4,.12};
  std::vector<std::uint8_t> cells(100*100);
  cells[24*100+29]=255;
  r.grid=std::make_shared<h::GridSnapshot>(100,100,.125,fwis_planning_core::Point2d{},cells);
  auto s=straight(12.); s.mode=h::Mode::S; s.heading0[0]=-pi/4; s.heading1[0]=pi/4;
  s.pieces[0].x=line(3,0);
  auto b=stopped_at(r,s);
  EXPECT_FALSE(audit(r,{{b,s}}).executable);
}
TEST(HybridCertificate, TamperedPhaseAndInvalidDurationFailClosed) {
  auto r=request(); auto p=straight();
  p.duration=.05;
  EXPECT_EQ(audit(r,{{p}}).reason,"MODE_DWELL");
  p=straight(); p.pieces.push_back(p.pieces[0]); p.pieces[1].x=line(5,1);
  EXPECT_EQ(audit(r,{{p}}).reason,"INTERNAL_POLYNOMIAL_JOIN");
  p=straight(); p.duration=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(h::certify(r,{{p}},h::Clock::now()+std::chrono::seconds(1)).reason,"INVALID_PHASE");
}
TEST(HybridCertificate, EnforcesReversePolicyAndPreparationTargetAndWheelGeometry) {
  auto r=request();
  auto p=straight(); p.gear[0]=-1;
  r.allow_reverse=false;
  EXPECT_EQ(audit(r,{{p}}).reason,"REVERSE_DISALLOWED");
  p.gear[0]=1;
  p.mode=h::Mode::O; p.pieces[0].x=line(3,0);p.pieces[0].y=line(3,.8);
  auto prep=stopped_at(r,p); prep.mode=h::Mode::S;
  EXPECT_EQ(audit(r,{{prep,p}}).reason,"PREPARE_TARGET_MISMATCH");
  p=straight(20.);p.pieces[0].y={3,3,3,3.1,3.2,3.4};
  r.limits.wheelbase=8; r.limits.steer_angle=.08;
  EXPECT_FALSE(audit(r,{{p}}).executable);
  r.limits=request().limits;
  p.mode=h::Mode::S;
  EXPECT_EQ(audit(r,{{p}}).reason,"STATIONARY_TRANSLATION");
}
TEST(HybridCertificate, ReversePolicyCoversObliqueTractionAndStillAllowsSpin) {
  for(bool formation:{false,true}) {
    auto r=request(formation);
    for(int gear:{-1,1}) {
      auto oblique=straight(18.);
      oblique.mode=h::Mode::O;
      oblique.gear.fill(gear);
      oblique.pieces.front()={line(5.,gear),{5.,5.03,5.18,5.26,5.35,5.35},line(0.,0.)};
      auto prepare=stopped_at(r,oblique);
      prepare.gear=oblique.gear;
      const h::Plan plan{{prepare,oblique}};
      r.allow_reverse=true;
      const auto allowed=audit(r,plan);
      ASSERT_TRUE(allowed.executable)<<allowed.reason;
      r.allow_reverse=false;
      const auto restricted=audit(r,plan);
      EXPECT_EQ(restricted.executable,gear>0)<<restricted.reason;
      if(gear<0) EXPECT_EQ(restricted.reason,"REVERSE_DISALLOWED");
    }
    auto spin=straight(12.);
    spin.mode=h::Mode::S;
    spin.heading1.fill(.5);
    spin.pieces.front().x=line(3.,0.);
    const auto rotated=audit(r,{{stopped_at(r,spin),spin}});
    EXPECT_TRUE(rotated.executable)<<rotated.reason;
  }
}
TEST(HybridCertificate, SpinNeedsAFeasibleWheelAngleAtRest) {
  auto r=request();
  auto s=straight(12.); s.mode=h::Mode::S;s.heading1[0]=1.;
  s.pieces.front().x=line(3,0);
  auto b=stopped_at(r,s);
  r.limits.steer_angle=.2;
  EXPECT_EQ(audit(r,{{b,s}}).reason,"SPIN_WHEEL_ANGLE");
}
TEST(HybridCertificate, CruisePlateauRampsAndUnrecognizedTimeLaw) {
  auto r=request();auto p=straight(10.);p.cruise_ramp_fraction=.125;
  const auto ok=audit(r,{{p}});EXPECT_TRUE(ok.executable)<<ok.reason;
  EXPECT_NEAR(h::evaluate_phase(r,p,.25*p.duration).bodies[0].vx,
              h::evaluate_phase(r,p,.75*p.duration).bodies[0].vx,1e-12);
  p.duration=3.;
  EXPECT_FALSE(audit(r,{{p}}).executable);
  p.cruise_ramp_fraction=.10;
  EXPECT_EQ(h::certify(r,{{p}},h::Clock::now()+std::chrono::seconds(1)).reason,"UNKNOWN_TIME_LAW");
}
TEST(HybridCertificate, EveryCruiseWidthProvesRestAndRejectsDynamicAndStationaryMisuse) {
  auto r=request();
  for(double width:h::cruise_ramps) {
    auto p=straight(100.);p.cruise_ramp_fraction=width;
    EXPECT_TRUE(audit(r,{{p}}).executable);
    EXPECT_DOUBLE_EQ(h::evaluate_phase(r,p,0.).bodies[0].vx,0.);
    EXPECT_DOUBLE_EQ(h::evaluate_phase(r,p,p.duration).bodies[0].vx,0.);
    auto slow=r;slow.limits.speed=.019;
    EXPECT_EQ(audit(slow,{{p}}).reason,"BODY_SPEED");
    auto hold=stopped_at(r,p);hold.cruise_ramp_fraction=width;
    EXPECT_EQ(audit(r,{{hold}}).reason,"UNKNOWN_TIME_LAW");
  }
}
TEST(HybridCertificate, StationaryAndObliqueSpansCannotAccumulateUncommandedDrift) {
  auto r=request(true);
  auto still=stopped_at(r,straight());
  still.kind=h::Kind::Hold;
  for(int j=1;j<300;++j) {
    auto piece=still.pieces.front();piece.x.fill(piece.x.front()+j*5e-9);
    still.pieces.push_back(piece);
  }
  EXPECT_EQ(audit(r,{{still}}).reason,"STATIONARY_TRANSLATION");
  auto o=straight(20.);
  o.mode=h::Mode::O;
  for(int j=0;j<2;++j) {
    h::Piece part{line(3.+j,1.),line(3.,0.),line(j*5.e-9,0.)};
    if(j==0)o.pieces.front()=part;else o.pieces.push_back(part);
  }
  auto prep=stopped_at(r,o);
  EXPECT_EQ(audit(r,{{prep,o}}).reason,"STATIONARY_REFERENCE_YAW");
}
TEST(HybridCertificate, RequestEndpointsFreeYawAndCancellation) {
  auto r=request();
  auto p=straight();
  r.start.reference={3,3,0}; r.goal.reference={5,3,.8};
  EXPECT_EQ(h::certify(r,{{p}},h::Clock::now()+std::chrono::seconds(2)).reason,"GOAL_ENDPOINT");
  r.goal.constrain_reference_yaw=false;
  EXPECT_TRUE(h::certify(r,{{p}},h::Clock::now()+std::chrono::seconds(2)).executable);
  EXPECT_EQ(h::certify(r,{{p}},h::Clock::now()+std::chrono::seconds(2),[]{return true;}).reason,"CERTIFICATION_BUDGET");
}
}
