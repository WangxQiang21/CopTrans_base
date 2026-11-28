#include "search_internal.hpp"
#include "channel_spline.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace fwis_hybrid_joint::search_detail {
namespace {
using fwis_planning_core::CollisionKernel;
using fwis_planning_core::CollisionReason;
constexpr double infinity = std::numeric_limits<double>::infinity();

double positive_turn(double angle) {
  double v = std::fmod(angle, 2*pi);
  if (v < 0) v += 2*pi;
  return v > 2*pi-1e-9 ? 0. : v;
}

struct Connection {
  double first{}, line{}, last{}, radius{};
  int left{}, right{}, gear{};
  double length() const { return radius * (first + last) + line; }

  // The spatial tangent, including on a reversing branch, points along travel.
  bool sample(Pose2d pose, const CollisionKernel& map, Route* points) const {
    if (gear < 0) pose.yaw += pi;
    for (int leg = 0; leg < 3; ++leg) {
      const double distance = leg == 1 ? line : radius * (leg == 0 ? first : last);
      if (distance < 1e-9) continue;
      const double curvature = (leg == 0 ? left : leg == 2 ? right : 0) / radius;
      const int steps = std::max(1, int(std::ceil(distance / .18)));
      const double ds = distance / steps, turn = curvature * ds;
      const double chord = curvature ? 2. * std::sin(.5*turn) / curvature : ds;
      for (int i = 0; i < steps; ++i) {
        pose.x += chord * std::cos(pose.yaw + .5*turn);
        pose.y += chord * std::sin(pose.yaw + .5*turn);
        pose.yaw += turn;
        const double chassis = pose.yaw + (gear < 0 ? pi : 0.);
        if (map.check(pose, &chassis, 1).reason != CollisionReason::clear) return false;
        if (points) points->push_back({pose.x, pose.y});
      }
    }
    return true;
  }
};

std::vector<Connection> shots(Pose2d from, Pose2d to, int gear, double radius) {
  if (gear < 0) { from.yaw += pi; to.yaw += pi; }
  std::vector<Connection> out;
  for (int left : {1,-1}) for (int right : {1,-1}) {
    // External/internal common tangent of the two turning circles (CSC).
    const double dx = to.x-from.x - radius*(right*std::sin(to.yaw)-left*std::sin(from.yaw));
    const double dy = to.y-from.y + radius*(right*std::cos(to.yaw)-left*std::cos(from.yaw));
    const double separation = std::hypot(dx,dy), offset = radius*(right-left);
    if (separation <= std::abs(offset)+1e-9) continue;
    const double heading = std::atan2(dy,dx)-std::asin(offset/separation);
    out.push_back({positive_turn(left*(heading-from.yaw)), std::sqrt(separation*separation-offset*offset),
                   positive_turn(right*(to.yaw-heading)), radius, left, right, gear});
  }
  std::sort(out.begin(),out.end(),[](const auto& a,const auto& b){return a.length()<b.length();});
  return out;
}

struct Label {
  Pose2d pose;
  int gear;
  double cost{infinity};
  int parent{-1};
  Connection edge;
};

using Layer = std::vector<Label>;
std::vector<Layer> stations(const Request& r,const Route& guide) {
  const int last = int(guide.size())-1;
  std::vector<Layer> layers(guide.size());
  for (int i=0;i<=last;++i) {
    const auto p=guide[i], a=guide[std::max(i-1,0)], b=guide[std::min(last,i+1)];
    const double tangent=std::atan2(b.y-a.y,b.x-a.x);
    for (int gear : {1,-1}) {
      if (gear<0 && !r.allow_reverse) continue;
      auto add=[&](double yaw){layers[i].push_back({{p.x,p.y,yaw},gear,i==0?0.:infinity,-1,{}});};
      if (i==0) add(r.start.heading[0]);
      else if (i==last && (r.goal.constrain_heading[0] || r.goal.constrain_reference_yaw))
        add(r.goal.constrain_heading[0]?r.goal.heading[0]:r.goal.reference.yaw);
      else for (double shift : {0.,-.48,.48,-.96,.96}) add(tangent+(gear<0?pi:0.)+shift);
    }
  }
  return layers;
}

std::vector<Label> connect(const Request& r,const Route& guide,const CollisionKernel& map,
                           double radius,const Run& run) {
  auto layers=stations(r,guide);
  for (std::size_t i=1;i<layers.size();++i) {
    const auto& before=layers[i-1];
    for (auto& target:layers[i]) {
      if (run.stop()) return {};
      for (int j=0;j<int(before.size());++j) {
        const auto& source=before[j];
        // A direction reversal is a stationary gear handoff, with the same
        // chassis tangent on both sides. Rank it below an ordinary running edge.
        const double prior=source.cost+(i>1 && source.gear!=target.gear?3.*radius:0.);
        if (prior + std::hypot(target.pose.x-source.pose.x,target.pose.y-source.pose.y)>=target.cost) continue;
        for (const auto& edge:shots(source.pose,target.pose,target.gear,radius)) {
          const double cost=prior+edge.length()*(target.gear<0?1.03:1.)+.12*radius*(edge.first+edge.last);
          if (cost>=target.cost || !edge.sample(source.pose,map,nullptr)) continue;
          target.parent=j;target.cost=cost;target.edge=edge;
          break;
        }
      }
    }
    if (std::none_of(layers[i].begin(),layers[i].end(),
        [](const Label& label){return label.parent>=0;})) return {};
  }
  const auto& last=layers.back();
  auto winner=std::min_element(last.begin(),last.end(),[](const Label& a,const Label& b){return a.cost<b.cost;});
  if (!std::isfinite(winner->cost)) return {};
  std::vector<Label> route(layers.size());
  int index=int(winner-last.begin());
  for (int i=int(layers.size())-1;i>=0;--i) {route[i]=layers[i][index];index=route[i].parent;}
  return route;
}

double route_length(const Route& route) {
  double length=0.;
  for (std::size_t i=1;i<route.size();++i) length+=distance(route[i-1],route[i]);
  return length;
}

// Approximate the geometric guide with a clamped C4 quintic B-spline. The
// endpoint controls impose the exact signed tangents and boundary curvature.
// A common clock spans all bends with unchanged gear.
Phase cruise_phase(const Route& route,double entry,double exit,int gear,double length,double spacing,double entry_curvature,double exit_curvature,
                   const Limits& limits) {
  const int n=std::clamp(int(std::ceil(length/spacing)),2,100);
  std::vector<double> knots(6,0.);
  for(int i=1;i<n;++i)knots.push_back(i);
  knots.insert(knots.end(),6,n);
  std::vector<double> x(n+5),y(n+5);
  for(int i=0;i<n+5;++i){
    double station=0.;
    for(int j=1;j<=5;++j)station+=knots[i+j]/(5.*n);
    const auto pos=interpolate(route,station);
    x[i]=pos.x;y[i]=pos.y;
    for(int k=1;k<=2;++k){
      if(i==k){const double s=station*length;x[i]=route.front().x+s*std::cos(entry);y[i]=route.front().y+s*std::sin(entry);}
      if(i==n+4-k){const double s=(1.-station)*length;x[i]=route.back().x-s*std::cos(exit);y[i]=route.back().y-s*std::sin(exit);}
    }
  }
  // For a clamped quintic, the normal component of its second derivative
  // at the first knot is 10*(P2-3*P1+2*P0). A short parking arc can require
  // a nonzero wheel angle at rest. append_motion times that steering change.
  const double gain=.1*std::pow(length/n,2);
  x[2]-=gain*entry_curvature*std::sin(entry);
  y[2]+=gain*entry_curvature*std::cos(entry);
  x[n+2]-=gain*exit_curvature*std::sin(exit);
  y[n+2]+=gain*exit_curvature*std::cos(exit);
  const auto xs=hybrid_channel::bezier_spans(std::move(x),n),ys=hybrid_channel::bezier_spans(std::move(y),n);
  Phase ph;ph.gear[0]=gear;ph.cruise_ramp_fraction=.125;
  ph.heading0.fill(entry+(gear<0?pi:0.));ph.heading1.fill(exit+(gear<0?pi:0.));
  ph.duration=std::max(limits.min_motion_duration,length/(.875*std::min(limits.speed,limits.wheel_speed)));
  for(int i=0;i<n;++i){Piece piece;piece.x=xs[i];piece.y=ys[i];ph.pieces.push_back(piece);}
  return ph;
}

std::optional<Plan> smooth(const Request& r,const std::vector<Label>& route,const CollisionKernel& map,
                           double spacing,const Run& run,bool match_rest_curvature=false) {
  Plan plan;
  std::size_t a=0;
  while(a+1<route.size()){
    std::size_t b=a+1;
    const auto start=route[a].pose;
    Route points{{start.x,start.y}};
    const int gear=route[b].gear;
    while(true){
      if(!route[b].edge.sample(route[b-1].pose,map,&points)) return {};
      if(b+1==route.size() || route[b+1].gear!=gear)break;
      ++b;
    }
    const double chart=gear<0?pi:0.;
    const auto& first=route[a+1].edge;
    const auto& last=route[b].edge;
    const double k0=match_rest_curvature && first.first>1e-8 ? first.left/first.radius : 0.;
    const double k1=match_rest_curvature && last.last>1e-8 ? last.right/last.radius : 0.;
    auto ph=cruise_phase(points,start.yaw+chart,route[b].pose.yaw+chart,gear,route_length(points),
                         spacing,k0,k1,r.limits);
    // Compute a tight initial duration from sampled first-order bounds.
    // Final bounds, including all higher derivatives, are certified below.
    double scale=1.;
    for(int i=0;i<=int(12*ph.pieces.size());++i){
      const auto body=evaluate_phase(r,ph,ph.duration*i/(12*ph.pieces.size())).bodies[0];
      if(std::abs(body.curvature)>1./r.limits.radius+1e-8)return {};
      scale=std::max({scale,std::abs(body.wz)/r.limits.yaw_rate,std::abs(body.vx)/r.limits.speed});
    }
    ph.duration*=1.025*scale;append_motion(r,plan,std::move(ph));
    a=b;
  }
  while(!run.stop() && duration(plan)<=r.task_duration_cap){
    const auto audit=certify(r,plan,run.end,run.cancel);
    if(audit.executable)return plan;
    const auto& reason=audit.reason;
    if(reason.find("SPEED")==std::string::npos && reason.find("ACCELERATION")==std::string::npos &&
       reason.find("JERK")==std::string::npos && reason.find("RATE")==std::string::npos)return {};
    auto& ph=plan.phases[audit.phase];ph.duration*=1.16;
  }
  return {};
}
}

std::optional<Plan> single_cruise(const Request& r,const Route& guide,const Run& run) {
  if(guide.size()<2 || r.vehicles.size()!=1 || r.cargo || !(r.limits.radius>0.))return {};
  const CollisionKernel map(*r.grid,r.vehicles);
  const double length=route_length(guide);
  if(length<1e-3)return {};
  // Try a direct tangent connection, then follow the topology of the obstacle
  // guide. Each candidate may use forward or reverse within Ackermann mode.
  const Route direct{guide.front(),guide.back()};
  struct Chart {
    double radius_factor;
    bool match_rest_curvature;
    std::array<double,3> spacing;
  };
  // The zero-wheel-angle chart preserves the straight departure/arrival.
  // Near the turning-radius bound it can introduce a curvature spike at a
  // short first arc. Then also fit the actual circle curvature at rest.
  constexpr Chart charts[]={{1.65,false,{1.15,.70,.42}}, {1.18,false,{1.15,.70,.42}},
                            {1.35,true,{.85,1.15,.55}}};
  for(const auto& chart:charts)for(const auto* channel:{&direct,&guide}){
    if(run.stop())return {};
    if(channel==&guide && guide.size()==2)continue;
    if(chart.match_rest_curvature && length<=8.*r.limits.radius)continue;
    auto route=connect(r,*channel,map,chart.radius_factor*r.limits.radius,run);
    if(route.empty())continue;
    for(double spacing:chart.spacing){
      if(run.stop())return {};
      auto plan=smooth(r,route,map,spacing,run,chart.match_rest_curvature);
      if(plan)return plan;
    }
  }
  // A constrained parking approach can run out of room to rotate while
  // entering the bay. Look for a cusp on its extended terminal tangent,
  // then drive straight into the slot in the opposite gear.
  if(!r.allow_reverse || length>8.*r.limits.radius ||
     !(r.goal.constrain_heading[0] || r.goal.constrain_reference_yaw))return {};
  auto start=r.start.reference,goal=r.goal.reference;
  start.yaw=r.start.heading[0];
  if(r.goal.constrain_heading[0])goal.yaw=r.goal.heading[0];
  for(double reach:{1.5,2.,2.5,3.,3.5,4.})for(int gear:{1,-1}){
    const double tail=reach*r.limits.radius;
    Pose2d cusp{goal.x+gear*tail*std::cos(goal.yaw),goal.y+gear*tail*std::sin(goal.yaw),goal.yaw};
    for(double turn_factor:{1.8,1.4,1.2}){
      if(run.stop())return {};
      const double radius=turn_factor*r.limits.radius;
      const Connection straight{0.,tail,0.,radius,1,1,-gear};
      if(!straight.sample(cusp,map,nullptr))continue;
      for(const auto& entry:shots(start,cusp,gear,radius)){
        if(entry.length()>1.7*length || !entry.sample(start,map,nullptr))continue;
        const std::vector<Label> parked{{start,gear,0.,-1,{}},{cusp,gear,0.,-1,entry},{goal,-gear,0.,-1,straight}};
        for(double spacing:{.7,.4,.25}){
          if(run.stop())return {};
          auto plan=smooth(r,parked,map,spacing,run,true);
          if(plan)return plan;
        }
      }
    }
  }
  return {};
}

std::optional<Plan> single_corridor_cruise(const Request& r,const Route& guide,const Run& run) {
  if(guide.size()<3 || !(r.limits.radius>0.))return {};
  const double length=route_length(guide);
  const double radius=r.limits.radius;
  const CollisionKernel map(*r.grid,r.vehicles);
  // Station spacing follows the turning radius, rather than the fixed knot
  // count of the nonlinear seed. Keep the tangent and gear choices until
  // the whole corridor has been connected.
  for(double stride:{3.6,2.8,1.7}){
    if(run.stop())return {};
    const int edges=std::clamp(int(std::ceil(length/(stride*radius))),2,96);
    Route stations;
    stations.reserve(edges+1);
    for(int i=0;i<=edges;++i)stations.push_back(interpolate(guide,double(i)/edges));
    for(double radius_factor:{1.35,1.65}){
      const auto route=connect(r,stations,map,radius_factor*radius,run);
      if(route.empty())continue;
      // Match the circular arc curvature at rest, including at a gear
      // reversal. The native certificate checks geometry and timing.
      for(double mesh:{.85,.70,.42}){
        if(run.stop())return {};
        try{
          if(auto fitted=smooth(r,route,map,mesh,run,true))return fitted;
        }catch(const std::domain_error&){
          // A singular tangent invalidates this fit. Try the next mesh.
        }
      }
    }
  }
  return {};
}

std::optional<Plan> single_narrow_docking(const Request& r,const Route& guide,const Run& run){
  const double length=route_length(guide);
  if(length<4.*r.limits.radius)return {};
  const CollisionKernel map(*r.grid,r.vehicles);
  const auto start=guide.front(),goal=guide.back();
  const double arrival=r.goal.constrain_heading[0]?r.goal.heading[0]:r.goal.reference.yaw;
  for(double station:{1.,.65,1.6}) {
    const auto after=interpolate(guide,station/length);
    const auto before=interpolate(guide,1.-station/length);
    const double entry=std::atan2(after.y-start.y,after.x-start.x);
    const double exit=std::atan2(goal.y-before.y,goal.x-before.x);
    for(double spacing:{1.,.65,1.30,.45,1.60}) {
      if(run.stop())return {};
      auto phase=cruise_phase(guide,entry,exit,1,length,spacing,0.,0.,r.limits);
      double time_scale=1.;
      bool clear=true;
      for(int j=0;j<=int(12*phase.pieces.size());++j) {
        const auto sample=evaluate_phase(r,phase,phase.duration*j/(12*phase.pieces.size()));
        const auto& vehicle=sample.bodies.front();
        if(std::abs(vehicle.curvature)>1./r.limits.radius ||
           map.check(sample.reference,&vehicle.pose.yaw,1).reason!=CollisionReason::clear) {
          clear=false;
          break;
        }
        time_scale=std::max(time_scale,std::abs(vehicle.wz)/r.limits.yaw_rate);
      }
      if(!clear)continue;
      phase.duration*=1.03*time_scale;
      for(int i=0;i<6 && !run.stop();++i) {
        Plan plan;
        Sample rest;rest.reference=r.start.reference;rest.bodies.front().pose=r.start.reference;
        rest.bodies.front().pose.yaw=r.start.heading[0];
        if(std::abs(delta(rest.bodies.front().pose.yaw,entry))>pi/180.)
          append_motion(r,plan,stationary_spin(r,rest,{entry,0.,0.}));
        append_motion(r,plan,phase);
        const auto end=evaluate_phase(r,phase,phase.duration);
        if(std::abs(delta(end.bodies.front().pose.yaw,arrival))>pi/180.)
          append_motion(r,plan,stationary_spin(r,end,{arrival,0.,0.}));
        const auto proof=certify(r,plan,run.end,run.cancel);
        if(proof.executable)return plan;
        if(proof.reason.find("RATE")==std::string::npos &&
           proof.reason.find("ACCELERATION")==std::string::npos &&
           proof.reason.find("SPEED")==std::string::npos &&
           proof.reason.find("JERK")==std::string::npos)break;
        phase.duration*=1.18;
      }
    }
  }
  return {};
}
}
