#include "fwis_hybrid_joint/model.hpp"
#include "policy_values.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace fwis_hybrid_joint {
namespace {
using Wheels=std::array<double,4>;
Wheels wheel_angles(Mode mode,const Body& body,const Limits& limits){
  Wheels angles{};int i=0;
  for(double x:{limits.wheelbase/2,-limits.wheelbase/2})for(double y:{limits.track/2,-limits.track/2}){
    if(mode==Mode::A)angles[i++]=std::atan2(x*body.curvature,1-y*body.curvature);
    else if(mode==Mode::O)angles[i++]=body.oblique_angle;
    else angles[i++]=std::remainder(std::atan2(x,-y),std::acos(-1.));
  }
  return angles;
}
double physical_speed(const Sample& s,std::size_t i){
  const auto& b=s.bodies[i];
  if(s.mode==Mode::S)return b.wz;
  if(s.mode==Mode::A)return b.vx;
  return std::hypot(b.vx,b.vy);
}
double clearance_penalty(const fwis_planning_core::CollisionKernel& map,const Sample& s,std::size_t count){
  Headings headings{};
  for(std::size_t v=0;v<count;++v)headings[v]=s.bodies[v].pose.yaw;
  // Largest certified symmetric footprint padding among equally spaced levels.
  int safe=0,blocked=policy::clearance_levels+1;
  while(blocked-safe>1){
    const int mid=(safe+blocked)/2;
    const double padding=policy::comfort_padding_m*mid/policy::clearance_levels;
    if(map.check(s.reference,headings.data(),count,padding).reason==fwis_planning_core::CollisionReason::clear)safe=mid;
    else blocked=mid;
  }
  return std::pow(1.-double(safe)/policy::clearance_levels,2);
}
}

bool PolicyScore::better_than(const PolicyScore& other)const{
  if((auxiliary_blocks!=0)!=(other.auxiliary_blocks!=0))return auxiliary_blocks==0;
  if(std::abs(auxiliary_cost-other.auxiliary_cost)>policy::comparison_tolerance)return auxiliary_cost<other.auxiliary_cost;
  return execution_cost<other.execution_cost;
}

PolicyScore score_policy(const Request& r,const Plan& plan,Clock::time_point deadline,const Cancel& cancel){
  const auto& l=r.limits;
  const double t0=l.radius/l.speed;
  fwis_planning_core::CollisionKernel map(*r.grid,r.vehicles,r.cargo);
  PolicyScore out;
  const auto count=r.vehicles.size();
  std::array<Wheels,3> previous_wheels{};
  std::optional<Mode> last_mode;
  Gears last_gear{};
  for(std::size_t index=0;index<plan.phases.size();++index){
    const auto& ph=plan.phases[index];
    auto check_deadline=[&](){
      if(Clock::now()>=deadline || (cancel&&cancel()))throw std::runtime_error("POLICY_EVALUATION_DEADLINE");
    };
    check_deadline();
    if(ph.kind!=Kind::Motion){
      auto stay=evaluate_phase(r,ph,0.);
      out.clearance_effort+=ph.duration/t0*clearance_penalty(map,stay,count);
      if(ph.kind==Kind::Prepare && index+1<plan.phases.size()){
        const auto& next=plan.phases[index+1];auto start=evaluate_phase(r,next,0.);
        const double travel=ph.duration-l.prepare_time;
        for(std::size_t v=0;v<count && travel>0.;++v){
          const auto targets=wheel_angles(next.mode,start.bodies[v],l);
          // Exact integral of squared speed for a quintic rest-to-rest steering move.
          for(int j=0;j<4;++j)out.steering_effort+=(10./7.)*std::pow((targets[j]-previous_wheels[v][j])/l.steer_rate,2)/(travel*t0);
        }
      }
      continue;
    }
    const int intervals=policy::intervals_per_piece*static_cast<int>(ph.pieces.size());
    const double dt=ph.duration/intervals;
    double length=0.,spin=0.;
    for(std::size_t v=0;v<count;++v)if(ph.mode==Mode::S)spin+=std::abs(ph.heading1[v]-ph.heading0[v]);
    auto left=evaluate_phase(r,ph,0.);
    for(int j=0;j<intervals;++j){
      check_deadline();
      auto mid=evaluate_phase(r,ph,(j+.5)*dt),right=evaluate_phase(r,ph,(j+1)*dt);
      // Midpoint quadrature with symmetric derivatives of the analytic body
      // feed-forward. Differences never cross a mode or prepare boundary.
      length+=std::hypot(mid.reference.x-left.reference.x,mid.reference.y-left.reference.y)+
              std::hypot(right.reference.x-mid.reference.x,right.reference.y-mid.reference.y);
      out.clearance_effort+=dt/t0*clearance_penalty(map,mid,count);
      for(std::size_t v=0;v<count;++v){
        const auto a=wheel_angles(ph.mode,left.bodies[v],l),b=wheel_angles(ph.mode,right.bodies[v],l);
        for(int w=0;w<4;++w)out.steering_effort+=dt/t0*std::pow((b[w]-a[w])/(dt*l.steer_rate),2);
        const double jerk=4*(physical_speed(left,v)-2*physical_speed(mid,v)+physical_speed(right,v))/(dt*dt);
        out.jerk_effort+=dt/t0*std::pow(jerk/(ph.mode==Mode::S?l.spin_jerk:l.jerk),2);
        previous_wheels[v]=b;
      }
      left=right;
    }
    out.reference_length+=length;
    if(ph.mode==Mode::O)out.oblique_length+=length;
    out.spin_angle+=spin;
    if(length<=policy::zero_motion_tolerance && spin<=policy::zero_motion_tolerance)continue;
    const bool same=last_mode==ph.mode && std::equal(last_gear.begin(),last_gear.begin()+count,ph.gear.begin());
    if(ph.mode!=Mode::A && !same)++out.auxiliary_blocks;
    last_mode=ph.mode;last_gear=ph.gear;
  }
  out.auxiliary_cost=policy::aux_block_weight*out.auxiliary_blocks+
    policy::oblique_length_weight*out.oblique_length/l.radius+policy::rotation_weight*out.spin_angle/policy::rotation_scale_rad;
  out.execution_cost=policy::time_weight*duration(plan)/t0+policy::length_weight*out.reference_length/l.radius+
    policy::steering_effort_weight*out.steering_effort+policy::jerk_effort_weight*out.jerk_effort+
    policy::clearance_weight*out.clearance_effort;
  return out;
}
}
