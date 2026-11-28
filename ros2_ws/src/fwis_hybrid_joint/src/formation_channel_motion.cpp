#include "search_internal.hpp"
#include "formation_lattice.hpp"
#include "channel_repair.hpp"

namespace fwis_hybrid_joint::search_detail {
namespace {
// Bound derivatives of each supported clock for a linear displacement and
// choose the quickest rest-to-rest motion within those bounds.
void time_linear_phase(Phase& motion,double length,double speed,double acceleration,double jerk,double dwell) {
  double best=std::numeric_limits<double>::infinity();
  for(double ramp:{0.,cruise_ramps[0],cruise_ramps[1],cruise_ramps[2],cruise_ramps[3]}){
    // The narrow ramps mix two clocks. The sum of their derivative
    // bounds remains valid even when the two acceleration peaks differ.
    auto bound=[&](int order){
      auto peak=[&](double w){
        if(order==1)return 1./(1.-w);
        if(order==2)return 1.5/(w*(1.-w));
        return 6./(w*w*(1.-w));
      };
      if(!ramp)return order==1?1.875:order==2?10./std::sqrt(3.):60.;
      return .875*peak(ramp)+.125*peak(.125);
    };
    const double peak=bound(1),accel=bound(2),third=bound(3);
    const double tau=1.025*std::max({dwell,peak*length/speed,
      std::sqrt(accel*length/acceleration),std::cbrt(third*length/jerk)});
    if(tau<best){best=tau;motion.duration=tau;motion.cruise_ramp_fraction=ramp;}
  }
}

void match_headings(const Request& r,Plan& plan,Sample& rest,const Headings& target,
                    bool at_ackermann_start=false){
  // The first measured pose already has a certified one-degree heading window.
  // Internal joins always use the exact continuity threshold.
  const double tolerance=at_ackermann_start && plan.phases.empty()?pi/180.:1e-8;
  for(std::size_t v=0;v<r.vehicles.size();++v){
    if(std::abs(delta(rest.bodies[v].pose.yaw,target[v]))>tolerance){
      append_motion(r,plan,stationary_spin(r,rest,target));
      rest=evaluate_phase(r,plan.phases.back(),plan.phases.back().duration);
      return;
    }
  }
}

// A clamped approximating B-spline rounds a visibility polyline without
// interpolating past its corners. Greville stations reproduce a straight
// channel exactly. The resulting quintic spans share a single station clock.
Phase translation_spline(const Route& route,double yaw,int spans){
  std::vector<double> knots(6,0.);
  for(int j=1;j<spans;++j)knots.push_back(j);
  knots.insert(knots.end(),6,spans);
  std::vector<double> xs,ys;
  for(int i=0;i<spans+5;++i){
    double station=0.;
    for(int k=1;k<=5;++k)station+=knots[i+k];
    const auto q=interpolate(route,station/(5.*spans));
    xs.push_back(q.x);ys.push_back(q.y);
  }
  const auto x=hybrid_channel::bezier_spans(std::move(xs),spans);
  const auto y=hybrid_channel::bezier_spans(std::move(ys),spans);
  Phase motion;motion.duration=1.;
  for(int i=0;i<spans;++i){
    Piece piece;piece.x=x[i];piece.y=y[i];piece.yaw.fill(yaw);
    motion.pieces.push_back(piece);
  }
  return motion;
}

Audit timed_audit(const Request& r,const Plan& plan,const Run& run,double& seconds){
  const auto start=Clock::now();
  auto audit=certify(r,plan,run.end,run.cancel);
  seconds+=std::chrono::duration<double>(Clock::now()-start).count();
  return audit;
}

std::optional<Phase> continuous_translation(const Request& r,const std::vector<Pose2d>& path,
    std::size_t start,std::size_t end,const Sample& rest,const Run& run,double& proof_s){
  Route route;
  double length=0.;
  for(std::size_t i=start;i<=end;++i){
    Point p{path[i].x,path[i].y};
    if(!route.empty())length+=distance(route.back(),p);
    route.push_back(p);
  }
  if(length<r.limits.radius || route.size()<3)return {};
  Request local=r;local.start.reference=path[start];local.goal.reference=path[end];
  local.start.constrain_heading.fill(true);local.goal.constrain_heading.fill(false);
  for(std::size_t v=0;v<r.vehicles.size();++v)local.start.heading[v]=rest.bodies[v].pose.yaw;
  std::optional<Phase> best;
  double best_cost=std::numeric_limits<double>::infinity();
  int last_spans=0;
  for(double spacing:{3.0,1.35,.65}){
    const int spans=std::clamp(int(std::ceil(length/spacing)),2,24);
    if(spans==last_spans)continue;
    last_spans=spans;
    const Phase geometry=translation_spline(route,path[start].yaw,spans);
    const auto entry=evaluate_phase(r,geometry,0.);
    const double tangent=entry.bodies[0].pose.yaw;
    for(Mode mode:{Mode::A,Mode::O}){
      if(run.stop())return best;
      Phase motion=geometry;motion.mode=mode;
      Headings headings{};
      bool aligned=true;
      for(std::size_t v=1;v<r.vehicles.size();++v)
        aligned&=std::abs(delta(local.start.heading[0],local.start.heading[v]))<1e-7;
      auto heading_travel=[&](double target){
        double maximum=0.,sum=0.;
        for(double previous:local.start.heading){
          const double d=std::abs(delta(previous,target));maximum=std::max(maximum,d);sum+=d;
        }
        return maximum+.05*sum+.04*std::abs(delta(path[end].yaw,target));
      };
      const int common_gear=r.allow_reverse && heading_travel(tangent+pi)<heading_travel(tangent)?-1:1;
      for(std::size_t v=0;v<r.vehicles.size();++v){
        const double previous=local.start.heading[v];
        if(mode==Mode::A){
          motion.gear[v]=common_gear;
          headings[v]=tangent+(motion.gear[v]<0?pi:0.);
        }else{
          headings[v]=aligned?previous:path[start].yaw;
          // Pick the branch using the whole tangent range, including entries
          // that are exactly perpendicular and can leave on either branch.
          const double central=central_tangent(route);
          motion.gear[v]=r.allow_reverse && std::cos(central-headings[v])<0.?-1:1;
        }
      }
      motion.heading0=motion.heading1=headings;
      double vmax=std::min(r.limits.speed,r.limits.wheel_speed);
      if(r.limits.cargo_center_speed>0.)vmax=std::min(vmax,r.limits.cargo_center_speed);
      if(mode==Mode::O && r.limits.high_steer_speed>0.)vmax=std::min(vmax,r.limits.high_steer_speed);
      time_linear_phase(motion,length,vmax,r.limits.acceleration,r.limits.jerk,r.limits.min_motion_duration);
      Plan block;auto at=rest;
      match_headings(r,block,at,headings);
      append_motion(r,block,motion);
      double rejected_duration=0.;
      for(int trial=0;trial<4 && !run.stop();++trial){
        const double cost=duration(block)+(mode==Mode::O?.20*length/vmax:0.);
        const double tested_duration=block.phases.back().duration;
        if(cost>=best_cost || duration(block)>r.task_duration_cap)break;
        const auto proof=timed_audit(local,block,run,proof_s);
        if(proof.executable){
          auto& phase=block.phases.back();
          double feasible=phase.duration;
          for(int k=0;k<3 && rejected_duration>0. && feasible-rejected_duration>.6 && !run.stop();++k){
            phase.duration=.5*(feasible+rejected_duration);
            if(timed_audit(local,block,run,proof_s).executable)feasible=phase.duration;
            else rejected_duration=phase.duration;
          }
          best_cost=cost-tested_duration+feasible;
          phase.duration=feasible;best=phase;break;
        }
        const auto& why=proof.reason;
        if(why.find("SPEED")==std::string::npos && why.find("ACCELERATION")==std::string::npos &&
           why.find("JERK")==std::string::npos && why.find("RATE")==std::string::npos)break;
        rejected_duration=block.phases.back().duration;block.phases.back().duration*=1.22;
      }
    }
    if(best)return best;
  }
  return best;
}

Plan realize_channel(const Request& r,const std::vector<Pose2d>& path,const Run& run,bool smooth,double& proof_s){
  Plan plan;
  Sample rest;rest.reference=r.start.reference;
  for(std::size_t v=0;v<r.vehicles.size();++v)rest.bodies[v].pose.yaw=r.start.heading[v];
  const auto& l=r.limits;
  for(std::size_t i=1;i<path.size();++i){
    const auto a=path[i-1],b=path[i];
    Phase motion;
    if(run.stop())return {};
    if(smooth){
      std::size_t end=i-1;
      while(end+1<path.size() && std::abs(path[end+1].yaw-a.yaw)<1e-8)++end;
      const auto first_leg=i;
      for(;end>first_leg;--end){
        if(auto curve=continuous_translation(r,path,i-1,end,rest,run,proof_s)){
          const auto first=evaluate_phase(r,*curve,0.);
          Headings headings{};
          for(std::size_t v=0;v<r.vehicles.size();++v)headings[v]=first.bodies[v].pose.yaw;
          match_headings(r,plan,rest,headings);
          append_motion(r,plan,std::move(*curve));
          rest=evaluate_phase(r,plan.phases.back(),plan.phases.back().duration);
          i=end;break;
        }
      }
      if(i>first_leg)continue;
    }
    const double turn=b.yaw-a.yaw;
    if(std::abs(turn)>1e-8){
      // F stays fixed, each mount follows its own circle in Ackermann.
      // S only orients the chassis at rest; it cannot rotate the cargo.
      motion.pieces={stationary_piece(a)};motion.duration=1.;
      for(int j=0;j<6;++j)motion.pieces[0].yaw[j]=a.yaw+turn*j/5.;
      double vmax=l.yaw_rate,amax=std::numeric_limits<double>::infinity(),jmax=amax;
      if(l.yaw_acceleration>0.)amax=l.yaw_acceleration;
      if(l.yaw_jerk>0.)jmax=l.yaw_jerk;
      if(l.cargo_center_speed>0.){
        const double arm=std::hypot((*r.cargo)[0],(*r.cargo)[1]);
        if(arm>0.)vmax=std::min(vmax,l.cargo_center_speed/arm);
      }
      const auto tangent=evaluate_phase(r,motion,0.);
      Headings headings{};
      for(std::size_t v=0;v<r.vehicles.size();++v){
        const auto& f=r.vehicles[v];const double radius=std::hypot(f[0],f[1]);
        headings[v]=tangent.bodies[v].pose.yaw;
        if(r.allow_reverse && std::abs(delta(rest.bodies[v].pose.yaw,headings[v]))>pi/2.){
          motion.gear[v]=-1;headings[v]+=pi;
        }
        const double outer=std::hypot(radius+l.track/2.,l.wheelbase/2.);
        double speed=l.speed;
        if(l.high_steer_speed>0. && std::atan2(l.wheelbase/2.,radius-l.track/2.)>l.high_steer_angle)
          speed=std::min(speed,l.high_steer_speed);
        vmax=std::min({vmax,speed/radius,l.wheel_speed/outer,std::sqrt(l.lateral_acceleration/radius)});
        amax=std::min(amax,l.acceleration/radius);jmax=std::min(jmax,l.jerk/radius);
      }
      match_headings(r,plan,rest,headings);
      time_linear_phase(motion,std::abs(turn),vmax,amax,jmax,l.min_motion_duration);
    }else{
      const double dx=b.x-a.x,dy=b.y-a.y,tangent=std::atan2(dy,dx);
      motion=straight_motion(r,a,b,1);
      Headings headings{};headings.fill(a.yaw);
      // Preserve the initial mount headings through the first translation.
      bool share_heading=true;
      for(std::size_t v=1;v<r.vehicles.size();++v)
        share_heading&=std::abs(delta(rest.bodies[0].pose.yaw,rest.bodies[v].pose.yaw))<1e-7;
      if(i==1 || share_heading || (i+1<path.size() && std::hypot(dx,dy)<r.limits.radius &&
                   std::abs(path[i+1].yaw-b.yaw)>1e-8)){
        for(std::size_t v=0;v<r.vehicles.size();++v)headings[v]=rest.bodies[v].pose.yaw;
      }
      match_headings(r,plan,rest,headings);
      double vmax=std::min(l.speed,l.wheel_speed);
      if(l.cargo_center_speed>0.)vmax=std::min(vmax,l.cargo_center_speed);
      for(std::size_t v=0;v<r.vehicles.size();++v){
        const double relative=delta(headings[v],tangent);
        const int gear=r.allow_reverse && std::cos(relative)<0.?-1:1;
        motion.gear[v]=gear;
        const double steer=std::abs(delta(headings[v],tangent+(gear<0?pi:0.)));
        if(steer>1e-8)motion.mode=Mode::O;
        if(l.high_steer_speed>0. && steer>l.high_steer_angle)vmax=std::min(vmax,l.high_steer_speed);
      }
      motion.heading0=motion.heading1=headings;
      time_linear_phase(motion,std::hypot(dx,dy),vmax,l.acceleration,l.jerk,l.min_motion_duration);
    }
    append_motion(r,plan,std::move(motion));
    rest=evaluate_phase(r,plan.phases.back(),plan.phases.back().duration);
  }
  Headings target{};
  for(std::size_t v=0;v<r.vehicles.size();++v)
    target[v]=r.goal.constrain_heading[v]?r.goal.heading[v]:rest.bodies[v].pose.yaw;
  match_headings(r,plan,rest,target);
  return plan;
}

// Gear is a discrete body-heading branch of the same mount curve. Choose it
// across the whole channel: a locally shorter Spin can leave all three cars
// facing backwards at the next dock. The final sweep is checked independently.
Plan choose_channel_gears(const Request& r,const Plan& source,const Run& run){
  if(source.phases.empty())return source;
  struct Label {Phase motion;Sample end;int parent;double cost;};
  std::vector<std::vector<Label>> layers;
  Sample initial;initial.reference=r.start.reference;
  for(std::size_t v=0;v<3;++v)initial.bodies[v].pose.yaw=r.start.heading[v];
  for(const auto& original:source.phases){
    if(run.stop())return source;
    if(original.kind!=Kind::Motion || original.mode==Mode::S)continue;
    const auto& q=original.pieces.front();
    const bool pivot=original.mode==Mode::A && std::abs(q.yaw.front()-original.pieces.back().yaw.back())>1e-8;
    std::vector<Label> layer;
    for(int mask=0;mask<(r.allow_reverse?8:1);++mask){
      if(!pivot && mask!=0 && mask!=7)continue;
      Phase motion=original;
      for(int v=0;v<3;++v)if(mask&(1<<v)){
        motion.gear[v]*=-1;
        if(motion.mode==Mode::O)motion.heading0[v]=motion.heading1[v]=original.heading0[v]+pi;
      }
      const auto first=evaluate_phase(r,motion,0.);
      Headings headings{};
      for(int v=0;v<3;++v)headings[v]=first.bodies[v].pose.yaw;
      Label label{motion,evaluate_phase(r,motion,motion.duration),-1,std::numeric_limits<double>::infinity()};
      const auto count=layers.empty()?1:static_cast<int>(layers.back().size());
      for(int j=0;j<count;++j){
        Plan link;
        auto rest=initial;
        double cost=0.;
        if(!layers.empty()){
          const auto& prev=layers.back()[j];link.phases.push_back(prev.motion);
          rest=prev.end;cost=prev.cost-prev.motion.duration;
        }
        match_headings(r,link,rest,headings,motion.mode==Mode::A);
        append_motion(r,link,motion);
        cost+=duration(link);
        if(cost<label.cost){label.cost=cost;label.parent=j;}
      }
      layer.push_back(std::move(label));
    }
    layers.push_back(std::move(layer));
  }
  if(layers.empty())return source;
  auto goal_headings=[&](const Sample& rest,bool align){
    Headings target{};
    for(int v=0;v<3;++v)target[v]=r.goal.constrain_heading[v]?r.goal.heading[v]:
      align?r.goal.reference.yaw:rest.bodies[v].pose.yaw;
    return target;
  };
  int best=0;
  double lowest=std::numeric_limits<double>::infinity();
  for(std::size_t i=0;i<layers.back().size();++i){
    const auto& label=layers.back()[i];
    auto rest=label.end;Plan tail{{label.motion}};
    match_headings(r,tail,rest,goal_headings(rest,r.align_terminal_chassis));
    const double cost=label.cost+duration(tail)-label.motion.duration;
    if(cost<lowest){lowest=cost;best=int(i);}
  }
  std::vector<int> motion_indices(layers.size());
  for(std::size_t i=layers.size();i-->0;){
    motion_indices[i]=best;best=layers[i][best].parent;
  }
  Plan plan;auto rest=initial;
  for(std::size_t i=0;i<layers.size();++i){
    const auto& label=layers[i][motion_indices[i]];
    const auto first=evaluate_phase(r,label.motion,0.);
    Headings headings{};
    for(int v=0;v<3;++v)headings[v]=first.bodies[v].pose.yaw;
    match_headings(r,plan,rest,headings,label.motion.mode==Mode::A);append_motion(r,plan,label.motion);
    rest=label.end;
  }
  match_headings(r,plan,rest,goal_headings(rest,false));
  return plan;
}

// Replace a sequence of translations and fixed-reference yaw corrections with
// one Ackermann phase. Position and cargo yaw share the same station, while
// the tangent of each actual mount determines that vehicle's own body yaw.
struct CruiseGuide {
  Route xy;
  double length{}, initial_yaw{};
  struct Turn { double station, angle; };
  std::vector<Turn> turns;
  void append(Point next,double yaw_change){
    const double step=distance(xy.back(),next);
    if(std::abs(yaw_change)>1e-8)turns.push_back({length+.5*step,yaw_change});
    if(step>1e-7){xy.push_back(next);length+=step;}
  }

  CruiseGuide(const std::vector<Phase>& motions,std::size_t first,std::size_t end){
    const auto& start=motions[first].pieces.front();
    xy.push_back({start.x.front(),start.y.front()});initial_yaw=start.yaw.front();
    for(std::size_t j=first;j<end;++j)for(const auto& piece:motions[j].pieces)
      append({piece.x.back(),piece.y.back()},piece.yaw.back()-piece.yaw.front());
  }

  explicit CruiseGuide(const std::vector<Pose2d>& path){
    xy.push_back({path.front().x,path.front().y});initial_yaw=path.front().yaw;
    for(std::size_t i=1;i<path.size();++i)
      append({path[i].x,path[i].y},path[i].yaw-path[i-1].yaw);
  }

  Phase curve(double spacing,double turn_range,std::optional<double> entry,
              std::optional<double> exit=std::nullopt,double station_power=1.)const{
    const int spans=std::clamp(int(std::ceil(length/spacing)),1,24);
    std::vector<double> knots(6,0.);
    for(int i=1;i<spans;++i)knots.push_back(i);
    knots.insert(knots.end(),6,spans);
    std::vector<double> x,y,yaw;
    for(int i=0;i<spans+5;++i){
      double u=0.;
      for(int k=1;k<=5;++k)u+=knots[i+k];
      u/=5.*spans;
      if(station_power!=1.)u=std::pow(u,station_power);
      const auto p=interpolate(xy,u);
      x.push_back(p.x);y.push_back(p.y);
      double psi=initial_yaw;
      for(const auto& rotation:turns){
        // A broad turn window distributes cargo yaw over the neighboring
        // cruise legs. A narrow window preserves a constrained doorway.
        const auto lo=std::max(0.,rotation.station-turn_range);
        const auto hi=std::min(length,rotation.station+turn_range);
        psi+=rotation.angle*std::clamp((length*u-lo)/(hi-lo),0.,1.);
      }
      yaw.push_back(psi);
    }
    auto set_tangent=[&](int anchor,int step,double heading){
      const int next=anchor+step;
      const double dx=x[next]-x[anchor],dy=y[next]-y[anchor];
      if(dx*std::cos(heading)+dy*std::sin(heading)<0.)heading+=pi;
      // The tangent axis admits either gear. Two collinear control edges
      // start or finish with zero curvature at a stationary join.
      for(int j=1;j<=2;++j){
        const int k=anchor+j*step;
        const double d=std::hypot(x[k]-x[anchor],y[k]-y[anchor]);
        x[k]=x[anchor]+d*std::cos(heading);y[k]=y[anchor]+d*std::sin(heading);
      }
    };
    if(entry)set_tangent(0,1,*entry);
    if(exit)set_tangent(static_cast<int>(x.size())-1,-1,*exit);
    // A zero endpoint yaw derivative gives the three mounts parallel
    // tangents at a join to a translation with parallel chassis.
    yaw[1]=yaw.front();yaw[yaw.size()-2]=yaw.back();
    const auto xs=hybrid_channel::bezier_spans(std::move(x),spans);
    const auto ys=hybrid_channel::bezier_spans(std::move(y),spans);
    const auto ps=hybrid_channel::bezier_spans(std::move(yaw),spans);
    Phase phase;phase.duration=1.;
    for(int j=0;j<spans;++j)phase.pieces.push_back({xs[j],ys[j],ps[j]});
    return phase;
  }
};

// A sampled rejection / duration estimate only. The entire assembled plan
// must still pass the ordinary continuous certificate, including new joins.
bool cruise_timing(const Request& r,const fwis_planning_core::CollisionKernel& map,
    Phase& phase,double length){
  const auto& l=r.limits;
  double vmax=std::min(l.speed,l.wheel_speed);
  if(l.cargo_center_speed>0.)vmax=std::min(vmax,l.cargo_center_speed);
  time_linear_phase(phase,length,vmax,l.acceleration,l.jerk,l.min_motion_duration);
  double stretch=1.;
  const auto samples=std::max(20,int(phase.pieces.size())*10);
  try{
    for(int j=0;j<=samples;++j){
      auto s=evaluate_phase(r,phase,phase.duration*j/samples);
      Headings headings{};
      for(std::size_t i=0;i<r.vehicles.size();++i){
        const auto& b=s.bodies[i];
        if(std::abs(b.curvature)>1./l.radius)return false;
        stretch=std::max({stretch,std::abs(b.vx)/vmax,std::abs(b.wz)/l.yaw_rate});
        headings[i]=b.pose.yaw;
      }
      if(map.check(s.reference,headings.data(),r.vehicles.size()).reason!=fwis_planning_core::CollisionReason::clear)
        return false;
    }
  }catch(const std::domain_error&){return false;}
  phase.duration*=1.04*stretch;
  return true;
}

// Use the same joint footprint/curvature least-squares repair as the direct
// Ackermann solver, warm-started in the homotopy found by the posture lattice.
bool shape_cruise(const Request& r,const std::vector<channel_repair::Box>& boxes,
    Phase& curve,const Run& run){
  const int spans=curve.pieces.size();
  if(spans>HJ_MAX_SPANS || spans<2)return false;
  HJProblem problem{};
  auto& ph=problem.phase[0];ph.spans=spans;ph.gear[0]=1;
  auto local=r;
  const auto& start=curve.pieces.front();
  const auto& end=curve.pieces.back();
  ph.h0[0]=std::atan2(start.y[1]-start.y[0],start.x[1]-start.x[0]);
  local.start.reference={start.x.front(),start.y.front(),start.yaw.front()};
  local.goal.reference={end.x.back(),end.y.back(),end.yaw.back()};
  for(int j=0;j<spans;++j){
    const auto& q=curve.pieces[j];
    for(int k=0;k<6;++k){
      ph.coefficient[j][0][k]=q.x[k];ph.coefficient[j][1][k]=q.y[k];
      ph.coefficient[j][2][k]=q.yaw[k];
    }
  }
  if(!channel_repair::repair(local,problem,boxes,spans,0.,1.,[&](){return run.stop();},true))
    return false;
  for(int j=0;j<spans;++j){
    auto& q=curve.pieces[j];
    std::copy_n(ph.coefficient[j][0],6,q.x.begin());
    std::copy_n(ph.coefficient[j][1],6,q.y.begin());
    std::copy_n(ph.coefficient[j][2],6,q.yaw.begin());
  }
  return true;
}

// A shorter phase may carry a more costly final spin. Compare command runs
// and predicted duration all the way through the requested terminal alignment.
struct CruiseQuality {int switches{}, stops{};double seconds{};bool aligned{};};
CruiseQuality cruise_quality(const Plan& plan){
  std::optional<Mode> last;
  CruiseQuality quality;
  quality.seconds=duration(plan);
  for(const auto& phase:plan.phases){
    if(phase.kind!=Kind::Motion)continue;
    quality.switches+=last && *last!=phase.mode;
    quality.stops+=last.has_value();last=phase.mode;
  }
  return quality;
}

CruiseQuality terminal_quality(const Request& r,const Plan& plan,const Run& run,double& proof_s){
  if(!r.align_terminal_chassis)return cruise_quality(plan);
  const auto start=Clock::now();
  Result terminal;terminal.plan=plan;
  align_terminal(r,run,terminal);
  proof_s+=std::chrono::duration<double>(Clock::now()-start).count();
  auto quality=cruise_quality(*terminal.plan);
  quality.aligned=terminal.terminal_alignment=="aligned" || terminal.terminal_alignment=="already_aligned";
  return quality;
}

void refine_cruise(const Request& r,const std::vector<channel_repair::Box>& boxes,const Run& run,Plan& incumbent,double& proof_s){
  const fwis_planning_core::CollisionKernel map(*r.grid,r.vehicles,r.cargo);
  auto motions_of=[](const Plan& plan){
    std::vector<Phase> motions;
    for(const auto& phase:plan.phases)
      if(phase.kind==Kind::Motion && phase.mode!=Mode::S)motions.push_back(phase);
    return motions;
  };
  auto motions=motions_of(incumbent);
  auto best=terminal_quality(r,incumbent,run,proof_s);
  for(std::size_t width=motions.size();width>=1 && !run.stop();--width){
    for(std::size_t first=0;first+width<=motions.size() && !run.stop();++first){
      const auto end=first+width;
      if(width==1 && motions[first].mode!=Mode::O)continue;
      CruiseGuide guide(motions,first,end);
      if(guide.length<r.limits.radius)continue;
      bool accepted=false;
      for(double spacing:{3.0,5.0,2.0,1.25}){
        if(accepted || run.stop())break;
        for(double radius:{6.0,3.0,1.5}){
          if(run.stop())break;
          std::optional<double> entry;
          if(!first)entry=r.start.heading[0];
          else if(width==1)entry=motions[first].heading0[0];
          auto curve=guide.curve(spacing,radius,entry);
          if(!cruise_timing(r,map,curve,guide.length)){
            const Run repair_run{std::min(run.end,Clock::now()+std::chrono::milliseconds(180)),run.cancel};
            if(spacing!=3. || radius!=6. || !shape_cruise(r,boxes,curve,repair_run) ||
               !cruise_timing(r,map,curve,guide.length)){
              if(guide.turns.empty())break;
              continue;
            }
          }
          for(int k=0;k<4 && !run.stop();++k){
            Plan proposed;
            proposed.phases.assign(motions.begin(),motions.begin()+first);
            proposed.phases.push_back(curve);
            proposed.phases.insert(proposed.phases.end(),motions.begin()+end,motions.end());
            // Recompute all mode preparations and spin headings after the splice.
            proposed=choose_channel_gears(r,proposed,run);
            if(run.stop())break;
            const auto quality=cruise_quality(proposed);
            if(quality.seconds>=best.seconds || quality.switches>best.switches || quality.stops>best.stops)break;
            const auto proof=timed_audit(r,proposed,run,proof_s);
            if(proof.executable){
              const auto complete=terminal_quality(r,proposed,run,proof_s);
              if(run.stop() || (best.aligned && !complete.aligned) || complete.seconds>=best.seconds ||
                 complete.switches>best.switches || complete.stops>best.stops)break;
              incumbent=std::move(proposed);best=complete;
              motions=motions_of(incumbent);
              accepted=true;break;
            }
            const auto& why=proof.reason;
            if(why.find("SPEED")==std::string::npos && why.find("ACCELERATION")==std::string::npos &&
               why.find("JERK")==std::string::npos && why.find("RATE")==std::string::npos)break;
            curve.duration*=1.20;
          }
          if(accepted || guide.turns.empty())break;
        }
      }
      if(accepted){width=motions.size()+1;break;}
    }
  }
}

// Check the complete start, cruise and docking operation. The optional
// shape-fitting clearance is only a seed; certification always uses r.
bool certify_cruise(const Request& r,const Run& run,const CruiseQuality& previous,
                    Phase curve,Plan& incumbent,double& proof_s){
  for(int attempt=0;attempt<7 && !run.stop();++attempt){
    auto proposed=choose_channel_gears(r,Plan{{curve}},run);
    if(duration(proposed)>=previous.seconds)return false;
    const auto proof=timed_audit(r,proposed,run,proof_s);
    if(proof.executable){
      const auto next=terminal_quality(r,proposed,run,proof_s);
      if(run.stop() || (previous.aligned && !next.aligned) || next.seconds>=previous.seconds ||
         next.switches>previous.switches || next.stops>previous.stops)return false;
      incumbent=std::move(proposed);
      return true;
    }
    const auto& why=proof.reason;
    if(why.find("SPEED")==std::string::npos && why.find("ACCELERATION")==std::string::npos &&
       why.find("JERK")==std::string::npos && why.find("RATE")==std::string::npos)break;
    curve.duration*=1.14;
  }
  return false;
}

// A collocation fit can skim a doorpost between samples. Give its geometry
// a fraction of a cell of extra clearance, then certify on the original map.
auto fitting_obstacles(const std::vector<channel_repair::Box>& boxes,double resolution){
  auto padded=boxes;
  for(auto& box:padded){box[2]+=.30*resolution;box[3]+=.30*resolution;}
  return padded;
}

// Fit in two local posture charts when a long constant-yaw aisle separates
// manoeuvring regions. Each chart has its own fitting resolution. Their
// geometric C3 join lies inside the aisle and uses one rest-to-rest clock.
std::optional<Pose2d> aisle_join(const Plan& source,double radius,const std::vector<Pose2d>& path){
  double longest=0.;
  std::optional<Pose2d> join;
  for(const auto& phase:source.phases){
    if(phase.kind!=Kind::Motion || phase.mode!=Mode::O || phase.pieces.size()<3)continue;
    const auto& first=phase.pieces.front();const auto& last=phase.pieces.back();
    const double length=std::hypot(last.x.back()-first.x.front(),last.y.back()-first.y.front());
    if(length<=std::max(6.*radius,longest))continue;
    const auto& inside=phase.pieces[phase.pieces.size()-2];
    join=Pose2d{inside.x.back(),inside.y.back(),inside.yaw.back()};longest=length;
  }
  // An earlier repair may already have absorbed the oblique phase. Split
  // the longest straight approach to a rotation, inside the aisle so both
  // turn regions have room for a smooth connection.
  if(!join){
    for(std::size_t i=1;i+1<path.size();++i){
      const auto &a=path[i-1],&b=path[i],&c=path[i+1];
      if(std::abs(delta(a.yaw,b.yaw))>1e-8 || std::abs(delta(b.yaw,c.yaw))<1e-4)continue;
      const double length=std::hypot(b.x-a.x,b.y-a.y);
      if(length<=std::max(6.*radius,longest))continue;
      join=Pose2d{a.x+(b.x-a.x)*.68,a.y+(b.y-a.y)*.68,b.yaw};longest=length;
    }
  }
  return join;
}

// Two quintics constrained by position and the first three derivatives
// at each outside end and by the same four conditions at their midpoint.
std::array<Piece,2> bridge_quintics(const Piece& before,const Piece& after){
  std::array<Piece,2> bridge;
  for(auto axis:{&Piece::x,&Piece::y,&Piece::yaw}){
    const auto& p=before.*axis;const auto& q=after.*axis;
    auto& a=bridge[0].*axis;auto& b=bridge[1].*axis;
    auto extrapolate=[](double end,double one,double two,double three){
      return std::array<double,4>{end,2.*end-one,4.*end-4.*one+two,
                                  8.*end-12.*one+6.*two-three};
    };
    const auto left=extrapolate(p[5],p[4],p[3],p[2]);
    const auto right=extrapolate(q[0],q[1],q[2],q[3]);
    std::copy(left.begin(),left.end(),a.begin());
    std::copy(right.begin(),right.end(),b.rbegin());
    a[5]=b[0]=.25*(3.*a[3]+3.*b[2]-a[2]-b[3]);
    a[4]=a[5]+.25*(a[3]-b[2]);b[1]=2.*a[5]-a[4];
  }
  return bridge;
}

bool aisle_cruise(const Request& r,const std::vector<channel_repair::Box>& boxes,
    const Pose2d& join,int angles,double step,const Run& run,Plan& incumbent,double& proof_s){
  double lo=0.,hi=0.;
  for(double yaw:r.start.heading){
    const double a=delta(r.start.heading[0],yaw);lo=std::min(lo,a);hi=std::max(hi,a);
  }
  if(hi-lo>2.*pi/180. || run.stop())return false;
  const double entry=r.start.heading[0]+.5*(hi+lo);
  Request left=r,right=r;left.goal.reference=right.start.reference=join;
  auto left_path=formation_channel(left,boxes,angles,step,[&](){return run.stop();});
  auto right_path=formation_channel(right,boxes,angles,step,[&](){return run.stop();});
  if(left_path.size()<2 || right_path.size()<2)return false;
  const CruiseGuide head(left_path),tail(right_path);
  if(head.turns.empty() || tail.turns.empty() ||
     head.length<r.limits.radius || tail.length<r.limits.radius)return false;
  const fwis_planning_core::CollisionKernel map(*r.grid,r.vehicles,r.cargo);
  const auto padded=fitting_obstacles(boxes,r.grid->resolution());
  const int n=HJ_MAX_SPANS;
  auto first=head.curve(head.length/(n-.25),head.length*.55,entry,std::nullopt,1.5);
  if(!shape_cruise(left,padded,first,run))return false;
  const auto& end=first.pieces.back();
  const double tangent=std::atan2(end.y.back()-end.y[4],end.x.back()-end.x[4]);
  auto second=tail.curve(tail.length/(n-.25),tail.length*.30,tangent);
  if(!shape_cruise(right,padded,second,run))return false;
  // Fitters unwrap yaw relative to their own chart origin. Put both pieces
  // on the same angular branch before matching derivatives in the aisle.
  const double offset=end.yaw.back()-second.pieces.front().yaw.front();
  const double winding=offset-std::remainder(offset,2*pi);
  for(auto& piece:second.pieces)for(double& yaw:piece.yaw)yaw+=winding;
  first.pieces.pop_back();
  const auto middle=bridge_quintics(first.pieces.back(),second.pieces[1]);
  first.pieces.insert(first.pieces.end(),middle.begin(),middle.end());
  first.pieces.insert(first.pieces.end(),second.pieces.begin()+1,second.pieces.end());
  if(!cruise_timing(r,map,first,head.length+tail.length))return false;
  return certify_cruise(r,run,terminal_quality(r,incumbent,run,proof_s),std::move(first),incumbent,proof_s);
}

// The first certified channel can include two spins around a fixed-F pivot.
// Fit the cargo turn while translating. Both the certified curve and the
// original lattice bends are useful seeds, since the first fit may already
// have rounded away a turn needed to negotiate a narrow passage.
bool moving_turn(const Request& r,const std::vector<channel_repair::Box>& boxes,
                 const std::vector<Pose2d>& path,const Run& run,Plan& incumbent,double& proof_s,
                 bool prefer_lattice=false){
  std::vector<Phase> motions;
  for(const auto& phase:incumbent.phases)
    if(phase.kind==Kind::Motion && phase.mode!=Mode::S)motions.push_back(phase);
  if(motions.size()<2 || run.stop())return false;
  const CruiseGuide fitted(motions,0,motions.size()),lattice(path);
  if(lattice.turns.empty() || lattice.length<r.limits.radius || fitted.length<r.limits.radius)return false;
  const auto previous=terminal_quality(r,incumbent,run,proof_s);
  const fwis_planning_core::CollisionKernel map(*r.grid,r.vehicles,r.cargo);
  auto attempt=[&](const CruiseGuide& guide,int spans,double fraction,
                   std::optional<double> entry,std::optional<double> exit,
                   const std::vector<channel_repair::Box>& obstacles){
    if(run.stop())return false;
    auto curve=guide.curve(guide.length/(spans-.25),guide.length*fraction,entry,exit);
    return shape_cruise(r,obstacles,curve,run) && cruise_timing(r,map,curve,guide.length) &&
      certify_cruise(r,run,previous,std::move(curve),incumbent,proof_s);
  };
  const double heading=r.start.heading[0];
  // For an alternate yaw grid the incumbent may have substantially longer
  // angular winding. Start from the new lattice's exact boundary poses.
  if(prefer_lattice){
    for(double window:{.56,.45,.30,.67})
      if(attempt(lattice,HJ_MAX_SPANS,window,heading,std::nullopt,boxes))return true;
  }
  const double departure=std::atan2(fitted.xy[1].y-fitted.xy[0].y,fitted.xy[1].x-fitted.xy[0].x);
  const bool crosswise=std::abs(std::cos(departure-heading))<.70;
  // A sideways departure can use one endpoint spin and enter A along the
  // aisle. Matching the far dock's axis can eliminate the final spin too.
  if(crosswise)for(double window:{.30,.45,.525,.67})
    if(attempt(fitted,HJ_MAX_SPANS,window,departure,r.goal.reference.yaw,boxes))return true;
  for(double window:{.30,.67,.45})
    if(attempt(fitted,HJ_MAX_SPANS,window,heading,std::nullopt,boxes))return true;

  const auto padded=fitting_obstacles(boxes,r.grid->resolution());
  const double tangent=std::atan2(lattice.xy[1].y-lattice.xy[0].y,lattice.xy[1].x-lattice.xy[0].x);
  for(double window:{.30,.45,.67}){
    if(attempt(lattice,HJ_MAX_SPANS,window,tangent,r.goal.reference.yaw,padded) ||
       attempt(lattice,HJ_MAX_SPANS,window,std::nullopt,std::nullopt,padded) ||
       attempt(lattice,HJ_MAX_SPANS,window,tangent,std::nullopt,padded))return true;
    if(run.stop())return false;
  }
  for(double window:{.20,.85,1.})
    if(attempt(fitted,HJ_MAX_SPANS,window,heading,std::nullopt,boxes))return true;
  for(double window:{.30,.67,.45,.20,.85,1.})
    if(attempt(fitted,HJ_MAX_SPANS-2,window,heading,std::nullopt,boxes))return true;
  return false;
}

} // namespace

std::optional<Plan> search_formation_channel(const Request& r,const Run& run,Result& result,Clock::time_point& first_proof){
  const auto stopped=[&](){return run.stop();};
  // A pivot about F requires a nonzero admissible Ackermann radius for
  // every mount. Other formations still have the ordinary joint search.
  for(const auto& car:r.vehicles)
    if(std::hypot(car[0],car[1])<r.limits.radius+1e-8)return {};
  const auto boxes=channel_repair::rectangles(*r.grid,stopped);
  if(boxes.empty())return {};
  for(const auto& [angles,step]:{std::pair<int,double>{4,.20},{16,.15},{72,.10}}){
    if(stopped())break;
    const auto start=Clock::now();
    auto path=formation_channel(r,boxes,angles,step,stopped);
    double proof_s=0.;
    std::string status="NO_CHANNEL";
    std::optional<Plan> solution;
    if(path.size()>=2 && !stopped()){
      PolicyScore best;
      for(bool smooth:{false,true}){
        // Retain a ranked, certified incumbent while rounding corners. If
        // refinement expires it cannot evict an executable segmented plan.
        const auto now=Clock::now();
        const Run trial{solution?now+(run.end-now)/2:run.end,run.cancel};
        auto candidate=realize_channel(r,path,trial,smooth,proof_s);
        auto optimized=choose_channel_gears(r,candidate,trial);
        for(Plan* variant:{&optimized,&candidate}){
          if(trial.stop())break;
          const auto audit=timed_audit(r,*variant,trial,proof_s);
          if(audit.executable){
            if(first_proof==Clock::time_point{})first_proof=Clock::now();
            try{
              auto rank=score_policy(r,*variant,trial.end,trial.cancel);
              if(!solution || rank.better_than(best)){best=rank;solution=std::move(*variant);}
            }catch(const std::runtime_error&){if(!trial.stop())throw;}
            break;
          }
          status=audit.reason+"@"+std::to_string(audit.phase);
        }
        if(!solution)result.trace.push_back("FORMATION_LATTICE_"+std::to_string(angles)+
          (smooth?":SPLINE:":":SEGMENTS:")+status+":duration_s="+std::to_string(duration(candidate)));
      }
    }
    if(solution && !stopped()){
      const auto previous=duration(*solution);
      const auto polish_end=std::min(run.end,Clock::now()+std::chrono::milliseconds(2600));
      refine_cruise(r,boxes,{polish_end,run.cancel},*solution,proof_s);
      const auto turn_end=std::min(run.end-(run.end-Clock::now())/8,
                                  Clock::now()+std::chrono::milliseconds(1500));
      const auto join=aisle_join(*solution,r.limits.radius,path);
      bool cruising=moving_turn(r,boxes,path,{turn_end,run.cancel},*solution,proof_s);
      // The start-anchored yaw bins can pick an unnecessarily long turn
      // when the measured start yaw changes slightly. Keep that certified
      // fallback and try a different angular discretization.
      if(!cruising && std::abs(path.back().yaw-path.front().yaw)>pi){
        const auto retry_end=std::min(run.end-(run.end-Clock::now())/8,
                                      Clock::now()+std::chrono::milliseconds(1900));
        const Run retry{retry_end,run.cancel};
        auto alternate=formation_channel_on_goal_grid(r,boxes,angles,step,[&](){return retry.stop();});
        if(alternate.size()>=2 && std::abs(alternate.back().yaw-alternate.front().yaw)<=pi)
          cruising=moving_turn(r,boxes,alternate,retry,*solution,proof_s,true);
        if(cruising)result.trace.push_back("CERTIFIED_GOAL_GRID");
      }
      if(cruising)result.trace.push_back("CERTIFIED_MOVING_CARGO_TURN");
      if(join && !stopped() && std::any_of(solution->phases.begin(),solution->phases.end(),
          [](const Phase& phase){return phase.kind==Kind::Motion && phase.mode!=Mode::A;})){
        const Run local{std::min(run.end-(run.end-Clock::now())/8,Clock::now()+std::chrono::milliseconds(420)),run.cancel};
        if(aisle_cruise(r,boxes,*join,angles,step,local,*solution,proof_s))
          result.trace.push_back("CERTIFIED_C3_AISLE_CRUISE");
      }
      if(duration(*solution)<previous)result.trace.push_back("CERTIFIED_ACKERMANN_CRUISE_SPLICE");
    }
    if(solution)status="CERTIFIED";
    const std::string label="FORMATION_LATTICE_"+std::to_string(angles);
    result.trace.push_back(label+":"+status);
    const double wall=std::chrono::duration<double>(Clock::now()-start).count();
    result.attempts.push_back({label,status,0.,0.,proof_s,wall});
    result.seed_seconds+=wall-proof_s;
    if(solution)return solution;
  }
  return {};
}

} // namespace fwis_hybrid_joint::search_detail
