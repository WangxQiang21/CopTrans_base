#include "fwis_hybrid_joint/model.hpp"
#include "nlp_api.h"
#include "clock.hpp"
#include "channel_spline.hpp"
#include "channel_repair.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace fwis_hybrid_joint {
namespace {
using Point=fwis_planning_core::Point2d;
using fwis_planning_core::CollisionKernel;
using fwis_planning_core::CollisionReason;
constexpr double pi=3.14159265358979323846;
double delta(double a,double b){return std::remainder(b-a,2*pi);}
double distance(Point a,Point b){return std::hypot(a.x-b.x,a.y-b.y);}
struct Run {
  Clock::time_point end;
  const Cancel& cancel;
  bool stop()const{return Clock::now()>=end || (cancel && cancel());}
};
extern "C" int stop_nlp(void *r){return static_cast<Run*>(r)->stop();}
using Route=std::vector<Point>;
using Sequence=std::vector<Mode>;
struct Seed { Sequence seq; int sign; Route route; std::string edit; int reverse_from{-1}; bool quintic{}, general{}, relaxed{}; int priority{8}; bool shortest_yaw{}, smooth_channel{}; std::vector<double> channel_yaw{}; bool curved_oblique{}; double switch_station{.5}; };
// Occupancy A* only proposes continuous optimization starts. Full swept footprint
// checks (including independent vehicle headings) are performed by the certifier.
Route grid_seed(const Request& r,const Run& run,bool wide=false,int gear=1,std::vector<double>* yaw_seed=nullptr,const Route* fallback=nullptr,bool oblique=false) {
  const auto& map=*r.grid;
  const int w=map.width(),h=map.height(); const double res=map.resolution();
  const auto org=map.origin();
  if(static_cast<std::uint64_t>(w)*h>16000000) return {};
  auto cell=[&](Point p) {const int x=std::floor((p.x-org.x)/res),y=std::floor((p.y-org.y)/res);
    return x<0 || y<0 || x>=w || y>=h?-1:y*w+x;};
  auto point=[&](int id){return Point{org.x+(id%w+.5)*res,org.y+(id/w+.5)*res};};
  Point start{r.start.reference.x,r.start.reference.y}, goal{r.goal.reference.x,r.goal.reference.y};
  int from=cell(start),to=cell(goal);
  if(from<0 || to<0 || from==to)return {};
  // Chamfer clearance is a seed cost only. Sweeps use exact occupied cells.
  std::vector<float> clear(std::size_t(w)*h, std::numeric_limits<float>::infinity());
  for(int y=0;y<h;++y)for(int x=0;x<w;++x){
    int id=y*w+x;
    if (map.occupied(x,y)) clear[id]=0;
    else clear[id]=std::min({x+.5f,y+.5f,w-x-.5f,h-y-.5f});
    if(x>0)clear[id]=std::min(clear[id],clear[id-1]+1);
    if(y>0) {
      clear[id]=std::min(clear[id],clear[id-w]+1);
      if(x>0)clear[id]=std::min(clear[id],clear[id-w-1]+1.414214f);
      if(x+1<w)clear[id]=std::min(clear[id],clear[id-w+1]+1.414214f);
    }
  }
  for(int y=h-1;y>=0;--y)for(int x=w-1;x>=0;--x){
    int id=y*w+x;
    if(x+1<w)clear[id]=std::min(clear[id],clear[id+1]+1);
    if(y+1<h){
      clear[id]=std::min(clear[id],clear[id+w]+1);
      if(x>0)clear[id]=std::min(clear[id],clear[id+w-1]+1.414214f);
      if(x+1<w)clear[id]=std::min(clear[id],clear[id+w+1]+1.414214f);
    }
  }
  double reach=0;
  for(const auto& f:r.vehicles) reach=std::max(reach,std::hypot(f[0],f[1])+.5*std::hypot(f[2],f[3]));
  if(r.cargo){auto f=*r.cargo;reach=std::max(reach,std::hypot(f[0],f[1])+.5*std::hypot(f[2],f[3]));}
  struct ClearanceDisk {double x,y,radius;};
  std::vector<ClearanceDisk> cover;
  if(wide){
    // A small union of disks follows cargo orientation. A single enclosing
    // circle would close legitimate three-metre corridors on all headings.
    if(r.cargo){
      const auto& f=*r.cargo;
      const int nx=std::max(1,int(std::ceil(f[2]/.60))),ny=std::max(1,int(std::ceil(f[3]/.60)));
      const double sx=f[2]/nx,sy=f[3]/ny;
      for(int iy=0;iy<ny;++iy)for(int ix=0;ix<nx;++ix)
        cover.push_back({f[0]+(ix+.5)*sx-f[2]/2,f[1]+(iy+.5)*sy-f[3]/2,.5*std::hypot(sx,sy)});
    }
    for(const auto& f:r.vehicles)cover.push_back({f[0],f[1],.5*std::hypot(f[2],f[3])});
  }
  auto clearance_at=[&](Point q,double yaw){
    const double co=std::cos(yaw),si=std::sin(yaw);
    double gap=std::numeric_limits<double>::infinity();
    for(const auto& disk:cover){
      const int at=cell({q.x+co*disk.x-si*disk.y,q.y+si*disk.x+co*disk.y});
      gap=std::min(gap,(at<0?0.:clear[at]*res)-disk.radius);
    }
    return gap;
  };
  std::vector<float> gap_cache(wide?w*h:0,std::numeric_limits<float>::quiet_NaN());
  auto body_gap=[&](int id){
    if(std::isnan(gap_cache[id])){
      const auto q=point(id);
      const double ds=distance(q,start),dg=distance(q,goal),u=ds/std::max(1.e-8,ds+dg);
      const double yaw=r.start.reference.yaw+u*u*(3.-2.*u)*delta(r.start.reference.yaw,r.goal.reference.yaw);
      gap_cache[id]=static_cast<float>(clearance_at(q,yaw));
    }
    return gap_cache[id];
  };
  bool broad_clear=true;
  const int line_n=std::max(1,int(std::ceil(distance(start,goal)/res)));
  for(int j=0;j<=line_n;++j){double u=double(j)/line_n;int id=cell({start.x+u*(goal.x-start.x),start.y+u*(goal.y-start.y)});
    if(id<0 || clear[id]*res<reach+.75){broad_clear=false;break;}}
  if(broad_clear)return {};
  std::vector<double> g(w*h,std::numeric_limits<double>::infinity());
  std::vector<int> parent(w*h,-1);
  using Node=std::pair<double,int>;
  std::priority_queue<Node,std::vector<Node>,std::greater<Node>> frontier;
  g[from]=0;frontier.emplace(distance(point(from),point(to)),from);
  while(!frontier.empty() && !run.stop()){
    auto [f,id]=frontier.top();frontier.pop();
    if(id==to)break;
    if(f>g[id]+distance(point(id),point(to))+1.e-7)continue;
    for(int y=-1;y<=1;++y) for(int x=-1;x<=1;++x){
      const int nx=id%w+x,ny=id/w+y;
      if((!x&&!y) || nx<0 || ny<0 || nx>=w || ny>=h)continue;
      int next=ny*w+nx;
      if(clear[next]<1 || (x && y && (clear[id+x]<1||clear[id+y*w]<1)))continue;
      double clearance=clear[next]*res;
      double tangent_cost=0.;
      if(wide){
        const auto q=point(next);
        const double initial=distance(q,start);
        const double gap=body_gap(next);
        if(gap<-.025)continue;
        tangent_cost=.65/std::pow(.3+std::max(0.,gap),2);
        const double approach=2.5*r.limits.radius;
        if(!oblique && initial<approach){
          const double alignment=gear*(x*std::cos(r.start.heading[0])+y*std::sin(r.start.heading[0]))/std::hypot(x,y);
          tangent_cost+=5.*(1.-alignment)*std::pow(1.-initial/approach,2);
        }
      }
      double ng=g[id]+std::hypot(x,y)*res*(1+tangent_cost+1.5*std::pow(reach/std::max(.15,clearance),3));
      if(ng+1e-9>=g[next])continue;
      g[next]=ng;parent[next]=id;
      frontier.emplace(ng+distance(point(next),point(to)),next);
    }
  }
  Route out;
  if(parent[to]<0){
    // A prescribed yaw progression can block a channel that is physically
    // traversable with a different posture. Keep its centerline for the DP.
    if(!fallback || !yaw_seed || !wide || run.stop())return {};
    out=*fallback;
  }else{
    Route rev{goal};
    for(int v=parent[to];v!=from && v>=0;v=parent[v])rev.push_back(point(v));
    rev.push_back(start);std::reverse(rev.begin(),rev.end());
    std::vector<double> arc{0};for(std::size_t i=1;i<rev.size();++i)arc.push_back(arc.back()+distance(rev[i-1],rev[i]));
    const int spans=std::clamp(int(std::ceil(arc.back()/(oblique?1.75:4.0))),2,HJ_MAX_SPANS);
    out.push_back(start);
    for(int i=1;i<spans;++i){
      double at=arc.back()*i/spans;
      auto j=std::upper_bound(arc.begin(),arc.end(),at)-arc.begin();
      double f=(at-arc[j-1])/(arc[j]-arc[j-1]);
      out.push_back({rev[j-1].x+f*(rev[j].x-rev[j-1].x),rev[j-1].y+f*(rev[j].y-rev[j-1].y)});
    }
    out.push_back(goal);
  }
  if(wide && yaw_seed){
    const auto last=out[out.size()-2];
    const double entry=r.start.heading[0]+(gear<0?pi:0);
    const double exit=r.goal.constrain_heading[0]?r.goal.heading[0]+(gear<0?pi:0):std::atan2(goal.y-last.y,goal.x-last.x);
    const auto xy=hybrid_channel::fit(out,entry,exit);
    if(!xy[0].empty())
      *yaw_seed=hybrid_channel::yaw_profile(xy,r.start.reference.yaw,
                                             delta(r.start.reference.yaw,r.goal.reference.yaw),clearance_at);
  }
  return out;
}

Point interpolate(const Route& r,double fraction){
  double length=0;for(std::size_t k=1;k<r.size();++k)length+=distance(r[k-1],r[k]);
  double left=std::clamp(fraction,0.,1.)*length;
  for(std::size_t k=1;k<r.size();++k){
    const auto a=r[k-1],b=r[k];const double d=distance(a,b);
    if(left<=d || k+1==r.size()){double f=d>0?std::min(1.,left/d):0;return {a.x+(b.x-a.x)*f,a.y+(b.y-a.y)*f};}
    left-=d;
  } return r.back();
}

// Cut a channel by arc length. Its interior nodes only initialize the NLP;
// the shared mode-switch position is still one of its decision variables.
Route channel_part(const Route& route,double a,double b){
  const int spans=std::clamp(int(std::ceil((b-a)*(route.size()-1))),1,HJ_MAX_SPANS);
  Route part;
  for(int j=0;j<=spans;++j)part.push_back(interpolate(route,a+(b-a)*j/spans));
  return part;
}
double central_tangent(const Route& route){
  double heading=std::atan2(route[1].y-route[0].y,route[1].x-route[0].x),low=heading,high=heading;
  for(std::size_t j=1;j<route.size();++j){
    heading+=delta(heading,std::atan2(route[j].y-route[j-1].y,route[j].x-route[j-1].x));
    low=std::min(low,heading);high=std::max(high,heading);
  }
  return .5*(low+high);
}

// Moving an interpolation knot keeps the entire clamped spline C4. The
// local score steers only the initial geometry; admission always certifies
// the original footprint and the continuous normal/steering dynamics.
void shape_translation_seed(const Request& r,Route& path,double entry,double exit,double yaw,
                        const double* headings,const Run& run){
  if(path.size()<3 || run.stop())return;
  CollisionKernel map(*r.grid,r.vehicles,r.cargo);
  const auto original=path;
  const double cell=r.grid->resolution();
  auto score=[&](const Route& proposal){
    const auto curve=hybrid_channel::fit(proposal,entry,exit);
    if(curve[0].empty())return std::numeric_limits<double>::infinity();
    double penalty=0.;
    for(std::size_t j=0;j<curve[0].size();++j){
      const int steps=std::clamp(int(std::ceil(distance(proposal[j],proposal[j+1])/.10)),8,48);
      for(int k=0;k<steps;++k){
        const double t=(k+.5)/steps;
        double xy[2]{};
        for(int axis=0;axis<2;++axis){
          auto v=curve[axis][j];
          for(int d=5;d>0;--d)for(int i=0;i<d;++i)v[i]+=t*(v[i+1]-v[i]);
          xy[axis]=v[0];
        }
        const fwis_planning_core::Pose2d pose{xy[0],xy[1],yaw};
        if(map.check(pose,headings,r.vehicles.size(),2*cell).reason==CollisionReason::clear)continue;
        penalty+=.15;
        if(map.check(pose,headings,r.vehicles.size(),cell).reason!=CollisionReason::clear)penalty+=1.;
        if(map.check(pose,headings,r.vehicles.size()).reason!=CollisionReason::clear)penalty+=10.;
      }
    }
    for(std::size_t j=1;j+1<proposal.size();++j)penalty+=.05*std::pow(distance(original[j],proposal[j]),2);
    return penalty;
  };
  double best=score(path);
  if(best<1.)return;
  for(double step:{std::max(.18,4*cell),std::max(.09,2*cell),cell}){
    for(int sweep=0;sweep<3 && !run.stop();++sweep){
      const double before=best;
      for(std::size_t j=1;j+1<path.size() && !run.stop();++j){
        const auto initial=path[j];auto picked=initial;
        for(auto shift:{Point{step,0},Point{-step,0},Point{0,step},Point{0,-step}}){
          path[j]={initial.x+shift.x,initial.y+shift.y};
          const double current=score(path);
          if(current+1.e-8<best){best=current;picked=path[j];}
        }
        path[j]=picked;
      }
      if(before-best<1.e-8 || best<.15)break;
    }
  }
}

// A clamped, uniform quintic B-spline initialized from the channel tangent.
// Insert every interior knot to multiplicity five to extract exact Bernstein
// spans. The fixed-yaw chart has C4 joins, with zero angular rate at both ends.
std::vector<Quintic> route_orientation(const Route& path,double start,double end,bool shortest,const std::vector<double>& optimized){
  const int n=path.size()-1,degree=5;
  std::vector<double> knots(6,0.);for(int i=1;i<n;++i)knots.push_back(i);
  knots.insert(knots.end(),6,n);
  std::vector<double> a(n+1,start);
  const double first=std::atan2(path[1].y-path[0].y,path[1].x-path[0].x);
  double last=first;
  for(int j=1;j<=n;++j){
    int k=std::min(j+1,n);
    double current=std::atan2(path[k].y-path[j-1].y,path[k].x-path[j-1].x);
    a[j]=a[j-1]+delta(last,current);last=current;
  }
  const double residual=delta(a.back(),end);
  for(int j=1;j<=n;++j)a[j]=shortest?start+delta(start,end)*j/n:a[j]+residual*j/n;
  if(optimized.size()==a.size())a=optimized;
  std::vector<double> controls(n+degree);
  for(int i=0;i<n+degree;++i){
    double t=0;for(int j=1;j<=degree;++j)t+=knots[i+j]/degree;
    int j=std::clamp(int(t),0,n-1);
    controls[i]=a[j]+(t-j)*(a[j+1]-a[j]);
  }
  for(int i=0;i<3;++i){controls[i]=start;controls[controls.size()-1-i]=end;}
  return hybrid_channel::bezier_spans(std::move(controls),n);
}
HJProblem problem(const Request& r,const Seed& seed,const SolveOptions& options,Run& run) {
  const auto& l=r.limits;
  HJProblem p{};
  p.general_chart=seed.general;
  p.terminal_heading_window=seed.relaxed?.97*pi/180.:0.;
  p.count=seed.seq.size();p.bodies=r.vehicles.size();p.movable=options.movable_switches;
  p.radius=l.radius;p.speed=l.speed;p.acceleration=l.acceleration;p.jerk=l.jerk;
  p.yaw_rate=l.yaw_rate;p.spin_rate=l.spin_speed;p.spin_acc=l.spin_acceleration;p.spin_jerk=l.spin_jerk;
  p.min_motion_duration=l.min_motion_duration;p.prepare_time=l.prepare_time;p.lateral_acceleration=l.lateral_acceleration;p.steer_acceleration=l.steer_acceleration;
  p.yaw_acceleration=l.yaw_acceleration;p.yaw_jerk=l.yaw_jerk;
  p.wheelbase=l.wheelbase;p.track=l.track;p.wheel_speed=l.wheel_speed;p.steer_angle=l.steer_angle;p.steer_rate=l.steer_rate;
  p.duration_cap=r.task_duration_cap;p.stop=stop_nlp;p.opaque=&run;
  p.wall_seconds=std::max(.001,std::chrono::duration<double>(run.end-Clock::now()).count());
  for(int k=0;k<p.bodies;++k) {p.offset[k][0]=r.vehicles[k][0];p.offset[k][1]=r.vehicles[k][1];}
  p.start[0]=r.start.reference.x;p.start[1]=r.start.reference.y;p.start[2]=r.start.reference.yaw;
  p.goal[0]=r.goal.reference.x;p.goal[1]=r.goal.reference.y;
  p.goal[2]=p.start[2]+delta(p.start[2],r.goal.reference.yaw);
  p.fixed_goal[0]=r.goal.constrain_reference_yaw;
  for(int i=0;i<p.bodies;++i){
    p.start[3+i]=r.start.heading[i];p.goal[3+i]=r.start.heading[i]+delta(r.start.heading[i],r.goal.heading[i]);
    p.fixed_goal[1+i]=r.goal.constrain_heading[i] || (p.bodies==1 && p.fixed_goal[0]);
    if(p.bodies==1 && !r.goal.constrain_heading[i] && p.fixed_goal[0])p.goal[3]=p.goal[2];
  }
  const int moving=std::count_if(seed.seq.begin(),seed.seq.end(),[](Mode m){return m!=Mode::S;});
  int m=0;
  Headings headings=r.start.heading;
  double psi=p.start[2];
  for(int k=0;k<p.count;++k){
    auto& ph=p.phase[k];ph.mode=static_cast<int>(seed.seq[k]);ph.spans=1;
    std::fill_n(ph.gear,3,1);
    auto fraction=[&](int index){
      if(seed.curved_oblique && moving==2 && index==1)return seed.switch_station;
      return index/double(std::max(1,moving));
    };
    const auto start=interpolate(seed.route,fraction(m));
    const auto end=ph.mode==2?start:interpolate(seed.route,fraction(m+1));
    Route path{start,end};
    if(ph.mode==0 && moving==1)path=seed.route;
    if(seed.curved_oblique && ph.mode!=2)path=channel_part(seed.route,fraction(m),fraction(m+1));
    const int sign=seed.reverse_from>=0 && k>=seed.reverse_from?-seed.sign:seed.sign;
    const double dx=end.x-start.x,dy=end.y-start.y,length=std::hypot(dx,dy);
    const double travel=length>1e-9?std::atan2(dy,dx):p.start[2];
    const double oblique_direction=ph.mode==1 && seed.curved_oblique?central_tangent(path):travel;
    double endpsi=psi;
    if(ph.mode==0){
      const int remaining=seed.curved_oblique?std::count(seed.seq.begin()+k,seed.seq.end(),Mode::A):moving-m;
      endpsi=psi+delta(psi,p.goal[2])/remaining;
    }
    for(int i=0;i<p.bodies;++i){
      ph.h0[i]=headings[i];
      const int orientation=std::cos(headings[i]-ph.h0[0])>=0?1:-1;
      // All gears start forward. O's per-car direction uses the curved
      // channel tangent or the straight chord and honors the same policy as A.
      if(r.allow_reverse){
        if(ph.mode==0)ph.gear[i]=sign*orientation;
        if(ph.mode==1)ph.gear[i]=std::cos(oblique_direction-headings[i])>=0?1:-1;
      }
      if(ph.mode==2) {
        bool last=k+1==p.count;
        double next=p.goal[3+i];
        if(!last){
          if(seed.curved_oblique){
            const auto after=channel_part(seed.route,fraction(m),fraction(m+1));
            next=seed.seq[k+1]==Mode::O?central_tangent(after):
              std::atan2(after[1].y-after[0].y,after[1].x-after[0].x);
          }else{
            next=std::atan2(seed.route[1].y-seed.route[0].y,seed.route[1].x-seed.route[0].x);
          }
          if(seed.sign<0)next+=pi;
        }
        headings[i]+=delta(headings[i],next);
      } else if(ph.mode==0) {
        const double next=k+1==p.count && p.fixed_goal[i+1]?p.goal[3+i]:travel+(ph.gear[i]<0?pi:0);
        headings[i]+=delta(headings[i],next);
      }
      ph.h1[i]=headings[i];
    }
    double spin=0;for(int i=0;i<p.bodies;++i)spin=std::max(spin,std::abs(ph.h1[i]-ph.h0[i]));
    double amount=ph.mode==2?spin:length;
    if(path.size()>2){
      amount=0;for(std::size_t j=1;j<path.size();++j)amount+=distance(path[j-1],path[j]);
    }
    // A cubic velocity ramp reaching v requires t >= 1.5 v/a and
    // t >= sqrt(6 v/j). For ramp fraction f, a plateau covering arc L at v
    // gives each ramp f L / ((1-f) v) seconds. Choose the shortest that fits.
    const double min_ramp=std::max(1.5*l.speed/l.acceleration,std::sqrt(6*l.speed/l.jerk));
    if(!seed.quintic && ph.mode!=2){
      for(double f:cruise_ramps){
        if(f*amount >= (1-f)*l.speed*min_ramp){ph.cruise_ramp_fraction=f;break;}
      }
    }
    const double f=ph.cruise_ramp_fraction;
    const double peak=hybrid_clock::state(.5,f)[1];
    const double acceleration_peak=f>0?1.5*peak/f:5.775;
    const double jerk_peak=f>0?6*peak/(f*f):60.;
    ph.tau=std::clamp(1.15*std::max({.5,peak*amount/(ph.mode==2?l.spin_speed:l.speed),
      std::sqrt(acceleration_peak*amount/(ph.mode==2?l.spin_acceleration:l.acceleration)),
      std::cbrt(jerk_peak*amount/(ph.mode==2?l.spin_jerk:l.jerk))}),.05,p.duration_cap);
    ph.spans=path.size()-1;
    for(int j=0;j<ph.spans;++j){
      const auto a=path[j],b=path[j+1];double len=distance(a,b);
      double d0=j?std::atan2(path[j+1].y-path[j-1].y,path[j+1].x-path[j-1].x):ph.h0[0]+(sign<0?pi:0);
      double d1=j+1<ph.spans?std::atan2(path[j+2].y-path[j].y,path[j+2].x-path[j].x):ph.h1[0]+(sign<0?pi:0);
      if(seed.smooth_channel && j+1==ph.spans && !p.fixed_goal[1])d1=std::atan2(b.y-a.y,b.x-a.x);
      for(int c=0;c<6;++c){
        double t=c/5.;
        ph.coefficient[j][0][c]=a.x+(b.x-a.x)*t;ph.coefficient[j][1][c]=a.y+(b.y-a.y)*t;
        if(ph.mode==0 && c>0 && c<5){
          double h=c<3?d0:d1,w=(c<3?t:t-1)*len;
          ph.coefficient[j][0][c]=(c<3?a.x:b.x)+w*std::cos(h);
          ph.coefficient[j][1][c]=(c<3?a.y:b.y)+w*std::sin(h);
        }
        // At a stopped A/O junction the adjacent translation chooses one
        // chassis heading per mount. Give a short cargo-yaw adjustment zero
        // station derivatives at the endpoints to preserve compatible tangents.
        const double angle_fraction=seed.curved_oblique && ph.mode==0 && ph.spans==1?
          (c<3?0.:1.):(j+t)/ph.spans;
        ph.coefficient[j][2][c]=psi+(endpsi-psi)*angle_fraction;
      }
    }
    if((seed.smooth_channel || seed.curved_oblique) && ph.spans>1 && ph.mode!=2) {
      const auto a=path[ph.spans-1],b=path.back();
      const double entry=ph.mode==1?std::atan2(path[1].y-path[0].y,path[1].x-path[0].x):ph.h0[0]+(sign<0?pi:0);
      const double exit=ph.mode==0 && k+1==p.count && p.fixed_goal[1]?ph.h1[0]+(sign<0?pi:0):std::atan2(b.y-a.y,b.x-a.x);
      if(seed.curved_oblique && ph.mode==1){
        shape_translation_seed(r,path,entry,exit,psi,ph.h0,run);
      }
      const auto smooth=hybrid_channel::fit(path,entry,exit);
      if(!smooth[0].empty())
        for(int axis=0;axis<2;++axis)for(int j=0;j<ph.spans;++j)
          std::copy(smooth[axis][j].begin(),smooth[axis][j].end(),ph.coefficient[j][axis]);
    }
    if(ph.mode==0 && p.bodies==3 && moving==1 && ph.spans>1 &&
       (std::abs(delta(psi,endpsi))>1e-8) && !seed.general){
      auto yaw=route_orientation(path,psi,endpsi,seed.shortest_yaw,seed.channel_yaw);
      for(int j=0;j<ph.spans;++j)std::copy(yaw[j].begin(),yaw[j].end(),ph.coefficient[j][2]);
      p.route_yaw_chart=1;
    }
    if(ph.mode!=2)++m;
    psi=endpsi;
  }
  return p;
}
// Pick local separating axes from the map for smooth fixed-feature constraints.
// A seed in collision may use a negative initial gap and move back into free space.
// The independent swept certificate still accounts for every occupied/unknown cell.
struct WitnessStation {int phase;double u;};
std::vector<HJSeparation> separators(const Request& r,const HJProblem& graph,const Run& run,
                                     std::optional<WitnessStation> witness={}){

  const auto& grid=*r.grid;const auto org=grid.origin();const double res=grid.resolution();
  std::vector<HJSeparation> all;
  for(int k=0;k<graph.count&&!run.stop();++k){
    const auto& a=graph.phase[k];if(a.mode==2 || (witness && witness->phase!=k))continue;
    Phase phase;phase.mode=static_cast<Mode>(a.mode);phase.duration=1;
    std::copy_n(a.h0,3,phase.heading0.begin());std::copy_n(a.gear,3,phase.gear.begin());
    for(int j=0;j<a.spans;++j) {
      Piece q;std::copy_n(a.coefficient[j][0],6,q.x.begin());std::copy_n(a.coefficient[j][1],6,q.y.begin());
      std::copy_n(a.coefficient[j][2],6,q.yaw.begin());phase.pieces.push_back(q);
    }
    for(int j=0;j<a.spans&&!run.stop();++j){
      std::vector<double> stations;
      if(witness){
        const double local=witness->u*a.spans-j;
        if(local>=0 && local<=1)stations={6*local};
      }else stations={0,1,2,3,4,5,6};
      for(double z:stations){
        double low=0,high=1,station=(j+z/6.)/a.spans;
        for(int iter=0;iter<32;++iter){double u=(low+high)/2;if(u*u*u*(10+u*(-15+6*u))<station)low=u;else high=u;}
        Sample sample;
        try{sample=evaluate_phase(r,phase,(low+high)/2);}catch(const std::domain_error&){continue;}
        if(witness && graph.bodies==3){
          // At an actual contact choose the best separating axis of the two
          // OBBs. Both positions and headings remain variables in the next NLP.
          for(int v=0;v<3;++v)for(int other=v+1;other<3;++other){
            auto a0=sample.bodies[v].pose,b0=sample.bodies[other].pose;
            auto sa=r.vehicles[v],sb=r.vehicles[other];
            double best=-1e99;Point axis{};
            for(auto angle:{a0.yaw,a0.yaw+pi/2,b0.yaw,b0.yaw+pi/2}){
              Point n{std::cos(angle),std::sin(angle)};
              double travel=n.x*(a0.x-b0.x)+n.y*(a0.y-b0.y);
              if(travel<0){n.x=-n.x;n.y=-n.y;travel=-travel;}
              auto extent=[&](const Footprint& f,double h){
                return .5*(f[2]*std::abs(std::cos(h-angle))+f[3]*std::abs(std::sin(h-angle)));
              };
              double gap=travel-extent(sa,a0.yaw)-extent(sb,b0.yaw);
              if(gap>best){best=gap;axis=n;}
            }
            if(best<.03)all.push_back({k,j,z,v,sa[0],sa[1],sa[2]/2,sa[3]/2,axis.x,axis.y,0,
                                        other,sb[0],sb[1],sb[2]/2,sb[3]/2});
          }
        }
        for(int v=r.cargo?-1:0;v<graph.bodies;++v) {
          const auto& shape=v<0?*r.cargo:r.vehicles[v];const double a0=shape[2]/2,b0=shape[3]/2;
          Pose2d pose=sample.reference;
          if(v<0){pose.x+=std::cos(pose.yaw)*shape[0]-std::sin(pose.yaw)*shape[1];
            pose.y+=std::sin(pose.yaw)*shape[0]+std::cos(pose.yaw)*shape[1];}
          else pose=sample.bodies[v].pose;
          const double c=std::cos(pose.yaw),s=std::sin(pose.yaw),radius=std::hypot(a0,b0)+.5+res;
          const std::array<Point,8> axes{{{1,0},{-1,0},{0,1},{0,-1},{c,s},{-c,-s},{-s,c},{s,-c}}};
          std::array<Point,8> feature{};
          std::array<double,8> tight,gaps;tight.fill(-1.e99);gaps.fill(1.e99);
          int left=std::max(0,int(std::floor((pose.x-radius-org.x)/res)));
          int right=std::min(int(grid.width())-1,int(std::floor((pose.x+radius-org.x)/res)));
          int bottom=std::max(0,int(std::floor((pose.y-radius-org.y)/res)));
          int top=std::min(int(grid.height())-1,int(std::floor((pose.y+radius-org.y)/res)));
          for(int cy=bottom;cy<=top;++cy)for(int cx=left;cx<=right;++cx)if(grid.occupied(cx,cy)){
            const Point center{org.x+(cx+.5)*res,org.y+(cy+.5)*res};
            double largest=-1e99,support=0;int active=0;
            for(int i=0;i<8;++i){const auto n=axes[i];
              const double bound=n.x*center.x+n.y*center.y+.5*res*(std::abs(n.x)+std::abs(n.y));
              const double gap=n.x*pose.x+n.y*pose.y-bound-a0*std::abs(c*n.x+s*n.y)-b0*std::abs(-s*n.x+c*n.y);
              if(gap>largest){largest=gap;active=i;support=bound;}
            }
            if(largest<gaps[active]){tight[active]=support;gaps[active]=largest;feature[active]=center;}
          }
          // Two opposing seed faces may define an empty strip for every yaw.
          // Preserve the SAT disjunction for this body's local obstacle features.
          bool impossible=false;
          for(int i=0;i<8;i+=2) {
            if(gaps[i]<.22 && gaps[i+1]<.22 &&
               tight[i]+tight[i+1]+2*(std::min(a0,b0)+1e-5)>0)
              impossible=true;
          }
          for(int i=0;i<8;++i)if(gaps[i]<.22){
            const auto n=axes[i];
            if(impossible) {
              all.push_back({k,j,z,v,shape[0],shape[1],a0,b0,n.x,n.y,tight[i],HJSeparation::map_cell,feature[i].x,feature[i].y,.5*res,.5*res});
            } else {
              all.push_back({k,j,z,v,shape[0],shape[1],a0,b0,n.x,n.y,tight[i]});
            }
          }
        }
      }
    }
  }
  return all;
}
std::array<double,4> wheels(Mode mode,const Body& b,const Limits& l) {
  std::array<double,4> out{};int k=0;
  for(double a:{l.wheelbase*.5,-l.wheelbase*.5})for(double y:{l.track*.5,-l.track*.5}){
    if(mode==Mode::A)out[k++]=std::atan2(a*b.curvature,1-y*b.curvature);
    else if(mode==Mode::O)out[k++]=b.oblique_angle;
    else out[k++]=std::remainder(std::atan2(a,-y),pi);
  }return out;
}

Piece stationary_piece(const Pose2d& pose) {
  Piece piece;
  piece.x.fill(pose.x);
  piece.y.fill(pose.y);
  piece.yaw.fill(pose.yaw);
  return piece;
}

Phase wheel_preparation(const Request& r,const Sample& first,const Gears& gear,double range) {
  Phase prep;
  prep.kind=Kind::Prepare;
  prep.mode=first.mode;
  prep.gear=gear;
  prep.pieces={stationary_piece(first.reference)};
  prep.duration=r.limits.prepare_time+.0001+
    std::max(1.875*range/r.limits.steer_rate,std::sqrt(5.774*range/r.limits.steer_acceleration));
  for(std::size_t i=0;i<r.vehicles.size();++i) prep.heading0[i]=prep.heading1[i]=first.bodies[i].pose.yaw;
  return prep;
}

Plan expand(const Request& r,const HJProblem& p) {
  Plan plan;
  std::array<std::array<double,4>,3> prior{};
  Mode prev=Mode::A;Gears gear{1,1,1};
  for(int k=0;k<p.count;++k) {
    const auto& a=p.phase[k];
    Phase ph; ph.mode=static_cast<Mode>(a.mode);ph.duration=std::max(a.tau,r.limits.min_motion_duration);ph.cruise_ramp_fraction=a.cruise_ramp_fraction;
    std::copy_n(a.h0,3,ph.heading0.begin());std::copy_n(a.h1,3,ph.heading1.begin());
    std::copy_n(a.gear,3,ph.gear.begin());
    for(int j=0;j<a.spans;++j){Piece piece;
      std::copy_n(a.coefficient[j][0],6,piece.x.begin());std::copy_n(a.coefficient[j][1],6,piece.y.begin());
      std::copy_n(a.coefficient[j][2],6,piece.yaw.begin());ph.pieces.push_back(piece);}
    if(ph.mode==Mode::S){double turn=0;for(int v=0;v<p.bodies;++v)turn=std::max(turn,std::abs(ph.heading1[v]-ph.heading0[v]));if(turn<1e-7)continue;}
    auto first=evaluate_phase(r,ph,0.),last=evaluate_phase(r,ph,ph.duration);
    if(ph.mode!=Mode::S && !plan.phases.empty() && plan.phases.back().mode==Mode::S){
      auto& spin=plan.phases.back();
      // The next motion fixes the car headings at the stationary mounts.
      // Reconstruct the shared spin endpoint exactly on its unwrapped branch,
      // then subject the entire rotation and both joins to continuous audit.
      for(int i=0;i<p.bodies;++i)spin.heading1[i]+=delta(spin.heading1[i],first.bodies[i].pose.yaw);
    }
    double range=0; bool gear_change=false;
    for(int i=0;i<p.bodies;++i){
      auto next=wheels(ph.mode,first.bodies[i],r.limits);
      for(int j=0;j<4;++j)range=std::max(range,std::abs(next[j]-prior[i][j]));
      prior[i]=wheels(ph.mode,last.bodies[i],r.limits);
      gear_change|=gear[i]!=ph.gear[i];
    }
    if(prev!=ph.mode || gear_change || range>1e-7)
      plan.phases.push_back(wheel_preparation(r,first,ph.gear,range));
    plan.phases.push_back(ph);prev=ph.mode;gear=ph.gear;
  }
  return plan;
}

// Optimize duration on a continuous channel before releasing its geometry to
// the full shape-and-time NLP. A feasible spatial iterate survives subsequent
// nonconvergence. Every tested duration uses the original continuous checker.
std::optional<Plan> retime_channel(const Request& r,const HJProblem& graph,const Run& run,Clock::time_point& first_proof){
  if(graph.count!=1 || graph.phase[0].mode==2 || graph.phase[0].spans<2 || run.stop())return {};
  Plan plan=expand(r,graph);
  auto& motion=plan.phases.back();
  const double cap=r.task_duration_cap-(duration(plan)-motion.duration);
  if(cap<r.limits.min_motion_duration)return {};
  auto test=[&](double trial){
    motion.duration=trial;
    return certify(r,plan,run.end,run.cancel);
  };
  double hi=std::min(motion.duration,cap),lo=r.limits.min_motion_duration;
  auto proof=test(hi);
  if(!proof.executable){
    const auto& failure=proof.reason;
    if(run.stop() || hi>=cap || !(failure.find("SPEED")!=std::string::npos ||
       failure.find("ACCELERATION")!=std::string::npos || failure.find("JERK")!=std::string::npos ||
       failure.find("RATE")!=std::string::npos || failure.find("DYNAMIC")!=std::string::npos))return {};
    lo=hi;hi=cap;
    if(!test(hi).executable)return {};
  }
  first_proof=Clock::now();
  // Bisection keeps the last proven duration; a timeout can never admit the
  // unverified lower endpoint. The later joint solve can further reduce cost.
  for(int i=0;i<4 && !run.stop();++i){
    const double middle=(lo+hi)*.5;
    if(test(middle).executable)hi=middle;else lo=middle;
  }
  motion.duration=hi;
  return plan;
}
// Retime a multi-phase seed without moving its channel or any mode boundary.
// A failed spatial audit cannot be repaired by lengthening a phase. Keep its
// witness for the normal full shape-and-time NLP in that case.
std::optional<Plan> retime_mixed_channel(const Request& r,const HJProblem& graph,
                                        const Run& run,Clock::time_point& first_proof,std::string& status){
  auto plan=expand(r,graph);
  for(int attempt=0;attempt<16 && !run.stop();++attempt){
    const double total=duration(plan);
    if(total>r.task_duration_cap){status="TASK_TIME_CONSTRAINT";return {};}
    const auto audit=certify(r,plan,run.end,run.cancel);
    status=audit.reason+"@"+std::to_string(audit.phase)+"/"+std::to_string(.5*(audit.u0+audit.u1));
    if(audit.executable){first_proof=Clock::now();return plan;}
    if(audit.phase>=plan.phases.size() || plan.phases[audit.phase].kind!=Kind::Motion)return {};
    const auto& reason=audit.reason;
    if(reason.find("SPEED")==std::string::npos && reason.find("ACCELERATION")==std::string::npos &&
       reason.find("JERK")==std::string::npos && reason.find("RATE")==std::string::npos &&
       reason.find("DYNAMIC")==std::string::npos)return {};
    auto& tau=plan.phases[audit.phase].duration;
    const double next=std::min(tau*1.22,r.task_duration_cap-total+tau);
    if(next<=tau+1.e-8)return {};
    tau=std::nextafter(next,0.);
  }
  return {};
}
// A pointwise determinant excludes only an exact endpoint. With a heading
// tolerance the inactive tangent proof must hold over the whole neighborhood.
struct MountRank {bool exact{}, whole_neighborhood{};};
MountRank rank_at_boundary(const Request& r,const Boundary& b){
  if(r.vehicles.size()!=3 || !b.constrain_reference_yaw ||
     std::any_of(b.constrain_heading.begin(),b.constrain_heading.end(),[](bool v){return !v;}))return {};
  std::array<std::array<double,3>,3> row{};double radii2=0.;
  for(std::size_t i=0;i<3;++i){
    auto f=r.vehicles[i];double c=std::cos(b.reference.yaw),s=std::sin(b.reference.yaw);
    double rx=c*f[0]-s*f[1],ry=s*f[0]+c*f[1];double h=b.heading[i];
    row[i]={-std::sin(h),std::cos(h),std::sin(h)*ry+std::cos(h)*rx};
    radii2+=rx*rx+ry*ry;
  }
  double det=0., cofactors2=0.;
  for(int i=0;i<3;++i)for(int j=0;j<3;++j){
    const double c=row[(i+1)%3][(j+1)%3]*row[(i+2)%3][(j+2)%3]
                  -row[(i+1)%3][(j+2)%3]*row[(i+2)%3][(j+1)%3];
    if(i==0)det+=row[0][j]*c;
    cofactors2+=c*c;
  }
  // 1/||inverse(M)||_F lower-bounds the smallest singular value.
  double perturb=(pi/180.)*std::sqrt(3+4*radii2);
  return {std::abs(det)>1e-6,std::abs(det)>std::sqrt(cofactors2)*perturb+1e-10};
}
std::string name(const Sequence& seq){std::string s;for(auto m:seq)s+="AOS"[static_cast<int>(m)];return s;}

// Recompute preparations at a join. Both input motions stop at the boundary,
// but their nonzero geometric curvature can require different wheel angles.
void append_motion(const Request& r, Plan& plan, Phase motion) {
  const auto last=std::find_if(plan.phases.rbegin(),plan.phases.rend(),
                              [](const Phase& p){return p.kind==Kind::Motion;});
  const auto first=evaluate_phase(r,motion,0.);
  const auto previous=last==plan.phases.rend()?Sample{}:evaluate_phase(r,*last,last->duration);
  const auto previous_gear=last==plan.phases.rend()?Gears{1,1,1}:last->gear;
  double range=0.;
  for(std::size_t i=0;i<r.vehicles.size();++i){
    const auto from=wheels(previous.mode,previous.bodies[i],r.limits);
    const auto to=wheels(first.mode,first.bodies[i],r.limits);
    for(std::size_t w=0;w<from.size();++w) range=std::max(range,std::abs(to[w]-from[w]));
  }
  // Adjacent same-gear A phases intentionally stop at this certified junction.
  if(last!=plan.phases.rend() || first.mode!=Mode::A || motion.gear!=previous_gear || range>1e-7)
    plan.phases.push_back(wheel_preparation(r,first,motion.gear,range));
  plan.phases.push_back(std::move(motion));
}

Phase stationary_spin(const Request& r,const Sample& rest,const Headings& target) {
  Phase spin;
  spin.mode=Mode::S;
  spin.pieces={stationary_piece(rest.reference)};
  const auto& l=r.limits;
  double turn=0.;
  for(std::size_t i=0;i<r.vehicles.size();++i){
    spin.heading0[i]=rest.bodies[i].pose.yaw;
    const double angle=delta(spin.heading0[i],target[i]);
    spin.heading1[i]=spin.heading0[i]+angle;
    turn=std::max(turn,std::abs(angle));
  }
  const double wheel_radius=.5*std::hypot(l.wheelbase,l.track);
  const double speed=std::min(l.spin_speed,l.wheel_speed/wheel_radius);
  spin.duration=1.03*std::max({l.min_motion_duration,1.875*turn/speed,
                std::sqrt(5.774*turn/l.spin_acceleration),std::cbrt(60.*turn/l.spin_jerk)});
  return spin;
}

Phase straight_motion(const Request& r,const Pose2d& from,const Pose2d& to,int gear) {
  Phase move;
  const double dx=to.x-from.x,dy=to.y-from.y,length=std::hypot(dx,dy);
  const double heading=std::atan2(gear*dy,gear*dx);
  move.gear.fill(gear);
  move.heading0.fill(heading);
  move.heading1.fill(heading);
  move.pieces={stationary_piece(from)};
  for(std::size_t j=1;j<move.pieces[0].x.size();++j){
    const double u=double(j)/double(move.pieces[0].x.size()-1);
    move.pieces[0].x[j]=from.x+u*dx;
    move.pieces[0].y[j]=from.y+u*dy;
  }
  const auto& l=r.limits;
  double vmax=std::min(l.speed,l.wheel_speed);
  if(l.cargo_center_speed>0.)vmax=std::min(vmax,l.cargo_center_speed);
  move.duration=1.03*std::max({l.min_motion_duration,1.875*length/vmax,
               std::sqrt(5.774*length/l.acceleration),std::cbrt(60.*length/l.jerk)});
  return move;
}

enum class SolvePass { Full, Continuation, Arrival };
Result solve_route(const Request&,Clock::time_point,const Cancel&,const SolveOptions&,SolvePass);

constexpr double endpoint_heading_window=pi/180.;

void record_stage(Result& trace,const Result& continuation,const std::string& tag,
                  const std::string& status,Clock::time_point started,double certificate_s){
  Attempt attempt{tag,status,0.,0.,certificate_s,std::chrono::duration<double>(Clock::now()-started).count()};
  for(const auto& child:continuation.attempts){
    attempt.nlp_build_s+=child.nlp_build_s;
    attempt.nlp_solve_s+=child.nlp_solve_s;
    attempt.certify_s+=child.certify_s;
  }
  trace.attempts.push_back(std::move(attempt));
  trace.trace.push_back(tag+":"+status);
}

std::optional<Plan> depart_and_resume(const Request& r,const Route& route,const Run& run,Result& trace,
                                      bool short_straight=false) {
  if(route.size()<3 || !r.cargo || !r.start.constrain_reference_yaw ||
     !std::all_of(r.start.constrain_heading.begin(),r.start.constrain_heading.end(),[](bool x){return x;}))return {};
  const double anchor=r.start.heading.front();
  double lower=-endpoint_heading_window,upper=endpoint_heading_window,cos_sum=0.,sin_sum=0.;
  // Intersect the existing endpoint tolerance windows on the same yaw chart.
  for(double heading:r.start.heading){
    const double offset=delta(anchor,heading);
    lower=std::max(lower,offset-endpoint_heading_window);
    upper=std::min(upper,offset+endpoint_heading_window);
    cos_sum+=std::cos(offset);
    sin_sum+=std::sin(offset);
  }
  if(lower>upper)return {};
  const double start_heading=anchor+std::clamp(std::atan2(sin_sum,cos_sum),lower,upper);
  const auto& start=r.start.reference;
  const double initial_tangent=std::atan2(route[1].y-start.y,route[1].x-start.x);
  const int primary_gear=std::cos(delta(start_heading,initial_tangent))>=0.?1:-1;
  const auto& cargo=*r.cargo;
  const double reach=.5*std::hypot(cargo[2],cargo[3])+std::hypot(cargo[0],cargo[1]);
  if(short_straight && std::abs(delta(start_heading,initial_tangent+(primary_gear<0?pi:0.)))>=pi/8.)return {};
  const std::initializer_list<double> straight_lengths{.45,.36}, spin_lengths{1.35,1.,1.75,.7};
  const auto lengths=short_straight?straight_lengths:spin_lengths;
  unsigned continuations=0;
  for(int gear:{primary_gear,-primary_gear}){
    if(gear<0&&!r.allow_reverse)continue;
    for(double scale:lengths){
      if(run.stop())return {};
      const double length=scale*reach;
      const Pose2d exit{start.x+gear*length*std::cos(start_heading),start.y+gear*length*std::sin(start_heading),start.yaw};
      // Look beyond the local straight run before choosing the next travel
      // direction. Quarter turns offer a useful seed in rectilinear aisles.
      auto ahead=route.begin()+1;
      double along=distance(route.front(),*ahead);
      while(ahead+1!=route.end() && along<length+reach){
        along+=distance(*ahead,*(ahead+1));
        ++ahead;
      }
      const double aim=std::atan2(ahead->y-exit.y,ahead->x-exit.x);
      const double turn=delta(start_heading,aim);
      if(!short_straight && std::abs(turn)<pi/8.)continue;
      const double quarter=std::round(turn/(pi/2.))*(pi/2.);
      const double preferred=short_straight?start_heading:
        start_heading+(std::abs(quarter)>1e-9?quarter:turn);
      const auto move=straight_motion(r,start,exit,gear);
      const std::array<double,2> targets{preferred,aim};
      for(std::size_t i=0;i<(short_straight?1:targets.size());++i){
        if(run.stop())return {};
        const auto started=Clock::now();
        const double target=targets[i];
        Plan prefix;
        append_motion(r,prefix,move);
        Headings heads{target,target,target};
        const bool spin=std::abs(delta(start_heading,target))>1e-9;
        if(spin)
          append_motion(r,prefix,stationary_spin(r,evaluate_phase(r,move,move.duration),heads));
        const std::string tag=spin?"LOCAL_DEPARTURE:ASA":"LOCAL_DEPARTURE:AA";
        const auto prefix_duration=duration(prefix);
        if(prefix_duration>=r.task_duration_cap)continue;
        Request entry=r;
        entry.goal={exit,heads,true,{true,true,true}};
        auto audit_start=Clock::now();
        if(!certify(entry,prefix,run.end,run.cancel).executable)continue;
        double audit_s=std::chrono::duration<double>(Clock::now()-audit_start).count();
        Request remainder=r;
        remainder.start=entry.goal;
        remainder.task_duration_cap-=prefix_duration;
        // One recursive solve is sufficient for a candidate, with local
        // staging disabled so no request can grow an unbounded search tree.
        const auto slice=std::chrono::milliseconds(short_straight?3000:4200);
        const auto end=std::min(run.end-std::chrono::milliseconds(200),Clock::now()+slice);
        if(Clock::now()>=end)return {};
        ++continuations;
        const auto found=solve_route(remainder,end,run.cancel,{},SolvePass::Continuation);
        if(found.plan){
          for(const auto& motion:found.plan->phases)
            if(motion.kind==Kind::Motion)append_motion(r,prefix,motion);
          audit_start=Clock::now();
          const auto proof=certify(r,prefix,run.end,run.cancel);
          audit_s+=std::chrono::duration<double>(Clock::now()-audit_start).count();
          record_stage(trace,found,tag,proof.reason,started,audit_s);
          if(proof.executable)return prefix;
        }else{
          record_stage(trace,found,tag,found.reason,started,audit_s);
        }
        // Leave time for the original global candidates if the resumed route
        // does not solve from the first fully certified stop.
        if(continuations>=2 || run.end-Clock::now()<std::chrono::seconds(2))return {};
      }
    }
  }
  return {};
}

// Find a stopped approach state outside a constrained goal area. The cargo
// attains its requested yaw on the main segment and holds it through the gate.
std::optional<Plan> approach_from_channel(const Request& r,const Route& route,const Run& run,Result& trace,
                                          SolvePass pass=SolvePass::Continuation){
  if(!r.cargo || !r.goal.constrain_reference_yaw || route.size()<3 ||
     std::any_of(r.goal.constrain_heading.begin(),r.goal.constrain_heading.end(),[](bool fixed){return fixed;}))return {};
  const auto target=r.goal.reference;
  const auto previous=route[route.size()-2];
  const double tangent=std::atan2(target.y-previous.y,target.x-previous.x);
  constexpr double octant=pi/4.;
  const double grid_angle=std::round(tangent/octant)*octant;
  const std::array<double,9> headings{grid_angle,tangent,grid_angle+octant,grid_angle-octant,
    grid_angle+2*octant,grid_angle-2*octant,grid_angle+3*octant,grid_angle-3*octant,grid_angle+pi};
  const auto& cargo=*r.cargo;
  const double reach=.5*std::hypot(cargo[2],cargo[3])+std::hypot(cargo[0],cargo[1]);
  unsigned continuations=0;
  for(std::size_t i=0;i<headings.size();++i){
    const double heading=headings[i];
    if(i==1 && std::abs(delta(grid_angle,tangent))<1e-9)continue;
    for(double separation:{1.75,1.5,2.}){
      if(run.stop())return {};
      const auto began=Clock::now();
      const Pose2d entry{target.x-separation*reach*std::cos(heading),
                        target.y-separation*reach*std::sin(heading),target.yaw};
      const auto tail=straight_motion(r,entry,target,1);
      Request last=r;
      last.start={entry,tail.heading0,true,{true,true,true}};
      Plan approach{{tail}};
      auto audit_start=Clock::now();
      if(!certify(last,approach,run.end,run.cancel).executable)continue;
      double audit_s=std::chrono::duration<double>(Clock::now()-audit_start).count();
      Request first=r;
      first.goal=last.start;
      first.task_duration_cap-=tail.duration;
      const auto end=std::min(run.end-std::chrono::milliseconds(230),Clock::now()+std::chrono::milliseconds(3200));
      if(first.task_duration_cap<=.1 || end-Clock::now()<std::chrono::milliseconds(700))return {};
      ++continuations;
      auto found=solve_route(first,end,run.cancel,{},pass);
      auto status=found.reason;
      bool certified=false;
      if(found.plan){
        append_motion(r,*found.plan,tail);
        audit_start=Clock::now();
        const auto proof=certify(r,*found.plan,run.end,run.cancel);
        audit_s+=std::chrono::duration<double>(Clock::now()-audit_start).count();
        status=proof.reason;
        certified=proof.executable;
      }
      record_stage(trace,found,"LOCAL_ARRIVAL:STRAIGHT",status,began,audit_s);
      if(certified)return std::move(found.plan);
      if(continuations>=2)return {};
    }
  }
  return {};
}

std::optional<Plan> stage_backwards(const Request& r,const Route& route,const Run& run,Result& trace) {
  if(!r.allow_reverse || !r.goal.constrain_reference_yaw)return {};
  Request reverse=r;
  std::swap(reverse.start,reverse.goal);
  reverse.start.constrain_heading.fill(true);
  Route reversed(route.rbegin(),route.rend());
  auto found=depart_and_resume(reverse,reversed,run,trace);
  if(!found)return {};
  Plan plan;
  for(auto it=found->phases.rbegin();it!=found->phases.rend();++it){
    if(it->kind!=Kind::Motion)continue;
    auto phase=*it;
    std::reverse(phase.pieces.begin(),phase.pieces.end());
    for(auto& p:phase.pieces){
      std::reverse(p.x.begin(),p.x.end());
      std::reverse(p.y.begin(),p.y.end());
      std::reverse(p.yaw.begin(),p.yaw.end());
    }
    std::swap(phase.heading0,phase.heading1);
    if(phase.mode!=Mode::S) for(auto& gear:phase.gear)gear=-gear;
    append_motion(r,plan,std::move(phase));
  }
  const auto proof=certify(r,plan,run.end,run.cancel);
  trace.trace.push_back("LOCAL_ARRIVAL:REVERSED:"+proof.reason);
  return proof.executable?std::make_optional(std::move(plan)):std::nullopt;
}

Result solve_route(const Request& r,Clock::time_point deadline,const Cancel& cancel,const SolveOptions& opts,
                   SolvePass pass){
  const auto begin=Clock::now();Run run{deadline,cancel};Result result;
  auto elapsed=[&](){return std::chrono::duration<double>(Clock::now()-begin).count();};
  if(!r.grid || (r.vehicles.size()!=1 && r.vehicles.size()!=3) || !(r.task_duration_cap>=.1) ||
     opts.initial_sequence.size()>HJ_MAX_PHASES){result.status="INVALID_REQUEST";result.reason="invalid map, formation, time cap or sequence";return result;}
  for(const auto* endpoint:{&r.start,&r.goal}) {
    const auto& q=endpoint->reference;const auto org=r.grid->origin();const double res=r.grid->resolution();
    if(!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.yaw)){
      result.status="INVALID_REQUEST";result.reason="NONFINITE_ENDPOINT";return result;
    }
    // Only reject a reference-center cell if that point lies inside a body for
    // every heading allowed by the request.
    bool covered=r.vehicles.size()==1 || (r.cargo && std::abs((*r.cargo)[0])<=(*r.cargo)[2]/2 && std::abs((*r.cargo)[1])<=(*r.cargo)[3]/2);
    double i=std::floor((q.x-org.x)/res),j=std::floor((q.y-org.y)/res);
    if(covered && (i<0 || j<0 || i>=r.grid->width() || j>=r.grid->height() || r.grid->occupied(i,j))){
      result.status="INVALID_REQUEST";result.reason=endpoint==&r.goal?"OCCUPIED_GOAL_REFERENCE":"OCCUPIED_START_REFERENCE";return result;
    }
  }
  Route direct{{r.start.reference.x,r.start.reference.y},{r.goal.reference.x,r.goal.reference.y}};
  const double d=distance(direct.front(),direct.back());
  if(d<1e-10) {
    Phase rest;rest.mode=Mode::A;rest.kind=Kind::Hold;rest.duration=std::min(.1,r.task_duration_cap);
    Piece q;q.x.fill(r.start.reference.x);q.y.fill(r.start.reference.y);q.yaw.fill(r.start.reference.yaw);
    rest.heading0=rest.heading1=r.start.heading;rest.pieces={q};Plan hold{{rest}};
    if(certify(r,hold,deadline,cancel).executable){
      result.plan=hold;result.status="CERTIFIED_POLICY_COMPLIANT";result.reason="STATIONARY_GOAL";
      result.trace={"HOLD:CERTIFIED"};result.first_policy_compliant_s=result.first_certified_s=elapsed();return result;
    }
  }
  const double projection=(direct[1].x-direct[0].x)*std::cos(r.start.heading[0])+(direct[1].y-direct[0].y)*std::sin(r.start.heading[0]);
  const int sign=r.allow_reverse && projection<0?-1:1;
  std::vector<Seed> seeds;
  bool sharp_departure=false;
  Route staging_route;
  const auto rank_start=rank_at_boundary(r,r.start),rank_goal=rank_at_boundary(r,r.goal);
  const bool locked_start=rank_start.whole_neighborhood,locked_goal=rank_goal.whole_neighborhood;
  auto add=[&](Sequence seq,int gear,Route route,std::string edit,int reverse_from=-1){
    if(!opts.enable_edits && !opts.initial_sequence.empty() && seq!=opts.initial_sequence)return;
    for(auto& s:seeds)if(seq==s.seq && gear==s.sign && reverse_from==s.reverse_from && route.size()==s.route.size() &&
       std::equal(route.begin(),route.end(),s.route.begin(),[](Point a,Point b){return distance(a,b)<1e-8;}))return;
    seeds.push_back({seq,gear,std::move(route),std::move(edit),reverse_from});
  };
  if(!opts.initial_sequence.empty())add(opts.initial_sequence,sign,direct,"SEED");
  else if(d>1e-7)add({Mode::A},sign,direct,"CONTINUOUS_A");
  else add({Mode::S},sign,direct,"SPIN");
  if(d>1e-7 || (r.vehicles.size()==3 && std::abs(delta(r.start.reference.yaw,r.goal.reference.yaw))>1e-7)) {
    if(opts.initial_sequence.empty())add({Mode::A},sign,direct,"CONTINUOUS_A");
    auto route=grid_seed(r,run);
    if(!route.empty()){
      if(pass==SolvePass::Full && r.cargo && opts.initial_sequence.empty() && route.size()>=3)staging_route=route;
      const double heading=std::atan2(route[1].y-route[0].y,route[1].x-route[0].x);
      sharp_departure=std::abs(delta(r.start.heading[0]+(sign<0?pi:0),heading))>pi/2;
      double detour=0;
      for(auto p:route){double dx=direct[1].x-direct[0].x,dy=direct[1].y-direct[0].y;
        detour=std::max(detour,std::abs(dy*(p.x-direct[0].x)-dx*(p.y-direct[0].y))/std::max(1e-8,d));}
      add({Mode::A},sign,route,"CHANGE_ROUTE_SEED");
      if(opts.initial_sequence.empty() && detour>r.limits.radius*.35 && seeds.size()>1)
        std::rotate(seeds.begin(),std::prev(seeds.end()),seeds.end());
      if(route.size()>=4 && r.vehicles.size()==1 && opts.initial_sequence.empty() && r.goal.constrain_heading[0]){
        const auto before=route[route.size()-2],goal=route.back();
        const double terminal=r.goal.heading[0]+(sign<0?pi:0);
        const double approach=std::atan2(goal.y-before.y,goal.x-before.x);
        const double angle=delta(approach,terminal);
        // The position-only graph can approach the goal from the wrong side.
        // Extend the requested terminal tangent backwards on each feasible
        // turning branch, then seed one unconstrained continuous A phase.
        if(std::abs(angle)>pi/3){
          std::vector<Seed> turns;
          for(double rotation:{angle,angle-std::copysign(2*pi,angle)}){
            if(std::abs(rotation)>1.55*pi)continue;
            const double radius=1.65*r.limits.radius, signed_radius=std::copysign(radius,rotation);
            const Point center{goal.x-signed_radius*std::sin(terminal),goal.y+signed_radius*std::cos(terminal)};
            auto arc=[&](double t){const double h=terminal-(1.-t)*rotation;
              return Point{center.x+signed_radius*std::sin(h),center.y-signed_radius*std::cos(h)};
            };
            const auto incoming=arc(0.);
            auto intermediate=r;intermediate.goal.reference={incoming.x,incoming.y,approach};
            auto channel=grid_seed(intermediate,run,true,sign);
            if(channel.empty())channel=grid_seed(intermediate,run);
            if(channel.empty())channel={route.front(),incoming};
            for(int k=1;k<=8;++k)channel.push_back(arc(k/8.));
            double length=0;for(std::size_t j=1;j<channel.size();++j)length+=distance(channel[j-1],channel[j]);
            const int spans=std::clamp(int(std::ceil(length/2.8)),3,HJ_MAX_SPANS);
            Route smooth;
            for(int j=0;j<=spans;++j)smooth.push_back(interpolate(channel,double(j)/spans));
            Seed solved{{Mode::A},sign,std::move(smooth),"TERMINAL_TANGENT_ARC"};
            solved.smooth_channel=true;turns.push_back(std::move(solved));
          }
          auto base=std::find_if(seeds.begin(),seeds.end(),[](const Seed& v){return v.edit=="CHANGE_ROUTE_SEED";});
          seeds.insert(base,turns.begin(),turns.end());
        }
      }
      if(route.size()>=3){
        const double turn=std::abs(delta(r.start.reference.yaw,r.goal.reference.yaw));
        double length=0.,reach=r.limits.radius;
        for(std::size_t j=1;j<route.size();++j)length+=distance(route[j-1],route[j]);
        for(const auto& v:r.vehicles)reach=std::max(reach,std::hypot(v[0],v[1])+r.limits.radius);
        const double turn_station=std::clamp((reach*turn*1.5+r.limits.radius)/std::max(.01,length),.12,.45);
        auto curve=[&](Sequence sequence,double station){
          if(!opts.initial_sequence.empty() && !opts.enable_edits && opts.initial_sequence!=sequence)return;
          Seed seed{sequence,sign,route,"CURVED_OBLIQUE_CHANNEL"};
          seed.curved_oblique=true;seed.switch_station=station;
          seeds.push_back(std::move(seed));
        };
        if(!r.goal.constrain_reference_yaw || turn<1.e-8 || r.vehicles.size()==1)curve({Mode::O},.5);
        if(r.vehicles.size()==3 && turn>=1.e-8){
          // If the route crosses the steering angle branch, a shared spin
          // before O and another before A keep both stopped joins feasible.
          curve({Mode::S,Mode::O,Mode::S,Mode::A},1.-turn_station);
          curve({Mode::O,Mode::A},1.-turn_station);
          curve({Mode::A,Mode::O},turn_station);
          curve({Mode::O,Mode::S,Mode::A},1.-turn_station);
          curve({Mode::S,Mode::O,Mode::A},1.-turn_station);
          curve({Mode::S,Mode::A,Mode::O},turn_station);
        }
      }
      if(r.vehicles.size()==3 && (opts.initial_sequence.empty() || opts.enable_edits || opts.initial_sequence==Sequence{Mode::A})){
        Seed general{{Mode::A},sign,route,"GENERAL_F_YAW"};
        general.general=true;
        seeds.push_back(std::move(general));
      }
      if(detour>r.limits.radius*.35 && (opts.initial_sequence.empty() || opts.enable_edits ||
                                       opts.initial_sequence==Sequence{Mode::S,Mode::A})){
        // Reorient the cars at their stationary mounts, then optimize a
        // continuous Ackermann phase through the unobstructed route.
        Seed aligned{{Mode::S,Mode::A},sign,route,"ALIGN_TO_CHANNEL"};
        aligned.shortest_yaw=true;
        if(sharp_departure){
          auto smooth=aligned;
          smooth.smooth_channel=true;
          smooth.edit="ALIGNED_TANGENT_CHANNEL";
          seeds.push_back(std::move(smooth));
        }
        seeds.push_back(std::move(aligned));
      }
    }
    if(r.vehicles.size()==3 && route.empty() && (opts.initial_sequence.empty() || opts.enable_edits)){
      Seed general{{Mode::A},sign,direct,"GENERAL_F_YAW"};
      general.general=true;seeds.push_back(std::move(general));
    }
    if(r.allow_reverse && opts.initial_sequence.empty())add({Mode::A},-sign,direct,"CHANGE_GEAR");
    add({Mode::O},sign,direct,"INSERT_MODE");
    // Expand both ends of a motion proposal using the same insertion grammar.
    // Try the endpoint with incompatible mount tangents first, if any.
    std::array<bool,2> prepend{locked_start,!locked_start};
    std::vector<Sequence> parents{{Mode::A},{Mode::O}};
    for(auto move:{Mode::S,Mode::O})for(auto q:parents)for(bool left:prepend){
      if(q.front()==move)continue;
      q.insert(left?q.begin():q.end(),move);add(q,sign,direct,"INSERT_MODE");
      q.erase(left?q.begin():std::prev(q.end()));
    }
    Sequence both{Mode::A};
    both.insert(both.begin(),Mode::S);both.push_back(Mode::S);add(both,sign,direct,"INSERT_MODE");
    add({Mode::A,Mode::O,Mode::A},sign,direct,"REPLACE_SUBSEQUENCE");
    if(r.allow_reverse)add({Mode::A,Mode::A},sign,direct,"CHANGE_GEAR",1);
  }
  // A one-block proposal has no auxiliary joins to tune. Explore it before
  // spending the remaining improvement budget on local edits of a long seed.
  std::stable_partition(seeds.begin(),seeds.end(),[](const auto& s){return s.edit=="SEED" || s.seq.size()==1;});
  const auto straight_oblique=[](const Seed& seed){
    return seed.seq==Sequence{Mode::O} && !seed.curved_oblique && seed.route.size()==2;
  };
  const auto first_curve=std::find_if(seeds.begin(),seeds.end(),[](const Seed& s){return s.curved_oblique;});
  const auto straight=std::find_if(first_curve,seeds.end(),straight_oblique);
  if(straight!=seeds.end())std::rotate(first_curve,straight,straight+1);
  // Matching end attitudes can admit a straight translation. Keep time to
  // check it after the Ackermann starts, even if a local solve stalls.
  bool translation_boundary=opts.initial_sequence.empty() &&
    (!r.goal.constrain_reference_yaw || std::abs(delta(r.start.reference.yaw,r.goal.reference.yaw))<1e-8);
  for(std::size_t i=0;i<r.vehicles.size() && translation_boundary;++i)
    translation_boundary=!r.goal.constrain_heading[i] || std::abs(delta(r.start.heading[i],r.goal.heading[i]))<1e-8;
  const auto translation_start=deadline-(deadline-begin)/4;
  // Time laws are native initializations of the same spatial NLP. A near-wall
  // stationary point found with a cruise plateau need not be the one reached
  // with a smooth quintic clock. Both variants must pass the identical audit.
  if(opts.initial_sequence.empty() || opts.enable_edits){
    for(std::size_t i=0;i<seeds.size();++i){
      if(seeds[i].seq==Sequence{Mode::A} && seeds[i].route.size()>2 && seeds[i].edit=="CHANGE_ROUTE_SEED"){
        auto alternative=seeds[i];alternative.edit="CHANGE_TIME_LAW";alternative.quintic=true;
        seeds.insert(seeds.begin()+i+1,std::move(alternative));++i;
      }
    }
  }
  // Cargo yaw can move independently of the tangents of the three mount
  // trajectories. Give the NLP both the low-rotation and channel-following charts.
  if(r.vehicles.size()==3 && r.cargo && opts.initial_sequence.empty() &&
     std::abs(delta(r.start.reference.yaw,r.goal.reference.yaw))>1.e-8) {
    auto route=std::find_if(seeds.begin(),seeds.end(),[](const Seed& s){return s.edit=="CHANGE_ROUTE_SEED";});
    if(route!=seeds.end() && route->route.size()>=4){
      const auto cargo=*r.cargo;
      const double arm=std::hypot(cargo[0],cargo[1])+.5*std::hypot(cargo[2],cargo[3]);
      const bool subcell_rotation=arm*std::abs(delta(r.start.reference.yaw,r.goal.reference.yaw))<=r.grid->resolution();
      auto low_rotation=*route;
      low_rotation.edit="MINIMUM_YAW_ROUTE";
      low_rotation.shortest_yaw=true;
      if(!subcell_rotation){
        auto wide=grid_seed(r,run,true,sign,&low_rotation.channel_yaw,&route->route);
        if(!wide.empty())low_rotation.route=std::move(wide);
        low_rotation.smooth_channel=true;
      }
      if(!low_rotation.channel_yaw.empty()){
        auto minimum=low_rotation;
        minimum.channel_yaw.clear();
        low_rotation.edit="CLEARANCE_YAW_ROUTE";
        // Keep the original channel in second place. Both orientation hypotheses
        // go through the same fixed-sequence optimizer and full interval check.
        const auto index=route-seeds.begin();
        seeds.insert(seeds.begin()+index,std::move(low_rotation));
        seeds.insert(seeds.begin()+index+2,std::move(minimum));
      }else seeds.insert(route,std::move(low_rotation));
    }
  }
  if(sharp_departure && r.allow_reverse && r.cargo && opts.initial_sequence.empty()){
    // Terminal displacement may suggest forward while the free channel at
    // the initial footprint actually points backward. Search the opposite
    // gear on that same channel before introducing any steering-mode switch.
    auto base=std::find_if(seeds.begin(),seeds.end(),[](const Seed& s){return s.edit=="CHANGE_ROUTE_SEED";});
    if(base!=seeds.end()){
      auto opposite=*base;
      opposite.sign=-sign;opposite.shortest_yaw=true;opposite.smooth_channel=true;
      opposite.edit="OPPOSITE_GEAR_CHANNEL";
      seeds.insert(seeds.begin(),std::move(opposite));
    }
  }
  // Near a rank transition, constrain the terminal tangent against its
  // certified angle window. The original request is always used in certify().
  if(rank_goal.exact && !locked_goal){
    for(auto& seed:seeds)if(seed.seq.back()==Mode::A)seed.relaxed=true;
  }
  result.seed_seconds=elapsed();
  for(auto& seed:seeds){
    if(seed.edit=="SEED")seed.priority=-1;
    else if(seed.seq==Sequence{Mode::A})seed.priority=0;
    else if(seed.seq.size()==1)seed.priority=2;
    else seed.priority=8+int(seed.seq.size());
  }
  std::array<std::optional<Route>,2> oblique_channels;
  bool tried_oblique_departure=false;
  PolicyScore best{};bool unranked_incumbent=false;std::string why;
  bool repaired_channel=false;
  bool tried_local_stage=false;
  const bool large_cargo_turn=r.goal.constrain_reference_yaw &&
    std::abs(delta(r.start.reference.yaw,r.goal.reference.yaw))>=pi/4.;
  std::array<double,3> heading_offsets;
  std::transform(r.start.heading.begin(),r.start.heading.end(),heading_offsets.begin(),
    [&](double h){return delta(r.start.heading.front(),h);});
  const auto [lo,hi]=std::minmax_element(heading_offsets.begin(),heading_offsets.end());
  const bool dispersed_start=*hi-*lo>endpoint_heading_window && *hi-*lo<=2*endpoint_heading_window;
  const auto early_stage_slice=std::min((deadline-begin)/3,
    std::chrono::duration_cast<Clock::duration>(std::chrono::milliseconds(3500)));
  bool lateral_arrival=false;
  if(!staging_route.empty()){
    const auto before=staging_route[staging_route.size()-2];
    const auto goal=staging_route.back();
    const double tangent=std::atan2(goal.y-before.y,goal.x-before.x);
    lateral_arrival=std::abs(std::cos(delta(r.goal.reference.yaw,tangent)))<.75;
  }
  auto accept_stage=[&](Plan staged){
    result.first_certified_s=result.first_policy_compliant_s=elapsed();
    result.plan=std::move(staged);
    result.status="CERTIFIED_POLICY_COMPLIANT";
    result.reason="CERTIFIED_LOCAL_STAGE";
  };
  for(std::size_t ordinal=0;ordinal<seeds.size();++ordinal) {
    const auto total_budget=std::chrono::duration<double>(deadline-begin).count();
    const auto rejection=result.attempts.empty()?std::string{}:result.attempts.back().status;
    const bool departure_collision=rejection.find("OBB_COLLISION")!=std::string::npos;
    const bool departure_rejection=rejection=="START_ENDPOINT" || rejection=="STEER_RATE" ||
      rejection=="STEER_ACCELERATION" || rejection=="ACKERMANN_CURVATURE";
    // The final active constraint can vary with a shortened NLP slice. A
    // whole-route fit that already consumed a third of the budget also
    // warrants local staging, even when curvature was the last rejection.
    const bool slow_initial_fit=ordinal==1 && elapsed()>.32*total_budget;
    const bool early_staging=ordinal==1 && large_cargo_turn &&
      (dispersed_start || departure_rejection || departure_collision);
    const bool try_staging=early_staging || (sharp_departure?
      ((ordinal==1 && departure_collision)||slow_initial_fit||tried_oblique_departure):
      elapsed()>.40*total_budget);
    if(!result.plan && !tried_local_stage && !staging_route.empty() && ordinal>0 && try_staging &&
       deadline-Clock::now()>=std::chrono::seconds(3)){
      tried_local_stage=true;
      std::optional<Plan> staged;
      if(sharp_departure){
        staged=depart_and_resume(r,staging_route,run,result);
        if(!staged)staged=stage_backwards(r,staging_route,run,result);
      }else if(large_cargo_turn && sign>0){
        if(dispersed_start || departure_rejection){
          // Route the known boundary mismatch independently of the last NLP
          // rejection, which can change with solver rounding or CPU time.
          const Run departure{std::min(deadline,Clock::now()+
            std::chrono::milliseconds(dispersed_start?5900:3250)),cancel};
          staged=depart_and_resume(r,staging_route,departure,result,true);
          if(!staged)staged=approach_from_channel(r,staging_route,run,result);
        }else{
          staged=approach_from_channel(r,staging_route,run,result);
          if(!staged)staged=depart_and_resume(r,staging_route,run,result);
        }
      }else{
        staged=stage_backwards(r,staging_route,run,result);
        if(!staged && large_cargo_turn)staged=approach_from_channel(r,staging_route,run,result);
        if(!staged)staged=depart_and_resume(r,staging_route,run,result);
      }
      if(staged){
        accept_stage(std::move(*staged));
        return result;
      }
    }
    auto translation=seeds.end();
    if(!result.plan && translation_boundary)
      translation=std::find_if(seeds.begin()+ordinal,seeds.end(),straight_oblique);
    if(translation!=seeds.end() && Clock::now()>=translation_start){
      std::rotate(seeds.begin()+ordinal,translation,translation+1);
    }else if(!result.plan && sharp_departure && !tried_oblique_departure &&
       elapsed()>.36*std::chrono::duration<double>(deadline-begin).count()){
      // A direct tangent fit can fold back when the outgoing corridor is
      // behind the chosen gear. Try a stopped alignment before one continuous
      // A phase; fully oblique alternatives remain available farther down.
      const auto curve=std::find_if(seeds.begin()+ordinal,seeds.end(),[](const Seed& s){
        return s.edit=="ALIGNED_TANGENT_CHANNEL";});
      if(curve!=seeds.end())std::rotate(seeds.begin()+ordinal,curve,curve+1);
      tried_oblique_departure=true;
    }else if (!result.plan && elapsed()>.58*std::chrono::duration<double>(deadline-begin).count()) {
      auto repair=std::find_if(seeds.begin()+ordinal,seeds.end(),[](const Seed& s){return s.edit=="ALIGN_TO_CHANNEL";});
      if(repair!=seeds.end())std::rotate(seeds.begin()+ordinal,repair,repair+1);
    }
    Seed seed=seeds[ordinal]; // appending child proposals may reallocate the queue
    const bool aligned_tangent=seed.edit=="ALIGNED_TANGENT_CHANNEL";
    const bool opposite_tangent=seed.edit=="OPPOSITE_GEAR_CHANNEL";
    if(seed.curved_oblique || aligned_tangent || opposite_tangent){
      const auto first=std::find_if(seed.seq.begin(),seed.seq.end(),[](Mode mode){return mode!=Mode::S;});
      const bool starts_oblique=aligned_tangent || opposite_tangent || *first==Mode::O;
      auto& channel=oblique_channels[starts_oblique?0:1];
      if(!channel){
        auto fixed=r;
        fixed.start.reference.yaw=fixed.goal.reference.yaw=starts_oblique?r.start.reference.yaw:r.goal.reference.yaw;
        channel=grid_seed(fixed,run,true,seed.sign,nullptr,nullptr,true);
      }
      if(!channel->empty())seed.route=*channel;
      if(aligned_tangent || opposite_tangent){
        const auto& points=seed.route;
        const auto& before=points[points.size()-2];
        const double entry=std::atan2(points[1].y-points[0].y,points[1].x-points[0].x);
        const double exit=std::atan2(points.back().y-before.y,points.back().x-before.x);
        auto headings=r.start.heading;headings.fill(central_tangent(points));
        shape_translation_seed(r,seed.route,entry,exit,r.start.reference.yaw,headings.data(),run);
      }
    }
    auto propose=[&](std::size_t at,bool accepted){
      if(!opts.enable_edits || seed.seq.empty() || run.stop())return;
      const auto edit_start=Clock::now();
      struct EditTimer {
        double& total;Clock::time_point start;
        ~EditTimer(){total+=std::chrono::duration<double>(Clock::now()-start).count();}
      } timer{result.edit_seconds,edit_start};
      at=std::min(at,seed.seq.size()-1);
      auto child=[&](Sequence sequence,const char* operation,int reverse=-1,bool first=false){
        if(sequence.empty() || sequence.size()>HJ_MAX_PHASES || (sequence==seed.seq && reverse==seed.reverse_from))return;
        Seed next{sequence,seed.sign,seed.route,operation,reverse};
        // Prefer pure-A replacements, then short edits of a certified parent,
        // then repairs near a failed phase. These are ordering heuristics.
        next.priority=sequence==Sequence{Mode::A}?0:(accepted?3:5)+int(sequence.size());
        next.relaxed=rank_goal.exact && !locked_goal && sequence.back()==Mode::A;
        auto duplicate=std::find_if(seeds.begin(),seeds.end(),[&](const Seed& existing){
          return existing.seq==next.seq && existing.sign==next.sign && existing.reverse_from==reverse &&
                 existing.general==next.general && existing.relaxed==next.relaxed && existing.route.size()==next.route.size() &&
                 std::equal(existing.route.begin(),existing.route.end(),next.route.begin(),[](Point a,Point b){return distance(a,b)<1e-8;});
        });
        if(duplicate!=seeds.end()){
          if(static_cast<std::size_t>(duplicate-seeds.begin())>ordinal)duplicate->priority=std::min(duplicate->priority,next.priority);
          if(first && static_cast<std::size_t>(duplicate-seeds.begin())>ordinal){
            duplicate->edit=operation;
            std::rotate(seeds.begin()+ordinal+1,duplicate,duplicate+1);
          }
          return;
        }
        if(seeds.size()>=48)return;
        if(first)seeds.insert(seeds.begin()+ordinal+1,std::move(next));else seeds.push_back(std::move(next));
      };
      const bool has_aux=std::any_of(seed.seq.begin(),seed.seq.end(),[](Mode m){return m!=Mode::A;});
      if(has_aux){
        // Rebuild the neighboring boundaries instead of forcing a zero-length phase.
        for(std::size_t j=0;j<seed.seq.size();++j)if(seed.seq[j]!=Mode::A){
          auto reduced=seed.seq;reduced.erase(reduced.begin()+j);child(reduced,"REMOVE_BLOCK");
          auto replace=seed.seq;replace[j]=Mode::A;
          replace.erase(std::unique(replace.begin(),replace.end()),replace.end());
          child(replace,"REPLACE_SUBSEQUENCE");
        }
        if(d>1e-7)child({Mode::A},"REPLACE_SUBSEQUENCE",-1,true);
      }
      if(seed.reverse_from<0){
        auto merged=seed.seq;merged.erase(std::unique(merged.begin(),merged.end()),merged.end());
        child(merged,"MERGE_SAME_MODE");
      }
      if(!accepted){
        for(auto mode:{Mode::S,Mode::O})for(std::size_t pos:{at,at+1}){
          auto extended=seed.seq;extended.insert(extended.begin()+pos,mode);child(extended,"INSERT_MODE");
        }
        if(r.allow_reverse && seed.seq[at]==Mode::A){
          auto mixed=seed.seq;mixed.insert(mixed.begin()+at,Mode::A);child(mixed,"CHANGE_GEAR",int(at+1));
        }
      }
      std::stable_sort(seeds.begin()+ordinal+1,seeds.end(),[](const Seed& a,const Seed& b){return a.priority<b.priority;});
    };
    if(run.stop())break;
    const auto improve_until=begin+std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(result.first_certified_s+.65));
    const bool improving_aux=result.plan && best.auxiliary_blocks && seed.seq!=Sequence{Mode::A};
    if(improving_aux && Clock::now()>=improve_until)break;
    try{
      if(seed.seq.empty())continue;
      const auto tag=seed.edit+":"+name(seed.seq)+":"+std::to_string(seed.sign);
      Attempt metrics{tag,"REJECTED"};
      const auto started=Clock::now();
      struct Finish {
        std::vector<Attempt>& attempts;
        Attempt& a;
        Clock::time_point start;
        double staging_s{};
        ~Finish(){
          a.wall_s=std::chrono::duration<double>(Clock::now()-start).count()-staging_s;
          attempts.push_back(std::move(a));
        }
      } finish{result.attempts,metrics,started};
      if((seed.seq.front()==Mode::A && locked_start) || (seed.seq.back()==Mode::A && locked_goal)){
        why="ENDPOINT_MOUNT_TANGENT_RANK";result.trace.push_back(tag+":"+why);propose(locked_start?0:seed.seq.size()-1,false);continue;
      }
      // Keep time for edits after a poor local initialization.
      double slice=seed.route.size()>2?5.6:(seed.seq.size()>2?2.15:2.8);
      Run trial{std::min(deadline,Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(slice))),cancel};
      if(translation!=seeds.end() && !straight_oblique(seed) && Clock::now()<translation_start)
        trial.end=std::min(trial.end,translation_start);
      if(result.plan && best.auxiliary_blocks)trial.end=std::min(trial.end,improve_until);
      auto graph=problem(r,seed,opts,trial);
      const bool continuous_channel=graph.count==1 && graph.phase[0].mode!=2 && graph.phase[0].spans>1;
      if(!result.plan && (continuous_channel || seed.curved_oblique || aligned_tangent)){
        const auto before=Clock::now();
        Run timing{std::min(trial.end,before+std::chrono::milliseconds(300)),cancel};
        Clock::time_point first_proof;
        std::optional<Plan> timed;
        std::string timing_status;
        try{timed=continuous_channel?retime_channel(r,graph,timing,first_proof):
              retime_mixed_channel(r,graph,timing,first_proof,timing_status);}
        catch(const std::domain_error&){/* The full NLP can repair a singular initial tangent. */}
        double repair_seed_s=0.;
        if(!timed && !repaired_channel && graph.count==1 && graph.phase[0].mode==0 &&
           r.cargo && graph.bodies==3 && graph.phase[0].spans>=4 &&
           (opposite_tangent || seed.edit=="CLEARANCE_YAW_ROUTE" || seed.edit=="MINIMUM_YAW_ROUTE")) {
          const auto repair_start=Clock::now();
          double repair_certification_s=0.;
          const auto cuts=separators(r,graph,trial);
          const bool near_cargo=std::any_of(cuts.begin(),cuts.end(),[](const auto& plane){return plane.vehicle<0;});
          if(near_cargo) {
            if(!opposite_tangent)repaired_channel=true;
            auto stopped=[&](){return trial.stop();};
            const auto boxes=channel_repair::rectangles(*r.grid,stopped);
            // Move the cargo rotation to either side of the tight passage, then
            // refine the early-turn chart by one knot at the opposite shoulder.
            const int original=graph.phase[0].spans;
            const int fine=std::min(HJ_MAX_SPANS,original+4);
            struct PostureChart {int spans; double start,end;};
            std::vector<PostureChart> charts{{std::min(HJ_MAX_SPANS,original+2),.5,1.},
              {std::min(HJ_MAX_SPANS,original+3),0.,1./3},{fine,1./(3*fine),1./3}};
            if(opposite_tangent)charts={{original,0.,1.}};

            for(const auto& chart:charts) {
              if(timed || boxes.empty() || stopped())break;
              HJProblem candidate=graph;
              if(!channel_repair::repair(r,candidate,boxes,chart.spans,chart.start,chart.end,stopped))continue;
              Clock::time_point first;
              const auto certification_start=Clock::now();
              try{timed=retime_channel(r,candidate,trial,first);}
              catch(const std::domain_error&){/* A singular mount tangent invalidates this initial guess. */}
              repair_certification_s+=std::chrono::duration<double>(Clock::now()-certification_start).count();
              if(timed){
                graph=candidate;first_proof=first;
                result.trace.push_back(tag+":CERTIFIED_POSTURE_REPAIR");
              }
            }
          }
          repair_seed_s=std::chrono::duration<double>(Clock::now()-repair_start).count()-repair_certification_s;
          result.seed_seconds+=repair_seed_s;
        }
        metrics.certify_s+=std::chrono::duration<double>(Clock::now()-before).count()-repair_seed_s;
        if(!timed && !timing_status.empty())result.trace.push_back(tag+":INITIAL_CHANNEL:"+timing_status);
        if(timed){
          result.first_certified_s=std::chrono::duration<double>(first_proof-begin).count();
          try{best=score_policy(r,*timed,deadline,cancel);}
          catch(const std::runtime_error&){
            if(!run.stop())throw;
            result.plan=std::move(*timed);
            unranked_incumbent=true;
            result.trace.push_back(tag+":CERTIFIED_CHANNEL_TIMING:POLICY_EVALUATION_DEADLINE");
            break;
          }
          if(graph.count>1){
            // Keep the jointly feasible rest/steering-compatible duration
            // estimate when switching from the shape preflight to Ipopt.
            int ordinal=0;
            for(const auto& phase:timed->phases)if(phase.kind==Kind::Motion){
              while(ordinal<graph.count && graph.phase[ordinal].mode!=static_cast<int>(phase.mode))++ordinal;
              if(ordinal<graph.count)graph.phase[ordinal++].tau=phase.duration;
            }
          }
          result.plan=std::move(*timed);
          result.trace.push_back(tag+":CERTIFIED_CHANNEL_TIMING");
          // A local arrival still has to be joined and audited with its
          // straight tail. Preserve the feasible pure-A lead-in for that
          // proof instead of polishing it with a full NLP first.
          if(pass==SolvePass::Arrival && !best.auxiliary_blocks){
            metrics.status="CERTIFIED";
            break;
          }
          trial.end=std::min(trial.end,Clock::now()+std::chrono::milliseconds(1600));
        }
      }
      // A fast continuous certificate retains priority. If it fails at a
      // constrained boundary, try the applicable local stage before spending
      // the whole-route NLP budget rebuilding collision and curvature cuts.
      if(ordinal==0 && !result.plan && !staging_route.empty() && large_cargo_turn &&
         seed.seq==Sequence{Mode::A} && (sharp_departure || sign<0 || dispersed_start || lateral_arrival)){
        const auto staged_begin=Clock::now();
        const auto early_end=std::min(deadline-std::chrono::seconds(2),staged_begin+early_stage_slice);
        if(early_end-staged_begin>=std::chrono::seconds(1)){
          const Run early{early_end,cancel};
          std::optional<Plan> staged;
          if(sharp_departure)staged=depart_and_resume(r,staging_route,early,result);
          else if(sign<0)staged=stage_backwards(r,staging_route,early,result);
          else if(dispersed_start)staged=depart_and_resume(r,staging_route,early,result,true);
          else staged=approach_from_channel(r,staging_route,early,result,SolvePass::Arrival);
          const auto local_time=Clock::now()-staged_begin;
          finish.staging_s=std::chrono::duration<double>(local_time).count();
          if(staged){
            metrics.status="DEFERRED_TO_CERTIFIED_STAGE";
            accept_stage(std::move(*staged));
            break;
          }
          // The speculative stage has a separate budget. Give the original
          // candidate its remaining slice, still under the caller's deadline.
          trial.end=std::min(deadline,trial.end+local_time);
        }
      }
      auto active=separators(r,graph,run);
      std::vector<HJCurvatureStation> curvature_stations;
      graph.separations=active.data();graph.sep_count=active.size();
      char msg[512]{};
      bool solved=hj_optimize(&graph,msg,sizeof(msg));
      metrics.nlp_build_s=graph.nlp_build_s;metrics.nlp_solve_s=graph.nlp_solve_s;
      if(!solved){why=msg;metrics.status=why;result.trace.push_back(tag+":"+why);if(result.plan && !best.auxiliary_blocks)break;propose(locked_goal?seed.seq.size()-1:0,false);continue;}
      if(graph.primal_status[0])result.trace.push_back(tag+":UNCONVERGED_PRIMAL:"+graph.primal_status);
      auto plan=expand(r,graph);
      bool accepted=false;
      std::size_t failed_motion=0;
      int geometric_refinements=0;
      std::set<std::pair<std::size_t,double>> tried_capped_ramps;
      for(int retry=0;retry<12 && !run.stop();++retry){
        if(plan.phases.empty() || duration(plan)>r.task_duration_cap+1e-8){why="TASK_TIME_CONSTRAINT";break;}
        const auto audit_start=Clock::now();const auto audit=certify(r,plan,deadline,cancel);
        metrics.certify_s+=std::chrono::duration<double>(Clock::now()-audit_start).count();
        metrics.status=audit.reason;
        if(audit.executable) {
          accepted=true;
          const bool first=!result.plan;
          if(first)result.first_certified_s=elapsed();
          PolicyScore ranking;
          try{ranking=score_policy(r,plan,deadline,cancel);}
          catch(const std::runtime_error&){
            if(!run.stop())throw;
            if(first){result.plan=plan;unranked_incumbent=true;}
            result.trace.push_back(tag+":CERTIFIED:POLICY_EVALUATION_DEADLINE");
            break;
          }
          result.trace.push_back(tag+":CERTIFIED");
          if(first || ranking.better_than(best)){result.plan=plan;best=ranking;}
          break;
        }
        why=audit.reason;result.trace.push_back(tag+":"+why);
        if(audit.phase>=plan.phases.size() || plan.phases[audit.phase].kind!=Kind::Motion)break;
        failed_motion=std::count_if(plan.phases.begin(),plan.phases.begin()+audit.phase,
                                  [](const Phase& ph){return ph.kind==Kind::Motion;});
        std::string refinement;
        if(why.find("OBB_COLLISION")!=std::string::npos && geometric_refinements<3 && !trial.stop()){
          // Use the position of the failed continuous proof, not just the
          // original collocation points. Preserve all earlier active planes.
          std::size_t before=active.size();
          const double ramp=plan.phases[audit.phase].cruise_ramp_fraction;
          const double witness_time=.5*(hybrid_clock::inverse(audit.u0,ramp)+hybrid_clock::inverse(audit.u1,ramp));
          const double witness_station=hybrid_clock::state(witness_time,ramp)[0];
          for(double u:{witness_station,.25*audit.u0+.75*audit.u1,.5*(audit.u0+audit.u1),.75*audit.u0+.25*audit.u1}){
            auto cuts=separators(r,graph,trial,WitnessStation{static_cast<int>(failed_motion),u});
            for(const auto& cut:cuts){
              auto same=[&](const HJSeparation& v){return v.phase==cut.phase && v.span==cut.span &&
                v.vehicle==cut.vehicle && v.other==cut.other && std::abs(v.station-cut.station)<1e-7 &&
                (v.other==HJSeparation::map_cell
                  ? std::abs(v.other_x-cut.other_x)+std::abs(v.other_y-cut.other_y)<1e-7
                  : std::abs(v.nx-cut.nx)+std::abs(v.ny-cut.ny)+std::abs(v.obstacle_support-cut.obstacle_support)<1e-7);};
              if(std::none_of(active.begin(),active.end(),same))active.push_back(cut);
            }
          }
          if(active.size()>before)refinement="REFINE_WITNESS";
        }else if(why=="ACKERMANN_CURVATURE" && geometric_refinements<3 && !trial.stop()){
          // The audit returns the first unresolved interval. Constrain its midpoint
          // and its neighboring collocation cell to cover a peak past that first crossing.
          const int spans=graph.phase[failed_motion].spans;
          const double station=.5*(audit.u0+audit.u1)*spans;
          const int span=std::min(int(station),spans-1);
          const double local=station-span;
          const int nodes=graph.bodies==3 && spans>=3 ? 3:6;
          const int cell=std::min(int(local*nodes),nodes-1);
          for(double u:{local,(cell+.25)/nodes,(cell+.5)/nodes,(cell+.75)/nodes}){
            auto already=[&](const auto& v){return v.phase==int(failed_motion) && v.span==span && std::abs(v.local-u)<1e-6;};
            if(std::none_of(curvature_stations.begin(),curvature_stations.end(),already)){
              curvature_stations.push_back({int(failed_motion),span,u});refinement="REFINE_CURVATURE";
            }
          }
        }
        if(!refinement.empty() && !trial.stop()){
          ++geometric_refinements;
          result.trace.push_back(tag+":"+refinement+":"+std::to_string(failed_motion)+":"+std::to_string(audit.u0)+":"+std::to_string(audit.u1));
          graph.separations=active.data();graph.sep_count=active.size();
          graph.curvature_stations=curvature_stations.data();graph.curvature_station_count=curvature_stations.size();
          graph.phase[failed_motion].tau=plan.phases[audit.phase].duration;
          graph.nlp_build_s=graph.nlp_solve_s=0.;
          const bool converged=hj_optimize(&graph,msg,sizeof(msg));
          metrics.nlp_build_s+=graph.nlp_build_s;metrics.nlp_solve_s+=graph.nlp_solve_s;
          if(!converged){why=msg;result.trace.push_back(tag+":REFINEMENT_FAILED:"+why);break;}
          plan=expand(r,graph);
          continue;
        }
        // The certificate distinguishes spatial violations from time-law violations.
        // Retiming preserves geometry, boundary rest and the authoritative mount identities.
        if(why.find("SPEED")==std::string::npos && why.find("ACCELERATION")==std::string::npos &&
           why.find("JERK")==std::string::npos && why.find("RATE")==std::string::npos &&
           why.find("DYNAMIC")==std::string::npos)break;
        auto& phase_duration=plan.phases[audit.phase].duration;
        const double previous=phase_duration;
        const double available=previous+(r.task_duration_cap-duration(plan));
        phase_duration=std::min(previous*1.25,available);
        // Sum all actual phases in the same order as the certificate. Round
        // toward the feasible side only when their floating-point sum exceeds the cap.
        while(phase_duration>previous && duration(plan)>r.task_duration_cap)
          phase_duration=std::nextafter(phase_duration,0.);
        if(phase_duration>previous)continue;
        phase_duration=previous;
        auto& ramp=plan.phases[audit.phase].cruise_ramp_fraction;
        if(ramp==0.)break;
        // A path can have high mount speed near one shoulder or on its
        // plateau. At the cap, try each other supported law on the same geometry.
        tried_capped_ramps.emplace(audit.phase,ramp);
        const auto next=std::find_if(cruise_ramps.begin(),cruise_ramps.end(),[&](double candidate){
          return !tried_capped_ramps.count({audit.phase,candidate});});
        if(next==cruise_ramps.end())break;
        ramp=*next;
        result.trace.push_back(tag+":RETIME_AT_CAP:"+std::to_string(ramp));
      }
      propose(failed_motion,accepted);
      if(result.plan && (best.auxiliary_blocks==0 ||
         ((locked_start||locked_goal) && seed.seq.size()<=2 && seed.seq.front()!=Mode::O && seed.seq.back()!=Mode::O)))break;
      if(result.plan && elapsed()-result.first_certified_s>.75) break;
    }catch(const std::exception& e){why=e.what();result.trace.push_back(why);propose(0,false);}
  }
  if(cancel&&cancel()){result.plan.reset();result.status="CANCELLED";result.reason=result.status;}
  else if(result.plan){
    if(result.reason=="CERTIFIED_LOCAL_STAGE")return result;
    result.status=unranked_incumbent || (run.stop() && best.auxiliary_blocks)?"FEASIBLE_POLICY_DEGRADED":"CERTIFIED_POLICY_COMPLIANT";
    result.reason=unranked_incumbent?"POLICY_EVALUATION_DEADLINE":!best.auxiliary_blocks?"PURE_A":locked_start||locked_goal?"ENDPOINT_CONFIGURATION":
      run.stop()?"A_SOLVE_TIMEOUT":"A_SEARCH_SPACE_EXHAUSTED";
    // The synchronous solver and ROS adapter expose only the terminal policy
    // decision. Earlier certified incumbents remain a separate diagnostic.
    if(result.status=="CERTIFIED_POLICY_COMPLIANT")result.first_policy_compliant_s=elapsed();
  }
  else{result.status="NO_CERTIFIED_SOLUTION_WITHIN_BUDGET";result.reason=run.stop()?"A_SOLVE_TIMEOUT":why;}
  return result;
}

void align_terminal(const Request& r,const Run& run,Result& result) {
  if(!r.align_terminal_chassis || r.vehicles.size()!=3 || !r.cargo)return;
  if(!result.plan){result.terminal_alignment="skipped_no_plan";return;}
  if(run.stop()){result.terminal_alignment="skipped_budget";return;}
  const auto end=evaluate(r,*result.plan,duration(*result.plan));
  const double yaw=end.reference.yaw;
  const bool aligned=std::all_of(end.bodies.begin(),end.bodies.end(),
      [&](const Body& b){return std::abs(delta(b.pose.yaw,yaw))<=pi/180.;});
  if(aligned){result.terminal_alignment="already_aligned";return;}
  for(std::size_t i=0;i<r.vehicles.size();++i){
    if(r.goal.constrain_heading[i] && std::abs(delta(r.goal.heading[i],yaw))>pi/180.){
      result.terminal_alignment="skipped_heading_constraint";return;
    }
  }
  auto candidate=*result.plan;
  append_motion(r,candidate,stationary_spin(r,end,Headings{yaw,yaw,yaw}));
  const auto proof=certify(r,candidate,run.end,run.cancel);
  if(proof.executable){
    result.plan=std::move(candidate);
    result.terminal_alignment="aligned";
  }else{
    result.terminal_alignment="skipped_"+proof.reason;
  }
}
} // namespace

Result solve(const Request& r,Clock::time_point deadline,const Cancel& cancel,const SolveOptions& opts){
  auto result=solve_route(r,deadline,cancel,opts,SolvePass::Full);
  align_terminal(r,Run{deadline,cancel},result);
  if(cancel&&cancel()){
    result.plan.reset();
    result.status=result.reason="CANCELLED";
  }
  return result;
}
} // namespace fwis_hybrid_joint
