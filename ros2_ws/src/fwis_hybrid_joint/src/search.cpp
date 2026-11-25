#include "fwis_hybrid_joint/model.hpp"
#include "nlp_api.h"

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
struct Seed { Sequence seq; int sign; Route route; std::string edit; int reverse_from{-1}; bool quintic{}; };
struct Ranked {
  int presence;
  double use,cost;
  bool operator<(const Ranked& b)const{return std::tie(presence,use,cost)<std::tie(b.presence,b.use,b.cost);}
};

// Occupancy A* only proposes continuous optimization starts. Full swept footprint
// checks (including independent vehicle headings) are performed by the certifier.
Route grid_seed(const Request& r,const Run& run) {
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
      double ng=g[id]+std::hypot(x,y)*res*(1+1.5*std::pow(reach/std::max(.15,clearance),3));
      if(ng+1e-9>=g[next])continue;
      g[next]=ng;parent[next]=id;
      frontier.emplace(ng+distance(point(next),point(to)),next);
    }
  }
  if(parent[to]<0)return {};
  Route rev{goal};
  for(int v=parent[to];v!=from && v>=0;v=parent[v])rev.push_back(point(v));
  rev.push_back(start);std::reverse(rev.begin(),rev.end());
  std::vector<double> arc{0};for(std::size_t i=1;i<rev.size();++i)arc.push_back(arc.back()+distance(rev[i-1],rev[i]));
  const int spans=std::clamp(int(std::ceil(arc.back()/4.0)),2,HJ_MAX_SPANS);
  Route out{start};
  for(int i=1;i<spans;++i){
    double at=arc.back()*i/spans;
    auto j=std::upper_bound(arc.begin(),arc.end(),at)-arc.begin();
    double f=(at-arc[j-1])/(arc[j]-arc[j-1]);
    out.push_back({rev[j-1].x+f*(rev[j].x-rev[j-1].x),rev[j-1].y+f*(rev[j].y-rev[j-1].y)});
  }
  out.push_back(goal);return out;
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

// A clamped, uniform quintic B-spline initialized from the channel tangent.
// Insert every interior knot to multiplicity five to extract exact Bernstein
// spans. The fixed-yaw chart has C4 joins, with zero angular rate at both ends.
std::vector<Quintic> route_orientation(const Route& path,double start,double end){
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
  for(int j=1;j<=n;++j)a[j]+=residual*j/n;
  std::vector<double> controls(n+degree);
  for(int i=0;i<n+degree;++i){
    double t=0;for(int j=1;j<=degree;++j)t+=knots[i+j]/degree;
    int j=std::clamp(int(t),0,n-1);
    controls[i]=a[j]+(t-j)*(a[j+1]-a[j]);
  }
  for(int i=0;i<3;++i){controls[i]=start;controls[controls.size()-1-i]=end;}
  for(int x=1;x<n;++x)for(int rep=1;rep<degree;++rep){
    int k=std::upper_bound(knots.begin(),knots.end(),x)-knots.begin()-1;
    std::vector<double> refined(controls.size()+1);
    for(int j=0;j<=k-degree;++j)refined[j]=controls[j];
    for(int j=k-rep;j<(int)controls.size();++j)refined[j+1]=controls[j];
    for(int j=k-degree+1;j<=k-rep;++j){
      double f=(x-knots[j])/(knots[j+degree]-knots[j]);
      refined[j]=controls[j-1]+f*(controls[j]-controls[j-1]);
    }
    controls.swap(refined);knots.insert(knots.begin()+k+1,x);
  }
  std::vector<Quintic> pieces(n);for(int j=0;j<n;++j)std::copy_n(controls.begin()+5*j,6,pieces[j].begin());
  return pieces;
}
HJProblem problem(const Request& r,const Seed& seed,const SolveOptions& options,Run& run) {
  const auto& l=r.limits;
  HJProblem p{};
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
    const auto start=interpolate(seed.route,m/double(std::max(1,moving)));
    const auto end=ph.mode==2?start:interpolate(seed.route,(m+1)/double(moving));
    const int sign=seed.reverse_from>=0 && k>=seed.reverse_from?-seed.sign:seed.sign;
    const double dx=end.x-start.x,dy=end.y-start.y,length=std::hypot(dx,dy);
    const double travel=length>1e-9?std::atan2(dy,dx):p.start[2];
    double endpsi=psi;
    if(ph.mode==0) endpsi=psi+delta(psi,p.goal[2])/(moving-m);
    for(int i=0;i<p.bodies;++i){
      ph.h0[i]=headings[i];
      const int orientation=std::cos(headings[i]-ph.h0[0])>=0?1:-1;
      ph.gear[i]=ph.mode==0?sign*orientation:(ph.mode==2 || std::cos(travel-headings[i])>=0?1:-1);
      if(ph.mode==0 && ph.gear[i]<0 && !r.allow_reverse)ph.gear[i]=1;
      if(ph.mode==2) {
        bool last=k+1==p.count;
        const double next=last?p.goal[3+i]:std::atan2(seed.route.back().y-seed.route.front().y,seed.route.back().x-seed.route.front().x)+(seed.sign<0?pi:0);
        headings[i]+=delta(headings[i],next);
      } else if(ph.mode==0) {
        const double next=k+1==p.count && p.fixed_goal[i+1]?p.goal[3+i]:travel+(ph.gear[i]<0?pi:0);
        headings[i]+=delta(headings[i],next);
      }
      ph.h1[i]=headings[i];
    }
    double spin=0;for(int i=0;i<p.bodies;++i)spin=std::max(spin,std::abs(ph.h1[i]-ph.h0[i]));
    double amount=ph.mode==2?spin:length;
    // At target speed, a cubic velocity ramp needs at least this much time
    // for the configured acceleration and jerk. Use a cruise plateau only
    // where both ramps fit naturally in a long translating motion.
    const double min_ramp=std::max(1.5*l.speed/l.acceleration,std::sqrt(6*l.speed/l.jerk));
    if(!seed.quintic && ph.mode!=2 && length>=7*l.speed*min_ramp) ph.cruise_ramp_fraction=.125;

    const double peak=ph.cruise_ramp_fraction>0?1./(1-ph.cruise_ramp_fraction):1.875;
    if(moving==1 && seed.route.size()>2 && ph.mode==0){
      amount=0;for(std::size_t j=1;j<seed.route.size();++j)amount+=distance(seed.route[j-1],seed.route[j]);
    }
    ph.tau=std::clamp(1.15*std::max({.5,peak*amount/(ph.mode==2?l.spin_speed:l.speed),
      std::sqrt(5.775*amount/(ph.mode==2?l.spin_acceleration:l.acceleration)),
      std::cbrt(60*amount/(ph.mode==2?l.spin_jerk:l.jerk))}),.05,p.duration_cap);
    Route path{start,end};if(ph.mode==0 && moving==1)path=seed.route;
    ph.spans=path.size()-1;
    for(int j=0;j<ph.spans;++j){
      const auto a=path[j],b=path[j+1];double len=distance(a,b);
      double d0=j?std::atan2(path[j+1].y-path[j-1].y,path[j+1].x-path[j-1].x):ph.h0[0]+(sign<0?pi:0);
      double d1=j+1<ph.spans?std::atan2(path[j+2].y-path[j].y,path[j+2].x-path[j].x):ph.h1[0]+(sign<0?pi:0);
      for(int c=0;c<6;++c){
        double t=c/5.;
        ph.coefficient[j][0][c]=a.x+(b.x-a.x)*t;ph.coefficient[j][1][c]=a.y+(b.y-a.y)*t;
        if(ph.mode==0 && c>0 && c<5){
          double h=c<3?d0:d1,w=(c<3?t:t-1)*len;
          ph.coefficient[j][0][c]=(c<3?a.x:b.x)+w*std::cos(h);
          ph.coefficient[j][1][c]=(c<3?a.y:b.y)+w*std::sin(h);
        }
        ph.coefficient[j][2][c]=psi+(endpsi-psi)*(j+t)/ph.spans;
      }
    }
    if(ph.mode==0 && p.bodies==3 && moving==1 && ph.spans>1 && p.count==1 &&
       (std::abs(delta(psi,endpsi))>1e-8) && seed.edit!="GENERAL_F_YAW"){
      auto yaw=route_orientation(path,psi,endpsi);
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
std::vector<HJSeparation> separators(const Request& r,const HJProblem& graph,const Run& run){
  const auto& grid=*r.grid;const auto org=grid.origin();const double res=grid.resolution();
  std::vector<HJSeparation> all;
  for(int k=0;k<graph.count&&!run.stop();++k){
    const auto& a=graph.phase[k];if(a.mode==2)continue;
    Phase phase;phase.mode=static_cast<Mode>(a.mode);phase.duration=1;
    std::copy_n(a.h0,3,phase.heading0.begin());std::copy_n(a.gear,3,phase.gear.begin());
    for(int j=0;j<a.spans;++j) {
      Piece q;std::copy_n(a.coefficient[j][0],6,q.x.begin());std::copy_n(a.coefficient[j][1],6,q.y.begin());
      std::copy_n(a.coefficient[j][2],6,q.yaw.begin());phase.pieces.push_back(q);
    }
    for(int j=0;j<a.spans&&!run.stop();++j)for(int z=0;z<=6;++z){
      double low=0,high=1,station=(j+z/6.)/a.spans;
      for(int iter=0;iter<32;++iter){double u=(low+high)/2;if(u*u*u*(10+u*(-15+6*u))<station)low=u;else high=u;}
      Sample sample;
      try{sample=evaluate_phase(r,phase,(low+high)/2);}catch(const std::domain_error&){continue;}
      for(int v=r.cargo?-1:0;v<graph.bodies;++v) {
        const auto& shape=v<0?*r.cargo:r.vehicles[v];const double a0=shape[2]/2,b0=shape[3]/2;
        Pose2d pose=sample.reference;
        if(v<0){pose.x+=std::cos(pose.yaw)*shape[0]-std::sin(pose.yaw)*shape[1];
          pose.y+=std::sin(pose.yaw)*shape[0]+std::cos(pose.yaw)*shape[1];}
        else pose=sample.bodies[v].pose;
        const double c=std::cos(pose.yaw),s=std::sin(pose.yaw),radius=std::hypot(a0,b0)+.5+res;
        const std::array<Point,8> axes{{{1,0},{-1,0},{0,1},{0,-1},{c,s},{-c,-s},{-s,c},{s,-c}}};
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
          if(largest<gaps[active]){tight[active]=support;gaps[active]=largest;}
        }
        for(int i=0;i<8;++i)if(gaps[i]<.22){
          const auto n=axes[i];
          all.push_back({k,j,z,v,shape[0],shape[1],a0,b0,n.x,n.y,tight[i]});
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
Plan expand(const Request& r,const HJProblem& p) {
  Plan plan;
  std::array<std::array<double,4>,3> prior{};
  Mode prev=Mode::A;Gears gear{1,1,1};
  for(int k=0;k<p.count;++k) {
    const auto& a=p.phase[k];
    Phase ph; ph.mode=static_cast<Mode>(a.mode);ph.duration=a.tau;ph.cruise_ramp_fraction=a.cruise_ramp_fraction;
    std::copy_n(a.h0,3,ph.heading0.begin());std::copy_n(a.h1,3,ph.heading1.begin());
    std::copy_n(a.gear,3,ph.gear.begin());
    for(int j=0;j<a.spans;++j){Piece piece;
      std::copy_n(a.coefficient[j][0],6,piece.x.begin());std::copy_n(a.coefficient[j][1],6,piece.y.begin());
      std::copy_n(a.coefficient[j][2],6,piece.yaw.begin());ph.pieces.push_back(piece);}
    if(ph.mode==Mode::S){double turn=0;for(int v=0;v<p.bodies;++v)turn=std::max(turn,std::abs(ph.heading1[v]-ph.heading0[v]));if(turn<1e-7)continue;}
    auto first=evaluate_phase(r,ph,0.),last=evaluate_phase(r,ph,ph.duration);
    double range=0; bool gear_change=false;
    for(int i=0;i<p.bodies;++i){
      auto next=wheels(ph.mode,first.bodies[i],r.limits);
      for(int j=0;j<4;++j)range=std::max(range,std::abs(next[j]-prior[i][j]));
      prior[i]=wheels(ph.mode,last.bodies[i],r.limits);
      gear_change|=gear[i]!=ph.gear[i];
    }
    if(prev!=ph.mode || gear_change || range>1e-7){
      Phase b;b.kind=Kind::Prepare;b.mode=ph.mode;b.gear=ph.gear;
      b.duration=r.limits.prepare_time+.0001+std::max(1.875*range/r.limits.steer_rate,std::sqrt(5.774*range/r.limits.steer_acceleration));
      Piece q;q.x.fill(first.reference.x);q.y.fill(first.reference.y);q.yaw.fill(first.reference.yaw);b.pieces.push_back(q);
      for(int v=0;v<p.bodies;++v)b.heading0[v]=b.heading1[v]=first.bodies[v].pose.yaw;
      plan.phases.push_back(b);
    }
    plan.phases.push_back(ph);prev=ph.mode;gear=ph.gear;
  }
  return plan;
}
Ranked score(const Plan& plan,const Request& r){
  Ranked s{0,0,duration(plan)};
  for(const auto& p:plan.phases) if(p.kind==Kind::Motion && p.mode!=Mode::A){
    s.presence=1;s.use+=.5;
    if(p.mode==Mode::O)for(const auto& c:p.pieces)s.use+=std::hypot(c.x[5]-c.x[0],c.y[5]-c.y[0])/r.limits.radius;
    if(p.mode==Mode::S)for(std::size_t k=0;k<r.vehicles.size();++k)s.use+=.25*std::abs(p.heading1[k]-p.heading0[k]);
  } return s;
}
bool locked_mount_tangents(const Request& r,const Boundary& b){
  if(r.vehicles.size()!=3 || !b.constrain_reference_yaw ||
     std::any_of(b.constrain_heading.begin(),b.constrain_heading.end(),[](bool v){return !v;}))return false;
  // A full-rank no-side-slip matrix admits no nonzero reference twist at
  // this boundary. This excludes the regular tangent chart only.
  std::array<std::array<double,3>,3> row{};
  for(std::size_t i=0;i<3;++i){
    auto f=r.vehicles[i];double c=std::cos(b.reference.yaw),s=std::sin(b.reference.yaw);
    double rx=c*f[0]-s*f[1],ry=s*f[0]+c*f[1];double h=b.heading[i];
    row[i]={-std::sin(h),std::cos(h),std::sin(h)*ry+std::cos(h)*rx};
  }
  auto a=row[0],c=row[1],d=row[2];
  double det=a[0]*(c[1]*d[2]-c[2]*d[1])+a[1]*(c[2]*d[0]-c[0]*d[2])+a[2]*(c[0]*d[1]-c[1]*d[0]);
  return std::abs(det)>1.e-6;
}
std::string name(const Sequence& seq){std::string s;for(auto m:seq)s+="AOS"[static_cast<int>(m)];return s;}
}

Result solve(const Request& r,Clock::time_point deadline,const Cancel& cancel,const SolveOptions& opts){
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
  auto add=[&](Sequence seq,int gear,Route route,std::string edit,int reverse_from=-1){
    if(!opts.enable_edits && !opts.initial_sequence.empty() && seq!=opts.initial_sequence)return;
    for(auto& s:seeds)if(seq==s.seq && gear==s.sign && reverse_from==s.reverse_from && route.size()==s.route.size() &&
       std::equal(route.begin(),route.end(),s.route.begin(),[](Point a,Point b){return distance(a,b)<1e-8;}))return;
    seeds.push_back({seq,gear,std::move(route),std::move(edit),reverse_from});
  };
  if(!opts.initial_sequence.empty())add(opts.initial_sequence,sign,direct,"SEED");
  else if(d>1e-7)add({Mode::A},sign,direct,"CONTINUOUS_A");
  else add({Mode::S},sign,direct,"SPIN");
  if(opts.enable_edits && !opts.initial_sequence.empty()){
    Sequence reduced;
    for(auto mode:opts.initial_sequence)if(reduced.empty()||reduced.back()!=mode)reduced.push_back(mode);
    if(reduced!=opts.initial_sequence)add(reduced,sign,direct,"MERGE_SAME_MODE");
    if(d>1e-7)add({Mode::A},sign,direct,"REPLACE_SUBSEQUENCE");
    for(std::size_t i=0;i<reduced.size();++i)if(reduced[i]!=Mode::A){
      auto q=reduced;q.erase(q.begin()+i);if(!q.empty())add(q,sign,direct,"REMOVE_BLOCK");}
  }
  bool locked_start=locked_mount_tangents(r,r.start),locked_goal=locked_mount_tangents(r,r.goal);
  if(d>1e-7 || (r.vehicles.size()==3 && std::abs(delta(r.start.reference.yaw,r.goal.reference.yaw))>1e-7)) {
    add({Mode::A},sign,direct,"CONTINUOUS_A");
    auto route=grid_seed(r,run);
    if(!route.empty()){
      double detour=0;
      for(auto p:route){double dx=direct[1].x-direct[0].x,dy=direct[1].y-direct[0].y;
        detour=std::max(detour,std::abs(dy*(p.x-direct[0].x)-dx*(p.y-direct[0].y))/std::max(1e-8,d));}
      add({Mode::A},sign,route,"CHANGE_ROUTE_SEED");
      if(opts.initial_sequence.empty() && detour>r.limits.radius*.35 && seeds.size()>1)
        std::rotate(seeds.begin(),std::prev(seeds.end()),seeds.end());
      if(r.vehicles.size()==3 && std::abs(delta(r.start.reference.yaw,r.goal.reference.yaw))>1e-8)
        seeds.push_back({{Mode::A},sign,route,"GENERAL_F_YAW"});
    }
    if(r.allow_reverse)add({Mode::A},-sign,direct,"CHANGE_GEAR");
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
  // Time laws are native initializations of the same spatial NLP. A near-wall
  // stationary point found with a cruise plateau need not be the one reached
  // with a smooth quintic clock. Both variants must pass the identical audit.
  if(opts.initial_sequence.empty() || opts.enable_edits){
    for(std::size_t i=0;i<seeds.size();++i){
      if(seeds[i].seq==Sequence{Mode::A} && seeds[i].route.size()>2 && seeds[i].edit!="GENERAL_F_YAW"){
        auto alternative=seeds[i];alternative.edit="CHANGE_TIME_LAW";alternative.quintic=true;
        seeds.insert(seeds.begin()+i+1,std::move(alternative));++i;
      }
    }
  }
  result.seed_seconds=elapsed();
  Ranked best{2,0,0};std::string why;
  for(const auto& seed:seeds) {
    if(run.stop())break;
    const auto improve_until=begin+std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(result.first_certified_s+.65));
    const bool improving_aux=result.plan && best.presence && seed.seq!=Sequence{Mode::A};
    if(improving_aux && Clock::now()>=improve_until)break;
    try{
      if(seed.seq.empty())continue;
      const auto tag=seed.edit+":"+name(seed.seq)+":"+std::to_string(seed.sign);
      result.attempts.push_back({tag,"REJECTED"});auto& metrics=result.attempts.back();
      const auto started=Clock::now();
      struct Finish{Attempt& a;Clock::time_point start;~Finish(){a.wall_s=std::chrono::duration<double>(Clock::now()-start).count();}} finish{metrics,started};
      if((seed.seq.front()==Mode::A && locked_start) || (seed.seq.back()==Mode::A && locked_goal)){
        why="ENDPOINT_MOUNT_TANGENT_RANK";result.trace.push_back(tag+":"+why);continue;
      }
      // Keep time for edits after a poor local initialization.
      double slice=seed.route.size()>2?5.6:(seed.seq.size()>2?2.15:2.8);
      Run trial{std::min(deadline,Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(slice))),cancel};
      if(improving_aux)trial.end=std::min(trial.end,improve_until);
      auto graph=problem(r,seed,opts,trial);
      auto active=separators(r,graph,run);
      graph.separations=active.data();graph.sep_count=active.size();
      char msg[512]{};
      bool solved=hj_optimize(&graph,msg,sizeof(msg));
      metrics.nlp_build_s=graph.nlp_build_s;metrics.nlp_solve_s=graph.nlp_solve_s;
      if(!solved){why=msg;metrics.status=why;result.trace.push_back(tag+":"+why);continue;}
      if(graph.primal_status[0])result.trace.push_back(tag+":UNCONVERGED_PRIMAL:"+graph.primal_status);
      auto plan=expand(r,graph);
      for(int retry=0;retry<12 && !run.stop();++retry){
        if(plan.phases.empty() || duration(plan)>r.task_duration_cap+1e-8){why="TASK_TIME_CONSTRAINT";break;}
        const auto audit_start=Clock::now();const auto audit=certify(r,plan,deadline,cancel);
        metrics.certify_s+=std::chrono::duration<double>(Clock::now()-audit_start).count();
        metrics.status=audit.reason;
        if(audit.executable) {
          const auto ranking=score(plan,r);if(!result.plan)result.first_certified_s=elapsed();
          result.trace.push_back(tag+":CERTIFIED");
          if(ranking<best){result.plan=plan;best=ranking;result.first_policy_compliant_s=elapsed();}
          break;
        }
        why=audit.reason;result.trace.push_back(tag+":"+why);
        if(audit.phase>=plan.phases.size() || plan.phases[audit.phase].kind!=Kind::Motion)break;
        // The certificate distinguishes spatial violations from time-law violations.
        // Retiming preserves geometry, boundary rest and the authoritative mount identities.
        if(why.find("SPEED")==std::string::npos && why.find("ACCELERATION")==std::string::npos &&
           why.find("JERK")==std::string::npos && why.find("RATE")==std::string::npos &&
           why.find("DYNAMIC")==std::string::npos)break;
        plan.phases[audit.phase].duration*=1.25;
      }
      if(result.plan && (best.presence==0 ||
         ((locked_start||locked_goal) && seed.seq.size()<=2 && seed.seq.front()!=Mode::O && seed.seq.back()!=Mode::O)))break;
      if(result.plan && elapsed()-result.first_certified_s>.75) break;
    }catch(const std::exception& e){why=e.what();result.trace.push_back(why);}
  }
  if(cancel&&cancel()){result.plan.reset();result.status="CANCELLED";result.reason=result.status;}
  else if(result.plan){
    result.status=run.stop() && best.presence?"FEASIBLE_POLICY_DEGRADED":"CERTIFIED_POLICY_COMPLIANT";
    result.reason=!best.presence?"PURE_A":locked_start||locked_goal?"ENDPOINT_CONFIGURATION":
      run.stop()?"A_SOLVE_TIMEOUT":"A_SEARCH_SPACE_EXHAUSTED";
  }
  else{result.status="NO_CERTIFIED_SOLUTION_WITHIN_BUDGET";result.reason=run.stop()?"A_SOLVE_TIMEOUT":why;}
  return result;
}
} // namespace fwis_hybrid_joint
