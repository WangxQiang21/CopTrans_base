#include "formation_lattice.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>

namespace fwis_hybrid_joint {
namespace {
using Point=fwis_planning_core::Point2d;
using Box=std::array<double,4>;
constexpr double pi=3.14159265358979323846;
double turn(double a,double b){return std::remainder(b-a,2*pi);}
double length(Point a,Point b){return std::hypot(b.x-a.x,b.y-a.y);}
struct Axis {double x,y,reach;};

// At a fixed formation yaw the forbidden reference positions are the
// Minkowski sum of each occupied rectangle and the reflected envelope.
// Both shapes are rectangles, so four separating axes suffice.
struct Envelope {
  Point offset;
  double hx,hy,radius;
  explicit Envelope(const Request& r){
    const auto& cargo=*r.cargo;
    double left=cargo[0]-cargo[2]/2,right=cargo[0]+cargo[2]/2;
    double bottom=cargo[1]-cargo[3]/2,top=cargo[1]+cargo[3]/2;
    for(const auto& car:r.vehicles){
      left=std::min(left,car[0]-car[2]/2);right=std::max(right,car[0]+car[2]/2);
      bottom=std::min(bottom,car[1]-car[3]/2);top=std::max(top,car[1]+car[3]/2);
    }
    hx=(right-left)/2;hy=(top-bottom)/2;offset={(right+left)/2,(top+bottom)/2};
    radius=std::hypot(std::max(-left,right),std::max(-bottom,top));
  }
};

struct Placement {
  Point offset;
  std::array<Axis,4> axes;
  Placement(const Envelope& e,double yaw,double pad,Point travel={}){
    const double c=std::cos(yaw),s=std::sin(yaw);
    offset={c*e.offset.x-s*e.offset.y+travel.x/2,s*e.offset.x+c*e.offset.y+travel.y/2};
    axes={{{1,0,std::abs(c)*e.hx+std::abs(s)*e.hy+pad},
           {0,1,std::abs(s)*e.hx+std::abs(c)*e.hy+pad},
           {c,s,e.hx+pad},{-s,c,e.hy+pad}}};
    for(auto& a:axes)a.reach+=.5*std::abs(a.x*travel.x+a.y*travel.y);
  }
  // Intersect the obstacle's projection on each axis with a parametric
  // line. The retained interval is the part of the line inside the obstacle.
  bool blocked_interval(const Box& b,Point from,Point direction,double& lo,double& hi)const{
    const double x=from.x+offset.x-b[0],y=from.y+offset.y-b[1];
    for(const auto& a:axes){
      const double v=direction.x*a.x+direction.y*a.y,p=x*a.x+y*a.y;
      const double reach=a.reach+b[2]*std::abs(a.x)+b[3]*std::abs(a.y);
      if(std::abs(v)<1.e-12){if(std::abs(p)>reach)return false;continue;}
      const double centre=-p/v,half=reach/std::abs(v);
      lo=std::max(lo,centre-half);hi=std::min(hi,centre+half);
      if(lo>hi)return false;
    }
    return true;
  }
};

class Lattice {
  const Request& r_;
  const std::vector<Box>& obstacles_;
  const std::function<bool()>& stop_;
  Envelope envelope_;
  double step_,left_,bottom_,right_,top_;
  int width_,height_,plane_;
  Point origin_;
  std::vector<double> angles_;
  using Mask=std::vector<std::uint8_t>;
  std::vector<Mask> layers_,turns_,near_wall_;
  std::vector<std::array<Mask,4>> translations_;
  Point point(int xy)const{return {origin_.x+step_*(xy%width_),origin_.y+step_*(xy/width_)};}
  bool inside(const Placement& p,Point q)const{
    const double x=q.x+p.offset.x,y=q.y+p.offset.y;
    return x-p.axes[0].reach>left_ && x+p.axes[0].reach<right_ &&
           y-p.axes[1].reach>bottom_ && y+p.axes[1].reach<top_;
  }
  bool line(Point from,Point to,double yaw,double margin=.012)const{
    const Placement placement(envelope_,yaw,margin);
    if(!inside(placement,from)||!inside(placement,to))return false;
    for(const auto& b:obstacles_){
      double lo=0,hi=1;
      if(placement.blocked_interval(b,from,{to.x-from.x,to.y-from.y},lo,hi))return false;
    }
    return true;
  }
  void paint(Mask& blocked,double yaw,double padding,Point travel={})const{
    const Placement p(envelope_,yaw,padding,travel);
    for(int y=0;y<height_;++y){
      const auto row=point(y*width_);auto first=blocked.begin()+y*width_;
      const double cy=row.y+p.offset.y;
      if(cy-p.axes[1].reach<=bottom_ || cy+p.axes[1].reach>=top_){
        std::fill_n(first,width_,1);continue;
      }
      const double cx=row.x+p.offset.x;
      int a=std::clamp(int(std::floor((left_+p.axes[0].reach-cx)/step_))+1,0,width_);
      int b=std::clamp(int(std::ceil((right_-p.axes[0].reach-cx)/step_)),0,width_);
      std::fill(first,first+a,1);std::fill(first+b,first+width_,1);
    }
    for(const auto& box:obstacles_){
      const double cy=box[1]-p.offset.y,reach=box[3]+p.axes[1].reach;
      const int y0=std::max(0,int(std::ceil((cy-reach-origin_.y)/step_-1e-10)));
      const int y1=std::min(height_-1,int(std::floor((cy+reach-origin_.y)/step_+1e-10)));
      for(int y=y0;y<=y1;++y){
        double lo=0,hi=width_-1;
        if(!p.blocked_interval(box,point(y*width_),{step_,0},lo,hi))continue;
        const int x0=std::max(0,int(std::ceil(lo-1e-10)));
        const int x1=std::min(width_-1,int(std::floor(hi+1e-10)));
        if(x0<=x1)std::fill_n(blocked.begin()+y*width_+x0,x1-x0+1,1);
      }
    }
  }
  int heading(double yaw)const{
    return int(std::min_element(angles_.begin(),angles_.end(),[&](double a,double b){
      return std::abs(turn(yaw,a))<std::abs(turn(yaw,b));})-angles_.begin());
  }
  std::vector<int> connectors(const Pose2d& q,int h)const{
    std::vector<int> nodes;
    const int x0=std::lround((q.x-origin_.x)/step_),y0=std::lround((q.y-origin_.y)/step_);
    for(int y=std::max(0,y0-2);y<=std::min(height_-1,y0+2);++y)
      for(int x=std::max(0,x0-2);x<=std::min(width_-1,x0+2);++x){
        int xy=y*width_+x;
        if(!layers_[h][xy] && line({q.x,q.y},point(xy),q.yaw))nodes.push_back(h*plane_+xy);
      }
    return nodes;
  }
public:
  Lattice(const Request& r,const std::vector<Box>& obstacles,int headings,double spacing,
          const std::function<bool()>& stop,bool anchor_at_goal=false):
    r_(r),obstacles_(obstacles),stop_(stop),envelope_(r),step_(spacing){
    const auto& map=*r.grid;
    left_=map.origin().x;bottom_=map.origin().y;
    right_=left_+map.width()*map.resolution();top_=bottom_+map.height()*map.resolution();
    width_=int(std::ceil((right_-left_)/step_));height_=int(std::ceil((top_-bottom_)/step_));
    origin_={left_+.5*step_,bottom_+.5*step_};
    // Offset the alternate angular chart by a fraction of one bin to
    // sample passages rejected by the start-aligned discretization.
    const double origin_yaw=anchor_at_goal?r.goal.reference.yaw+.10*2*pi/headings:r.start.reference.yaw;
    const double other_yaw=anchor_at_goal?r.start.reference.yaw:r.goal.reference.yaw;
    for(int i=0;i<headings;++i)angles_.push_back(std::remainder(origin_yaw+2*pi*i/headings,2*pi));
    if(std::abs(turn(angles_[heading(other_yaw)],other_yaw))>1e-8)
      angles_.push_back(std::remainder(other_yaw,2*pi));
    if(anchor_at_goal && std::abs(turn(angles_[heading(r.goal.reference.yaw)],r.goal.reference.yaw))>1e-8)
      angles_.push_back(std::remainder(r.goal.reference.yaw,2*pi));
    std::sort(angles_.begin(),angles_.end());
    plane_=0;
    if(width_>0 && height_>0 && std::uint64_t(width_)*height_*angles_.size()<=8500000)
      plane_=width_*height_;
  }
  std::vector<Pose2d> route(){
    if(!plane_ || stop_())return {};
    const auto start=r_.start.reference,goal=r_.goal.reference;
    const int count=angles_.size(),hs=heading(start.yaw),hg=heading(goal.yaw);
    for(int h=0;h<count;++h){
      if(stop_())return {};
      layers_.emplace_back(plane_,0);
      paint(layers_.back(),angles_[h],.012);
      near_wall_.emplace_back(plane_,0);paint(near_wall_.back(),angles_[h],.18);
      // Rasterize each undirected edge family once. Only the extent along
      // that edge grows, so a parallel wall does not narrow a whole aisle.
      translations_.emplace_back();
      const std::array<Point,4> steps{{{step_,0},{0,step_},{step_,step_},{-step_,step_}}};
      for(std::size_t k=0;k<steps.size();++k){
        auto& edges=translations_.back()[k];edges.resize(plane_);
        paint(edges,angles_[h],.012,steps[k]);
      }
      turns_.emplace_back(plane_,0);
      const double d=turn(angles_[h],angles_[(h+1)%count]);
      const int n=std::max(1,int(std::ceil(std::abs(d)/(pi/24))));
      const double pad=.012+2*envelope_.radius*std::sin(std::abs(d)/(4*n));
      for(int i=0;i<n;++i)paint(turns_.back(),angles_[h]+d*(i+.5)/n,pad);
    }
    const auto starts=connectors(start,hs),goals=connectors(goal,hg);
    if(starts.empty()||goals.empty())return {};
    // Translating and turning reach the same pose with different wheel
    // setups. Keep both labels so consecutive angular steps pay for one
    // stopped mode change, even on the five-degree refinement.
    const int poses=plane_*count;
    std::vector<double> cost(2*poses,std::numeric_limits<double>::infinity());
    std::vector<int> parent(cost.size(),-1);
    std::vector<bool> terminal(plane_,false);
    for(int id:goals)terminal[id%plane_]=true;
    using Node=std::pair<double,int>;
    std::priority_queue<Node,std::vector<Node>,std::greater<Node>> queue;
    const double rotation_cost=3.*envelope_.radius;
    auto heuristic=[&](int h,int xy){
      return length(point(xy),{goal.x,goal.y})+rotation_cost*std::abs(turn(angles_[h],goal.yaw));};
    auto add=[&](int id,int source,double g){
      if(g+1e-8>=cost[id])return;
      parent[id]=source;cost[id]=g;queue.emplace(g+1.18*heuristic((id%poses)/plane_,id%plane_),id);
    };
    for(int id:starts)add(id,-1,length({start.x,start.y},point(id%plane_)));
    int end=-1;unsigned ticks=0;
    while(!queue.empty()){
      if((++ticks%256)==0 && stop_())return {};
      auto [score,id]=queue.top();queue.pop();
      int h=(id%poses)/plane_,xy=id%plane_,x=xy%width_,y=xy/width_;
      const bool turning=id>=poses;
      if(score>cost[id]+1.18*heuristic(h,xy)+1e-7)continue;
      if(h==hg && terminal[xy]){end=id;break;}
      for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
        if((!dx&&!dy)||x+dx<0||x+dx>=width_||y+dy<0||y+dy>=height_)continue;
        const int next=xy+dy*width_+dx;
        const int family=dy==0?0:dx==0?1:dx==dy?2:3;
        const int edge=(dy>0 || (dy==0 && dx>0))?xy:next;
        if(!translations_[h][family][edge] && !layers_[h][next])
          add(h*plane_+next,id,cost[id]+step_*(dx&&dy?std::sqrt(2.):1.)*(near_wall_[h][next]?1.40:1.));
      }
      for(int dh:{-1,1}){
        const int next=(h+count+dh)%count,edge=dh>0?h:next;
        if(!turns_[edge][xy] && !layers_[next][xy])
          add(poses+next*plane_+xy,id,cost[id]+(turning?0.:rotation_cost)+rotation_cost*std::abs(turn(angles_[h],angles_[next])));
      }
    }
    if(end<0)return {};
    std::vector<Pose2d> raw{goal};
    for(int id=end;id>=0;id=parent[id]){
      const auto p=point(id%plane_);
      raw.push_back({p.x,p.y,angles_[(id%poses)/plane_]});
    }
    raw.push_back(start);std::reverse(raw.begin(),raw.end());
    // Merge visible translations and same-direction turns, respecting the
    // exact (unsnapped) endpoints. Preserve winding when crossing +/-pi.
    std::vector<Pose2d> path{start};
    for(std::size_t i=1;i<raw.size();){
      auto a=path.back(),b=raw[i];
      const double rotation=turn(raw[i-1].yaw,b.yaw);
      if(std::abs(rotation)>1e-8){
        b.yaw=a.yaw+rotation;
        while(i+1<raw.size()){
          const double next=turn(raw[i].yaw,raw[i+1].yaw);
          if(rotation*next<=1e-12)break;
          b.yaw+=next;++i;
        }
      }else{
        auto last=i;
        while(last+1<raw.size() && std::abs(turn(a.yaw,raw[last+1].yaw))<1e-8)++last;
        const auto limit=last;
        while(last>i && !line({a.x,a.y},{raw[last].x,raw[last].y},a.yaw,.10))--last;
        if(last==i){
          last=limit;
          while(last>i && !line({a.x,a.y},{raw[last].x,raw[last].y},a.yaw))--last;
        }
        i=last;b=raw[i];b.yaw=a.yaw;
      }
      if(std::hypot(a.x-b.x,a.y-b.y)>1e-8 || std::abs(a.yaw-b.yaw)>1e-8)path.push_back(b);
      ++i;
    }
    return path;
  }
};
} // namespace
std::vector<Pose2d> formation_channel(const Request& r,const std::vector<Box>& obstacles,
  int headings,double spacing,const std::function<bool()>& stop){
  return Lattice(r,obstacles,headings,spacing,stop).route();
}
std::vector<Pose2d> formation_channel_on_goal_grid(const Request& r,const std::vector<Box>& obstacles,
  int headings,double spacing,const std::function<bool()>& stop){
  return Lattice(r,obstacles,headings,spacing,stop,true).route();
}
} // namespace fwis_hybrid_joint
