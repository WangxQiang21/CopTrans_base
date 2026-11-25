// An independent closed-polygon oracle for the production row-indexed OBB checker.
// Uses segment crossings and winding, including contact; no SAT implementation is shared.
#include "fwis_planning_core/collision.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>

namespace pc = fwis_planning_core;
struct Point {long double x, y;};
using Quad = std::array<Point, 4>;
long double cross(Point a, Point b, Point c)
{
  return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
}
bool on_edge(Point a, Point b, Point p)
{
  return cross(a,b,p)==0 && p.x>=std::min(a.x,b.x) && p.x<=std::max(a.x,b.x)
      && p.y>=std::min(a.y,b.y) && p.y<=std::max(a.y,b.y);
}
bool segments_cross(Point a, Point b, Point c, Point d)
{
  const auto l = cross(a,b,c), r = cross(a,b,d), p = cross(c,d,a), q = cross(c,d,b);
  return on_edge(a,b,c) || on_edge(a,b,d) || on_edge(c,d,a) || on_edge(c,d,b)
      || (((l>0 && r<0)||(l<0 && r>0)) && ((p>0 && q<0)||(p<0 && q>0)));
}
bool contains(const Quad &a, Point b)
{
  for (int i=0;i<4;++i) if (cross(a[i],a[(i+1)%4],b)<0) return false;
  return true;
}
bool overlap(const Quad &a, const Quad &b)
{
  if (contains(a,b[0]) || contains(b,a[0])) return true;
  for (int i=0;i<4;++i) for (int j=0;j<4;++j) {
    if (segments_cross(a[i],a[(i+1)%4],b[j],b[(j+1)%4])) return true;
  }
  return false;
}
Quad corners(Point center, long double heading, long double length, long double width)
{
  Quad result{};
  const auto c=cosl(heading), s=sinl(heading);
  const int sx[]{-1,1,1,-1}, sy[]{-1,-1,1,1};
  for (int i=0;i<4;++i) {
    const auto x=sx[i]*length/2, y=sy[i]*width/2;
    result[i]={center.x+x*c-y*s, center.y+x*s+y*c};
  }
  return result;
}
Quad place(pc::Pose2d ref, const pc::Footprint &shape, double heading, double inflate)
{
  const auto c=cosl(ref.yaw), s=sinl(ref.yaw);
  const Point center{ref.x+c*shape[0]-s*shape[1],ref.y+s*shape[0]+c*shape[1]};
  return corners(center,heading,shape[2]+2*inflate,shape[3]+2*inflate);
}
int main(int argc, char **argv)
{
  constexpr unsigned seed=31271189, samples=24000;
  constexpr unsigned side=40;
  constexpr double res=0.125;
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> unit(0,1);
  auto range=[&](double lo,double hi){return lo+(hi-lo)*unit(rng);};
  std::ofstream raw;
  if (argc>1) {raw.open(argv[1]); if (!raw) return 2; raw<<"sample,category,oracle_collision,kernel_collision\n";}
  unsigned false_neg=0, false_pos=0, hits=0, clears=0;
  for (unsigned k=0; k<samples;++k) {
    const unsigned category=k%6;
    pc::Pose2d pose{range(0.8,4.2),range(0.8,4.2),range(-3.14,3.14)};
    std::vector<pc::Footprint> bodies{{0.,0.,range(.3,.75),range(.18,.45)}};
    std::vector<double> headings{range(-3.14,3.14)};
    std::optional<pc::Footprint> cargo;
    if (category==1 || category==2) {
      cargo=pc::Footprint{range(-.65,.65),range(-.45,.45),range(.75,1.8),range(.3,.9)};
      bodies.push_back({range(.2,1.1),range(-.3,.3),.5,.3});
      headings.push_back(range(-3.14,3.14));
    }
    if (category==3) pose.x=range(0,.5);
    const double inflate=category==4?range(0,.12):0.;
    std::vector<std::uint8_t> cells(side*side);
    std::vector<Quad> obstacles;
    auto obstacle=[&](int col,int row,std::uint8_t value){
      col=std::clamp(col,0,int(side)-1);
      row=std::clamp(row,0,int(side)-1);
      cells[row*side+col]=value;
      obstacles.push_back(corners({(col+.5L)*res,(row+.5L)*res},0,res,res));
    };
    if (category==5) {
      // Dyadic coordinates deliberately place an edge exactly on the occupied cell
      // and one ULP to either side of contact. The fixed cell is unknown.
      pose={2.,2.,0.};
      pose.x=(k/6)%3==0?2.:std::nextafter(2.,(k/6)%2?0.:4.);
      bodies[0]={0.,0.,.5,.25}; headings[0]=0.;
      obstacle(18,16,255);
    } else {
      for (int j=0;j<3;++j)
        obstacle(int((pose.x+range(-1,1))/res),int((pose.y+range(-1,1))/res),j==1?255:100);
    }
    std::vector<Quad> bs;
    for (std::size_t i=0;i<bodies.size();++i) bs.push_back(place(pose,bodies[i],headings[i],inflate));
    bool expected=false;
    for (std::size_t i=0;i<bs.size();++i) for (std::size_t j=i+1;j<bs.size();++j)
      expected=expected || overlap(bs[i],bs[j]);
    if (cargo) bs.push_back(place(pose,*cargo,pose.yaw,inflate));
    for (const auto &q:bs) {
      for (const auto &pt:q) expected=expected || pt.x<=0 || pt.y<=0 || pt.x>=side*res || pt.y>=side*res;
      for (const auto &obs:obstacles) expected=expected || overlap(q,obs);
    }
    const pc::GridSnapshot grid(side,side,res,{0.,0.},std::move(cells));
    const auto actual=pc::CollisionKernel(grid,bodies,cargo).check(pose,headings,inflate).reason!=pc::CollisionReason::clear;
    if (expected) ++hits; else ++clears;
    if (expected && !actual) ++false_neg;
    if (!expected && actual) ++false_pos;
    if (raw) raw<<k<<','<<category<<','<<expected<<','<<actual<<'\n';
  }
  std::cout<<"{\"seed\":"<<seed<<",\"random_collision_cases\":"<<samples
           <<",\"oracle_collision\":"<<hits<<",\"oracle_clear\":"<<clears
           <<",\"false_negatives\":"<<false_neg<<",\"false_positives\":"<<false_pos<<"}\n";
  return false_neg || false_pos || !hits || !clears;
}
