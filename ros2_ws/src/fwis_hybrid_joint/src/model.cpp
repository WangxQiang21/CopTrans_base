#include "fwis_hybrid_joint/model.hpp"
#include "clock.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fwis_hybrid_joint {
namespace {
struct Jet { double p,d,dd; };
double bernstein(const double *p, int n, double s) {
  double a[6]{};
  std::copy(p,p+n+1,a);
  for (int r=n;r>0;--r)
    for (int j=0;j<r;++j) a[j]=a[j]*(1-s)+a[j+1]*s;
  return a[0];
}
Jet coordinate(const Quintic& p, double s, double scale) {
  double d[5],dd[4];
  for (int k=0;k<5;++k) d[k]=5*(p[k+1]-p[k]);
  for (int k=0;k<4;++k) dd[k]=4*(d[k+1]-d[k]);
  return {bernstein(p.data(),5,s),scale*bernstein(d,4,s),scale*scale*bernstein(dd,3,s)};
}
}
double duration(const Plan& p) {
  double t=0; for (const auto& phase:p.phases) t+=phase.duration; return t;
}
Sample evaluate_phase(const Request& r,const Phase& p,double t) {
  if (!(p.duration>0) || !std::isfinite(t) || p.pieces.empty() ||
      (r.vehicles.size()!=1 && r.vehicles.size()!=3))
    throw std::invalid_argument("invalid hybrid phase or vehicle count");
  const double u=std::clamp(t/p.duration,0.,1.);
  const auto law=hybrid_clock::state(u,p.cruise_ramp_fraction);
  const double s=law[0],ds=law[1]/p.duration;
  const double n=static_cast<double>(p.pieces.size()), pos=std::clamp(s,0.,1.)*n;
  const auto i=std::min(p.pieces.size()-1,static_cast<std::size_t>(pos));
  const auto& piece=p.pieces[i];
  const auto x=coordinate(piece.x,pos-i,n), y=coordinate(piece.y,pos-i,n), psi=coordinate(piece.yaw,pos-i,n);
  Sample sample;
  sample.mode=p.mode; sample.kind=p.kind; sample.reference={x.p,y.p,psi.p};
  for (std::size_t k=0;k<r.vehicles.size();++k) {
    auto& b=sample.bodies[k];
    const auto& foot=r.vehicles[k];
    const double rx=std::cos(psi.p)*foot[0]-std::sin(psi.p)*foot[1];
    const double ry=std::sin(psi.p)*foot[0]+std::cos(psi.p)*foot[1];
    const double dx=x.d-psi.d*ry, dy=y.d+psi.d*rx;
    const double ax=x.dd-psi.dd*ry-psi.d*psi.d*rx;
    const double ay=y.dd+psi.dd*rx-psi.d*psi.d*ry;
    b.pose={x.p+rx,y.p+ry,p.heading0[k]};
    if (p.kind!=Kind::Motion) continue;
    if (p.mode==Mode::A) {
      const double norm=std::hypot(dx,dy);
      if (!(norm>1e-10)) throw std::domain_error("singular Ackermann mount tangent");
      const double dtheta=(dx*ay-dy*ax)/(norm*norm);
      b.pose.yaw=std::atan2(p.gear[k]*dy,p.gear[k]*dx);
      b.curvature=p.gear[k]*dtheta/norm;
      b.vx=p.gear[k]*norm*ds; b.wz=dtheta*ds;
    } else if (p.mode==Mode::O) {
      const double c=std::cos(b.pose.yaw), h=std::sin(b.pose.yaw);
      const double vx=c*dx+h*dy, vy=-h*dx+c*dy;
      b.vx=vx*ds; b.vy=vy*ds;
      b.oblique_angle=std::atan2(p.gear[k]*vy,p.gear[k]*vx);
    } else {
      const double turn=p.heading1[k]-p.heading0[k];
      b.pose.yaw+=turn*s; b.wz=turn*ds;
    }
  }
  if(r.vehicles.size()==1) sample.reference.yaw=sample.bodies[0].pose.yaw;
  return sample;
}
Sample evaluate(const Request& r,const Plan& p,double t) {
  if (!std::isfinite(t) || p.phases.empty())
    throw std::invalid_argument("invalid hybrid plan sample");
  double start=0.;
  for (const auto& phase:p.phases) {
    const double end=start+phase.duration;
    if(t<end) return evaluate_phase(r,phase,t-start);
    start=end;
  }
  // duration() sums the same boundaries. Subtracting several non-integral
  // phase durations from their sum can leave a moving sample just before rest.
  return evaluate_phase(r,p.phases.back(),p.phases.back().duration);
}
}
