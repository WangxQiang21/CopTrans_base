#include "fwis_hybrid_joint/model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fwis_hybrid_joint {
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double inf = std::numeric_limits<double>::infinity();

// Each elementary operation encloses its result outwards. atan2 is only used
// at point samples; the swept enclosure uses a proven angular travel bound.
struct I {
  double l{}, h{};
  I() = default;
  I(double x) : l(x), h(x) {}
  I(double a, double b) : l(a), h(b) {}
};
double down(double x) {return std::nextafter(x,-inf);}
double up(double x) {return std::nextafter(x,inf);}
I operator+(I a,I b) {return {down(a.l+b.l),up(a.h+b.h)};}
I operator-(I a,I b) {return {down(a.l-b.h),up(a.h-b.l)};}
I operator*(I a,I b) {
  const auto vals=std::array<double,4>{a.l*b.l,a.h*b.h,a.l*b.h,a.h*b.l};
  const auto extremes=std::minmax_element(vals.begin(),vals.end());
  return {down(*extremes.first),up(*extremes.second)};
}
I operator/(I a,I b) {
  if (b.l<=0 && b.h>=0) throw std::domain_error("SINGULAR_TANGENT_OR_WHEEL");
  return a*I(down(1/b.h),up(1/b.l));
}
double magnitude(I a) {return std::max(std::abs(a.l),std::abs(a.h));}
I square(I a) {
  return {a.l*a.h<=0?0:down(std::min(a.l*a.l,a.h*a.h)),up(magnitude(a)*magnitude(a))};
}
I root(I a) {
  if (a.l<=0) throw std::domain_error("SINGULAR_MOUNT_TANGENT");
  return {down(std::sqrt(a.l)),up(std::sqrt(a.h))};
}
I sin_bounds(I x) {
  if (x.h-x.l>=2*pi || magnitude(x)>1.e6) return {-1,1};
  // Bound platform libm and reduction at extrema. The input width is also
  // enlarged in proportion to |x| before testing for interior critical points.
  const double err=16*std::numeric_limits<double>::epsilon()*(1+magnitude(x));
  const double l=x.l-err, h=x.h+err;
  double a=std::min(std::sin(l),std::sin(h)), b=std::max(std::sin(l),std::sin(h));
  if (std::ceil((l-pi/2)/(2*pi))<=std::floor((h-pi/2)/(2*pi))) b=1;
  if (std::ceil((l+pi/2)/(2*pi))<=std::floor((h+pi/2)/(2*pi))) a=-1;
  return {std::max(-1.,down(a-err)),std::min(1.,up(b+err))};
}
// Taylor coefficients in local piece coordinate. The largest required order
// is four: two derivatives of steering angle require up to p''''.
struct Jet { std::array<I,5> c{}; };
Jet operator+(const Jet &a,const Jet &b) {
  Jet r; for (int k=0;k<=4;++k) r.c[k]=a.c[k]+b.c[k]; return r;
}
Jet operator-(const Jet &a,const Jet &b) {
  Jet r; for (int k=0;k<=4;++k) r.c[k]=a.c[k]-b.c[k]; return r;
}
Jet scale(const Jet &a,I b) {
  Jet r; for (int k=0;k<=4;++k) r.c[k]=a.c[k]*b; return r;
}
template<int Order=4>
std::pair<Jet,Jet> sin_cos(const Jet &p) {
  Jet s,c; s.c[0]=sin_bounds(p.c[0]); c.c[0]=sin_bounds(p.c[0]+I(pi/2));
  for (int k=1;k<=Order;++k) {
    for (int j=1;j<=k;++j) {
      s.c[k]=s.c[k]+I(j)*p.c[j]*c.c[k-j];
      c.c[k]=c.c[k]-I(j)*p.c[j]*s.c[k-j];
    }
    s.c[k]=s.c[k]/I(k); c.c[k]=c.c[k]/I(k);
  }
  return {s,c};
}
using Poly=std::array<I,6>;
std::pair<Poly,Poly> split(Poly p,int n,I v) {
  Poly a{},b{}; a[0]=p[0]; b[n]=p[n];
  for (int i=1;i<=n;++i) {
    for (int j=0;j<=n-i;++j) p[j]=p[j]*(I(1)-v)+p[j+1]*v;
    a[i]=p[0]; b[n-i]=p[n-i];
  }
  return {a,b};
}
I hull(const Poly &p,int degree,double a,double b) {
  // Subdivide with de Casteljau. The restricted control hull contains the
  // polynomial everywhere on [a,b], including common endpoints.
  Poly sub=p;
  if (b<1) sub=split(sub,degree,I(b)).first;
  if (a>0 && b>0) sub=split(sub,degree,I(a)/I(b)).second;
  I bounds=sub[0];
  for (int i=1;i<=degree;++i) {
    bounds.l=std::min(bounds.l,sub[i].l); bounds.h=std::max(bounds.h,sub[i].h);
  }
  return bounds;
}
template<int Order=4>
Jet polynomial(const Quintic &coeff,double a,double b) {
  Poly p; for (int i=0;i!=6;++i) p[i]=I(coeff[i]);
  Jet result;
  for (int k=0;k<=Order;++k) {
    result.c[k]=hull(p,5-k,a,b);
    if (k==Order) break;
    for (int i=0;i<5-k;++i) p[i]=I(5-k)*(p[i+1]-p[i])/I(k+1);
  }
  return result;
}
struct ClockBounds {I z1,z2,z3;};
using GridIK=std::array<double,2>;
constexpr Quintic progress{0,0,0,1,1,1};
I base_progress(double u,double ramp) {
  if(ramp>0) {
    const bool tail=u>0.5;
    const I x=tail?I(1)-I(u):I(u);
    I s;
    if ((tail?1-u:u)>=ramp) s=(x-I(.5)*I(ramp))/(I(1)-I(ramp));
    else {
      const I r=x/I(ramp);
      s=I(ramp)*r*r*r*(I(1)-I(.5)*r)/(I(1)-I(ramp));
    }
    return tail?I(1)-s:s;
  }
  // Symmetry keeps the endpoint near 1 well conditioned.
  const bool tail=u>0.5;
  I x=tail?I(1)-I(u):I(u);
  const auto s=x*x*x*(I(10)+x*(I(-15)+I(6)*x));
  return tail?I(1)-s:s;
}
I progress_at(double u,double ramp) {
  const auto base=base_progress(u,ramp);
  if(ramp>0 && ramp<.125)return I(.875)*base+I(.125)*base_progress(u,.125);
  return base;
}
GridIK inverse_clock(double station,double ramp) {
  if (station<=0 || station>=1) return {station,station};
  // Two one-sided searches preserve the bracket even if the function
  // evaluation contains the target due to floating-point rounding. Share
  // the enclosure whenever the bisections evaluate the same midpoint.
  double lower_lo=0,lower_hi=1,upper_lo=0,upper_hi=1;
  for (int k=0;k<45;++k) {
    const auto lower_m=(lower_lo+lower_hi)/2,upper_m=(upper_lo+upper_hi)/2;
    const auto lower_progress=progress_at(lower_m,ramp);
    const auto upper_progress=lower_m==upper_m?lower_progress:progress_at(upper_m,ramp);
    if (lower_progress.h<station) lower_lo=lower_m; else lower_hi=lower_m;
    if (upper_progress.l>station) upper_hi=upper_m; else upper_lo=upper_m;
  }
  return {lower_lo,upper_hi};
}
I envelope(I a,I b) {return {std::min(a.l,b.l),std::max(a.h,b.h)};}
ClockBounds base_derivatives(double from,double to,double ramp) {
  if(ramp==0) {
    const auto law=polynomial<3>(progress,from,to);
    return {law.c[1],I(2)*law.c[2],I(6)*law.c[3]};
  }
  const I width(ramp), peak=I(1)/(I(1)-width);
  ClockBounds out;
  bool set=false;
  auto add=[&](I v,I a,I j) {
    if(!set) {out={v,a,j};set=true;return;}
    out={envelope(out.z1,v),envelope(out.z2,a),envelope(out.z3,j)};
  };
  // Enclose both one-sided jerks at a junction. There is no impulse:
  // acceleration and velocity are continuous with the constant-speed span.
  for(bool tail:{false,true}) {
    const double lo=tail?1-ramp:0.,hi=tail?1.:ramp;
    if(to<lo || from>hi) continue;
    I x(std::max(lo,from),std::min(hi,to));
    const I r=(tail?I(1)-x:x)/width;
    const I v=square(r)*(I(3)-I(2)*r)*peak;
    const I acceleration=I(6)*r*(I(1)-r)*peak/width;
    add(v,tail?I(0)-acceleration:acceleration,I(6)*(I(1)-I(2)*r)*peak/square(width));
  }
  if(to>=ramp && from<=1-ramp) add(peak,I(0),I(0));
  return out;
}
ClockBounds time_derivatives(double from,double to,double ramp) {
  const auto narrow=base_derivatives(from,to,ramp);
  if(ramp<=0 || ramp>=.125)return narrow;
  const auto wide=base_derivatives(from,to,.125);
  return {I(.875)*narrow.z1+I(.125)*wide.z1,I(.875)*narrow.z2+I(.125)*wide.z2,
          I(.875)*narrow.z3+I(.125)*wide.z3};
}
double separation(double a,double b){return std::abs(std::remainder(a-b,2*pi));}
bool same(double a,double b,double tol){return std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=tol;}
bool positive(double x) {return std::isfinite(x)&&x>0;}
std::array<double,4> wheels(const Body &b,Mode mode,const Limits &l) {
  std::array<double,4> result; int i=0;
  for (double x:{l.wheelbase/2,-l.wheelbase/2}) for(double y:{l.track/2,-l.track/2}) {
    if (mode==Mode::A) result[i++]=std::atan2(x*b.curvature,1-y*b.curvature);
    else if(mode==Mode::O) result[i++]=b.oblique_angle;
    else result[i++]=std::remainder(std::atan2(x,-y),pi);
  }
  return result;
}
double wheel_delta(const Request &r,const Sample &a,const Sample &b) {
  double d=0;
  for (std::size_t i=0;i<r.vehicles.size();++i) {
    const auto w0=wheels(a.bodies[i],a.mode,r.limits),w1=wheels(b.bodies[i],b.mode,r.limits);
    for (int k=0;k<4;++k) d=std::max(d,std::abs(w1[k]-w0[k]));
  }
  return d;
}
struct CheckFailure {std::string reason;std::size_t vehicle;};
void below(I value,double limit,const char *name,std::size_t vehicle) {
  if (!std::isfinite(value.l) || !std::isfinite(value.h) || magnitude(value)>limit)
    throw CheckFailure{name,vehicle};
}
I signed_arc_rate(I tx,I ty,I ax,I ay,I norm2) {return (tx*ay-ty*ax)/norm2;}
// Midpoint location/orientation errors are enclosed independently from the
// double evaluator. These bounds apply to the exact Bernstein model, including
// its reference rotation and atan2 at vehicle tangents. Inflating footprints
// covers the rounding error and also the motion on either half interval.
double reconstruction_padding(const Request &r,const Phase &ph,std::size_t j,double t,
    const Sample &sample) {
  const auto n=double(ph.pieces.size()); const auto s=progress_at(t,ph.cruise_ramp_fraction);
  const auto local=s*I(n)-I(double(j));
  const double a=std::clamp(local.l,0.,1.), b=std::clamp(local.h,0.,1.);
  const auto& c=ph.pieces[j];
  const auto x=polynomial<1>(c.x,a,b),y=polynomial<1>(c.y,a,b),psi=polynomial<1>(c.yaw,a,b);
  const double displacement=up(std::hypot(magnitude(x.c[0]-I(sample.reference.x)),
      magnitude(y.c[0]-I(sample.reference.y))));
  const bool formation=r.vehicles.size()==3;
  const double refangle=formation?magnitude(psi.c[0]-I(sample.reference.yaw)):0.;
  const auto sc=sin_cos<1>(psi);
  auto geometry_error=[&](const Footprint &f,double heading_error) {
    return up(up(std::hypot(f[0],f[1]))*refangle+
              up(.5*std::hypot(f[2],f[3]))*heading_error);
  };
  double angular=r.cargo?geometry_error(*r.cargo,refangle):0.;
  for(std::size_t i=0;i<r.vehicles.size();++i) {
    const auto &f=r.vehicles[i]; const auto heading=sample.bodies[i].pose.yaw;
    double err=0;
    if(ph.kind==Kind::Motion && ph.mode==Mode::A) {
      const auto dx=formation?x.c[1]+I(f[0])*sc.second.c[1]-I(f[1])*sc.first.c[1]:x.c[1];
      const auto dy=formation?y.c[1]+I(f[0])*sc.first.c[1]+I(f[1])*sc.second.c[1]:y.c[1];
      const auto cs=sin_bounds(I(heading)+I(pi/2)), sn=sin_bounds(I(heading));
      const auto dot=I(ph.gear[i])*(cs*dx+sn*dy),cross=I(ph.gear[i])*(cs*dy-sn*dx);
      if (!(dot.l>0)) throw std::domain_error("SINGULAR_MIDPOINT_HEADING");
      err=up(std::atan2(magnitude(cross),dot.l)+4*std::numeric_limits<double>::epsilon());
    } else if(ph.kind==Kind::Motion && ph.mode==Mode::S) {
      err=magnitude(I(ph.heading0[i])+(I(ph.heading1[i])-I(ph.heading0[i]))*s-I(heading));
    } else err=magnitude(I(ph.heading0[i])-I(heading));
    angular=std::max(angular,geometry_error(f,err));
  }
  // SAT transforms, projections and cell-centre construction in the reused
  // double precision collision kernel: sum an absolute per-operation error
  // bound at the largest coordinate and body scale used by that kernel.
  const auto origin=r.grid->origin();
  double scale=1+std::abs(origin.x)+std::abs(origin.y)+r.grid->resolution()*(double(r.grid->height())+double(r.grid->width()))+
    std::abs(sample.reference.x)+std::abs(sample.reference.y);
  for(const auto &f:r.vehicles) for(double v:f) scale+=std::abs(v);
  if(r.cargo) for(double v:*r.cargo) scale+=std::abs(v);
  return up(displacement+angular+128*std::numeric_limits<double>::epsilon()*scale);
}
struct IntervalAudit {
  const Request &r;
  const Phase &ph;
  const fwis_planning_core::CollisionKernel &map;
  Clock::time_point deadline;
  const Cancel &cancel;
  std::size_t index;
  Audit result{true,"CERTIFIED",0,0,0,0};
  bool stopped() const {return Clock::now()>=deadline || (cancel&&cancel());}
  void fail(const std::string &why,std::size_t body,double a,double b) {
    result={false,why,index,body,a,b};
  }
  bool visit(std::size_t piece,double a,double b,double ta,double tb,int depth);
};
bool IntervalAudit::visit(std::size_t piece,double a,double b,double ta,double tb,int depth) {
  const double n=static_cast<double>(ph.pieces.size());
  auto normalized=[&](double local) {return (piece+local)/n;};
  if(stopped()) {fail("CERTIFICATION_BUDGET",0,normalized(a),normalized(b));return false;}
  const auto &p=ph.pieces[piece];
  const auto tm=(ta+tb)/2;
  const auto point=evaluate_phase(r,ph,tm*ph.duration);
  double head[3]{};
  for (std::size_t i=0;i<r.vehicles.size();++i) head[i]=point.bodies[i].pose.yaw;
  const auto exact=map.check(point.reference,head,r.vehicles.size());
  if(exact.reason!=fwis_planning_core::CollisionReason::clear) {
    fail(fwis_planning_core::reason_name(exact.reason),exact.first,normalized(a),normalized(b));
    return false;
  }
  CheckFailure uncertainty{"CERTIFICATION_INCONCLUSIVE",0};
  try {
    const auto x=polynomial(p.x,a,b),y=polynomial(p.y,a,b),psi=polynomial(p.yaw,a,b);
    const auto law=time_derivatives(ta,tb,ph.cruise_ramp_fraction);
    const I T(ph.duration);
    const ClockBounds z{law.z1*I(n)/T,law.z2*I(n)/(T*T),law.z3*I(n)/(T*T*T)};
    const auto z1sq=square(z.z1);
    const auto z1cube=z1sq*z.z1;
    const auto speedF=up(std::hypot(magnitude(x.c[1]*z.z1),magnitude(y.c[1]*z.z1)));
    double corner_rate=0;

    const auto &l=r.limits;
    const bool formation=r.vehicles.size()==3;
    const auto sc=sin_cos(psi);
    if (r.cargo) {
      const auto &f=*r.cargo;
      const auto reach=up(std::hypot(f[0],f[1])+.5*std::hypot(f[2],f[3]));
      corner_rate=up(speedF+reach*magnitude(psi.c[1]*z.z1));
      if(l.cargo_center_speed>0 && ph.kind==Kind::Motion && ph.mode!=Mode::S){
        const auto cx=x+scale(sc.second,I(f[0]))-scale(sc.first,I(f[1]));
        const auto cy=y+scale(sc.first,I(f[0]))+scale(sc.second,I(f[1]));
        below(root(square(cx.c[1])+square(cy.c[1]))*z.z1,l.cargo_center_speed,"CARGO_CENTER_SPEED",0);
      }
    }
    for (std::size_t i=0;i<r.vehicles.size();++i) {
      const auto& f=r.vehicles[i];
      const auto mx=formation?x+scale(sc.second,I(f[0]))-scale(sc.first,I(f[1])):x;
      const auto my=formation?y+scale(sc.first,I(f[0]))+scale(sc.second,I(f[1])):y;
      I speed(0),omega(0);
      if (ph.kind==Kind::Motion && ph.mode==Mode::S) {
        const auto angle=I(ph.heading1[i])-I(ph.heading0[i]);
        omega=angle*z.z1/I(n);
        below(omega,l.spin_speed,"SPIN_SPEED",i);
        below(angle*z.z2/I(n),l.spin_acceleration,"SPIN_ACCELERATION",i);
        below(angle*z.z3/I(n),l.spin_jerk,"SPIN_JERK",i);
        below(omega*I(up(.5*std::hypot(l.wheelbase,l.track))),l.wheel_speed,"WHEEL_SPEED",i);
        const double spin_steer=std::atan2(l.wheelbase,l.track);
        below(I(up(spin_steer)),l.steer_angle,"SPIN_WHEEL_ANGLE",i);
      } else if(ph.kind==Kind::Motion) {
        const auto tx=mx.c[1],ty=my.c[1],ax=I(2)*mx.c[2],ay=I(2)*my.c[2];
        const auto jx=I(6)*mx.c[3],jy=I(6)*my.c[3],sx=I(24)*mx.c[4],sy=I(24)*my.c[4];
        const auto q=square(tx)+square(ty);
        const auto norm=root(q);
        if (norm.l*n<=1e-10) throw CheckFailure{"SINGULAR_MOUNT_TANGENT",i};
        const auto dnorm=(tx*ax+ty*ay)/norm;
        const auto d2norm=(square(ax)+square(ay)+tx*jx+ty*jy-square(dnorm))/norm;
        speed=norm*z.z1;
        const auto acceleration=dnorm*z1sq+norm*z.z2;
        const auto jerk=d2norm*z1cube+I(3)*dnorm*z.z1*z.z2+norm*z.z3;
        below(speed,l.speed,"BODY_SPEED",i);
        below(acceleration,l.acceleration,"PATH_ACCELERATION",i);
        below(jerk,l.jerk,"PATH_JERK",i);
        const auto h1=signed_arc_rate(tx,ty,ax,ay,q);
        const auto h2=(tx*jy-ty*jx-I(2)*norm*dnorm*h1)/q;
        const auto h3=(ax*jy-ay*jx+tx*sy-ty*sx
            -I(2)*(square(dnorm)+norm*d2norm)*h1-I(4)*norm*dnorm*h2)/q;
        const auto tangent_w=h1*z.z1;
        below(speed*tangent_w,l.lateral_acceleration,"LATERAL_ACCELERATION",i);

        if (ph.mode==Mode::A) {
          omega=tangent_w;
          below(omega,l.yaw_rate,"YAW_RATE",i);
          if (l.yaw_acceleration>0) below(h2*z1sq+h1*z.z2,l.yaw_acceleration,"YAW_ACCELERATION",i);
          if (l.yaw_jerk>0) below(h3*z1cube+I(3)*h2*z.z1*z.z2+h1*z.z3,l.yaw_jerk,"YAW_JERK",i);
          const auto k=I(ph.gear[i])*h1/norm;
          below(k,1/l.radius,"ACKERMANN_CURVATURE",i);
          const auto dk=(I(ph.gear[i])*h2-k*dnorm)/norm;
          const auto ddk=(I(ph.gear[i])*h3-I(2)*dk*dnorm-k*d2norm)/norm;
          const auto kdot=dk*z.z1,kddot=ddk*z1sq+dk*z.z2;
          for (double wx:{l.wheelbase/2,-l.wheelbase/2}) for (double wy:{l.track/2,-l.track/2}) {
            const auto forward=I(1)-I(wy)*k,side=I(wx)*k;
            if (forward.l<=0) throw CheckFailure{"WHEEL_FORWARD_SINGULARITY",i};
            const auto steer=up(std::atan2(magnitude(side),forward.l));
            below(I(steer),l.steer_angle,"WHEEL_ANGLE",i);
            if(l.high_steer_speed>0 && steer>l.high_steer_angle)
              below(speed,l.high_steer_speed,"HIGH_STEER_BODY_SPEED",i);
            const auto denom=square(forward)+square(side);
            const auto angle_k=I(wx)/denom;
            const auto ddenom=I(2)*(I(-wy)*forward+I(wx)*side);
            below(angle_k*kdot,l.steer_rate,"STEER_RATE",i);
            below(angle_k*kddot-angle_k*ddenom*square(kdot)/denom,l.steer_acceleration,"STEER_ACCELERATION",i);
            below(speed*root(denom),l.wheel_speed,"WHEEL_SPEED",i);
          }
        } else {
          // Physical steering in [-pi/2,+pi/2]. A projected tangent may be
          // exactly perpendicular (legitimate crab). Direct angle containment
          // also detects crossing its atan2 branch while the base moves.
          const double midangle=point.bodies[i].oblique_angle;
          const double span=up(magnitude(h1)*(b-a));
          // Mid-angle plus arc variation is conservative, including at zero speed.
          if (std::abs(midangle)+span>l.steer_angle+2e-13) throw CheckFailure{"OBLIQUE_ANGLE",i};
          if(l.high_steer_speed>0 && std::abs(midangle)+span>l.high_steer_angle)
            below(speed,l.high_steer_speed,"HIGH_STEER_BODY_SPEED",i);
          below(tangent_w,l.steer_rate,"STEER_RATE",i);
          below(h2*z1sq+h1*z.z2,l.steer_acceleration,"STEER_ACCELERATION",i);
          below(speed,l.wheel_speed,"WHEEL_SPEED",i);
        }
      }
      const double rho=up(.5*std::hypot(f[2],f[3]));
      corner_rate=std::max(corner_rate,up(magnitude(speed)+rho*magnitude(omega)));
    }
    const double eval_bound=reconstruction_padding(r,ph,piece,tm,point);
    const double pad=up(corner_rate*up((tb-ta)*ph.duration/2)+eval_bound);
    const auto sweep=map.check(point.reference,head,r.vehicles.size(),pad);
    if (sweep.reason==fwis_planning_core::CollisionReason::clear) return true;
    uncertainty={"SWEPT_CLEARANCE_INCONCLUSIVE",sweep.first};
  } catch (const std::domain_error &e) {uncertainty={e.what(),0};}
    catch (const CheckFailure &e) {uncertainty=e;}
  // A small interval failing a conservative inequality proves neither a
  // collision nor physical infeasibility. Never promote it to executable.
  if(depth>=22) {fail(uncertainty.reason,uncertainty.vehicle,normalized(a),normalized(b));return false;}
  const double m=(a+b)/2;
  const auto clock=inverse_clock(normalized(m),ph.cruise_ramp_fraction);
  return visit(piece,a,m,ta,clock[1],depth+1) && visit(piece,m,b,clock[0],tb,depth+1);
}
bool same_pose(const Sample &a,const Sample &b,std::size_t count) {
  if (std::hypot(a.reference.x-b.reference.x,a.reference.y-b.reference.y)>1e-6
      || separation(a.reference.yaw,b.reference.yaw)>1e-6) return false;
  for(std::size_t i=0;i<count;++i) {
    if (std::hypot(a.bodies[i].pose.x-b.bodies[i].pose.x,a.bodies[i].pose.y-b.bodies[i].pose.y)>1e-6
        || separation(a.bodies[i].pose.yaw,b.bodies[i].pose.yaw)>1e-6) return false;
  }
  return true;
}
bool boundary(const Boundary& b,const Sample& s,std::size_t n) {
  if (!std::isfinite(b.reference.x)||!std::isfinite(b.reference.y)||!std::isfinite(b.reference.yaw) ||
      std::hypot(b.reference.x-s.reference.x,b.reference.y-s.reference.y)>.02 ||
      (b.constrain_reference_yaw && separation(b.reference.yaw,s.reference.yaw)>pi/180)) return false;
  for(std::size_t i=0;i<n;++i)
    if(!std::isfinite(b.heading[i]) || (b.constrain_heading[i] && separation(b.heading[i],s.bodies[i].pose.yaw)>pi/180)) return false;
  return true;
}
double endpoint_diff(const Quintic& p,int order,bool end) {
  Quintic v=p;
  for (int k=0;k<order;++k) for(int i=0;i<5-k;++i) v[i]=(5-k)*(v[i+1]-v[i]);
  return v[end?5-order:0];
}
bool active_join(const Request &r,const Phase &ph,const Piece &left,const Piece &right) {
  if(ph.kind!=Kind::Motion || ph.mode==Mode::S) return true;
  auto boundary_motion=[&](const Piece& p,bool last,std::size_t body) {
    const double z=last?1.:0.;
    const auto px=polynomial<2>(p.x,z,z),py=polynomial<2>(p.y,z,z),angle=polynomial<2>(p.yaw,z,z);
    const auto sc=sin_cos<2>(angle);
    const auto& f=r.vehicles[body];
    const auto x=px+scale(sc.second,I(f[0]))-scale(sc.first,I(f[1]));
    const auto y=py+scale(sc.first,I(f[0]))+scale(sc.second,I(f[1]));
    const I tx=x.c[1],ty=y.c[1],norm2=square(tx)+square(ty),length=root(norm2);
    const auto theta_s=(tx*y.c[2]-ty*x.c[2])*I(2)/norm2;
    return std::array<I,4>{tx,ty,theta_s/length, length};
  };
  const I ds=I(1.875)*I(double(ph.pieces.size()))/I(ph.duration);
  for(std::size_t i=0;i<r.vehicles.size();++i) {
    const auto a=boundary_motion(left,true,i),b=boundary_motion(right,false,i);
    if(magnitude(ds*(a[3]-b[3]))>1e-6 || magnitude(a[2]-b[2])>1e-5) return false;
    const auto dot=a[0]*b[0]+a[1]*b[1], cross=a[0]*b[1]-a[1]*b[0];
    if(!(dot.l>0) || std::atan2(magnitude(cross),dot.l)>1e-6) return false;
  }
  return true;
}
bool finite_coeff(const Quintic &p) {
  return std::all_of(p.begin(),p.end(),[](double x){return std::isfinite(x);});
}
bool constant(const Quintic &p) {
  return std::all_of(p.begin()+1,p.end(),[&](double x){return x==p.front();});
}
} // namespace

Audit certify(const Request& r,const Plan& plan,Clock::time_point deadline,const Cancel& cancel) {
  const auto n=r.vehicles.size();
  if (!r.grid || (n!=1 && n!=3) || (n==3 && !r.cargo) ||
      (n==1 && (r.cargo || r.vehicles[0][0]!=0 || r.vehicles[0][1]!=0)) ||
      !positive(r.task_duration_cap) || plan.phases.empty()) return {false,"INVALID_REQUEST_OR_EMPTY_PLAN"};
  const auto &l=r.limits;
  for (double v:{l.radius,l.speed,l.acceleration,l.jerk,l.lateral_acceleration,l.spin_speed,
    l.spin_acceleration,l.spin_jerk,l.yaw_rate,l.wheelbase,l.track,l.wheel_speed,l.steer_angle,
    l.steer_rate,l.steer_acceleration,l.prepare_time}) if (!positive(v)) return {false,"INVALID_LIMITS"};
  if (!std::isfinite(l.min_motion_duration)||l.min_motion_duration<0||
      !std::isfinite(l.cargo_center_speed)||l.cargo_center_speed<0||
      !std::isfinite(l.high_steer_speed)||l.high_steer_speed<0||
      !positive(l.high_steer_angle)||
      !std::isfinite(l.yaw_acceleration)||l.yaw_acceleration<0||!std::isfinite(l.yaw_jerk)||l.yaw_jerk<0)
    return {false,"INVALID_LIMITS"};
  try {
    const fwis_planning_core::CollisionKernel map(*r.grid,r.vehicles,r.cargo);
    double total=0,prepare=0;
    Sample prev_end,last_motion;
    last_motion.mode=Mode::A;
    Gears last_gear{1,1,1};
    bool have_motion=false;
    for (std::size_t idx=0;idx<plan.phases.size();++idx) {
      const auto &ph=plan.phases[idx];
      if (Clock::now()>=deadline || (cancel&&cancel())) return {false,"CERTIFICATION_BUDGET",idx};
      if (!positive(ph.duration) || ph.pieces.empty()) return {false,"INVALID_PHASE",idx};
      if (ph.cruise_ramp_fraction != 0. &&
          (!supported_cruise_ramp(ph.cruise_ramp_fraction) || ph.kind != Kind::Motion || ph.mode == Mode::S))
        return {false,"UNKNOWN_TIME_LAW",idx};
      if (ph.kind==Kind::Motion && ph.duration<l.min_motion_duration) return {false,"MODE_DWELL",idx};
      if (!(ph.mode==Mode::A || ph.mode==Mode::O || ph.mode==Mode::S)
          || !(ph.kind==Kind::Motion || ph.kind==Kind::Prepare || ph.kind==Kind::Hold)) return {false,"UNKNOWN_PHASE",idx};
      total+=ph.duration;
      if (total>r.task_duration_cap) return {false,"TASK_TIME_CONSTRAINT",idx};
      for (std::size_t i=0;i<n;++i) {
        if ((ph.gear[i]!=1 && ph.gear[i]!=-1) || !std::isfinite(ph.heading0[i]) || !std::isfinite(ph.heading1[i]))
          return {false,"INVALID_BODY_STATE",idx,i};
        if (!r.allow_reverse && ph.kind==Kind::Motion &&
            (ph.mode==Mode::A || ph.mode==Mode::O) && ph.gear[i]<0)
          return {false,"REVERSE_DISALLOWED",idx,i};
        if ((ph.kind!=Kind::Motion || ph.mode==Mode::O) && ph.heading0[i]!=ph.heading1[i])
          return {false,"STATIONARY_HEADING_CHANGE",idx,i};
      }
      for(std::size_t j=0;j<ph.pieces.size();++j) {
        const auto &p=ph.pieces[j];
        if(!finite_coeff(p.x)||!finite_coeff(p.y)||!finite_coeff(p.yaw)) return {false,"NONFINITE_COEFFICIENT",idx};
        // Stationary coordinates share a single authority over the entire
        // phase. A succession of tiny, locally tolerated position changes
        // would otherwise accumulate despite zero commanded body speed.
        const auto &first=ph.pieces.front();
        if ((ph.kind!=Kind::Motion || ph.mode==Mode::S) &&
            (!constant(p.x)||!constant(p.y)||p.x.front()!=first.x.front()||p.y.front()!=first.y.front()))
          return {false,"STATIONARY_TRANSLATION",idx};
        if ((ph.kind!=Kind::Motion || ph.mode!=Mode::A) &&
            (!constant(p.yaw)||p.yaw.front()!=first.yaw.front()))
          return {false,"STATIONARY_REFERENCE_YAW",idx};
        if (j) {
          const auto &q=ph.pieces[j-1];
          // Source representation uses one clock across equal-size polynomial
          // spans. C3 of mount positions implies continuity of wheel rates.
          for (int order=0;order<=3;++order) {
            constexpr double tol=1e-8;
            for (const auto &pair:{std::make_pair(&q.x,&p.x),std::make_pair(&q.y,&p.y),std::make_pair(&q.yaw,&p.yaw)})
              if (!same(endpoint_diff(*pair.first,order,true),endpoint_diff(*pair.second,order,false),tol))
                return {false,"INTERNAL_POLYNOMIAL_JOIN",idx,0,double(j)/ph.pieces.size(),double(j)/ph.pieces.size()};
          }
          if(!active_join(r,ph,q,p))
            return {false,"BODY_CONTINUITY_AT_JOIN",idx,0,double(j)/ph.pieces.size(),double(j)/ph.pieces.size()};
        }
      }
      const auto start=evaluate_phase(r,ph,0),end=evaluate_phase(r,ph,ph.duration);
      if ((!idx&&!boundary(r.start,start,n)) || (idx && !same_pose(prev_end,start,n)))
        return {false,idx?"PHASE_JOIN":"START_ENDPOINT",idx};
      if (idx+1==plan.phases.size() && !boundary(r.goal,end,n)) return {false,"GOAL_ENDPOINT",idx};
      for(const auto &s:{start,end}) {
        double h[3]{};
        for(std::size_t i=0;i<n;++i) h[i]=s.bodies[i].pose.yaw;
        const auto hit=map.check(s.reference,h,n);
        if (hit.reason!=fwis_planning_core::CollisionReason::clear)
          return {false,fwis_planning_core::reason_name(hit.reason),idx,hit.first};
      }
      if(ph.kind==Kind::Prepare) {
        if (idx+1==plan.phases.size() || plan.phases[idx+1].kind!=Kind::Motion ||
            ph.mode!=plan.phases[idx+1].mode || ph.gear!=plan.phases[idx+1].gear)
          return {false,"PREPARE_TARGET_MISMATCH",idx};
        prepare=ph.duration;
      }
      if(ph.kind==Kind::Motion) {
        const double delta=wheel_delta(r,last_motion,start);
        const bool protocol=start.mode!=last_motion.mode || (have_motion && last_gear!=ph.gear);
        const auto travel=std::max(1.875*delta/l.steer_rate,
            std::sqrt(10./std::sqrt(3.)*delta/l.steer_acceleration));
        if ((protocol || delta>1e-7) && prepare+1e-9<l.prepare_time+travel)
          return {false,"MISSING_WHEEL_PREPARATION",idx};
        if(have_motion && !protocol && ph.mode==Mode::A && !prepare)
          return {false,"UNNECESSARY_INTERNAL_A_STOP",idx};
        prepare=0; last_motion=end; last_gear=ph.gear; have_motion=true;
      }
      IntervalAudit audit{r,ph,map,deadline,cancel,idx};
      const auto count=ph.pieces.size();
      auto left=inverse_clock(0.,ph.cruise_ramp_fraction);
      for(std::size_t j=0;j<count;++j) {
        const auto right=inverse_clock(double(j+1)/count,ph.cruise_ramp_fraction);
        if (!audit.visit(j,0,1,left[0],right[1],0)) return audit.result;
        left=right;
      }
      prev_end=end;
    }
    return {true,"CERTIFIED"};
  } catch(const std::exception &e) {return {false,e.what()};}
}
} // namespace fwis_hybrid_joint
