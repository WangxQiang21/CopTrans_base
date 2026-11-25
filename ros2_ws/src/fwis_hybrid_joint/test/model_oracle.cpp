// Reconstruct every Bernstein coordinate from the binomial basis in long
// double, independently of the evaluator's double de Casteljau recursion.
// Integrate each emitted body Twist by RK4 without resetting integrated poses.
#include "fwis_hybrid_joint/model.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>

namespace h=fwis_hybrid_joint;
using C=std::complex<long double>;
constexpr long double pi=3.14159265358979323846264338327950288L;
struct Oracle {C pos; long double yaw,vx,vy,w,k;};
long double angle(long double a){return std::remainder(a,2*pi);}
int choose(int n,int j){
  int a=1;for(int k=0;k<j;++k) a=a*(n-k)/(k+1);return a;
}
long double basis(const h::Quintic &c,long double s,int derivative) {
  long double out=0;
  const int degree=5-derivative;
  for(int i=0;i<=degree;++i){
    long double v=0;
    for(int j=0;j<=derivative;++j) v+=((derivative-j)%2?-1:1)*choose(derivative,j)*static_cast<long double>(c[i+j]);
    for(int j=0;j<derivative;++j) v*=5-j;
    out+=v*choose(degree,i)*std::pow(s,i)*std::pow(1-s,degree-i);
  }
  return out;
}
Oracle oracle(const h::Phase &p,const h::Footprint &f,int i,long double u) {
  long double s=u*u*u*(10-15*u+6*u*u),sd=30*u*u*(1-u)*(1-u)/p.duration;
  if(p.cruise_ramp_fraction) {
    const auto a=static_cast<long double>(p.cruise_ramp_fraction);
    long double z=u<.5?u:1-u;
    if(z<a) {
      z/=a;
      s=a/(1-a)*(std::pow(z,3)-std::pow(z,4)/2);
      sd=(3*std::pow(z,2)-2*std::pow(z,3))/((1-a)*p.duration);
    } else {
      s=(z-a/2)/(1-a);
      sd=1/((1-a)*p.duration);
    }
    if(u>.5)s=1-s;
  }
  const auto &piece=p.pieces.front();
  C xy(basis(piece.x,s,0),basis(piece.y,s,0)),v(basis(piece.x,s,1),basis(piece.y,s,1));
  C a(basis(piece.x,s,2),basis(piece.y,s,2));
  const auto psi=basis(piece.yaw,s,0),w=basis(piece.yaw,s,1),wd=basis(piece.yaw,s,2);
  C offset=C(f[0],f[1])*std::polar(1.L,psi);
  v+=C(0,w)*offset;
  a+=C(-w*w,wd)*offset;
  Oracle out{xy+offset,p.heading0[i],0,0,0,0};
  if(p.mode==h::Mode::A){
    out.yaw=std::arg(v*static_cast<long double>(p.gear[i]));
    const auto rotation=std::imag(a/v);
    out.w=rotation*sd;out.k=rotation/std::abs(v)*p.gear[i];out.vx=std::abs(v)*sd*p.gear[i];
  } else if(p.mode==h::Mode::O){
    C body=v*std::polar(1.L,-out.yaw)*sd;out.vx=body.real();out.vy=body.imag();
  } else {
    const auto delta=p.heading1[i]-p.heading0[i];out.yaw+=s*delta;out.w=sd*delta;
  }
  return out;
}
struct State {double x,y,h;};
State operator+(State a,State b){return {a.x+b.x,a.y+b.y,a.h+b.h};}
State operator*(double a,State b){return {a*b.x,a*b.y,a*b.h};}
State rhs(const h::Body &cmd,const State &s){
  const auto c=cos(s.h),q=sin(s.h);
  return {cmd.vx*c-cmd.vy*q,cmd.vx*q+cmd.vy*c,cmd.wz};
}
int main(int argc,char** argv){
  constexpr unsigned seed=3177513;
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> un(0,1);
  auto range=[&](double l,double h){return l+(h-l)*un(rng);};
  std::ofstream raw;
  if(argc>1){raw.open(argv[1]);if(!raw)return 2;raw<<"task,mode,sample,position_error_m,yaw_error_rad,twist_error,curvature_error,mount_error_m\n";}
  double poserr=0,yawerr=0,twerr=0,kerr=0,mount=0,intxy=0,inth=0;
  unsigned loops=0,integ=0,boundaries=0;
  for(int bodies:{1,3}) for(int mode=0;mode<3;++mode) for(int k=0;k<10010;++k) {
    h::Request r;h::Phase p;p.duration=range(3,15);p.cruise_ramp_fraction=(k%2 || k>=10004)?.125:0.;p.mode=static_cast<h::Mode>(mode);
    r.vehicles={{0,0,.5,.3}};
    if(bodies==3) r.vehicles={{.9,.1,.5,.3},{-.6,-.68,.5,.3},{-.55,.74,.5,.3}};
    h::Piece piece; auto x=range(-4,4),y=range(-4,4),ang=range(-pi,pi),d=range(.6,2);
    for(int j=0;j<6;++j){
      double z=j/5.;
      piece.x[j]=x+(mode==2?0:d*z);piece.y[j]=y+(mode==2?0:.1*std::sin(z*2)+range(-.08,.08));
      piece.yaw[j]=ang+(mode==0?range(-.035,.035):0.);
    }
    p.pieces={piece};
    for(int i=0;i<bodies;i++){
      p.heading0[i]=range(-pi,pi);p.heading1[i]=p.heading0[i]+(mode==2?range(-pi,pi):0.);
      p.gear[i]=(rng()&1)?1:-1;
    }
    double u=range(.01,.99);
    if(k>=10000){++boundaries;
      const double boundary_points[10]{0.,1.,1e-13,1-1e-13,.125-1e-13,.125,.125+1e-13,.875-1e-13,.875,.875+1e-13};
      u=boundary_points[k-10000];}
    const auto s=h::evaluate_phase(r,p,u*p.duration);
    double dp=0,da=0,dt=0,dk=0,dm=0;
    for(int i=0;i<bodies;++i) {
      const auto expected=oracle(p,r.vehicles[i],i,u);const auto &b=s.bodies[i];
      dp=std::max(dp,double(std::abs(expected.pos-C(b.pose.x,b.pose.y))));
      da=std::max(da,double(std::abs(angle(b.pose.yaw-expected.yaw))));
      dt=std::max({dt,double(std::abs(b.vx-expected.vx)),double(std::abs(b.vy-expected.vy)),double(std::abs(b.wz-expected.w))});
      dk=std::max(dk,double(std::abs(b.curvature-expected.k)));
      const C local=C(b.pose.x-s.reference.x,b.pose.y-s.reference.y)*std::polar(1.L,-static_cast<long double>(s.reference.yaw));
      dm=std::max(dm,double(std::abs(local-C(r.vehicles[i][0],r.vehicles[i][1]))));
    }
    ++loops;
    if(raw) raw<<(bodies==1?"single":"formation")<<','<<mode<<','<<k<<','<<dp<<','<<da<<','<<dt<<','<<dk<<','<<dm<<'\n';
    poserr=std::max(poserr,dp);yawerr=std::max(yawerr,da);twerr=std::max(twerr,dt);
    kerr=std::max(kerr,dk);mount=std::max(mount,dm);
    if(k<100) {
      constexpr int steps=128;
      const double dt=p.duration/steps;
      auto last=h::evaluate_phase(r,p,0);
      State st[3]{};
      for(int i=0;i<bodies;++i) st[i]={last.bodies[i].pose.x,last.bodies[i].pose.y,last.bodies[i].pose.yaw};
      for(int j=1;j<=steps;++j){
        const auto mid=h::evaluate_phase(r,p,(j-.5)*dt),end=h::evaluate_phase(r,p,j*dt);
        for(int i=0;i<bodies;++i) {
          auto &a=st[i]; auto k1=rhs(last.bodies[i],a),k2=rhs(mid.bodies[i],a+(.5*dt)*k1);
          auto k3=rhs(mid.bodies[i],a+(.5*dt)*k2),k4=rhs(end.bodies[i],a+dt*k3);
          a=a+(dt/6)*(k1+2*k2+2*k3+k4);
          const auto &truth=end.bodies[i].pose;
          inth=std::max(inth,double(std::abs(angle(a.h-truth.yaw))));
          intxy=std::max(intxy,std::hypot(a.x-truth.x,a.y-truth.y));
        }
        last=end;
      }
      ++integ;
    }
  }
  std::cout<<std::setprecision(15)<<"{\"seed\":"<<seed<<",\"ordinary_per_task_per_mode\":10000,\"boundary_cases\":"<<boundaries
    <<",\"total_cases\":"<<loops<<",\"reintegrated_trajectories\":"<<integ<<",\"max_position_error_m\":"<<poserr
    <<",\"max_angle_error_rad\":"<<yawerr<<",\"max_twist_error\":"<<twerr<<",\"max_curvature_error\":"<<kerr
    <<",\"max_mount_error_m\":"<<mount<<",\"max_reintegration_position_m\":"<<intxy
    <<",\"max_reintegration_angle_rad\":"<<inth<<"}\n";
  return !(poserr<1e-7 && yawerr<1e-7 && twerr<1e-7 && kerr<1e-7 && mount<1e-8 && intxy<1e-4 && inth<1e-4);
}
