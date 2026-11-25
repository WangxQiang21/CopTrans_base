#include "nlp_api.h"
#include "clock.hpp"
#include "policy_values.hpp"
#include <casadi/casadi.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <deque>
#include <random>
#include <stdexcept>
#include <vector>

namespace {
namespace policy=fwis_hybrid_joint::policy;
using casadi::DM;
using casadi::SX;
const double inf=std::numeric_limits<double>::infinity();
constexpr double pi=3.14159265358979323846;
using Coordinates=std::array<std::array<SX,6>,3>;

class StopCallback : public casadi::Callback {
public:
  StopCallback(casadi_int nv,casadi_int nc):nx(nv),ng(nc){construct("check_deadline");}
  casadi_int get_n_in() override{return casadi::nlpsol_n_out();}
  casadi_int get_n_out() override{return 1;}
  std::string get_name_in(casadi_int i) override{return casadi::nlpsol_out(i);}
  std::string get_name_out(casadi_int) override{return "stop";}
  casadi::Sparsity get_sparsity_in(casadi_int i) override{
    auto name=casadi::nlpsol_out(i);
    if(name=="x"||name=="lam_x")return casadi::Sparsity::dense(nx,1);
    if(name=="g"||name=="lam_g")return casadi::Sparsity::dense(ng,1);
    if(name=="f")return casadi::Sparsity::scalar();
    return casadi::Sparsity(0,1);
  }
  HJProblem* active{};
  std::vector<DM> eval(const std::vector<DM>&) const override{return {DM(!active || active->stop(active->opaque))};}
private:
  casadi_int nx,ng;
};
struct CompiledGraph {
  std::string key;
  std::unique_ptr<StopCallback> stop;
  casadi::Function solver,expand;
  std::vector<double> last_parameters,last_guess;
  DM last_x,last_dual,last_multipliers;
};
thread_local std::deque<CompiledGraph> templates;
class Problem {
public:
  explicit Problem(HJProblem& data, HJDerivativeAudit* audit=nullptr):p(data), audit_(audit){}
  void run();
private:
  std::vector<SX> continuous_axis(const HJPhase& phase, int axis, const SX& start, const SX& end);
  HJProblem& p;
  HJDerivativeAudit* audit_;
  std::vector<SX> vars, cons, outputs;
  std::vector<double> guess, lb, ub, cl, cu;
  SX cost{0};
  std::vector<SX> parameters;
  std::vector<double> values;
  SX parameter(double v){
    parameters.push_back(SX::sym("p"));values.push_back(v);return parameters.back();
  }
  SX variable(double x,double lo=-inf,double hi=inf) {
    SX a=SX::sym("v"); vars.push_back(a);guess.push_back(x);lb.push_back(lo);ub.push_back(hi);return a;
  }
  void constraint(const SX& a,double lo,double hi) {
    if(a.is_constant()) {
      double n=static_cast<double>(a);
      if(n<lo-1e-8 || n>hi+1e-8) throw std::runtime_error("incompatible boundary contract");
      return;
    }
    cons.push_back(a);cl.push_back(lo);cu.push_back(hi);
  }
  void equal(const SX& a,double tolerance=2e-8){constraint(a,-tolerance,tolerance);}
  void direction(const SX& dx,const SX& dy,const SX& h,int gear) {
    equal(-sin(h)*dx+cos(h)*dy,1e-8);
    constraint(gear*(cos(h)*dx+sin(h)*dy),1e-4,inf);
  }
};

// A degree-five spline with double interior knots spans the entire C3
// piecewise-quintic space. Knot insertion exposes the Bernstein coefficients
// consumed by dynamics, collision constraints and the independent certificate.
// Even an iteration-limited primal therefore has exact position/jet joins.
std::vector<SX> Problem::continuous_axis(const HJPhase& phase, int axis, const SX& start, const SX& end) {
  constexpr int degree = 5;
  const int spans = phase.spans;
  std::vector<double> knots(degree + 1, 0.);
  for (int j = 1; j < spans; ++j) knots.insert(knots.end(), 2, j);
  knots.insert(knots.end(), degree + 1, spans);
  const int count = static_cast<int>(knots.size()) - degree - 1;
  std::vector<SX> control(count);
  control.front() = start;
  control.back() = end;
  DM basis = DM::zeros(count,count);
  basis(0,0) = 1.;
  basis(count-1,count-1) = 1.;
  std::vector<double> samples(count);
  samples.front() = phase.coefficient[0][axis][0];
  samples.back() = phase.coefficient[spans-1][axis][degree];
  for (int i = 1; i < count - 1; ++i) {
    double station = 0;
    for (int j = 1; j <= degree; ++j) station += knots[i+j] / degree;
    const int span = std::min(static_cast<int>(station), spans - 1);
    const double u = station - span;
    std::array<double, degree+1> v;
    std::copy_n(phase.coefficient[span][axis], degree+1, v.begin());
    for (int order = degree; order > 0; --order)
      for (int j = 0; j < order; ++j) v[j] += u * (v[j+1] - v[j]);
    samples[i] = v[0];
    std::vector<double> weights(knots.size()-1);
    for (size_t j=0; j<weights.size(); ++j)
      weights[j] = knots[j]<=station && station<knots[j+1] ? 1. : 0.;
    for (int level=1; level<=degree; ++level) {
      for (size_t j=0; j+level<weights.size(); ++j) {
        const double left = knots[j+level]-knots[j];
        const double right = knots[j+level+1]-knots[j+1];
        weights[j] = (left>0 ? (station-knots[j])*weights[j]/left : 0.) +
                     (right>0 ? (knots[j+level+1]-station)*weights[j+1]/right : 0.);
      }
    }
    for (int j=0; j<count; ++j) basis(i,j) = weights[j];
  }
  // Interpolate the supplied spatial seed at the Greville points. Sampling a
  // spline at those points and copying its positions as control vertices would
  // smooth it a second time and can move an already feasible seed into a wall.
  const auto initial = DM::solve(basis,DM(samples)).nonzeros();
  for (int i=1; i<count-1; ++i) control[i] = variable(initial[i]);
  for (int knot = 1; knot < spans; ++knot) {
    for (int multiplicity = 2; multiplicity < degree; ++multiplicity) {
      const int k = std::upper_bound(knots.begin(), knots.end(), knot) - knots.begin() - 1;
      std::vector<SX> refined(control.size() + 1);
      std::copy_n(control.begin(), k - degree + 1, refined.begin());
      for (int i = k - degree + 1; i <= k - multiplicity; ++i) {
        const double u = (knot - knots[i]) / (knots[i+degree] - knots[i]);
        refined[i] = (1-u)*control[i-1] + u*control[i];
      }
      std::copy(control.begin() + k - multiplicity, control.end(), refined.begin() + k - multiplicity + 1);
      control.swap(refined);
      knots.insert(knots.begin() + k + 1, knot);
    }
  }
  return control;
}

// Restore the affine C3 equations of an iteration-limited primal. This is the
// orthogonal projection onto exactly the same equations used by the NLP. Keep
// both endpoint positions and endpoint directions, then certify the resulting
// geometry, wheel dynamics and collision envelope from scratch.
void restore_joins(HJPhase& phase) {
  const int spans = phase.spans;
  if (spans < 2 || phase.mode != 0) return;
  constexpr int difference[4][4] = {{1}, {-1,1}, {1,-2,1}, {-1,3,-3,1}};
  constexpr int derivative_scale[] = {1,5,20,60};
  const int rows = 4*(spans-1);
  const int columns = 6*spans;
  DM all = DM::zeros(rows, columns);
  for (int join = 0; join < spans-1; ++join) {
    for (int order = 0; order <= 3; ++order) {
      for (int i = 0; i <= order; ++i) {
        all(4*join+order,6*join+5-order+i) = difference[order][i];
        all(4*join+order,6*join+6+i) = -difference[order][i];
      }
    }
  }
  const auto free = all(casadi::Slice(),casadi::Slice(2,columns-2));
  for (int axis=0; axis<3; ++axis) {
    std::vector<double> values;
    for (int j=0; j<spans; ++j)
      values.insert(values.end(), phase.coefficient[j][axis], phase.coefficient[j][axis]+6);
    const DM errors = DM::mtimes(all, DM(values));
    auto residual = errors.nonzeros();
    bool needs_restoration = false;
    for (int i=0; i<rows; ++i)
      if (std::abs(residual[i])*derivative_scale[i%4] > 1e-8) needs_restoration = true;
    if (!needs_restoration) continue;
    const auto multipliers = DM::solve(DM::mtimes(free,free.T()), errors);
    const auto correction = DM::mtimes(free.T(),multipliers).nonzeros();
    for (int i=2; i<columns-2; ++i)
      phase.coefficient[i/6][axis][i%6] -= correction[i-2];
  }
}

std::array<SX,3> position(const Coordinates& coeff,const SX& s) {
  std::array<SX,3> out{};
  // Bernstein expansion. Working differences keep flat coordinates exact.
  for(int axis=0;axis<3;++axis){
    auto a=coeff[axis];
    for(int n=5;n>0;--n) for(int j=0;j<n;++j) a[j]+=s*(a[j+1]-a[j]);
    out[axis]=a[0];
  }
  return out;
}

// Independent central differences of every row, including the objective.
// The same parameter vector is used for every step. Adjacent central
// differences provide independent O(h^4) Richardson estimates. Pick the
// most stable adjacent pair before consulting AD, avoiding truncation error
// on high-curvature factors and cancellation at the smallest step.
void differential_check(const SX &design,const SX &rows,const std::vector<double> &guess,HJDerivativeAudit &audit) {
  const SX direction=SX::sym("direction",design.numel());
  const auto evaluate=casadi::Function("values",{design},{rows});
  const auto linearize=casadi::Function("product",{design,direction},{SX::jtimes(rows,design,direction)});
  std::mt19937_64 rng(audit.seed);
  std::uniform_real_distribution<double> unit(-1.,1.);
  audit.rows=rows.numel();
  if (audit.cases<100 || audit.rows<1) throw std::invalid_argument("insufficient derivative evidence");
  for (int sample=0;sample<audit.cases;++sample) {
    auto x=guess,d=guess;
    for(std::size_t i=0;i<x.size();++i) {
      x[i]+=.015*unit(rng);
      d[i]=unit(rng);
    }
    const auto exact=linearize(std::vector<DM>{x,d}).at(0).get_elements();
    std::array<std::vector<double>,5> diffs;
    const double steps[]{1e-3,1e-4,1e-5,1e-6,1e-7};
    for(int j=0;j<5;++j) {
      auto left=x,right=x;
      for(std::size_t i=0;i<x.size();++i){left[i]-=steps[j]*d[i];right[i]+=steps[j]*d[i];}
      const auto v0=evaluate(std::vector<DM>{left}).at(0).get_elements();
      const auto v1=evaluate(std::vector<DM>{right}).at(0).get_elements();
      diffs[j].resize(v0.size());
      for(std::size_t row=0;row<v0.size();++row) diffs[j][row]=(v1[row]-v0[row])/(2*steps[j]);
    }
    for(std::size_t row=0;row<exact.size();++row) {
      const auto a=exact[row];
      const bool small=std::abs(a)<.01;
      const double tolerance=small?1.e-6:1.e-4*std::abs(a);
      std::array<double,4> estimates;
      bool finite=std::isfinite(a);
      for(int j=0;j<5;++j) {
        finite=finite && std::isfinite(diffs[j][row]);
        if(j) estimates[j-1]=(100.*diffs[j][row]-diffs[j-1][row])/99.;
      }
      if(!finite){++audit.nonfinite;continue;}
      int best=1;
      for(int j=2;j<4;++j)
        if(std::abs(estimates[j]-estimates[j-1])<std::abs(estimates[best]-estimates[best-1])) best=j;
      const auto error=std::abs(a-estimates[best]);
      const bool corroborated=std::abs(a-estimates[best-1])<=tolerance;
      if(small) audit.max_near_zero_absolute_error=std::max(audit.max_near_zero_absolute_error,error);
      else audit.max_relative_error=std::max(audit.max_relative_error,error/std::abs(a));
      if(error>tolerance || !corroborated) {
        if(audit.failed<8) std::fprintf(stderr,"derivative sample=%d row=%zu exact=%.12g fd=%g %g %g %g\n",sample,row,a,diffs[0][row],diffs[1][row],diffs[2][row],diffs[3][row]);
        ++audit.failed;
      }
    }
  }
}

using Jet=std::array<SX,5>; // raw station derivatives, orders 0 through 4
Jet trajectory_jet(std::array<SX,6> coeff,double s) {
  Jet out;
  for(int k=0;k<=4;++k){
    const int degree=5-k;
    auto a=coeff;
    for(int n=degree;n>0;--n)for(int i=0;i<n;++i)a[i]+=s*(a[i+1]-a[i]);
    out[k]=a[0];
    for(int i=0;i<degree;++i)coeff[i]=degree*(coeff[i+1]-coeff[i]);
  }
  return out;
}
std::pair<Jet,Jet> mount_jets(const Coordinates& curve,double u,double ox,double oy){
  Jet x=trajectory_jet(curve[0],u),y=trajectory_jet(curve[1],u);
  if(ox==0 && oy==0)return {x,y};
  const auto yaw=trajectory_jet(curve[2],u);
  Jet rx,ry;
  const auto c=cos(yaw[0]),s=sin(yaw[0]);
  rx[0]=ox*c-oy*s;ry[0]=ox*s+oy*c;
  constexpr int binom[4][4]={{1,0,0,0},{1,1,0,0},{1,2,1,0},{1,3,3,1}};
  // d(Rr)/ds = yaw' J Rr. Repeated product differentiation uses only
  // derivatives local to this spline span and preserves CasADi sparsity.
  for(int k=1;k<5;++k){
    rx[k]=0;ry[k]=0;
    for(int l=0;l<k;++l){
      const SX weight=binom[k-1][l]*yaw[k-l];
      rx[k]-=weight*ry[l];ry[k]+=weight*rx[l];
    }
  }
  for(int k=0;k<5;++k){x[k]+=rx[k];y[k]+=ry[k];}
  return {x,y};
}
void Problem::run(){
  const auto build_start=std::chrono::steady_clock::now();
  const bool multi=p.bodies==3;
  const double t0=p.radius/p.speed;
  // Fixed-mode NLP uses differentiable collocation surrogates and spatial
  // regularization; after certification the same archive policy compares every
  // candidate on the physical model independently of these local graph nodes.
  const double smooth_scale=.1*t0;
  SX x=parameter(p.start[0]),y=parameter(p.start[1]),yaw=parameter(p.start[2]),tau_sum=0;
  std::array<SX,3> h{SX(p.start[3]),SX(p.start[4]),SX(p.start[5])};
  std::array<std::array<SX,4>,3> prior{};
  for(auto& row:prior)row.fill(SX(0));
  int last_xy=-1,last_a=-1;
  for(int k=0;k<p.count;++k) {
    if(p.phase[k].mode<2) last_xy=k;
    if(p.phase[k].mode==0) last_a=k;
  }
  if(last_xy<0 && std::hypot(p.start[0]-p.goal[0],p.start[1]-p.goal[1])>1e-6)
    throw std::runtime_error("spin cannot translate mount");
  if(multi && last_a<0 && p.fixed_goal[0] && std::abs(std::remainder(p.start[2]-p.goal[2],2*pi))>1e-6)
    throw std::runtime_error("fixed formation yaw in O/S");
  for(int k=0;k<p.count;++k) {
    if(p.stop && p.stop(p.opaque)) throw std::runtime_error("NLP assembly deadline");
    HJPhase& ph=p.phase[k];
    const int N=ph.spans;
    bool translation_chart=multi && !p.general_chart && std::abs(std::remainder(p.start[2]-p.goal[2],2*pi))<1e-9;
    for(int i=1;i<p.bodies && translation_chart;++i)
      translation_chart=(k>0 || std::abs(std::remainder(p.start[3]-p.start[3+i],pi))<1e-9) &&
        (k+1<p.count || !p.fixed_goal[1+i] || (p.fixed_goal[1] && std::abs(std::remainder(p.goal[3]-p.goal[3+i],pi))<1e-9));
    // A fixed-F-yaw translating chart is valid inside a mixed graph only
    // when the other A phases share that chart. Otherwise retain all F DOFs.
    if(p.count>1 && std::count_if(p.phase,p.phase+p.count,[](const HJPhase& a){return a.mode==0;})!=1)
      translation_chart=false;

    const double time_scale=std::max(1.,ph.tau);
    const SX tau=time_scale*variable(ph.tau/time_scale,std::max(.04,p.min_motion_duration)/time_scale,p.duration_cap/time_scale);
    tau_sum+=tau;
    outputs.push_back(tau);
    std::array<SX,3> h0=h;
    auto entry=prior,exit=prior;
    SX endx=x,endy=y,endpsi=yaw;
    if(ph.mode<2) {
      const double gx=ph.coefficient[N-1][0][5],gy=ph.coefficient[N-1][1][5];
      endx=k==last_xy?parameter(p.goal[0]):(p.movable?variable(gx):SX(gx));
      endy=k==last_xy?parameter(p.goal[1]):(p.movable?variable(gy):SX(gy));
    }
    if(ph.mode==0 && multi && !translation_chart) {
      const double psi=ph.coefficient[N-1][2][5];
      endpsi=k==last_a && p.fixed_goal[0]?parameter(p.goal[2]):(p.movable?variable(psi):SX(psi));
    }

    std::vector<Coordinates> blocks(N);
    std::array<std::vector<SX>,3> channel;
    for(int j=0;j<N;++j) {
      auto& b=blocks[j];
      for(int axis=0;axis<3;++axis) {
        const SX start=axis==0?x:axis==1?y:yaw, end=axis==0?endx:axis==1?endy:endpsi;
        if(axis==2 && p.route_yaw_chart && ph.mode==0 && !translation_chart) {
          for(int i=0;i<6;++i)b[axis][i]=SX(ph.coefficient[j][axis][i]);
          continue;
        }
        if(ph.mode==2 || (axis==2 && (!multi || translation_chart || ph.mode!=0))) { b[axis].fill(start);continue; }
        if(ph.mode==1 && N==1){for(int i=0;i<6;++i)b[axis][i]=start+(end-start)*i/5.;continue;}
        if(N>1 && (p.general_chart || ph.mode==1)){
          if(j==0)channel[axis]=continuous_axis(ph,axis,start,end);
          std::copy_n(channel[axis].begin()+5*j,6,b[axis].begin());
          for(int i=1;i<=5;++i)if(j+1<N || i<5)
            cost+=.65*pow(b[axis][i]-parameter(ph.coefficient[j][axis][i]),2);
          for(int i=0;i<4;++i)cost+=.025*pow(b[axis][i+2]-2*b[axis][i+1]+b[axis][i],2);
          continue;
        }
        b[axis][0]=j?blocks[j-1][axis][5]:start;
        for(int i=1;i<=5;++i) {
          const double seed=ph.coefficient[j][axis][i];
          if(j==N-1 && i==5) {b[axis][i]=end;continue;}
          b[axis][i]=variable(seed);
          // Spatial regularization selects a channel among the native search seeds.
          cost+=(N>1?.65:.003)*pow(b[axis][i]-parameter(seed),2);
        }
        if(j) {
          const auto& a=blocks[j-1][axis];
          constraint(b[axis][1]-b[axis][0]-a[5]+a[4],0,0);
          constraint(b[axis][2]-2*b[axis][1]+b[axis][0]-a[5]+2*a[4]-a[3],0,0);
          constraint(b[axis][3]-3*b[axis][2]+3*b[axis][1]-b[axis][0]-a[5]+3*a[4]-3*a[3]+a[2],0,0);
        }
        for(int i=0;i<4;++i) cost+=.025*pow(b[axis][i+2]-2*b[axis][i+1]+b[axis][i],2);
      }
    }
    if(ph.mode==2) {
      for(int v=0;v<p.bodies;++v) {
        bool last_rotation=true;
        for(int j=k+1;j<p.count;++j) if(p.phase[j].mode!=1) last_rotation=false;
        h[v]=last_rotation && p.fixed_goal[v+1]?SX(p.goal[v+3]):variable(ph.h1[v]);
        SX d=h[v]-h0[v]; cost+=policy::rotation_weight*sqrt(d*d+1e-10)/policy::rotation_scale_rad;
        constraint(1.875*d/tau,-p.spin_rate*.96,p.spin_rate*.96);
        constraint(5.77351*d/(tau*tau),-p.spin_acc*.96,p.spin_acc*.96);
        constraint(60*d/(tau*tau*tau),-p.spin_jerk*.96,p.spin_jerk*.96);
        int wheel=0;
        for(double wx:{p.wheelbase/2,-p.wheelbase/2})for(double wy:{p.track/2,-p.track/2}){
          entry[v][wheel]=exit[v][wheel]=SX(std::remainder(std::atan2(wx,-wy),pi));++wheel;
        }
      }
    }
    for (int j=0;j<N && ph.mode!=2;++j) {
      if(p.stop && p.stop(p.opaque)) throw std::runtime_error("NLP assembly deadline");
      for(int v=0;v<p.bodies;++v){
        if(translation_chart && v>0 && ph.mode==0){
          const double shift=ph.gear[v]==ph.gear[0]?0:pi;
          if(j==0){equal(sin(h0[v]-h0[0]-shift),1e-8);constraint(cos(h0[v]-h0[0]-shift),.99,inf);}
          h[v]=h[0]+shift;entry[v]=entry[0];exit[v]=exit[0];continue;
        }
        for(int z=0;z<=6;++z){
          const bool large=multi && N>=3;
          if(large && z%2)continue;
          double local=z/6., station=(j+local)/N;
          const auto law=hybrid_clock::state(hybrid_clock::inverse(station,ph.cruise_ramp_fraction),ph.cruise_ramp_fraction);
          const double sd=law[1]*N,sdd=law[2]*N,sj=law[3]*N;
          const auto mount=mount_jets(blocks[j],local,p.offset[v][0],p.offset[v][1]);
          const auto& xj=mount.first; const auto& yj=mount.second;
          const auto dx=xj[1],dy=yj[1],ax=xj[2],ay=yj[2];
          const auto den=dx*dx+dy*dy+1e-14,d=sqrt(den),vel=d*sd/tau;
          const auto d1=(dx*ax+dy*ay)/d, d2=(ax*ax+ay*ay+dx*xj[3]+dy*yj[3]-d1*d1)/d;
          const auto turning=(dx*ay-dy*ax)/den;
          const auto w1=(dx*yj[3]-dy*xj[3]-2*d*d1*turning)/den;
          const auto w2=(ax*yj[3]-ay*xj[3]+dx*yj[4]-dy*xj[4]-
                         2*(d1*d1+d*d2)*turning-4*d*d1*w1)/den;
          const auto cur=ph.gear[v]*turning/d;
          const auto k1=(ph.gear[v]*w1-cur*d1)/d, k2=(ph.gear[v]*w2-2*k1*d1-cur*d2)/d;
          constraint(vel,0,p.speed*.95);
          const SX acc=(d1*sd*sd+d*sdd)/(tau*tau);
          constraint(acc,-p.acceleration*.91,p.acceleration*.91);
          const SX jerk=(d2*sd*sd*sd+3*d1*sd*sdd+d*sj)/(tau*tau*tau);
          constraint(jerk,-p.jerk*.95,p.jerk*.95);
          const double mass=((z==0 || z==6)?.5:1.)/(large?3.:6.);
          cost+=smooth_scale*policy::jerk_effort_weight*(mass*tau/(N*t0))*pow(jerk/p.jerk,2);
          if(v==0){
            const auto rx=trajectory_jet(blocks[j][0],local),ry=trajectory_jet(blocks[j][1],local);
            cost+=smooth_scale*policy::length_weight*mass*sqrt(rx[1]*rx[1]+ry[1]*ry[1]+1e-14)/p.radius;
          }
          if(p.lateral_acceleration>0)constraint(vel*turning*sd/tau,-p.lateral_acceleration*.97,p.lateral_acceleration*.97);
          if(ph.mode==1) {
            const auto angle=atan2(ph.gear[v]*(-sin(h0[v])*dx+cos(h0[v])*dy),
                                   ph.gear[v]*(cos(h0[v])*dx+sin(h0[v])*dy));
            // Under fixed F and chassis headings, every wheel sees the
            // common translational tangent. Its angular velocity is steering
            // rate, not body yaw rate. Zero speed occurs only at phase ends.
            constraint(den,1e-8,inf);
            constraint(angle,-p.steer_angle,p.steer_angle);
            constraint(vel,0.,p.wheel_speed*.97);
            const SX steer_velocity=turning*sd/tau;
            const SX steer_acceleration=(w1*sd*sd+turning*sdd)/(tau*tau);
            constraint(steer_velocity,-p.steer_rate*.97,p.steer_rate*.97);
            if(p.steer_acceleration>0)
              constraint(steer_acceleration,-p.steer_acceleration*.96,p.steer_acceleration*.96);
            cost+=4*smooth_scale*policy::steering_effort_weight*mass*(tau/(N*t0))*pow(steer_velocity/p.steer_rate,2);
            if(v==0)cost+=policy::oblique_length_weight*mass*d/p.radius;
            if(j==0 && z==0) entry[v].fill(angle);
            if(j+1==N && z==6) exit[v].fill(angle);
          }

          if(ph.mode==0) {
            constraint(den,1e-7,inf);
            constraint(cur,-.98/p.radius,.98/p.radius);
            constraint(turning*sd/tau,-p.yaw_rate*.96,p.yaw_rate*.96);
            const SX ct=k1*sd/tau;
            const SX ctt=(k2*sd*sd+k1*sdd)/(tau*tau);
            if(p.yaw_acceleration>0) constraint((w1*sd*sd+turning*sdd)/(tau*tau),-p.yaw_acceleration*.96,p.yaw_acceleration*.96);
            if(p.yaw_jerk>0) constraint((w2*sd*sd*sd+3*w1*sd*sdd+turning*sj)/(tau*tau*tau),-p.yaw_jerk*.96,p.yaw_jerk*.96);
            int wheel=0;
            for(double wx:{p.wheelbase/2,-p.wheelbase/2})for(double wy:{p.track/2,-p.track/2}){
              const SX a=1.-wy*cur,b=wx*cur,denom=a*a+b*b;
              const SX angle=atan2(b,a),rate=wx*ct/denom;
              if(j==0 && z==0)entry[v][wheel]=angle;
              if(j+1==N && z==6)exit[v][wheel]=angle;
              ++wheel;
              // Front/rear wheels are symmetric in magnitude on each side.
              if(wx<0)continue;
              constraint(rate,-.96*p.steer_rate,.96*p.steer_rate);
              constraint(vel*sqrt(denom),0,.96*p.wheel_speed);
              if(p.steer_acceleration>0){
                SX accel=wx*ctt/denom-2*wx*(-wy*a+wx*b)*ct*ct/(denom*denom);
                constraint(accel,-.96*p.steer_acceleration,.96*p.steer_acceleration);
              }
              cost+=2*smooth_scale*policy::steering_effort_weight*(mass*tau/(N*t0))*pow(rate/p.steer_rate,2);
            }
            if(j==0 && z==0) direction(dx,dy,h0[v],ph.gear[v]);
            if(j==N-1 && z==6) h[v]=atan2(ph.gear[v]*dy,ph.gear[v]*dx);
            cost+=.002*cur*cur;
          }
        }
      }
    }
    for(int i=0;i<p.curvature_station_count;++i){
      const auto& cut=p.curvature_stations[i];if(cut.phase!=k)continue;
      // A moving formation shares F, but each independently oriented body
      // has its own mount tangent, so a local shape repair constrains all of them.
      for(int v=0;v<p.bodies;++v){
        const auto [mx,my]=mount_jets(blocks[cut.span],cut.local,p.offset[v][0],p.offset[v][1]);
        const SX den=mx[1]*mx[1]+my[1]*my[1]+1e-14;
        constraint(den,1e-7,inf);
        const SX cur=(mx[1]*my[2]-my[1]*mx[2])/(den*sqrt(den));
        constraint(cur,-.98/p.radius,.98/p.radius);
      }
    }
    for(int i=0;i<p.sep_count;++i){
      const auto& plane=p.separations[i];if(plane.phase!=k)continue;
      const double at=plane.station/6.;
      const auto mounted=mount_jets(blocks[plane.span],at,plane.ox,plane.oy);
      const auto mx=mounted.first[0],my=mounted.second[0];
      SX heading=trajectory_jet(blocks[plane.span][2],at)[0];
      if(plane.vehicle>=0){
        if(ph.mode==0)heading=atan2(mounted.second[1],mounted.first[1]);
        else heading=h0[plane.vehicle];
      }
      if(plane.other==HJSeparation::map_cell) {
        const SX c=cos(heading),s=sin(heading);
        const SX dx=mx-parameter(plane.other_x),dy=my-parameter(plane.other_y);
        const double hx=plane.other_half_length,hy=plane.other_half_width;
        const auto above=[](const SX& v){return sqrt(v*v+1e-12);};
        const auto below=[&](const SX& v){return above(v)-1e-6;};
        // Conservative smooth lower bound on max(a,b), preserving collision semantics.
        const auto either=[&](const SX& a,const SX& b){return (a+b+below(a-b))*.5;};
        const SX gx=below(dx)-hx-plane.half_length*above(c)-plane.half_width*above(s);
        const SX gy=below(dy)-hy-plane.half_length*above(s)-plane.half_width*above(c);
        const SX gl=below(c*dx+s*dy)-plane.half_length-hx*above(c)-hy*above(s);
        const SX gw=below(-s*dx+c*dy)-plane.half_width-hx*above(s)-hy*above(c);
        constraint(either(either(gx,gy),either(gl,gw)),.00001,inf);
        continue;
      }
      SX forward=plane.nx*cos(heading)+plane.ny*sin(heading),side=-plane.nx*sin(heading)+plane.ny*cos(heading);
      SX gap=plane.nx*mx+plane.ny*my-parameter(plane.obstacle_support)-plane.half_length*sqrt(forward*forward+1e-12)-plane.half_width*sqrt(side*side+1e-12);
      if(plane.other>=0){
        const auto other=mount_jets(blocks[plane.span],at,plane.other_x,plane.other_y);
        const SX other_heading=ph.mode==0?atan2(other.second[1],other.first[1]):h0[plane.other];
        const SX along=plane.nx*cos(other_heading)+plane.ny*sin(other_heading);
        const SX across=-plane.nx*sin(other_heading)+plane.ny*cos(other_heading);
        gap-=plane.nx*other.first[0]+plane.ny*other.second[0]+plane.other_half_length*sqrt(along*along+1e-12)+plane.other_half_width*sqrt(across*across+1e-12);
      }
      constraint(gap,.00001,inf);
    }
    // Reserve protocol plus wheel preparation on the same task clock. At
    // a straight initial A pose this conservative reservation is released
    // when assembling the exact certified plan.
    const SX travel=variable(.5,.0,p.duration_cap);
    for(int v=0;v<p.bodies;++v)for(int w=0;w<4;++w){
      const SX d=entry[v][w]-prior[v][w];
      constraint(1.875*d-p.steer_rate*travel,-inf,0);
      constraint(-1.875*d-p.steer_rate*travel,-inf,0);
      if(p.steer_acceleration>0){
        constraint(5.773503*d-p.steer_acceleration*travel*travel,-inf,0);
        constraint(-5.773503*d-p.steer_acceleration*travel*travel,-inf,0);
      }
    }
    tau_sum+=travel+p.prepare_time;
    prior=exit;
    for(const SX& heading:h0) outputs.push_back(heading);
    for(const SX& heading:h) outputs.push_back(heading);
    for(const auto& block:blocks) for(const auto& axis:block) for(const SX& c:axis) outputs.push_back(c);
    x=endx;y=endy;yaw=endpsi;
  }
  for(int v=0;v<p.bodies;++v) if(p.fixed_goal[v+1]) {
    if(p.terminal_heading_window>0){
      constraint(sin(h[v]-p.goal[v+3]),-std::sin(p.terminal_heading_window),std::sin(p.terminal_heading_window));
    }else equal(sin(h[v]-p.goal[v+3]));
    constraint(cos(h[v]-p.goal[v+3]),.999,inf);
  }
  constraint(tau_sum,0,p.duration_cap);
  cost+=smooth_scale*policy::time_weight*tau_sum/t0;
  const auto design=SX::vertcat(vars);
  const auto params=SX::vertcat(parameters);
  const auto factor=SX::vertcat(cons),output=SX::vertcat(outputs);
  if (audit_) {
    auto bound=SX::substitute(SX::vertcat({cost,factor}),params,SX(values));
    differential_check(design,bound,guess,*audit_);
    return;
  }
  if(p.stop(p.opaque)) throw std::runtime_error("cancelled or deadline before NLP");
  // Complex one-shot route graphs are already bound to a map/channel by their
  // local active planes. Avoid serializing these transient multi-span tapes.
  const bool reusable=p.count==1 && p.phase[0].spans==1;
  // CasADi serialization includes the exact operation graph, all constants,
  // outputs and sparse structure. Numeric parameters and bounds are supplied
  // again on each request. A cache hit only reuses compiled functions.
  auto identity=reusable?casadi::Function("structure",{design,params},{cost,factor,output}).serialize():std::string{};
  auto it=std::find_if(templates.begin(),templates.end(),[&](auto& t){return reusable && t.key==identity;});
  if(it==templates.end()){
    CompiledGraph next;next.key=std::move(identity);
    next.stop=std::make_unique<StopCallback>(vars.size(),cons.size());
    casadi::Dict op;
    op["print_time"]=false;op["error_on_fail"]=false;op["ipopt.sb"]="yes";op["ipopt.print_level"]=0;
    op["ipopt.max_iter"]=100;op["ipopt.hessian_approximation"]="limited-memory";
    op["ipopt.tol"]=1e-7;op["ipopt.acceptable_tol"]=1e-5;
    op["ipopt.max_wall_time"]=12.;op["iteration_callback"]=*next.stop;
    const casadi::SXDict graph{{"x",design},{"p",params},{"f",cost},{"g",factor}};
    next.solver=casadi::nlpsol("joint","ipopt",graph,op);
    next.expand=casadi::Function("expand",{design,params},{output});
    if(templates.size()>=2)templates.pop_front();
    templates.push_back(std::move(next));it=std::prev(templates.end());
  }
  const auto solve_start=std::chrono::steady_clock::now();
  p.nlp_build_s=std::chrono::duration<double>(solve_start-build_start).count();
  if(p.stop(p.opaque)) throw std::runtime_error("cancelled or deadline building NLP");
  it->stop->active=&p;
  struct Detach{StopCallback& callback;~Detach(){callback.active=nullptr;}} detach{*it->stop};
  casadi::DMDict input{{"x0",guess},{"p",values},{"lbx",lb},{"ubx",ub},{"lbg",cl},{"ubg",cu}};
  if(it->last_parameters==values && it->last_guess==guess && !it->last_x.is_empty()){
    input["x0"]=it->last_x;input["lam_x0"]=it->last_dual;input["lam_g0"]=it->last_multipliers;
  }
  auto answer=it->solver(input);
  p.nlp_solve_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-solve_start).count();
  const std::string status=it->solver.stats().at("return_status").to_string();
  const bool converged=status=="Solve_Succeeded" || status=="Solved_To_Acceptable_Level";
  if(!converged && status!="Maximum_Iterations_Exceeded" && status!="Search_Direction_Becomes_Too_Small")
    throw std::runtime_error(status);
  // An iteration-limited primal may already satisfy all physical constraints.
  // It remains a research candidate and is independently audited from scratch.
  if(!converged) std::snprintf(p.primal_status,sizeof p.primal_status,"%s",status.c_str());
  it->last_parameters=values;it->last_guess=guess;it->last_x=answer.at("x");
  it->last_dual=answer.at("lam_x");it->last_multipliers=answer.at("lam_g");
  auto coefficients=it->expand(std::vector<DM>{it->last_x,DM(values)}).at(0).nonzeros();
  std::size_t o=0;
  for(int k=0;k<p.count;++k){
    auto& ph=p.phase[k];ph.tau=coefficients[o++];
    for(auto& h:ph.h0) h=coefficients[o++];
    for(auto& h:ph.h1) h=coefficients[o++];
    for(int j=0;j<ph.spans;++j)for(auto& axis:ph.coefficient[j])for(double& c:axis)c=coefficients[o++];
    if (!converged) restore_joins(ph);
  }
}
} // namespace

extern "C" int hj_optimize(HJProblem* p,char* reason,int capacity) {
  try { Problem problem(*p);problem.run();return 1; }
  catch(const std::exception& e){std::snprintf(reason,capacity,"%s",e.what());return 0;}
}

extern "C" int hj_audit_derivatives(HJProblem* p,HJDerivativeAudit* audit,char* reason,int capacity) {
  try {Problem problem(*p,audit);problem.run();return audit->failed==0 && audit->nonfinite==0;}
  catch(const std::exception& e){std::snprintf(reason,capacity,"%s",e.what());return 0;}
}
