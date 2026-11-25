#include "../src/nlp_api.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
int main() {
  int failures=0,nonfinite=0,nrows=0;
  double relative=0,absolute=0;
  constexpr int seed=178314;
  for(int bodies:{1,3}){
    HJProblem p{};
    p.bodies=bodies;p.count=3;p.movable=1;p.duration_cap=200;
    p.start[0]=p.start[1]=2;p.goal[0]=5;p.goal[1]=3;
    p.spin_rate=.55;p.spin_acc=.7;p.spin_jerk=3;p.radius=.9;p.speed=.5;
    p.acceleration=.45;p.jerk=2;p.yaw_rate=2.5;p.wheelbase=.494;p.track=.364;
    p.steer_rate=.8;p.steer_angle=1.570796;p.wheel_speed=3;
    p.steer_acceleration=1.5;p.lateral_acceleration=.6;p.yaw_acceleration=.8;p.yaw_jerk=3.;p.prepare_time=.7;
    HJSeparation features[3]={{0,0,2,0,0.,0.,.35,.25,.6,.8,-2.},
      {0,1,4,bodies==3?-1:0,.2,-.1,.7,.45,-.8,.6,-4.},
      {0,0,2.37,0,0.,0.,.35,.25,.6,.8,0.,1,-.4,.7,.35,.25}};
    p.separations=features;p.sep_count=bodies==3?3:2;
    p.general_chart=bodies==3;p.terminal_heading_window=bodies==3?.015:0.;
    p.offset[1][0]=-.4;p.offset[1][1]=.7;p.offset[2][0]=-.4;p.offset[2][1]=-.7;
    p.fixed_goal[0]=1;
    for(int i=0;i<bodies;++i) p.fixed_goal[i+1]=1;
    for(int mode=0;mode<3;++mode) {
      auto &ph=p.phase[mode];ph.mode=mode;ph.spans=mode==0?2:1;ph.tau=8.;
      for(int i=0;i<3;++i){ph.gear[i]=1;ph.h0[i]=.03;ph.h1[i]=.035;}
      for(int j=0;j<ph.spans;++j) for(int i=0;i<6;++i){
        double t=i/5.;
        ph.coefficient[j][0][i]=2+2*mode+j+t;
        ph.coefficient[j][1][i]=2+.25*(j+t)+.07*t*t;
        ph.coefficient[j][2][i]=.02*(j+t);
      }
    }
    HJDerivativeAudit a{};a.seed=seed+bodies;a.cases=128;
    char reason[500]{};const bool ok=hj_audit_derivatives(&p,&a,reason,sizeof reason);
    if(!ok) std::cerr<<"derivative failure for "<<bodies<<" vehicles: "<<reason<<" failed "<<a.failed<<" rows "<<a.rows<<"\n";
    failures+=a.failed+(!ok&&!a.failed&&!a.nonfinite);nonfinite+=a.nonfinite;nrows+=a.rows;
    absolute=std::max(absolute,a.max_near_zero_absolute_error);relative=std::max(relative,a.max_relative_error);
  }
  std::cout<<std::setprecision(15)<<"{\"seed_base\":"<<seed<<",\"cases_per_row\":128,\"assembled_rows\":"<<nrows
           <<",\"step_sizes\":[0.001,0.0001,0.00001,0.000001,0.0000001],\"nonfinite\":"<<nonfinite<<",\"failures\":"<<failures
           <<",\"max_relative_error\":"<<relative<<",\"max_near_zero_absolute_error\":"<<absolute<<"}\n";
  return failures||nonfinite;
}
