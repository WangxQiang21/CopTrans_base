// CasADi normally dlopens Ipopt with RTLD_DEEPBIND, which the sanitizer runtime
// rejects. Register the very same plugin through its official C entry point
// (linked at program startup); all project code keeps sanitizer instrumentation.
#include "../src/nlp_api.h"
#include "fwis_hybrid_joint/model.hpp"
#include <iostream>
extern "C" void casadi_load_nlpsol_ipopt();
int main() {
  casadi_load_nlpsol_ipopt();
  for(int bodies:{1,3}) {
    HJProblem p{};
    p.count=1;p.bodies=bodies;p.duration_cap=200;p.movable=1;p.start[0]=2;p.start[1]=2;
    p.goal[0]=4;p.goal[1]=2;p.fixed_goal[0]=1;
    p.radius=.9;p.speed=.5;p.acceleration=.45;p.jerk=2.;p.yaw_rate=2.;
    p.spin_rate=.5;p.spin_acc=.7;p.spin_jerk=3.;p.wheel_speed=3.;p.steer_rate=.8;p.steer_angle=1.57;
    p.wheelbase=.494;p.track=.364;p.steer_acceleration=1.5;p.lateral_acceleration=.6;p.prepare_time=.7;
    p.wall_seconds=6.;p.stop=[](void*){return 0;};
    auto &ph=p.phase[0];ph.mode=0;ph.spans=1;ph.tau=10;
    for(int i=0;i<3;++i){p.fixed_goal[i+1]=1;ph.gear[i]=1;}
    p.offset[1][0]=-.4;p.offset[1][1]=.68;p.offset[2][0]=-.4;p.offset[2][1]=-.68;
    for(int i=0;i<6;++i){ph.coefficient[0][0][i]=2.+2.*i/5.;ph.coefficient[0][1][i]=2.;}
    char reason[1000]{};
    if(!hj_optimize(&p,reason,sizeof(reason))){
      std::cerr<<bodies<<" body optimization failed: "<<reason<<'\n';return 1;
    }
    std::cout<<bodies<<" body native Ipopt solve completed\n";
  }
}
