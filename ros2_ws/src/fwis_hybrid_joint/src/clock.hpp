#pragma once
#include <algorithm>
#include <array>

// Normalized position and its first three derivatives with respect to time
// fraction. No ABI-dependent types cross the CasADi/ROS boundary.
namespace hybrid_clock {
inline std::array<double,4> single_ramp(double u,double ramp) {
  u=std::clamp(u,0.,1.);
  if(ramp==0) {
    const double s=(u<=.5?u:1-u);
    const double pos=s*s*s*(10+s*(-15+6*s));
    return {u<=.5?pos:1-pos,30*u*u*(1-u)*(1-u),60*u*(1-u)*(1-2*u),60-360*u+360*u*u};
  }
  const double peak=1/(1-ramp);
  if(u>ramp && u<1-ramp)return {(u-.5*ramp)*peak,peak,0.,0.};
  const bool tail=u>.5;
  const double r=(tail?1-u:u)/ramp;
  const double pos=ramp*peak*r*r*r*(1-.5*r);
  return {tail?1-pos:pos,peak*r*r*(3-2*r),
          (tail?-1:1)*6*peak*r*(1-r)/ramp,6*peak*(1-2*r)/(ramp*ramp)};
}
// The older 1/8 and quintic clocks retain their original meaning. Narrow
// ramps keep a 1/8-weight wide shoulder on acceleration and deceleration.
// The positive combination is monotone and preserves endpoint velocity and acceleration.
inline std::array<double,4> state(double u,double ramp) {
  auto result=single_ramp(u,ramp);
  if(ramp>0 && ramp<.125){
    const auto shoulder=single_ramp(u,.125);
    for(std::size_t i=0;i<result.size();++i)result[i]=.875*result[i]+.125*shoulder[i];
  }
  return result;
}
inline double inverse(double s,double ramp) {
  if(s<=0 || s>=1)return std::clamp(s,0.,1.);
  if(ramp==.125 && s>=ramp/(2*(1-ramp)) && s<=1-ramp/(2*(1-ramp)))
    return s*(1-ramp)+.5*ramp;
  if(ramp>0 && ramp<.125){
    const double edge=state(.125,ramp)[0];
    if(s>=edge && s<=1-edge)return .5+(s-.5)/state(.5,ramp)[1];
  }
  double lo=0,hi=1;
  for(int i=0;i<42;++i){const double u=(lo+hi)*.5;if(state(u,ramp)[0]<s)lo=u;else hi=u;}
  return (lo+hi)*.5;
}
}
