#pragma once
#include "fwis_hybrid_joint/model.hpp"
#include <cmath>

namespace fwis_hybrid_joint::search_detail {
using Point=fwis_planning_core::Point2d;
using Route=std::vector<Point>;
constexpr double pi=3.14159265358979323846;
inline double delta(double a,double b){return std::remainder(b-a,2*pi);}
inline double distance(Point a,Point b){return std::hypot(a.x-b.x,a.y-b.y);}
struct Run {
  Clock::time_point end;
  const Cancel& cancel;
  bool stop()const{return Clock::now()>=end || (cancel && cancel());}
};
Point interpolate(const Route&,double fraction);
double central_tangent(const Route&);
Piece stationary_piece(const Pose2d&);
void append_motion(const Request&,Plan&,Phase);
Phase stationary_spin(const Request&,const Sample&,const Headings&);
Phase straight_motion(const Request&,const Pose2d&,const Pose2d&,int gear);
std::optional<Plan> single_cruise(const Request&,const Route& guide,const Run&);
std::optional<Plan> single_corridor_cruise(const Request&,const Route& guide,const Run&);
std::optional<Plan> single_narrow_docking(const Request&,const Route& guide,const Run&);
void align_terminal(const Request&,const Run&,Result&);
std::optional<Plan> search_formation_channel(const Request&,const Run&,Result&,Clock::time_point& first_proof);
}
