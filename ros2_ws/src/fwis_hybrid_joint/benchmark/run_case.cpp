// Offline test adapter. In solve mode the input has no witness, label or case identity.
#include "fwis_hybrid_joint/model.hpp"
#include "../src/clock.hpp"
#include "../src/rigid_polygon.hpp"
#include <json/json.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace hj = fwis_hybrid_joint;
using Json::Value;
namespace {
template <typename T, std::size_t N>
std::array<T,N> array(const Value& value) {
  if (!value.isArray() || value.size() != N) throw std::invalid_argument("invalid array length");
  std::array<T,N> out{};
  for (unsigned i=0; i<N; ++i) out[i] = static_cast<T>(value[i].asDouble());
  return out;
}
template <typename T> Value json_array(const T& values) {
  Value out(Json::arrayValue);
  for (auto x: values) out.append(x);
  return out;
}
hj::Boundary boundary(const Value& v) {
  hj::Boundary b;
  auto q=array<double,3>(v["reference"]);
  b.reference={q[0],q[1],q[2]};
  b.heading=array<double,3>(v["heading"]);
  b.constrain_reference_yaw=v["constrain_reference_yaw"].asBool();
  for (unsigned i=0;i<3;++i) b.constrain_heading[i]=v["constrain_heading"][i].asBool();
  return b;
}
hj::Request request(const Value& v, bool footprint_only) {
  hj::Request r;
  auto& g=v["grid"];
  unsigned n=g["width"].asUInt(), m=g["height"].asUInt();
  if (!n || !m || n>8192 || m>8192) throw std::invalid_argument("invalid map size");
  std::vector<uint8_t> cells(n*m,0);
  for (auto& run:g["occupied_runs"]) {
    unsigned start=run[0].asUInt(), count=run[1].asUInt();
    if (start>cells.size() || count>cells.size()-start) throw std::invalid_argument("invalid RLE");
    std::fill_n(cells.begin()+start,count,100);
  }
  r.grid=std::make_shared<hj::GridSnapshot>(n,m,g["resolution"].asDouble(),
    fwis_planning_core::Point2d{g["origin"][0].asDouble(),g["origin"][1].asDouble()},std::move(cells));
  for (auto& car:v["vehicles"]) r.vehicles.push_back(array<double,4>(car));
  if (!v["cargo"].isNull()) r.cargo=array<double,4>(v["cargo"]);
  if (footprint_only) return r;
  r.start=boundary(v["start"]); r.goal=boundary(v["goal"]);
  r.allow_reverse=v["allow_reverse"].asBool();
  r.task_duration_cap=v["task_duration_cap"].asDouble();
  r.align_terminal_chassis=v.get("align_terminal_chassis",false).asBool();
  const auto& l=v["limits"];
#define READ_LIMIT(field) r.limits.field=l[#field].asDouble()
  READ_LIMIT(radius); READ_LIMIT(speed); READ_LIMIT(acceleration); READ_LIMIT(jerk);
  READ_LIMIT(lateral_acceleration); READ_LIMIT(spin_speed); READ_LIMIT(spin_acceleration);
  READ_LIMIT(spin_jerk); READ_LIMIT(yaw_rate); READ_LIMIT(yaw_acceleration); READ_LIMIT(yaw_jerk);
  READ_LIMIT(wheelbase); READ_LIMIT(track); READ_LIMIT(wheel_speed); READ_LIMIT(steer_angle);
  READ_LIMIT(steer_rate); READ_LIMIT(steer_acceleration); READ_LIMIT(prepare_time);
  if (l.isMember("min_motion_duration")) READ_LIMIT(min_motion_duration);
  if (l.isMember("cargo_center_speed")) READ_LIMIT(cargo_center_speed);
  if (l.isMember("high_steer_angle")) READ_LIMIT(high_steer_angle);
  if (l.isMember("high_steer_speed")) READ_LIMIT(high_steer_speed);
#undef READ_LIMIT
  return r;
}
hj::CargoPolygon cargo_polygon(const Value& v) {
  if (!v.isNull() && (!v.isArray() || v.size()<3 || v.size()>256))
    throw std::invalid_argument("cargo_polygon requires 3 to 256 ordered vertices");
  hj::CargoPolygon out;
  for (const auto& vertex:v) {
    const auto p=array<double,2>(vertex);
    out.push_back({p[0],p[1]});
  }
  return out;
}
Value check_footprint(const hj::Request& r,const hj::CargoPolygon& polygon,const Value& input) {
  const auto count=r.vehicles.size();
  if (count<1 || count>hj::Headings{}.size())
    throw std::invalid_argument("expected one to three vehicles");
  const auto& poses=input["poses"];
  if (!poses.isArray() || poses.empty())
    throw std::invalid_argument("expected non-empty footprint poses");
  const hj::FormationCollision map(r,polygon);
  Value out;
  const double default_padding=input.get("padding_m",0.).asDouble();
  for (Json::ArrayIndex i=0; i<poses.size(); ++i) {
    const auto& p=poses[i];
    const auto columns=3+count;
    if (!p.isArray() || (p.size()!=columns && p.size()!=columns+1))
      throw std::invalid_argument("expected x, y, yaw, one heading per vehicle and optional padding");
    for (const auto& v:p) if (!v.isNumeric() || !std::isfinite(v.asDouble()))
      throw std::invalid_argument("footprint poses require finite coordinates");
    hj::Headings head{};
    for (Json::ArrayIndex j=0; j<count; ++j) head[j]=p[3+j].asDouble();
    const double padding=p.size()==columns ? default_padding : p[Json::ArrayIndex(columns)].asDouble();
    const auto hit=map.check({p[0].asDouble(),p[1].asDouble(),p[2].asDouble()},head.data(),count,padding);
    if (hit.reason!=fwis_planning_core::CollisionReason::clear) {
      out["status"]="collision";
      out["reason"]=map.reason(hit.reason);
      out["samples_checked"]=i+1;
      out["sample_index"]=i;
      out["vehicle"]=Json::UInt64(hit.first);
      return out;
    }
  }
  out["status"]="clear";
  out["reason"]="ALL_FOOTPRINTS_CLEAR";
  out["samples_checked"]=poses.size();
  return out;
}
char letter(hj::Mode m) { return m==hj::Mode::A?'A':m==hj::Mode::O?'O':'S'; }
hj::Mode mode(char c) {
  if (c=='A') return hj::Mode::A;
  if (c=='O') return hj::Mode::O;
  if (c=='S') return hj::Mode::S;
  throw std::invalid_argument("unknown mode");
}
hj::Plan plan(const Value& raw) {
  hj::Plan p;
  for (auto& v:raw) {
    hj::Phase ph;
    ph.mode=mode(v["mode"].asString().at(0));
    auto k=v["kind"].asString();
    if (k!="motion" && k!="prepare" && k!="hold") throw std::invalid_argument("unknown phase");
    ph.kind=k=="motion"?hj::Kind::Motion:k=="prepare"?hj::Kind::Prepare:hj::Kind::Hold;
    ph.gear=array<int,3>(v["gear"]);
    ph.duration=v["duration"].asDouble();
    ph.cruise_ramp_fraction=v.get("cruise_ramp_fraction",0.0).asDouble();
    ph.heading0=array<double,3>(v["heading0"]); ph.heading1=array<double,3>(v["heading1"]);
    for (auto& q:v["pieces"])
      ph.pieces.push_back({array<double,6>(q[0]),array<double,6>(q[1]),array<double,6>(q[2])});
    p.phases.push_back(std::move(ph));
  }
  return p;
}
Value encode(const hj::Plan& p) {
  Value output(Json::arrayValue);
  for (auto& ph:p.phases) {
    Value v;
    v["mode"]=std::string(1,letter(ph.mode));
    v["kind"]=ph.kind==hj::Kind::Motion?"motion":ph.kind==hj::Kind::Prepare?"prepare":"hold";
    v["cruise_ramp_fraction"]=ph.cruise_ramp_fraction;
    v["gear"]=json_array(ph.gear); v["duration"]=ph.duration;
    v["heading0"]=json_array(ph.heading0); v["heading1"]=json_array(ph.heading1);
    v["pieces"]=Value(Json::arrayValue);
    for (auto& q:ph.pieces) {
      Value piece(Json::arrayValue);
      piece.append(json_array(q.x)); piece.append(json_array(q.y)); piece.append(json_array(q.yaw));
      v["pieces"].append(piece);
    }
    output.append(v);
  }
  return output;
}
Value audit(const hj::Audit& a) {
  Value out;
  out["executable"]=a.executable; out["reason"]=a.reason;
  out["phase"]=Json::UInt64(a.phase); out["vehicle"]=Json::UInt64(a.vehicle);
  out["u0"]=a.u0; out["u1"]=a.u1;
  return out;
}
double angle_error(double a,double b) {return std::abs(std::remainder(a-b,2.*std::acos(-1.)));}
Value diagnostics(const hj::Request& r,const hj::Plan& p) {
  const size_t n=r.vehicles.size();
  Value v;
  double pos=0,yaw=0,speed=0,curvature=0,odist=0,spin=0,cruise=0,longest=0;
  unsigned internal_stops=0,aux=0;
  std::string modes;
  const hj::Phase* previous=nullptr;
  for(const auto& ph:p.phases) {
    if(ph.kind!=hj::Kind::Motion)continue;
    modes+=letter(ph.mode);
    if(previous && ph.mode==hj::Mode::A && previous->mode==hj::Mode::A &&
        std::equal(ph.gear.begin(),ph.gear.begin()+n,previous->gear.begin())) ++internal_stops;
    previous=&ph;
    if(ph.mode!=hj::Mode::A)++aux;
    if(ph.mode==hj::Mode::S)
      for(size_t b=0;b<n;++b)spin+=std::abs(ph.heading1[b]-ph.heading0[b]);
    double distance=0;
    auto last=hj::evaluate_phase(r,ph,0.0);
    const unsigned steps=static_cast<unsigned>(std::ceil(ph.duration/.04));
    bool moving=false;unsigned slow=0;
    for(unsigned j=1;j<=steps;++j) {
      const auto sample=hj::evaluate_phase(r,ph,ph.duration*j/steps);
      distance+=std::hypot(sample.reference.x-last.reference.x,sample.reference.y-last.reference.y);
      if(ph.mode==hj::Mode::A) {
        bool stopped=true;
        for(size_t b=0;b<n;++b) stopped=stopped && std::abs(sample.bodies[b].vx)<.012;
        if(stopped && moving)++slow;
        if(!stopped){if(moving && slow*ph.duration/steps>=.08)++internal_stops;moving=true;slow=0;}
      }
      last=sample;
    }
    if(ph.mode==hj::Mode::O)odist+=distance;
    if(ph.mode!=hj::Mode::A)continue;
    cruise+=distance;longest=std::max(longest,distance);
    for(size_t j=1;j<ph.pieces.size();++j) {
      const double target=double(j)/ph.pieces.size();
      double low=0,high=1;
      for(int k=0;k<60;++k){double u=(low+high)/2.;
        if(hybrid_clock::state(u,ph.cruise_ramp_fraction)[0]<target)low=u;else high=u;}
      // A symmetric physical-time probe straddles the knot to avoid rounding to the same side.
      const auto left=hj::evaluate_phase(r,ph,(low-1e-11)*ph.duration);
      const auto right=hj::evaluate_phase(r,ph,(high+1e-11)*ph.duration);
      for(size_t b=0;b<n;++b){const auto& a=left.bodies[b];const auto& c=right.bodies[b];
        pos=std::max(pos,std::hypot(a.pose.x-c.pose.x,a.pose.y-c.pose.y));
        yaw=std::max(yaw,angle_error(a.pose.yaw,c.pose.yaw));speed=std::max(speed,std::abs(a.vx-c.vx));
        curvature=std::max(curvature,std::abs(a.curvature-c.curvature));}
    }
  }
  const auto last=hj::evaluate(r,p,hj::duration(p));
  v["terminal_position_m"]=std::hypot(last.reference.x-r.goal.reference.x,last.reference.y-r.goal.reference.y);
  double endpoint_yaw=r.goal.constrain_reference_yaw?angle_error(last.reference.yaw,r.goal.reference.yaw):0.;
  for(size_t i=0;i<n;++i)if(r.goal.constrain_heading[i])
    endpoint_yaw=std::max(endpoint_yaw,angle_error(last.bodies[i].pose.yaw,r.goal.heading[i]));
  v["constrained_terminal_yaw_rad"]=endpoint_yaw;v["mode_sequence"]=modes;v["aux_blocks"]=aux;
  v["oblique_distance_m"]=odist;v["total_spin_rad"]=spin;v["a_length_m"]=cruise;v["longest_A_m"]=longest;
  v["internal_stops"]=internal_stops;v["join_position_m"]=pos;v["join_yaw_rad"]=yaw;
  v["join_speed_mps"]=speed;v["curvature_jump"]=curvature;
  return v;
}
Value run(const Value& input) {
  auto begin=hj::Clock::now();
  const auto operation=input["operation"].asString();
  auto r=request(input["request"],operation=="check_footprint");
  const auto polygon=cargo_polygon(input["request"]["cargo_polygon"]);
  if (operation=="check_footprint") return check_footprint(r,polygon,input);
  const double budget=input["budget_s"].asDouble();
  auto deadline=begin+std::chrono::duration_cast<hj::Clock::duration>(std::chrono::duration<double>(budget));
  Value out;
  if (operation=="certify")
    return audit(hj::certify_polygon(r,plan(input["plan"]),polygon,deadline));
  if (!polygon.empty())
    throw std::invalid_argument("cargo_polygon is supported by certify and check_footprint");
  if ((operation!="solve" && operation!="docking") || input.isMember("plan"))
    throw std::invalid_argument("unrecognized or non-blind solve");
  hj::SolveOptions options;
  const auto& opt=input["options"];
  if (opt.isObject()) {
    for (char c:opt["sequence"].asString()) options.initial_sequence.push_back(mode(c));
    options.enable_edits=opt["enable_edits"].asBool(); options.movable_switches=opt["movable_switches"].asBool();
  }
  auto solution=operation=="docking"?hj::solve_docking(r,deadline):hj::solve(r,deadline,{},options);
  auto wall=std::chrono::duration<double>(hj::Clock::now()-begin).count();
  out["wall_s"]=wall; out["deadline_overrun_s"]=std::max(0.,wall-budget);
  out["status"]=solution.status; out["reason"]=solution.reason;
  out["terminal_alignment"]=solution.terminal_alignment;
  out["first_certified_s"]=solution.first_certified_s;
  out["first_policy_compliant_s"]=solution.first_policy_compliant_s;
  out["trace"]=json_array(solution.trace);
  out["stage_times"]["seed_s"]=solution.seed_seconds;
  out["stage_times"]["edit_s"]=solution.edit_seconds;
  out["stage_times"]["attempts"]=Value(Json::arrayValue);
  for(const auto& a:solution.attempts) {
    Value x;
    x["proposal"]=a.proposal;x["status"]=a.status;x["nlp_build_s"]=a.nlp_build_s;
    x["nlp_solve_s"]=a.nlp_solve_s;x["certify_s"]=a.certify_s;x["wall_s"]=a.wall_s;
    out["stage_times"]["attempts"].append(x);
  }
  if (solution.plan) {
    out["plan"]=encode(*solution.plan);
    out["duration_s"]=hj::duration(*solution.plan);
    out["metrics"]=diagnostics(r,*solution.plan);
    const auto scored=hj::score_policy(r,*solution.plan,hj::Clock::now()+std::chrono::seconds(5));
    auto& m=out["metrics"];
    m["aux_blocks"]=scored.auxiliary_blocks;m["auxiliary_cost"]=scored.auxiliary_cost;m["execution_cost"]=scored.execution_cost;
    m["policy_reference_length_m"]=scored.reference_length;m["steering_effort"]=scored.steering_effort;
    m["jerk_effort"]=scored.jerk_effort;m["clearance_effort"]=scored.clearance_effort;
    out["audit"]=audit(hj::certify(r,*solution.plan,hj::Clock::now()+std::chrono::seconds(5)));
  }
  out["actual_backend"]=r.vehicles.size()==1?"single_hybrid_joint_cpp":"formation_hybrid_joint_cpp";
  return out;
}
}
int main() {
  std::string line;
  Json::CharReaderBuilder reader;
  Json::StreamWriterBuilder writer; writer["indentation"]="";
  while (std::getline(std::cin,line)) {
    try {
      Value input; std::string err;
      std::unique_ptr<Json::CharReader> parser(reader.newCharReader());
      if (!parser->parse(line.data(),line.data()+line.size(),&input,&err)) throw std::runtime_error(err);
      std::cout << Json::writeString(writer,run(input)) << std::endl;
    } catch (const std::exception& e) {
      Value error; error["status"]="input_or_runtime_error"; error["reason"]=e.what();
      std::cout << Json::writeString(writer,error) << std::endl;
    }
  }
}
