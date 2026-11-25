#include "fwis_hybrid_joint_ros/transport.hpp"
#include <openssl/sha.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace hybrid_ros
{
namespace
{
using J = Json::Value;
using LimitMember = double hj::Limits::*;
const std::pair<const char *, LimitMember> limit_fields[] = {
  {"radius", &hj::Limits::radius}, {"speed", &hj::Limits::speed}, {"acceleration", &hj::Limits::acceleration},
  {"jerk", &hj::Limits::jerk}, {"lateral_acceleration", &hj::Limits::lateral_acceleration},
  {"spin_speed", &hj::Limits::spin_speed}, {"spin_acceleration", &hj::Limits::spin_acceleration},
  {"spin_jerk", &hj::Limits::spin_jerk}, {"yaw_rate", &hj::Limits::yaw_rate},
  {"yaw_acceleration", &hj::Limits::yaw_acceleration},
  {"yaw_jerk", &hj::Limits::yaw_jerk}, {"wheelbase", &hj::Limits::wheelbase}, {"track", &hj::Limits::track},
  {"wheel_speed", &hj::Limits::wheel_speed}, {"steer_angle", &hj::Limits::steer_angle},
  {"steer_rate", &hj::Limits::steer_rate},
  {"steer_acceleration", &hj::Limits::steer_acceleration}, {"prepare_time", &hj::Limits::prepare_time}, {"min_motion_duration", &hj::Limits::min_motion_duration}};
void require(bool condition, const char * reason)
{
  if (!condition) {
    throw std::invalid_argument(reason);
  }
}
template<class V> J array(const V & data)
{
  J a(Json::arrayValue);
  for (auto v:data) {
    a.append(v);
  }
  return a;
}
double number(const J & j)
{
  require(j.isNumeric() && std::isfinite(j.asDouble()), "invalid numeric model field");
  return j.asDouble();
}
template<class T, size_t N> std::array<T, N> fixed(const J & j)
{
  require(j.isArray() && j.size() == N, "invalid vector length");
  std::array<T, N> a{};
  for (unsigned i = 0; i < N; ++i) {
    if constexpr (std::is_same_v<T, bool>) {
      require(j[i].isBool(), "invalid boolean");
      a[i] = j[i].asBool();
    } else if constexpr (std::is_same_v<T, int>) {
      require(j[i].isInt(), "invalid integer");
      a[i] = j[i].asInt();
    } else {
      a[i] = number(j[i]);
    }
  }
  return a;
}
J boundary(const hj::Boundary & b)
{
  J j;
  j["reference"] = array(std::array<double, 3>{b.reference.x, b.reference.y, b.reference.yaw});
  j["heading"] = array(b.heading);
  j["reference_yaw_constrained"] = b.constrain_reference_yaw;
  j["headings_constrained"] = array(b.constrain_heading);
  return j;
}
hj::Boundary decode_boundary(const J & j)
{
  hj::Boundary b;
  auto p = fixed<double, 3>(j["reference"]);
  b.reference = {p[0], p[1], p[2]};
  b.heading = fixed<double, 3>(j["heading"]);
  require(j["reference_yaw_constrained"].isBool(), "invalid yaw constraint");
  b.constrain_reference_yaw = j["reference_yaw_constrained"].asBool();
  b.constrain_heading = fixed<bool, 3>(j["headings_constrained"]);
  return b;
}
J phase(const hj::Phase & p)
{
  J j;
  j["kind"] = int(p.kind);
  j["mode"] = int(p.mode);
  j["duration"] = p.duration;
  j["cruise_ramp_fraction"] = p.cruise_ramp_fraction;
  j["gear"] = array(p.gear);
  j["heading0"] = array(p.heading0);
  j["heading1"] = array(p.heading1);
  j["pieces"] = J(Json::arrayValue);
  for (const auto & piece:p.pieces) {
    J s;
    s["x"] = array(piece.x);
    s["y"] = array(piece.y);
    s["yaw"] = array(piece.yaw);
    j["pieces"].append(s);
  }
  return j;
}
hj::Phase decode_phase(const J & j, size_t vehicles, bool piecewise_clock)
{
  hj::Phase p;
  for (auto key:{"kind", "mode"}) {
    require(j[key].isInt() && j[key].asInt() >= 0 && j[key].asInt() <= 2, "invalid phase tag");
  }
  p.kind = hj::Kind(j["kind"].asInt());
  p.mode = hj::Mode(j["mode"].asInt());
  p.gear = fixed<int, 3>(j["gear"]);
  for (size_t i = 0; i < p.gear.size(); ++i) {
    require(p.gear[i] == 1 || p.gear[i] == -1 || (i >= vehicles && p.gear[i] == 0), "invalid gear");
  }
  p.cruise_ramp_fraction = j.isMember("cruise_ramp_fraction") ? number(j["cruise_ramp_fraction"]) : 0.;
  require(p.cruise_ramp_fraction == 0. ||
    (piecewise_clock && p.cruise_ramp_fraction == .125 && p.kind == hj::Kind::Motion && p.mode != hj::Mode::S),
    "unsupported phase clock");
  p.duration = number(j["duration"]);
  require(p.duration > 0. && p.duration <= 86400., "invalid phase time");
  p.heading0 = fixed<double, 3>(j["heading0"]);
  p.heading1 = fixed<double, 3>(j["heading1"]);
  const auto & pieces = j["pieces"];
  require(pieces.isArray() && pieces.size() > 0 && pieces.size() <= 1024, "invalid phase mesh");
  for (const auto & s:pieces) {
    p.pieces.push_back({fixed<double, 6>(s["x"]), fixed<double, 6>(s["y"]), fixed<double, 6>(s["yaw"])});
  }
  return p;
}
}
std::string build_digest()
{
  return HYBRID_BUILD_DIGEST;
}
std::string digest(const std::string & s)
{
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(s.data()), s.size(), hash);
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (auto byte:hash) {
    stream << std::setw(2) << unsigned(byte);
  }
  return stream.str();
}
std::string json(const J & j)
{
  Json::StreamWriterBuilder w;
  w["indentation"] = "";
  w["precision"] = 17;
  return Json::writeString(w, j);
}
J parse(const std::string & s)
{
  if (s.size() > 16000000) {
    throw std::invalid_argument("oversized model");
  }
  Json::CharReaderBuilder b;
  b["rejectDupKeys"] = true;
  b["allowComments"] = false;
  b["failIfExtra"] = true;
  b["stackLimit"] = 64;
  auto reader = std::unique_ptr<Json::CharReader>(b.newCharReader());
  J j;
  std::string err;
  require(reader->parse(s.data(), s.data() + s.size(), &j, &err), "invalid JSON");
  return j;
}
J contract(const Context & c)
{
  const auto & r = c.request;
  require(bool(r.grid) && !r.grid->hash().empty(), "missing map identity");
  J j;
  j["grid"] = r.grid->hash();
  j["occupancy"] = "nonzero_or_outside_occupied";
  j["id"] = c.id;
  j["formation_id"] = c.formation_id;
  j["vehicle_ids"] = array(c.ids);
  j["vehicles"] = J(Json::arrayValue);
  for (const auto & v:r.vehicles) {
    j["vehicles"].append(array(v));
  }
  j["cargo"] = r.cargo ? array(*r.cargo) : J();
  j["start"] = boundary(r.start);
  j["goal"] = boundary(r.goal);
  j["allow_reverse"] = r.allow_reverse;
  j["task_duration_cap"] = r.task_duration_cap;
  for (auto [name, member]:limit_fields) {
    j["limits"][name] = r.limits.*member;
  }
  return j;
}
Envelope encode(const Context & c,const hj::Plan & p,int64_t epoch)
{
  return encode(c,p,epoch,0);
}
Envelope encode(const Context & c, const hj::Plan & p, int64_t epoch, int64_t publication_steady_ns)
{
  require(c.id && epoch > 0, "invalid plan identity");
  J j;
  j["schema"] = schema;
  j["numeric_contract"] = "bernstein5/unwrapped_yaw/phase_clock/v2";
  j["certificate_version"] = "hybrid_continuous/v1";
  j["solver_build"] = HYBRID_BUILD_DIGEST;
  j["request"] = contract(c);
  j["epoch_ns"] = Json::Int64(epoch);
  j["publication_steady_ns"] = Json::Int64(publication_steady_ns);
  j["phases"] = J(Json::arrayValue);
  for (const auto & v:p.phases) {
    j["phases"].append(phase(v));
  }
  Envelope m;
  m.header.frame_id = "map";
  m.header.stamp = rclcpp::Time(epoch).operator builtin_interfaces::msg::Time();
  m.plan_id = c.id;
  m.formation_id = c.formation_id;
  m.backend = "hybrid_joint";
  m.model_version = schema;
  m.semantic_version = "hybrid_joint_transport/v1";
  m.certification = "native_continuous";
  m.model_json = json(j);
  m.model_sha256 = digest(m.model_json);
  return m;
}
Decoded decode(const Envelope & m, std::shared_ptr<const hj::GridSnapshot> grid, const Context * expected)
{
  require(
    m.header.frame_id == "map" && m.backend == "hybrid_joint" && m.model_version == schema && m.semantic_version == "hybrid_joint_transport/v1" &&
    m.certification == "native_continuous" && m.plan_id && ns(
      m.header.stamp) > 0 && m.header.stamp.nanosec < 1000000000, "invalid envelope");
  require(m.model_json.size() <= 16000000 && digest(m.model_json) == m.model_sha256, "model digest mismatch");
  auto j = parse(m.model_json);
  const auto & v = j["request"];
  const bool piecewise_clock = j["numeric_contract"] == "bernstein5/unwrapped_yaw/phase_clock/v2";
  require(
    j["schema"] == schema && (piecewise_clock || j["numeric_contract"] == "bernstein5/unwrapped_yaw/quintic_time/v1") &&
    j["certificate_version"] == "hybrid_continuous/v1" && j["solver_build"] == HYBRID_BUILD_DIGEST &&
    j["epoch_ns"].isInt64() && j["epoch_ns"].asInt64() == ns(m.header.stamp), "model contract mismatch");
  require(
    grid && v["grid"] == grid->hash() && v["occupancy"] == "nonzero_or_outside_occupied" &&
    v["id"].isUInt() && v["id"].asUInt() == m.plan_id && v["formation_id"] == m.formation_id,
    "request identity mismatch");
  if (expected) {
    require(json(v) == json(contract(*expected)), "local request contract mismatch");
  }
  Decoded out;
  require(
    j["publication_steady_ns"].isInt64() && j["publication_steady_ns"].asInt64() >= 0,
    "invalid publication timestamp");
  auto & c = out.context;
  auto & r = c.request;
  c.id = m.plan_id;
  c.formation_id = m.formation_id;
  r.grid = std::move(grid);
  require(
    v["vehicle_ids"].isArray() && (v["vehicle_ids"].size() == 1 || v["vehicle_ids"].size() == 3) &&
    v["vehicles"].isArray() && v["vehicles"].size() == v["vehicle_ids"].size(), "invalid vehicles");
  for (unsigned i = 0; i < v["vehicle_ids"].size(); ++i) {
    require(v["vehicle_ids"][i].isUInt() && v["vehicle_ids"][i].asUInt() == i + 1, "unknown vehicle");
    c.ids.push_back(i + 1);
    r.vehicles.push_back(fixed<double, 4>(v["vehicles"][i]));
  }
  if (!v["cargo"].isNull()) {
    r.cargo = fixed<double, 4>(v["cargo"]);
  }
  require((c.ids.size() == 3) == r.cargo.has_value(), "invalid cargo contract");
  r.start = decode_boundary(v["start"]);
  r.goal = decode_boundary(v["goal"]);
  require(v["allow_reverse"].isBool(), "invalid reverse flag");
  r.allow_reverse = v["allow_reverse"].asBool();
  r.task_duration_cap = number(v["task_duration_cap"]);
  for (auto [name, member]:limit_fields) {
    r.limits.*member = number(v["limits"][name]);
  }
  require(j["phases"].isArray() && j["phases"].size() > 0 && j["phases"].size() <= 256,
    "invalid phase count");
  for (auto & p:j["phases"]) {
    out.plan.phases.push_back(decode_phase(p, c.ids.size(), piecewise_clock));
  }
  require(hj::duration(out.plan) <= std::min(r.task_duration_cap, 86400.) + .000001, "duration cap");
  return out;
}
void display_samples(Envelope & m, const Context & c, const hj::Plan & p)
{
  auto & t = m.trajectory;
  t.header = m.header;
  t.plan_id = m.plan_id;
  t.formation_id = m.formation_id;
  t.trajectories.resize(c.ids.size());
  for (size_t i = 0; i < c.ids.size(); ++i) {
    t.trajectories[i].vehicle_id = c.ids[i];
  }
  double elapsed = 0.;
  int64_t previous = -1;
  auto record = [&](double time, const auto & s) {
      auto stamp = std::llround(time * 1e9);
      if (stamp <= previous) {
        return;
      }
      previous = stamp;
      const auto offset =
        rclcpp::Duration::from_nanoseconds(stamp).operator builtin_interfaces::msg::Duration();
      t.times_from_start.push_back(offset);
      t.formation_poses.push_back(pose2(s.reference));
      for (size_t i = 0; i < c.ids.size(); ++i) {
        auto & row = t.trajectories[i].points.emplace_back();
        const auto & b = s.bodies[i];
        row.time_from_start = offset;
        row.center_pose = pose2(b.pose);
        row.mode = uint8_t(s.mode);
        row.center_speed_mps = s.mode == hj::Mode::A ? b.vx : std::copysign(std::hypot(b.vx, b.vy), b.vx);
        row.center_yaw_rate_radps = b.wz;
        row.mode_control = s.mode == hj::Mode::A ? b.curvature : b.oblique_angle;
      }
    };
  for (const auto & ph:p.phases) {
    const size_t n = std::max<size_t>(1, std::ceil(ph.duration / .1));
    for (size_t k = 0; k < n; ++k) {
      double local = ph.duration * k / n;
      record(elapsed + local, hj::evaluate_phase(c.request, ph, local));
    }
    elapsed += ph.duration;
  }
  record(elapsed, hj::evaluate(c.request, p, elapsed));
}
} // namespace hybrid_ros
