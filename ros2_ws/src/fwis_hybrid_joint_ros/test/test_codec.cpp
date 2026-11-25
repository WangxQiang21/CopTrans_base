#include "fwis_hybrid_joint_ros/transport.hpp"
#include <iostream>
using namespace hybrid_ros;
int main()
{
  try {
    Context c;
    c.id = 1;
    c.formation_id = "single";
    c.ids = {1};
    auto & r = c.request;
    r.grid = std::make_shared<hj::GridSnapshot>(
      32, 32, 1., fwis_planning_core::Point2d{-16., -16.},
      std::vector<uint8_t>(
        1024), 0, "test-grid");
    r.vehicles = {{0, 0, .838, .60}};
    r.goal.reference.x = 1.;
    hj::Phase ph;
    ph.duration = 12.;
    ph.gear = {1, 0, 0};
    hj::Piece piece;
    for (size_t i = 0; i < 6; ++i) {
      piece.x[i] = i * .2;
    }
    ph.pieces = {piece};
    ph.cruise_ramp_fraction = .125;
    hj::Plan plan;
    plan.phases.push_back(ph);
    const auto m = encode(c, plan, 1000000001);
    auto x = decode(m, r.grid, &c);
    if (encode(x.context, x.plan, 1000000001).model_json != m.model_json) {
      return 1;
    }
    for (int fault = 0; fault < 14; ++fault) {
      auto bad = m;
      auto expected = c;
      auto j = parse(m.model_json);
      switch (fault) {
        case 0: bad.plan_id++;
          break;
        case 1: bad.header.stamp.nanosec++;
          break;
        case 2: bad.model_sha256[3] = '?';
          break;
        case 3: bad.model_version = "old";
          break;
        case 4: j["solver_build"] = "different";
          bad.model_json = json(j);
          bad.model_sha256 = digest(bad.model_json);
          break;
        case 5: expected.request.goal.constrain_heading[0] = false;
          break;
        case 6: expected.request.vehicles[0][2] += .01;
          break;
        case 7: expected.request.limits.speed *= 1.1;
          break;
        case 8: expected.request.start.reference.x += .01;
          break;
        case 11:
          j["phases"][0]["cruise_ramp_fraction"]=.124;
          bad.model_json=json(j);bad.model_sha256=digest(bad.model_json);
          break;
        case 12:
          j["phases"][0]["kind"]=int(hj::Kind::Prepare);
          bad.model_json=json(j);bad.model_sha256=digest(bad.model_json);
          break;
        case 13:
          j["numeric_contract"]="bernstein5/unwrapped_yaw/quintic_time/v1";
          bad.model_json=json(j);bad.model_sha256=digest(bad.model_json);
          break;
        case 10: j["phases"][0]["gear"][0] = 0;
          bad.model_json = json(j);
          bad.model_sha256 = digest(bad.model_json);
          break;
        case 9: j["phases"][0]["duration"] = -1.;
          bad.model_json = json(j);
          bad.model_sha256 = digest(bad.model_json);
          break;
      }
      try {
        decode(bad, r.grid, &expected);
        std::cerr << "accepted fault " << fault << '\n';
        return 1;
      } catch (const std::invalid_argument &) {
      }
    }
    // v1/v2 retain their exact numeric contracts, and a v2 reader must
    // reject a narrower (v3-only) width even if the payload is rehashed.
    for (double width : hj::cruise_ramps) {
      plan.phases.front().cruise_ramp_fraction = width;
      auto wire = encode(c, plan, 1000000001);
      auto payload = parse(wire.model_json);
      const bool narrow = width < .125;
      const char * version = narrow ? "bernstein5/unwrapped_yaw/dual_ramp/v3" :
        "bernstein5/unwrapped_yaw/phase_clock/v2";
      if (payload["numeric_contract"] != version ||
          decode(wire, r.grid, &c).plan.phases[0].cruise_ramp_fraction != width) return 1;
      if (narrow) {
        payload["numeric_contract"] = "bernstein5/unwrapped_yaw/phase_clock/v2";
        wire.model_json = json(payload);wire.model_sha256 = digest(wire.model_json);
        try { decode(wire, r.grid, &c);return 1; } catch (const std::invalid_argument &) {}
      }
    }
    auto legacy = m;auto old = parse(m.model_json);
    old["numeric_contract"] = "bernstein5/unwrapped_yaw/quintic_time/v1";
    old["phases"][0].removeMember("cruise_ramp_fraction");
    legacy.model_json = json(old);legacy.model_sha256 = digest(legacy.model_json);
    if (decode(legacy, r.grid, &c).plan.phases[0].cruise_ramp_fraction != 0.) return 1;
    std::cout << "14 envelope/context fault cases rejected\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
