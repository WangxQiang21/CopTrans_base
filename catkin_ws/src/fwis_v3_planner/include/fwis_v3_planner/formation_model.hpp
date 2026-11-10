// S4 rigid-formation model.  This header is deliberately ROS-independent:
// callers construct FormationRequest from the ROS message, then validate it
// before any geometry search.
#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace fwis {
namespace formation {

struct FormationPose {
  double x = 0.0;
  double y = 0.0;
  double psi = 0.0;  // formation reference heading, may be unwrapped
};

struct FormationPrimitive {
  static constexpr uint8_t TRANSLATION = 0;
  static constexpr uint8_t ROTATION = 1;
  static constexpr uint8_t COMPOSITE = 2;

  uint8_t geometry_type = TRANSLATION;
  // TRANSLATION: (dx, dy) in the current formation frame; psi_F unchanged.
  double dx = 0.0;
  double dy = 0.0;
  // ROTATION: dpsi about the explicit pivot in the current formation frame.
  double dpsi = 0.0;
  double pivot_x = 0.0;
  double pivot_y = 0.0;
};

struct FormationVehicle {
  uint32_t vehicle_id = 0;
  double offset_x = 0.0;
  double offset_y = 0.0;
  double offset_z = 0.0;
  double start_x = 0.0;
  double start_y = 0.0;
  double start_theta = 0.0;
  double goal_x = 0.0;
  double goal_y = 0.0;
  double goal_theta = 0.0;
  bool goal_heading_free = false;
  double body_length_m = 0.0;
  double body_width_m = 0.0;
};

struct FormationRequest {
  std::string frame_id;
  uint32_t stamp_secs = 0;
  uint32_t stamp_nsecs = 0;
  uint32_t plan_id = 0;
  std::string formation_id;
  FormationPose start_qf;
  FormationPose goal_qf;
  std::vector<FormationVehicle> vehicles;
  bool allow_reverse = false;
};

inline double wrap_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline bool primitive_finite(const FormationPrimitive& primitive) {
  return std::isfinite(primitive.dx) && std::isfinite(primitive.dy) &&
         std::isfinite(primitive.dpsi) &&
         std::isfinite(primitive.pivot_x) &&
         std::isfinite(primitive.pivot_y);
}

// Rigid integration of a formation primitive in the current formation frame.
// COMPOSITE is deliberately unsupported until S6 defines a common parameter.
inline bool integrate_primitive(const FormationPose& qf,
                                const FormationPrimitive& primitive,
                                FormationPose* out) {
  if (out == nullptr || !primitive_finite(primitive)) return false;
  if (primitive.geometry_type == FormationPrimitive::TRANSLATION) {
    const double c = std::cos(qf.psi);
    const double s = std::sin(qf.psi);
    out->x = qf.x + c * primitive.dx - s * primitive.dy;
    out->y = qf.y + s * primitive.dx + c * primitive.dy;
    out->psi = qf.psi;
    return true;
  }
  if (primitive.geometry_type == FormationPrimitive::ROTATION) {
    const double c = std::cos(qf.psi);
    const double s = std::sin(qf.psi);
    const double pivot_world_x =
        qf.x + c * primitive.pivot_x - s * primitive.pivot_y;
    const double pivot_world_y =
        qf.y + s * primitive.pivot_x + c * primitive.pivot_y;
    const double next_psi = qf.psi + primitive.dpsi;
    const double cn = std::cos(next_psi);
    const double sn = std::sin(next_psi);
    out->x = pivot_world_x - cn * primitive.pivot_x + sn * primitive.pivot_y;
    out->y = pivot_world_y - sn * primitive.pivot_x - cn * primitive.pivot_y;
    out->psi = next_psi;
    return true;
  }
  return false;
}

// Fractional application keeps search sampling and message integration on the
// same rigid-transform definition.
inline FormationPrimitive fraction_primitive(
    const FormationPrimitive& primitive, double fraction) {
  FormationPrimitive out = primitive;
  out.dx *= fraction;
  out.dy *= fraction;
  out.dpsi *= fraction;
  return out;
}

struct FormationModel {
  // Vehicles are ordered by vehicle_id after successful validation.
  std::vector<FormationVehicle> vehicles;

  size_t size() const { return vehicles.size(); }
  bool empty() const { return vehicles.empty(); }

  std::vector<uint32_t> vehicle_ids() const {
    std::vector<uint32_t> ids;
    ids.reserve(vehicles.size());
    for (const auto& vehicle : vehicles) ids.push_back(vehicle.vehicle_id);
    return ids;
  }

  // p_i = p_F + R(psi_F) r_i.  Returns one pose per ID-sorted vehicle; psi
  // is copied from q_F so unwrapped formation history is not lost.
  std::vector<FormationPose> centers(const FormationPose& qf) const {
    const double c = std::cos(qf.psi);
    const double s = std::sin(qf.psi);
    std::vector<FormationPose> out;
    out.reserve(vehicles.size());
    for (const auto& vehicle : vehicles) {
      out.push_back({qf.x + c * vehicle.offset_x - s * vehicle.offset_y,
                     qf.y + s * vehicle.offset_x + c * vehicle.offset_y,
                     qf.psi});
    }
    return out;
  }
};

struct FormationIssue {
  bool has_vehicle_id = false;
  uint32_t vehicle_id = 0;
  std::string field;
  std::string detail;
};

struct FormationValidation {
  bool valid = false;
  std::string reason;
  std::string field;
  bool has_vehicle_id = false;
  uint32_t vehicle_id = 0;
  std::string detail;
  FormationRequest request;  // canonical, vehicle_id sorted
  FormationModel model;

  bool ok() const { return valid; }
};

namespace detail {

inline bool finite(double value) { return std::isfinite(value); }

inline bool blank(const std::string& text) {
  for (char ch : text) {
    if (!std::isspace(static_cast<unsigned char>(ch))) return false;
  }
  return true;
}

inline FormationValidation invalid(const std::string& field,
                                   const std::string& detail,
                                   bool has_id = false,
                                   uint32_t id = 0) {
  FormationValidation result;
  result.valid = false;
  result.reason = "INVALID_REQUEST";
  result.field = field;
  result.detail = detail;
  result.has_vehicle_id = has_id;
  result.vehicle_id = id;
  return result;
}

inline bool pose_finite(const FormationPose& pose) {
  return finite(pose.x) && finite(pose.y) && finite(pose.psi);
}

inline bool offsets_coincident(const FormationVehicle& first,
                               const FormationVehicle& second) {
  return first.offset_x == second.offset_x &&
         first.offset_y == second.offset_y;
}

}  // namespace detail

// Validate the S3 request semantics independently of the Python adapter.
// This checks structure and rigid-center consistency only.  It never treats
// a valid request as an executable or collision-free plan.
inline FormationValidation validate_formation_request(
    const FormationRequest& raw, double position_tolerance_m = 1e-6) {
  if (!detail::finite(position_tolerance_m) || position_tolerance_m < 0.0) {
    return detail::invalid("position_tolerance_m",
                           "expected a finite value >= 0");
  }
  if (raw.frame_id != "map") {
    return detail::invalid("header.frame_id",
                           "expected 'map', got '" + raw.frame_id + "'");
  }
  if (raw.stamp_secs == 0 && raw.stamp_nsecs == 0) {
    return detail::invalid("header.stamp",
                           "zero stamp is not an explicit common time");
  }
  if (raw.stamp_nsecs >= 1000000000u) {
    return detail::invalid("header.stamp",
                           "nsecs outside [0, 1e9)");
  }
  if (detail::blank(raw.formation_id)) {
    return detail::invalid("formation_id", "expected a non-empty string");
  }
  if (!detail::pose_finite(raw.start_qf) || !detail::pose_finite(raw.goal_qf)) {
    return detail::invalid("request", "formation pose contains a non-finite value");
  }
  if (raw.vehicles.empty()) {
    return detail::invalid("vehicles", "empty vehicle list");
  }

  FormationRequest canonical = raw;
  for (size_t i = 0; i < raw.vehicles.size(); ++i) {
    const FormationVehicle& vehicle = raw.vehicles[i];
    const std::string prefix = "vehicles[" + std::to_string(i) + "]";
    for (size_t j = 0; j < i; ++j) {
      if (raw.vehicles[j].vehicle_id == vehicle.vehicle_id) {
        return detail::invalid(
            prefix + ".vehicle_id",
            "duplicate vehicle_id=" + std::to_string(vehicle.vehicle_id),
            true, vehicle.vehicle_id);
      }
    }
    if (!detail::finite(vehicle.offset_x) ||
        !detail::finite(vehicle.offset_y) ||
        !detail::finite(vehicle.offset_z)) {
      return detail::invalid(prefix + ".offset_in_formation",
                             "offset contains a non-finite value",
                             true, vehicle.vehicle_id);
    }
    if (vehicle.offset_z != 0.0) {
      return detail::invalid(prefix + ".offset_in_formation.z",
                             "z must be exactly 0",
                             true, vehicle.vehicle_id);
    }
    const double pose_values[] = {
        vehicle.start_x, vehicle.start_y, vehicle.start_theta,
        vehicle.goal_x, vehicle.goal_y, vehicle.goal_theta};
    for (double value : pose_values) {
      if (!detail::finite(value)) {
        return detail::invalid(prefix + ".pose",
                               "vehicle pose contains a non-finite value",
                               true, vehicle.vehicle_id);
      }
    }
    if (!detail::finite(vehicle.body_length_m) ||
        !detail::finite(vehicle.body_width_m) ||
        vehicle.body_length_m <= 0.0 || vehicle.body_width_m <= 0.0) {
      return detail::invalid(prefix + ".body_shape",
                             "body_length_m and body_width_m must be finite and > 0",
                             true, vehicle.vehicle_id);
    }
  }

  canonical.vehicles = raw.vehicles;
  std::sort(canonical.vehicles.begin(), canonical.vehicles.end(),
            [](const FormationVehicle& a, const FormationVehicle& b) {
              return a.vehicle_id < b.vehicle_id;
            });

  for (size_t i = 0; i < canonical.vehicles.size(); ++i) {
    for (size_t j = i + 1; j < canonical.vehicles.size(); ++j) {
      if (detail::offsets_coincident(canonical.vehicles[i],
                                     canonical.vehicles[j])) {
        return detail::invalid(
            "vehicles.offset_in_formation",
            "COINCIDENT_OFFSETS pair_id=(" +
                std::to_string(canonical.vehicles[i].vehicle_id) + ", " +
                std::to_string(canonical.vehicles[j].vehicle_id) + ")");
      }
    }
  }

  const double start_c = std::cos(canonical.start_qf.psi);
  const double start_s = std::sin(canonical.start_qf.psi);
  const double goal_c = std::cos(canonical.goal_qf.psi);
  const double goal_s = std::sin(canonical.goal_qf.psi);
  for (size_t i = 0; i < canonical.vehicles.size(); ++i) {
    const FormationVehicle& vehicle = canonical.vehicles[i];
    const std::string prefix = "vehicle_id=" + std::to_string(vehicle.vehicle_id);
    const double expected_start_x =
        canonical.start_qf.x + start_c * vehicle.offset_x - start_s * vehicle.offset_y;
    const double expected_start_y =
        canonical.start_qf.y + start_s * vehicle.offset_x + start_c * vehicle.offset_y;
    const double expected_goal_x =
        canonical.goal_qf.x + goal_c * vehicle.offset_x - goal_s * vehicle.offset_y;
    const double expected_goal_y =
        canonical.goal_qf.y + goal_s * vehicle.offset_x + goal_c * vehicle.offset_y;
    if (std::hypot(vehicle.start_x - expected_start_x,
                   vehicle.start_y - expected_start_y) >
        position_tolerance_m) {
      return detail::invalid(
          "vehicles.start_center",
          "center does not equal formation_start + R(psi_F) r_i (" + prefix + ")",
          true, vehicle.vehicle_id);
    }
    if (std::hypot(vehicle.goal_x - expected_goal_x,
                   vehicle.goal_y - expected_goal_y) >
        position_tolerance_m) {
      return detail::invalid(
          "vehicles.goal_center",
          "center does not equal formation_goal + R(psi_F) r_i; "
          "goal_heading_free does not relax center rigidity (" + prefix + ")",
          true, vehicle.vehicle_id);
    }
  }

  FormationValidation result;
  result.valid = true;
  result.request = canonical;
  result.model.vehicles = canonical.vehicles;
  return result;
}

}  // namespace formation
}  // namespace fwis
