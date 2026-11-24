// S4 formation geometry collision checks.  These checks are exact against the
// raster's occupied-cell rectangles (unknown cells count as occupied) and use
// a conservative padded sweep.  They do not assign or certify per-vehicle A/O/S
// modes, timing, actuators, or executable feasibility.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "fwis_v3_planner/formation_model.hpp"

namespace fwis {
namespace formation {

struct VehicleBody {
  uint32_t vehicle_id = 0;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;  // body heading, independent of psi_F
  double length_m = 0.0;
  double width_m = 0.0;
};

struct CellRun {
  int x0 = 0;
  int x1 = -1;
};

struct Obb {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double half_length = 0.0;
  double half_width = 0.0;
};

inline Obb body_obb(const VehicleBody& body, double padding_m = 0.0) {
  return {body.x, body.y, body.theta,
          body.length_m * 0.5 + padding_m,
          body.width_m * 0.5 + padding_m};
}

namespace collision_detail {

constexpr double kEps = 1e-12;

inline bool finite_box(const Obb& box) {
  return std::isfinite(box.x) && std::isfinite(box.y) &&
         std::isfinite(box.theta) && std::isfinite(box.half_length) &&
         std::isfinite(box.half_width) && box.half_length >= 0.0 &&
         box.half_width >= 0.0;
}

// True only when a separating axis proves the boxes do NOT collide.  Touching
// (and near-touching within kEps) is deliberately treated as collision.
inline bool separated(const Obb& a, const Obb& b) {
  const double ca = std::cos(a.theta), sa = std::sin(a.theta);
  const double cb = std::cos(b.theta), sb = std::sin(b.theta);
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double axes[4][2] = {
      {ca, sa}, {-sa, ca}, {cb, sb}, {-sb, cb}};
  for (const auto& axis : axes) {
    const double ra = a.half_length * std::abs(axis[0] * ca + axis[1] * sa) +
                      a.half_width * std::abs(-axis[0] * sa + axis[1] * ca);
    const double rb = b.half_length * std::abs(axis[0] * cb + axis[1] * sb) +
                      b.half_width * std::abs(-axis[0] * sb + axis[1] * cb);
    const double distance = std::abs(dx * axis[0] + dy * axis[1]);
    if (distance > ra + rb + kEps) return true;
  }
  return false;
}

inline bool obb_intersects_aabb(const Obb& box, double cx, double cy,
                                double half_x, double half_y) {
  return !separated(box, Obb{cx, cy, 0.0, half_x, half_y});
}

inline double projected_radius_x(const Obb& box) {
  return box.half_length * std::abs(std::cos(box.theta)) +
         box.half_width * std::abs(std::sin(box.theta));
}

inline double projected_radius_y(const Obb& box) {
  return box.half_length * std::abs(std::sin(box.theta)) +
         box.half_width * std::abs(std::cos(box.theta));
}

}  // namespace collision_detail

class FormationMap {
 public:
  FormationMap() = default;
  FormationMap(int width, int height, double resolution, double origin_x,
               double origin_y, std::vector<int8_t> data)
      : width_(width),
        height_(height),
        resolution_(resolution),
        origin_x_(origin_x),
        origin_y_(origin_y),
        data_(std::move(data)) {
    build();
  }

  bool valid() const { return valid_; }
  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  double origin_x() const { return origin_x_; }
  double origin_y() const { return origin_y_; }
  double max_x() const { return origin_x_ + width_ * resolution_; }
  double max_y() const { return origin_y_ + height_ * resolution_; }

  bool occupied(double x, double y) const {
    int ix = 0, iy = 0;
    if (!world_to_index(x, y, ix, iy)) return true;
    return data_[static_cast<size_t>(iy) * width_ + ix] != 0;
  }

  // Chessboard cell distance in metres.  This is a search-cost heuristic only;
  // exact OBB-vs-cell SAT remains the collision gate.
  double distance(double x, double y) const {
    int ix = 0, iy = 0;
    if (!world_to_index(x, y, ix, iy) || distance_.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    return static_cast<double>(
               distance_[static_cast<size_t>(iy) * width_ + ix]) *
           resolution_;
  }

  bool world_to_index(double x, double y, int& ix, int& iy) const {
    if (!valid_) return false;
    const double fx = (x - origin_x_) / resolution_;
    const double fy = (y - origin_y_) / resolution_;
    if (fx < 0.0 || fy < 0.0) return false;
    ix = static_cast<int>(fx);
    iy = static_cast<int>(fy);
    return ix >= 0 && iy >= 0 && ix < width_ && iy < height_;
  }

  bool obb_clear(const Obb& box) const {
    using collision_detail::projected_radius_x;
    using collision_detail::projected_radius_y;
    if (!valid_ || !collision_detail::finite_box(box)) return false;

    const double rx = projected_radius_x(box);
    const double ry = projected_radius_y(box);
    if (box.x - rx < origin_x_ - collision_detail::kEps ||
        box.y - ry < origin_y_ - collision_detail::kEps ||
        box.x + rx > max_x() + collision_detail::kEps ||
        box.y + ry > max_y() + collision_detail::kEps) {
      return false;
    }

    const int ix0 =
        std::max(0, static_cast<int>(std::floor((box.x - rx - origin_x_) /
                                                resolution_)) - 1);
    const int ix1 =
        std::min(width_ - 1,
                 static_cast<int>(std::floor((box.x + rx - origin_x_) /
                                             resolution_)) + 1);
    const int iy0 =
        std::max(0, static_cast<int>(std::floor((box.y - ry - origin_y_) /
                                                resolution_)) - 1);
    const int iy1 =
        std::min(height_ - 1,
                 static_cast<int>(std::floor((box.y + ry - origin_y_) /
                                             resolution_)) + 1);

    for (int iy = iy0; iy <= iy1; ++iy) {
      for (const CellRun& run : rows_[static_cast<size_t>(iy)]) {
        if (run.x1 < ix0) continue;
        if (run.x0 > ix1) break;
        const double rect_cx =
            origin_x_ + (run.x0 + run.x1 + 1) * 0.5 * resolution_;
        const double rect_cy =
            origin_y_ + (iy + 0.5) * resolution_;
        const double rect_hx =
            (run.x1 - run.x0 + 1) * 0.5 * resolution_;
        const double rect_hy = 0.5 * resolution_;
        if (collision_detail::obb_intersects_aabb(
                box, rect_cx, rect_cy, rect_hx, rect_hy)) {
          return false;
        }
      }
    }
    return true;
  }

 private:
  void build() {
    valid_ = false;
    rows_.clear();
    distance_.clear();
    if (width_ <= 0 || height_ <= 0 || !(resolution_ > 0.0) ||
        !std::isfinite(resolution_) || !std::isfinite(origin_x_) ||
        !std::isfinite(origin_y_) ||
        data_.size() != static_cast<size_t>(width_) * height_) {
      return;
    }
    rows_.resize(height_);
    for (int iy = 0; iy < height_; ++iy) {
      auto& row = rows_[static_cast<size_t>(iy)];
      int x = 0;
      while (x < width_) {
        const size_t index = static_cast<size_t>(iy) * width_ + x;
        if (data_[index] == 0) {
          ++x;
          continue;
        }
        const int x0 = x;
        while (x + 1 < width_ &&
               data_[static_cast<size_t>(iy) * width_ + x + 1] != 0) {
          ++x;
        }
        row.push_back({x0, x});
        ++x;
      }
    }

    // Two-pass chessboard distance field, matching the project's conservative
    // cell-distance semantics.  The field never substitutes for exact SAT.
    const uint16_t inf = std::numeric_limits<uint16_t>::max();
    distance_.assign(static_cast<size_t>(width_) * height_, inf);
    for (size_t i = 0; i < data_.size(); ++i) {
      if (data_[i] != 0) distance_[i] = 0;
    }
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const size_t index = static_cast<size_t>(y) * width_ + x;
        uint16_t& cell = distance_[index];
        if (cell == 0) continue;
        if (x > 0 && distance_[index - 1] < inf - 1)
          cell = std::min<uint16_t>(cell, distance_[index - 1] + 1);
        if (y > 0 && distance_[index - width_] < inf - 1)
          cell = std::min<uint16_t>(cell, distance_[index - width_] + 1);
      }
    }
    for (int y = height_ - 1; y >= 0; --y) {
      for (int x = width_ - 1; x >= 0; --x) {
        const size_t index = static_cast<size_t>(y) * width_ + x;
        uint16_t& cell = distance_[index];
        if (cell == 0) continue;
        if (x + 1 < width_ && distance_[index + 1] < inf - 1)
          cell = std::min<uint16_t>(cell, distance_[index + 1] + 1);
        if (y + 1 < height_ &&
            distance_[index + width_] < inf - 1)
          cell = std::min<uint16_t>(cell, distance_[index + width_] + 1);
      }
    }
    valid_ = true;
  }

  bool valid_ = false;
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  std::vector<int8_t> data_;
  std::vector<std::vector<CellRun>> rows_;
  std::vector<uint16_t> distance_;
};

struct SweepReport {
  int sample_count = 0;
  double max_corner_step_m = 0.0;
  double padding_m = 0.0;
  bool supported = false;
  bool map_clear = false;
  bool formation_clear = false;
};

class FormationCollisionChecker {
 public:
  FormationCollisionChecker(const FormationMap& map,
                            const FormationModel& model,
                            const FormationRequest& request)
      : map_(map), model_(model), request_(request) {}

  std::vector<VehicleBody> start_bodies() const {
    return bodies_at(request_.start_qf, start_headings());
  }

  std::vector<VehicleBody> goal_bodies() const {
    return bodies_at(request_.goal_qf, goal_headings());
  }

  // During the S4 q_F-only search body headings rotate rigidly with psi_F.
  // This is a geometry candidate convention, not a per-vehicle mode assignment.
  std::vector<VehicleBody> rigid_bodies(const FormationPose& qf) const {
    std::vector<double> headings;
    headings.reserve(model_.vehicles.size());
    for (const auto& vehicle : model_.vehicles) {
      headings.push_back(vehicle.start_theta +
                         (qf.psi - request_.start_qf.psi));
    }
    return bodies_at(qf, headings);
  }

  bool start_clear() const { return static_clear(start_bodies(), 0.0); }
  bool goal_clear() const { return static_clear(goal_bodies(), 0.0); }
  bool rigid_clear(const FormationPose& qf) const {
    return static_clear(rigid_bodies(qf), 0.0);
  }

  bool static_clear(const std::vector<VehicleBody>& bodies,
                    double padding_m) const {
    if (!map_.valid() || bodies.empty()) return false;
    for (const auto& body : bodies) {
      if (!map_.obb_clear(body_obb(body, padding_m))) return false;
    }
    for (size_t i = 0; i < bodies.size(); ++i) {
      for (size_t j = i + 1; j < bodies.size(); ++j) {
        if (!collision_detail::separated(body_obb(bodies[i], padding_m),
                                         body_obb(bodies[j], padding_m))) {
          return false;
        }
      }
    }
    return true;
  }

  bool sweep_clear(const FormationPose& qf,
                   const FormationPrimitive& primitive,
                   double max_corner_step_m,
                   SweepReport* report = nullptr) const {
    SweepReport local;
    auto finish = [&](bool ok) {
      local.supported = true;
      local.map_clear = ok;
      local.formation_clear = ok;
      if (report != nullptr) *report = local;
      return ok;
    };
    if (!map_.valid() ||
        !(std::isfinite(max_corner_step_m) && max_corner_step_m > 0.0) ||
        !primitive_finite(primitive)) {
      local.supported = false;
      if (report != nullptr) *report = local;
      return false;
    }

    FormationPose next;
    if (!integrate_primitive(qf, primitive, &next)) {
      local.supported = false;
      if (report != nullptr) *report = local;
      return false;
    }

    double total_corner_bound = 0.0;
    if (primitive.geometry_type == FormationPrimitive::TRANSLATION) {
      total_corner_bound = std::hypot(primitive.dx, primitive.dy);
    } else if (primitive.geometry_type == FormationPrimitive::ROTATION) {
      double max_radius = 0.0;
      for (const auto& vehicle : model_.vehicles) {
        const double center_radius =
            std::hypot(vehicle.offset_x - primitive.pivot_x,
                       vehicle.offset_y - primitive.pivot_y);
        const double body_radius =
            0.5 * std::hypot(vehicle.body_length_m, vehicle.body_width_m);
        max_radius = std::max(max_radius, center_radius + body_radius);
      }
      total_corner_bound = std::abs(primitive.dpsi) * max_radius;
    } else {
      local.supported = false;
      if (report != nullptr) *report = local;
      return false;
    }

    const int samples = std::max(
        1, static_cast<int>(std::ceil(total_corner_bound / max_corner_step_m)));
    const double padding = total_corner_bound / (2.0 * samples);
    local.sample_count = samples;
    local.max_corner_step_m = max_corner_step_m;
    local.padding_m = padding;

    for (int i = 1; i <= samples; ++i) {
      const double fraction = static_cast<double>(i) / samples;
      FormationPose sampled;
      if (!integrate_primitive(qf, fraction_primitive(primitive, fraction),
                               &sampled)) {
        local.supported = false;
        if (report != nullptr) *report = local;
        return false;
      }
      if (!static_clear(rigid_bodies(sampled), padding)) {
        return finish(false);
      }
    }
    return finish(true);
  }

 private:
  std::vector<double> start_headings() const {
    std::vector<double> headings;
    headings.reserve(model_.vehicles.size());
    for (const auto& vehicle : model_.vehicles) {
      headings.push_back(vehicle.start_theta);
    }
    return headings;
  }

  std::vector<double> goal_headings() const {
    std::vector<double> headings;
    headings.reserve(model_.vehicles.size());
    for (const auto& vehicle : model_.vehicles) {
      headings.push_back(vehicle.goal_theta);
    }
    return headings;
  }

  std::vector<VehicleBody> bodies_at(
      const FormationPose& qf, const std::vector<double>& headings) const {
    std::vector<VehicleBody> out;
    if (headings.size() != model_.vehicles.size()) return out;
    const double c = std::cos(qf.psi);
    const double s = std::sin(qf.psi);
    out.reserve(model_.vehicles.size());
    for (size_t i = 0; i < model_.vehicles.size(); ++i) {
      const FormationVehicle& vehicle = model_.vehicles[i];
      out.push_back({vehicle.vehicle_id,
                     qf.x + c * vehicle.offset_x - s * vehicle.offset_y,
                     qf.y + s * vehicle.offset_x + c * vehicle.offset_y,
                     headings[i], vehicle.body_length_m, vehicle.body_width_m});
    }
    return out;
  }

  const FormationMap& map_;
  const FormationModel& model_;
  const FormationRequest& request_;
};

}  // namespace formation
}  // namespace fwis
