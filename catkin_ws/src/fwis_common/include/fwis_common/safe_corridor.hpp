#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <utility>

#include "fwis_common/grid_map.hpp"
#include "fwis_common/kinematics.hpp"
#include "fwis_common/vehicle.hpp"

namespace fwis {

struct SafeCorridor {
  double cx, cy;
  double tan_x, tan_y;
  double norm_x, norm_y;
  double backward, forward;
  double right, left;
};

struct CorridorConfig {
  double resample_distance = 0.1;
  double max_distance = 5.0;
  double fast_step  = 0.25;
  double fine_step  = 0.05;
  bool   axis_aligned = true;
};

inline double norm_angle(double a) {
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

inline std::vector<Pose2D> resample_path(const std::vector<Pose2D>& path,
                                          double spacing) {
  if (path.size() < 2) return path;
  std::vector<double> cum;
  cum.push_back(0.0);
  for (size_t i = 1; i < path.size(); ++i)
    cum.push_back(cum.back() + std::hypot(path[i].x - path[i-1].x,
                                           path[i].y - path[i-1].y));
  double total = cum.back();
  if (total < spacing) return path;

  std::vector<Pose2D> out;
  out.push_back(path[0]);
  for (double d = spacing; d < total - 1e-6; d += spacing) {
    auto it = std::upper_bound(cum.begin(), cum.end(), d);
    size_t j = static_cast<size_t>(it - cum.begin()) - 1;
    if (j + 1 >= path.size()) break;
    double t = (d - cum[j]) / std::max(cum[j+1] - cum[j], 1e-9);
    Pose2D p;
    p.x = path[j].x + t * (path[j+1].x - path[j].x);
    p.y = path[j].y + t * (path[j+1].y - path[j].y);
    p.theta = path[j].theta + t * norm_angle(path[j+1].theta - path[j].theta);
    out.push_back(p);
  }
  out.push_back(path.back());
  return out;
}

inline bool strip_is_safe(const GridMap& map, double x0, double y0,
                           double dx, double dy, double L, double step_size,
                           int N) {
  for (int i = 0; i <= N; ++i) {
    double t = static_cast<double>(i) / std::max(N, 1);
    double sx = x0 + t * dx;
    double sy = y0 + t * dy;
    for (double s = -L * 0.5; s <= L * 0.5 + 1e-9; s += step_size) {
      double px = sx + s * dy / std::max(std::hypot(dx, dy), 1e-9);
      double py = sy - s * dx / std::max(std::hypot(dx, dy), 1e-9);
      if (map.is_occupied(px, py)) return false;
    }
  }
  return true;
}

inline SafeCorridor build_one_corridor(const std::vector<Pose2D>& path,
                                        size_t idx, const GridMap& map,
                                        const VehicleParams& vp,
                                        const CorridorConfig& cfg) {
  const auto& p = path[idx];
  SafeCorridor corr;
  corr.cx = p.x;
  corr.cy = p.y;

  double dx, dy;
  if (idx + 1 < path.size() && idx > 0) {
    dx = path[idx+1].x - path[idx-1].x;
    dy = path[idx+1].y - path[idx-1].y;
  } else if (idx + 1 < path.size()) {
    dx = path[idx+1].x - p.x;
    dy = path[idx+1].y - p.y;
  } else if (idx > 0) {
    dx = p.x - path[idx-1].x;
    dy = p.y - path[idx-1].y;
  } else {
    dx = 1.0; dy = 0.0;
  }
  double len = std::hypot(dx, dy);
  if (len < 1e-9) { dx = 1.0; len = 1.0; }
  corr.tan_x = dx / len;
  corr.tan_y = dy / len;

  if (cfg.axis_aligned) {
    if (std::abs(corr.tan_x) >= std::abs(corr.tan_y)) {
      corr.tan_x = (corr.tan_x >= 0) ? 1.0 : -1.0; corr.tan_y = 0.0;
    } else {
      corr.tan_x = 0.0; corr.tan_y = (corr.tan_y >= 0) ? 1.0 : -1.0;
    }
  }

  corr.norm_x = -corr.tan_y;
  corr.norm_y =  corr.tan_x;

  double half_diag = std::sqrt((vp.length * 0.5) * (vp.length * 0.5) +
                                (vp.width  * 0.5) * (vp.width  * 0.5));
  corr.forward  = half_diag;
  corr.backward = half_diag;
  corr.left     = half_diag;
  corr.right    = half_diag;

  auto expand_one_side = [&](double& bound, double step_size, double dir_x, double dir_y,
                              bool is_tangent) {
    while (bound + step_size <= cfg.max_distance) {
      double test_len = bound + step_size;
      double x0 = corr.cx + dir_x * bound;
      double y0 = corr.cy + dir_y * bound;
      double dx_extend = dir_x * step_size;
      double dy_extend = dir_y * step_size;

      double strip_length = is_tangent ? (corr.left + corr.right) : (corr.forward + corr.backward);
      double strip_dir_x = is_tangent ? corr.norm_x : corr.tan_x;
      double strip_dir_y = is_tangent ? corr.norm_y : corr.tan_y;

      int N = std::max(1, static_cast<int>(std::ceil(step_size / (map.resolution() * 2.0))));
      bool safe = strip_is_safe(map, x0, y0, dx_extend, dy_extend,
                                 strip_length, map.resolution() * 1.5, N);
      if (!safe) return false;
      bound = test_len;
    }
    return true;
  };

  // 快速步进
  for (int pass = 0; pass < 2; ++pass) {
    double step = (pass == 0) ? cfg.fast_step : cfg.fine_step;
    if (pass == 0) {
      expand_one_side(corr.forward,  step, corr.tan_x, corr.tan_y, true);
      expand_one_side(corr.backward, step, -corr.tan_x, -corr.tan_y, true);
      expand_one_side(corr.right,    step, corr.norm_x, corr.norm_y, false);
      expand_one_side(corr.left,     step, -corr.norm_x, -corr.norm_y, false);
    }
  }

  return corr;
}

inline std::pair<std::vector<SafeCorridor>, std::vector<SafeCorridor>>
build_corridors(const std::vector<Pose2D>& path, const GridMap& map,
                 const VehicleParams& vp, const CorridorConfig& cfg) {
  auto resampled = resample_path(path, cfg.resample_distance);
  if (resampled.size() < 2) {
    return {{}, {}};
  }

  // 计算前后轴中心轨迹
  double half_wb = vp.wheelbase * 0.5;
  std::vector<Pose2D> front_axle, rear_axle;
  for (const auto& p : resampled) {
    Pose2D fa;
    fa.x = p.x + half_wb * std::cos(p.theta);
    fa.y = p.y + half_wb * std::sin(p.theta);
    fa.theta = p.theta;
    front_axle.push_back(fa);

    Pose2D ra;
    ra.x = p.x - half_wb * std::cos(p.theta);
    ra.y = p.y - half_wb * std::sin(p.theta);
    ra.theta = p.theta;
    rear_axle.push_back(ra);
  }

  std::vector<SafeCorridor> front, rear;
  for (size_t i = 0; i < resampled.size(); ++i) {
    front.push_back(build_one_corridor(front_axle, i, map, vp, cfg));
    rear.push_back(build_one_corridor(rear_axle, i, map, vp, cfg));
  }
  return {front, rear};
}

}  // namespace fwis
