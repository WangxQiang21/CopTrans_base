#pragma once
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>

namespace fwis {

// Lightweight wrapper around nav_msgs::OccupancyGrid for fast collision lookup.
// Cell value: 0 = free, 100 = occupied, -1 = unknown. Treat unknown as occupied.
class GridMap {
 public:
  GridMap() = default;
  explicit GridMap(const nav_msgs::OccupancyGrid& g) { reset(g); }

  void reset(const nav_msgs::OccupancyGrid& g) {
    res_      = g.info.resolution;
    inv_res_  = 1.0 / res_;
    width_    = g.info.width;
    height_   = g.info.height;
    origin_x_ = g.info.origin.position.x;
    origin_y_ = g.info.origin.position.y;
    data_     = g.data;  // copy: we want a stable view
    compute_distance_field();  // precompute for fast collision pre-screen
  }

  bool valid() const { return width_ > 0 && height_ > 0 && !data_.empty(); }
  double resolution() const { return res_; }
  int width()  const { return width_; }
  int height() const { return height_; }
  double origin_x() const { return origin_x_; }
  double origin_y() const { return origin_y_; }

  // World -> grid index (no clamp). Returns false if out of bounds.
  bool world_to_idx(double x, double y, int& ix, int& iy) const {
    double dx = (x - origin_x_) * inv_res_;
    double dy = (y - origin_y_) * inv_res_;
    if (dx < 0.0 || dy < 0.0) return false;
    ix = static_cast<int>(dx);
    iy = static_cast<int>(dy);
    return ix < width_ && iy < height_;
  }

  inline void grid_clamp(int& ix, int& iy) const {
    if (ix < 0) ix = 0;
    if (ix >= width_) ix = width_ - 1;
    if (iy < 0) iy = 0;
    if (iy >= height_) iy = height_ - 1;
  }

  // Distance (meters) to nearest obstacle at grid cell.
  double dist_at_cell(int ix, int iy) const {
    grid_clamp(ix, iy);
    return dist_[iy * width_ + ix] * res_;
  }

  // Distance (meters) to nearest obstacle at world point. Returns 0 if OOB.
  double distance(double x, double y) const {
    int ix, iy;
    if (!world_to_idx(x, y, ix, iy)) return 0.0;
    return dist_at_cell(ix, iy);
  }

  // True if (x,y) is occupied or out-of-bounds (treated as obstacle)
  bool is_occupied(double x, double y) const {
    int ix, iy;
    if (!world_to_idx(x, y, ix, iy)) return true;
    int8_t v = data_[iy * width_ + ix];
    return v != 0;  // anything not free is treated as obstacle
  }

  // Holonomic obstacle-aware lower bound from every free cell to a fixed
  // goal.  This is deliberately computed on the raw occupancy grid: it is a
  // lower bound for the vehicle search, while OBB collision checking remains
  // responsible for the true footprint constraint.
  std::vector<float> goal_distance_field(double goal_x, double goal_y) const {
    const int n = width_ * height_;
    const float inf = std::numeric_limits<float>::infinity();
    std::vector<float> out(n, inf);
    int gx, gy;
    if (!world_to_idx(goal_x, goal_y, gx, gy) || data_[gy * width_ + gx] != 0)
      return out;

    struct Item { float d; int idx; };
    struct Cmp { bool operator()(const Item& a, const Item& b) const {
      return a.d > b.d;
    }};
    std::priority_queue<Item, std::vector<Item>, Cmp> open;
    const int start = gy * width_ + gx;
    out[start] = 0.0f;
    open.push({0.0f, start});
    constexpr int dx[8] = {1,-1,0,0,1,1,-1,-1};
    constexpr int dy[8] = {0,0,1,-1,1,-1,1,-1};
    while (!open.empty()) {
      Item cur = open.top(); open.pop();
      if (cur.d != out[cur.idx]) continue;
      const int x = cur.idx % width_;
      const int y = cur.idx / width_;
      for (int k = 0; k < 8; ++k) {
        const int nx = x + dx[k], ny = y + dy[k];
        if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) continue;
        const int ni = ny * width_ + nx;
        if (data_[ni] != 0) continue;
        const float step = (k < 4) ? static_cast<float>(res_)
                                  : static_cast<float>(res_ * M_SQRT2);
        const float nd = cur.d + step;
        if (nd < out[ni]) {
          out[ni] = nd;
          open.push({nd, ni});
        }
      }
    }
    return out;
  }

  double sample_distance_field(const std::vector<float>& field,
                               double x, double y) const {
    int ix, iy;
    if (!world_to_idx(x, y, ix, iy) ||
        field.size() != static_cast<size_t>(width_ * height_)) {
      return std::numeric_limits<double>::infinity();
    }
    return field[iy * width_ + ix];
  }

  // Fast OBB collision check using SDF pre-screen.
  // If vehicle diagonal / 2 < distance_to_obstacle(center), skip dense check.
  bool obb_collides(double cx, double cy, double theta,
                    double L, double W) const {
    double half_diag = std::sqrt((L * 0.5) * (L * 0.5) + (W * 0.5) * (W * 0.5));
    // SDF 快速预筛: 中心点距离够远 → 整体安全
    if (distance(cx, cy) > half_diag + res_) return false;

    double c = std::cos(theta);
    double s = std::sin(theta);
    double step = res_ * 1.5;  // 粗化: 用 1.5×res 代替 0.7×res，减少 ~4× 采样点
    if (step <= 0.0) step = 0.05;
    double half_L = L * 0.5;
    double half_W = W * 0.5;
    for (double u = -half_L; u <= half_L + 1e-9; u += step) {
      for (double v = -half_W; v <= half_W + 1e-9; v += step) {
        double x = cx + u * c - v * s;
        double y = cy + u * s + v * c;
        if (is_occupied(x, y)) return true;
      }
    }
    return false;
  }

  // 完全密集检测（用于终端 shot 验碰，不跳过任何采样点）
  bool obb_collides_dense(double cx, double cy, double theta,
                          double L, double W) const {
    double c = std::cos(theta);
    double s = std::sin(theta);
    double step = res_ * 0.7;
    if (step <= 0.0) step = 0.05;
    double half_L = L * 0.5;
    double half_W = W * 0.5;
    for (double u = -half_L; u <= half_L + 1e-9; u += step) {
      for (double v = -half_W; v <= half_W + 1e-9; v += step) {
        double x = cx + u * c - v * s;
        double y = cy + u * s + v * c;
        if (is_occupied(x, y)) return true;
      }
    }
    return false;
  }

 private:
  // 两趟 DP 距离变换 (chessboard 距离, 保守估计)
  // O(width * height), 每个 map 更新时执行一次
  void compute_distance_field() {
    int N = width_ * height_;
    dist_.assign(N, std::numeric_limits<uint16_t>::max());
    // Pass 1: top-left to bottom-right
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        int idx = y * width_ + x;
        if (data_[idx] != 0) { dist_[idx] = 0; continue; }
        if (x > 0 && dist_[idx - 1] < dist_[idx])
          dist_[idx] = dist_[idx - 1] + 1;
        if (y > 0) {
          int up = dist_[idx - width_] + 1;
          if (up < dist_[idx]) dist_[idx] = up;
        }
      }
    }
    // Pass 2: bottom-right to top-left
    for (int y = height_ - 1; y >= 0; --y) {
      for (int x = width_ - 1; x >= 0; --x) {
        int idx = y * width_ + x;
        if (data_[idx] != 0) continue;
        if (x + 1 < width_ && dist_[idx + 1] + 1 < dist_[idx])
          dist_[idx] = dist_[idx + 1] + 1;
        if (y + 1 < height_) {
          int dn = dist_[idx + width_] + 1;
          if (dn < dist_[idx]) dist_[idx] = dn;
        }
      }
    }
  }

  double res_ = 0.0;
  double inv_res_ = 0.0;
  int width_ = 0, height_ = 0;
  double origin_x_ = 0.0, origin_y_ = 0.0;
  std::vector<int8_t> data_;
  std::vector<uint16_t> dist_;  // chessboard distance (cell units)
};

}  // namespace fwis
