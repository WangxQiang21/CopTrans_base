#pragma once
#include <Eigen/Dense>
#include <map>
#include "channel_spline.hpp"
#include "fwis_hybrid_joint/model.hpp"
#include "nlp_api.h"

// Fit a smooth formation reference to an occupancy-grid channel. All map and
// dynamics residuals here generate an initial iterate. Only certify() can
// authorize the resulting continuous motion.
namespace fwis_hybrid_joint::channel_repair
{
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Box = std::array<double, 4>; // cx,cy,hx,hy.
using Controls = Eigen::Matrix<double, Eigen::Dynamic, 3>;

inline double bernstein(const double * controls, int degree, double u)
{
  double q[6]{};
  std::copy_n(controls, degree + 1, q);
  for (int level = degree; level > 0; --level) {
    for (int i = 0; i < level; ++i) {
      q[i] += u * (q[i + 1] - q[i]);
    }
  }
  return q[0];
}
inline std::array<double, 3> curve(const HJPhase & ph, double u)
{
  const double s = u * ph.spans;
  const int j = std::min(int(s), ph.spans - 1);
  std::array<double, 3> q;
  for (int a = 0; a < 3; ++a) {
    q[a] = bernstein(ph.coefficient[j][a], 5, s - j);
  }
  return q;
}
inline auto basis(int n, int samples)
{
  std::array<MatrixXd, 3> b;
  for (auto & m:b) {
    m.resize(samples, n + 5);
  }
  for (int j = 0; j < n + 5; ++j) {
    std::vector<double> unit(n + 5);
    unit[j] = 1.;
    const auto spans = hybrid_channel::bezier_spans(std::move(unit), n);
    for (int i = 0; i < samples; ++i) {
      const double st = double(i) * n / (samples - 1);
      const int k = std::min(int(st), n - 1);
      auto a = spans[k];
      int degree = 5;
      for (auto & m:b) {
        m(i, j) = bernstein(a.data(), degree, st - k);
        for (int h = 0; h < degree; ++h) {
          a[h] = degree * (a[h + 1] - a[h]);
        }
        --degree;
      }
    }
  }
  return b;
}
inline std::vector<Box> rectangles(const GridSnapshot & grid, const std::function<bool()> & stop)
{
  std::vector<std::array<int, 4>> sets;
  std::map<std::pair<int, int>, int> prior;
  for (int y = 0; y < int(grid.height()); ++y) {
    if (stop() || sets.size() > 4096) {
      return {};
    }
    std::map<std::pair<int, int>, int> next;
    for (int x = 0; x < int(grid.width()); ) {
      if (!grid.occupied(x, y)) {
        ++x;
        continue;
      }
      const int l = x;
      while (x < int(grid.width()) && grid.occupied(x, y)) {
        ++x;
      }
      const auto key = std::make_pair(l, x);
      auto it = prior.find(key);
      if (it == prior.end()) {
        next[key] = int(sets.size());
        sets.push_back({l, x, y, y + 1});
      } else {
        next[key] = it->second;
        sets[it->second][3] = y + 1;
      }
    }
    prior = std::move(next);
  }
  std::vector<Box> out;
  const double r = grid.resolution() * .5;
  const auto org = grid.origin();
  for (auto a:sets) {
    out.push_back(
      {org.x + (a[0] + a[1]) * r, org.y + (a[2] + a[3]) * r, (a[1] - a[0]) * r,
        (a[3] - a[2]) * r});
  }
  return out;
}
// Forward mode with one seeded B-spline control. At a max/abs kink choose
// its directional derivative, as for the active face of a separating axis.
struct Tangent
{
  double value, slope;
  Tangent(double x = 0., double dx = 0.)
  : value(x), slope(dx) {}
};
inline Tangent operator+(Tangent a, Tangent b)
{
  return {a.value + b.value, a.slope + b.slope};
}
inline Tangent operator-(Tangent a, Tangent b)
{
  return {a.value - b.value, a.slope - b.slope};
}
inline Tangent operator*(Tangent a, Tangent b)
{
  return {a.value * b.value, a.slope * b.value + a.value * b.slope};
}
inline Tangent operator/(Tangent a, Tangent b)
{
  const double q = a.value / b.value;
  return {q, (a.slope - q * b.slope) / b.value};
}
inline Tangent magnitude(Tangent a)
{
  return {std::abs(a.value), a.value == 0. ? std::abs(a.slope) : std::copysign(1., a.value) * a.slope};
}
inline Tangent maximum(Tangent a, Tangent b)
{
  return a.value == b.value ? Tangent(a.value, std::max(a.slope, b.slope)) : (a.value > b.value ? a : b);
}
inline Tangent root(Tangent x)
{
  const double y = std::sqrt(x.value);
  return {y, .5 * x.slope / y};
}
inline Tangent sinc(Tangent a)
{
  return {std::sin(a.value), std::cos(a.value) * a.slope};
}
inline Tangent cosc(Tangent a)
{
  return {std::cos(a.value), -std::sin(a.value) * a.slope};
}
struct Geometry
{
  const Request & r;
  int n, samples;
  double spacing;
  std::array<MatrixXd, 3> b;
  std::vector<std::vector<Box>> obstacles;
  Controls preferred;
  std::array<double, 2> start_dir, end_dir;
  int nvars()const {return 2 + 3 * (n - 1);}
  Controls controls(const VectorXd & v)const
  {
    Controls c = preferred;
    const int last = c.rows() - 1;
    for (int a = 0; a < 2; ++a) {
      for (int i = 1; i <= 2; ++i) {
        c(i, a) = c(0, a) + i * v[0] * start_dir[a];
        c(last - i, a) = c(last, a) - i * v[1] * end_dir[a];
      }
    }
    for (int i = 3; i < last - 2; ++i) {for (int a = 0; a < 3; ++a) {c(i, a) = v[2 + 3 * (i - 3) + a];}}
    return c;
  }
  VectorXd guess()const
  {
    VectorXd v(nvars());
    v[0] = (preferred.row(1) - preferred.row(0)).head<2>().dot(Eigen::Vector2d(start_dir[0], start_dir[1]));
    const int last = preferred.rows() - 1;
    v[1] =
      (preferred.row(last) -
      preferred.row(last - 1)).head<2>().dot(Eigen::Vector2d(end_dir[0], end_dir[1]));
    for (int i = 3; i < last - 2; ++i) {
      for (int a = 0; a < 3; ++a) {
        v[2 + 3 * (i - 3) + a] = preferred(i, a);
      }
    }
    return v;
  }
  using Jets = std::array<Controls, 4>; // control, position, first and second station derivative
  Jets jets(const Controls & c)const {return {c, b[0] * c, b[1] * c, b[2] * c};}
  void residual(const Jets & v, const Jets * direction, VectorXd & err)const
  {
    const bool derivative = direction != nullptr;
    auto at = [&](int order, int i, int a) {
        return Tangent(v[order](i, a), derivative ? (*direction)[order](i, a) : 0.);
      };
    int row = 0;
    auto push = [&](Tangent q) {err[row++] = derivative ? q.slope : q.value;};
    for (int i = 1; i < preferred.rows() - 1; ++i) {
      for (int a = 0; a < 3; ++a) {
        push(.07 * (at(0, i, a) - preferred(i, a)));
      }
    }
    for (int i = 0; i < samples; ++i) {
      const Tangent x = at(1, i, 0), y = at(1, i, 1), yaw = at(1, i, 2), co = cosc(yaw), si = sinc(yaw);
      const Tangent dx = at(2, i, 0), dy = at(2, i, 1), dh = at(2, i, 2), ax = at(3, i, 0), ay = at(3, i, 1),
        ah = at(3, i, 2);
      push(.20 * (root(dx * dx + dy * dy + 1.e-16) - spacing));
      push(.32 * ax);
      push(.32 * ay);
      push(.8 * ah);
      push(.15 * dh);
      for (int k = -1; k < 3; ++k) {
        const auto & f = k < 0 ? *r.cargo : r.vehicles[k];
        const Tangent rx = co * f[0] - si * f[1], ry = si * f[0] + co * f[1];
        Tangent cs = co, sn = si;
        if (k >= 0) {
          const Tangent tx = dx - dh * ry, ty = dy + dh * rx;
          const Tangent norm2 = tx * tx + ty * ty + 1e-9, norm = root(norm2);
          const Tangent curvature =
            (tx * (ay + ah * rx - dh * dh * ry) - ty * (ax - ah * ry - dh * dh * rx)) / (norm2 * norm);
          cs = tx / norm;
          sn = ty / norm;
          push(50. * maximum(0., magnitude(curvature) - .88 / r.limits.radius));
          push(.24 * curvature);
          push(10. * maximum(0., .44 * spacing - norm));
        }
        const double a = f[2] / 2, bh = f[3] / 2;
        const Tangent ac = magnitude(cs), as = magnitude(sn), wx = a * ac + bh * as, wy = bh * ac + a * as;
        for (const auto & rect : obstacles[i]) {
          const Tangent qx = x + rx - rect[0], qy = y + ry - rect[1];
          const double hx = rect[2], hy = rect[3];
          Tangent margin = maximum(magnitude(qx) - hx - wx, magnitude(qy) - hy - wy);
          if (margin.value <= .045) {
            const auto along = magnitude(qx * cs + qy * sn) - a - hx * ac - hy * as;
            const auto across = magnitude(qy * cs - qx * sn) - bh - hx * as - hy * ac;
            margin = maximum(margin, maximum(along, across));
          }
          push(110. * maximum(0., .045 - margin));
        }
      }
    }
  }
  int rows()const
  {
    int count = 3 * (preferred.rows() - 2) + 14 * samples;
    for (const auto & obs : obstacles) {count += 4 * obs.size();}
    return count;
  }
};
inline bool repair(
  const Request & r, HJProblem & g, const std::vector<Box> & boxes, int n, double lo, double hi,
  const std::function<bool()> & stop)
{
  const auto & ph = g.phase[0];
  const int nc = n + 5, samples = n * 18 + 1;
  Geometry geom{r, n, samples, 0., basis(n, samples), {}, Controls(nc, 3), {}, {}};
  const auto & p0 = r.start.reference;
  const auto & p1 = r.goal.reference;
  const double start = ph.h0[0] + (ph.gear[0] < 0 ? 3.141592653589793 : 0);
  const auto & tail = ph.coefficient[ph.spans - 1];
  const double finish = std::atan2(tail[1][5] - tail[1][4], tail[0][5] - tail[0][4]);
  geom.start_dir = {std::cos(start), std::sin(start)};
  geom.end_dir = {std::cos(finish), std::sin(finish)};
  MatrixXd target(101, 3);
  const double turn = std::remainder(p1.yaw - p0.yaw, 2 * 3.141592653589793);
  auto prev = curve(ph, 0);
  for (int i = 0; i <= 100; ++i) {
    double u = i / 100.;
    auto pt = curve(ph, u);
    const double t = std::clamp((u - lo) / (hi - lo), 0., 1.);
    target(i, 0) = pt[0];
    target(i, 1) = pt[1];
    target(i, 2) = p0.yaw + turn * (t * t * (3 - 2 * t));
    geom.spacing += std::hypot(pt[0] - prev[0], pt[1] - prev[1]) / n;
    prev = pt;
  }
  geom.preferred = basis(n, 101)[0].colPivHouseholderQr().solve(target);
  geom.preferred.row(0) << p0.x, p0.y, p0.yaw;
  geom.preferred.row(nc - 1) << p1.x, p1.y, p0.yaw + turn;
  for (int i = 1; i <= 2; ++i) {
    geom.preferred(i, 2) = p0.yaw;
    geom.preferred(nc - 1 - i, 2) = p0.yaw + turn;
  }
  auto v = geom.guess();
  geom.preferred = geom.controls(v);
  double reach = 0.;
  for (const auto & f : r.vehicles) {
    reach = std::max(reach, std::hypot(f[0], f[1]) + .5 * std::hypot(f[2], f[3]));
  }
  if (r.cargo) {
    const auto & f = *r.cargo;
    reach = std::max(reach, std::hypot(f[0], f[1]) + .5 * std::hypot(f[2], f[3]));
  }
  const double halo = reach + r.limits.radius + 3 * r.grid->resolution();
  for (int i = 0; i < samples; ++i) {
    if (stop()) return false;
    auto point = curve(ph, double(i) / (samples - 1));
    geom.obstacles.emplace_back();
    for (auto box : boxes) {
      if (std::abs(point[0] - box[0]) < box[2] + halo && std::abs(point[1] - box[1]) < box[3] + halo) {
        geom.obstacles.back().push_back(box);
      }
    }
  }
  const int m = geom.rows(), q = v.size();
  if (m > 50000 || stop()) {
    return false;
  }
  VectorXd f(m), tmp(m);
  const auto zero = geom.controls(VectorXd::Zero(q));
  std::vector<Geometry::Jets> seeds;
  for (int axis = 0; axis < q; ++axis) {
    VectorXd unit = VectorXd::Zero(q);
    unit[axis] = 1.;
    seeds.push_back(geom.jets(geom.controls(unit) - zero));
  }
  auto jets = geom.jets(geom.controls(v));
  MatrixXd jac(m, q);
  geom.residual(jets, nullptr, f);
  double cost = f.squaredNorm(), lambda = .001;
  int accepted = 0;
  for (int it = 0; it < 85 && !stop(); ++it) {
    for (int j = 0; j < q; ++j) {
      if (stop()) return false;
      geom.residual(jets, &seeds[j], tmp);
      jac.col(j) = tmp;
    }
    // Compact inactive map rows before the small damped least-squares step.
    int active = 0;
    for (int i = 0; i < m; ++i) {
      if (f[i] != 0. || jac.row(i).squaredNorm() > 0.) {
        if (i != active) {
          jac.row(active) = jac.row(i);
        }
        tmp[active] = f[i];
        ++active;
      }
    }
    auto jj = jac.topRows(active);
    const MatrixXd normal = jj.transpose() * jj;
    const VectorXd grad = jj.transpose() * tmp.head(active);
    const auto diagonal = normal.diagonal().cwiseMax(.01).eval();
    bool improved = false;
    for (int tick = 0; tick < 12 && !stop(); ++tick) {
      MatrixXd damped = normal;
      damped.diagonal() += lambda * diagonal;
      const VectorXd step = damped.ldlt().solve(-grad);
      const auto trial = (v + step).eval();
      auto new_jets = geom.jets(geom.controls(trial));
      geom.residual(new_jets, nullptr, tmp);
      const double next = tmp.squaredNorm();
      if (next < cost) {
        v = trial;
        jets = std::move(new_jets);
        f = tmp;
        const double diff = cost - next;
        cost = next;
        lambda = std::max(1e-9, lambda * .45);
        improved = true;
        ++accepted;
        if (diff < .000025 * std::max(1., cost)) {
          it = 85;
        }
        break;
      }
      lambda *= 3.;
    }
    if (!improved) {
      break;
    }
  }
  if (accepted == 0 || stop()) {
    return false;
  }
  const auto c = geom.controls(v);
  auto & output = g.phase[0];
  output.spans = n;
  for (int a = 0; a < 3; ++a) {
    std::vector<double> controls(nc);
    for (int i = 0; i < nc; ++i) {
      controls[i] = c(i, a);
    }
    const auto spans = hybrid_channel::bezier_spans(std::move(controls), n);
    for (int i = 0; i < n; ++i) {
      std::copy_n(spans[i].begin(), 6, output.coefficient[i][a]);
    }
  }
  return true;
}
} // namespace fwis_hybrid_joint::channel_repair
