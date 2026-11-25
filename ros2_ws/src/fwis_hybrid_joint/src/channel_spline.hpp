#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace hybrid_channel {
using Span = std::array<double, 6>;

// Fully insert each interior knot of the uniform clamped C4 quintic. The
// extracted spans retain the equality of the three derivatives used for audit.
inline std::vector<Span> bezier_spans(std::vector<double> control, int spans) {
  constexpr int degree = 5;
  std::vector<double> knots(degree + 1, 0.);
  for (int i = 1; i < spans; ++i) knots.push_back(i);
  knots.insert(knots.end(), degree + 1, spans);
  for (int knot = 1; knot < spans; ++knot) {
    for (int multiplicity = 1; multiplicity < degree; ++multiplicity) {
      int k = std::upper_bound(knots.begin(), knots.end(), knot) - knots.begin() - 1;
      std::vector<double> next(control.size() + 1);
      std::copy_n(control.begin(), k - degree + 1, next.begin());
      for (int i = k - multiplicity; i < static_cast<int>(control.size()); ++i) next[i+1] = control[i];
      for (int i = k - degree + 1; i <= k - multiplicity; ++i) {
        const double f = (knot - knots[i]) / (knots[i+degree] - knots[i]);
        next[i] = control[i-1] + f * (control[i] - control[i-1]);
      }
      control.swap(next);
      knots.insert(knots.begin()+k+1, knot);
    }
  }
  std::vector<Span> result(spans);
  for (int j = 0; j < spans; ++j) std::copy_n(control.begin()+degree*j, degree+1, result[j].begin());
  return result;
}

// Interpolate all channel stations with clamped endpoint tangents and zero
// endpoint curvature. The tiny shared collocation system has at most 17 rows.
template <class Point>
std::array<std::vector<Span>,2> fit(const std::vector<Point>& route, double start_angle, double end_angle) {
  const int n = static_cast<int>(route.size()) - 1, count = n + 5;
  std::vector<std::vector<double>> system(count, std::vector<double>(count + 2));
  for (int i = 0; i < count; ++i) {
    std::vector<double> basis(count);
    basis[i] = 1.;
    const auto pieces = bezier_spans(std::move(basis), n);
    for (int j = 0; j < n; ++j) system[j][i] = pieces[j][0];
    const auto& a = pieces.front();
    const auto& b = pieces.back();
    system[n][i] = b[5];
    system[n+1][i] = 5 * (a[1] - a[0]);
    system[n+2][i] = 5 * (b[5] - b[4]);
    system[n+3][i] = 20 * (a[2] - 2*a[1] + a[0]);
    system[n+4][i] = 20 * (b[5] - 2*b[4] + b[3]);
  }
  for (int j = 0; j <= n; ++j) {
    system[j][count] = route[j].x;
    system[j][count+1] = route[j].y;
  }
  const double entry = std::hypot(route[1].x - route[0].x, route[1].y - route[0].y);
  const double exit = std::hypot(route[n].x - route[n-1].x, route[n].y - route[n-1].y);
  system[n+1][count] = entry * std::cos(start_angle);
  system[n+1][count+1] = entry * std::sin(start_angle);
  system[n+2][count] = exit * std::cos(end_angle);
  system[n+2][count+1] = exit * std::sin(end_angle);
  for (int i = 0; i < count; ++i) {
    int best = i;
    for (int row = i+1; row < count; ++row)
      if (std::abs(system[row][i]) > std::abs(system[best][i])) best = row;
    if (std::abs(system[best][i]) < 1e-12) return {};
    std::swap(system[i], system[best]);
    const double pivot = system[i][i];
    for (int j = i; j < count+2; ++j) system[i][j] /= pivot;
    for (int row = 0; row < count; ++row) {
      if (row == i) continue;
      const double f = system[row][i];
      for (int j = i; j < count+2; ++j) system[row][j] -= f * system[i][j];
    }
  }
  std::array<std::vector<Span>,2> result;
  for (int axis=0; axis<2; ++axis) {
    std::vector<double> control(count);
    for (int i=0; i<count; ++i) control[i] = system[i][count+axis];
    result[axis] = bezier_spans(std::move(control), n);
  }
  return result;
}
}  // namespace hybrid_channel
