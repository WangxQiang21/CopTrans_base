#pragma once

#include <array>
#include <deque>
#include <optional>

#include <QColor>
#include <QImage>
#include <QPointF>
#include <QString>

namespace fwis_rviz_plugins
{

constexpr std::size_t cargo_index = 3;
constexpr std::size_t track_count = cargo_index + 1;
constexpr double feedback_timeout = 0.75;
constexpr double chart_gap = 0.35;

struct Feedback
{
  double stamp = 0;
  double x = 0;
  double y = 0;
  double yaw = 0;
  double vx = 0;
  double vy = 0;
  double wz = 0;
};

struct TrackView
{
  std::optional<Feedback> feedback;
  QString mode;
  std::deque<QPointF> errors;
  QString phase;
  std::optional<double> spacing_error_metres;
};

using Dashboard = std::array<TrackView, track_count>;

QColor cargoColor();
double dashboardScale(
  int width, int height, double pixel_ratio, bool show_experiment_id, std::size_t vehicles);
QImage drawStatusCards(const Dashboard & dashboard, int width, double scale, std::size_t vehicles);
QImage drawErrorCharts(
  const Dashboard & dashboard, int width, double scale, const QString & experiment_id,
  std::size_t vehicles);

}  // namespace fwis_rviz_plugins
