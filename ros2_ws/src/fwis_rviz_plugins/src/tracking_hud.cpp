#include "tracking_hud.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <utility>

#include <QFont>
#include <QFontMetrics>
#include <QPainter>
#include <QPainterPath>

namespace fwis_rviz_plugins
{
namespace
{

constexpr int margin = 12;
constexpr int gap = 10;
constexpr int status_height = 166;
constexpr int chart_height = 137;
constexpr int experiment_footer_height = 27;
constexpr double radians_to_degrees = 57.29577951308232;
const QColor primary("#f3f6fa");
const QColor secondary("#aebbc9");
const QColor grid("#455364");
const std::array<QColor, track_count> accents = {
  QColor("#edf1f7"), QColor("#ff912c"), QColor("#72a8ff"), cargoColor()};
const std::array<QString, track_count> names = {
  QStringLiteral("CopTransV1"), QStringLiteral("CopTransV2"),
  QStringLiteral("CopTransV3"), QStringLiteral("Cargo")};
const std::array<QString, track_count> chart_names = {
  QStringLiteral("V1 位置误差"), QStringLiteral("V2 位置误差"),
  QStringLiteral("V3 位置误差"), QStringLiteral("最大车间距偏差")};

void label(
  QPainter & painter, const QRectF & rect, const QString & text, const QColor & color,
  int pixels, int alignment = Qt::AlignLeft, bool bold = false)
{
  QFont font = painter.font();
  font.setPixelSize(pixels);
  font.setBold(bold);
  painter.setFont(font);
  painter.setPen(color);
  painter.drawText(rect, alignment | Qt::AlignVCenter, text);
}

void panel(QPainter & painter, const QRectF & rect, const QColor & accent)
{
  painter.setBrush(QColor(19, 27, 36, 233));
  painter.setPen(QPen(QColor("#657281"), 1));
  painter.drawRoundedRect(rect, 7, 7);
  painter.setBrush(accent);
  painter.setPen(Qt::NoPen);
  painter.drawRoundedRect(QRectF(rect.left() + 10, rect.top() + 1, 26, 2), 1, 1);
  painter.setBrush(Qt::NoBrush);
}

QString number(double value, int decimals = 2)
{
  const double half_step = 0.5 * std::pow(0.1, decimals);
  return QString::number(std::abs(value) < half_step ? 0.0 : value, 'f', decimals);
}

void fields(
  QPainter & painter, const QRectF & card, const QColor & value_color,
  std::initializer_list<std::pair<QString, QString>> rows)
{
  const QRectF row = card.adjusted(11, 31, -11, 0);
  int index = 0;
  for (const auto & [key, value] : rows) {
    const QRectF line(row.left(), row.top() + 18 * index++, row.width(), 17);
    label(painter, line, key, secondary, 12);
    label(painter, line, value, value_color, 12, Qt::AlignRight);
  }
}

void statusCard(
  QPainter & painter, const QRectF & card, const TrackView & view, std::size_t index)
{
  const QColor & accent = accents[index];
  const bool live = view.feedback.has_value();
  const QColor & value_color = live ? accent : secondary;
  panel(painter, card, accent);
  const QRectF title = card.adjusted(11, 5, -11, 0);
  const QRectF title_line(title.left(), title.top(), title.width(), 24);
  label(painter, title_line, names[index], accent, 15, Qt::AlignLeft, true);
  label(
    painter, title_line, live ? QStringLiteral("反馈") : QStringLiteral("无数据"),
    live ? accent : secondary, 11, Qt::AlignRight);

  const Feedback value = view.feedback.value_or(Feedback{});
  const QString speed = number(std::hypot(value.vx, value.vy)) + " m/s";
  const QString heading = number(value.yaw * radians_to_degrees, 1) + QStringLiteral("°");
  const QString position = QString("(%1, %2) m").arg(number(value.x), number(value.y));
  if (index == cargo_index) {
    const QString spacing = view.spacing_error_metres ?
      number(100 * *view.spacing_error_metres) + " cm" : QStringLiteral("—");
    fields(painter, card, value_color, {
      {"speed", speed}, {"heading", heading}, {"position", position},
      {QStringLiteral("最大间距偏差"), spacing}});
    return;
  }
  fields(painter, card, value_color, {
    {"mode", view.mode.isEmpty() ? QStringLiteral("—") : view.mode},
    {"speed", speed}, {"heading", heading},
    {"linear.x", number(value.vx) + " m/s"}, {"linear.y", number(value.vy) + " m/s"},
    {"angular.z", number(value.wz) + " rad/s"}, {"position", position}});
}

void errorChart(
  QPainter & painter, const QRectF & card, const TrackView & view, std::size_t index)
{
  const QColor & accent = accents[index];
  panel(painter, card, accent);
  const QRectF title(card.left() + 11, card.top() + 6, card.width() - 22, 18);
  label(painter, title, chart_names[index] + " (cm)", accent, 13, Qt::AlignLeft, true);
  if (view.phase == "COMPLETE" || view.phase == "STOPPED") {
    label(
      painter, title, view.phase == "COMPLETE" ? QStringLiteral("已完成") : QStringLiteral("已停止"),
      secondary, 11, Qt::AlignRight);
  }
  if (view.errors.empty()) {
    const auto detail = index == cargo_index ? QStringLiteral("等待编队反馈") :
      (view.feedback ? QStringLiteral("等待轨迹") : QStringLiteral("无跟踪数据"));
    label(painter, card.adjusted(0, 23, 0, 0), detail, secondary, 12, Qt::AlignHCenter);
    return;
  }

  const QRectF plot = card.adjusted(39, 33, -14, -24);
  const double t_last = view.errors.back().x();
  const double t_min = std::floor(view.errors.front().x() / 10) * 10;
  const double t_max = std::max(t_min + 10, std::ceil(t_last / 10) * 10);
  const auto peak = std::max_element(
    view.errors.begin(), view.errors.end(),
    [](const QPointF & a, const QPointF & b) {return a.y() < b.y();});
  const double y_max = std::max(1.0, std::ceil(peak->y()));
  for (int tick = 0; tick <= 2; ++tick) {
    const double fraction = tick / 2.0;
    const double y = plot.bottom() - fraction * plot.height();
    const double x = plot.left() + fraction * plot.width();
    painter.setPen(grid);
    painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
    painter.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
    const double tick_value = fraction * y_max;
    label(
      painter, QRectF(card.left() + 4, y - 7, 28, 14),
      number(tick_value, tick_value == std::floor(tick_value) ? 0 : 1), secondary, 10,
      Qt::AlignRight);
    const QString time = number(t_min + fraction * (t_max - t_min), 0);
    label(
      painter, QRectF(x - 22, plot.bottom() + 4, 44, 15),
      tick == 2 ? time + " s" : time, secondary, 10, Qt::AlignHCenter);
  }

  QPainterPath line;
  QPointF end;
  double previous = -1;
  for (const QPointF & point : view.errors) {
    end = QPointF(
      plot.left() + (point.x() - t_min) * plot.width() / (t_max - t_min),
      plot.bottom() - point.y() * plot.height() / y_max);
    if (previous < 0 || point.x() - previous > chart_gap) {
      line.moveTo(end);
    } else {
      line.lineTo(end);
    }
    previous = point.x();
  }
  painter.setPen(QPen(accent, 2));
  painter.drawPath(line);
  painter.setPen(Qt::NoPen);
  painter.setBrush(accent);
  painter.drawEllipse(end, 2.5, 2.5);
}

QImage drawRow(
  const Dashboard & dashboard, int width, double scale, int height, std::size_t vehicles,
  void (*draw)(QPainter &, const QRectF &, const TrackView &, std::size_t),
  const QString & experiment_id = {})
{
  const int footer_height = experiment_id.isEmpty() ? 0 : experiment_footer_height;
  QImage image(
    width, std::ceil((height + footer_height + margin) * scale),
    QImage::Format_RGBA8888_Premultiplied);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
  painter.scale(scale, scale);
  const auto columns = vehicles + 1;
  const double card_width = (width / scale - 2 * margin - vehicles * gap) / columns;
  for (std::size_t column = 0; column < columns; ++column) {
    const auto index = column == vehicles ? cargo_index : column;
    const double x = margin + column * (card_width + gap);
    const double y = height == status_height ? margin : 0;
    draw(painter, QRectF(x, y, card_width, height), dashboard[index], index);
  }
  if (!experiment_id.isEmpty()) {
    constexpr int font_size = 12;
    constexpr int padding = 10;
    QFont font = painter.font();
    font.setPixelSize(font_size);
    font.setBold(false);
    const QFontMetrics metrics(font);
    const QString text = metrics.elidedText(
      QStringLiteral("实验编号  %1").arg(experiment_id), Qt::ElideRight,
      width / scale - 2 * (margin + padding));
    const double badge_width = metrics.horizontalAdvance(text) + 2 * padding;
    const QRectF badge(
      width / scale - margin - badge_width, height + 5, badge_width, footer_height - 5);
    painter.setPen(QPen(QColor("#657281"), 1));
    painter.setBrush(QColor(19, 27, 36, 233));
    painter.drawRoundedRect(badge, 5, 5);
    label(painter, badge.adjusted(padding, 0, -padding, 0), text, primary, font_size);
  }
  painter.end();
  return image;
}

}  // namespace

QColor cargoColor()
{
  return QColor("#6cd9c4");
}

double dashboardScale(
  int width, int height, double pixel_ratio, bool show_experiment_id, std::size_t vehicles)
{
  // Keep the visible columns readable and leave half the height for the 3D scene.
  const auto natural_width = (vehicles + 1) * 185 + 2 * margin + vehicles * gap;
  const int natural_height = status_height + chart_height + 2 * margin +
    (show_experiment_id ? experiment_footer_height : 0);
  return std::min({pixel_ratio, width / double(natural_width), height / double(2 * natural_height)});
}

QImage drawStatusCards(const Dashboard & dashboard, int width, double scale, std::size_t vehicles)
{
  return drawRow(dashboard, width, scale, status_height, vehicles, statusCard);
}

QImage drawErrorCharts(
  const Dashboard & dashboard, int width, double scale, const QString & experiment_id,
  std::size_t vehicles)
{
  return drawRow(dashboard, width, scale, chart_height, vehicles, errorChart, experiment_id);
}

}  // namespace fwis_rviz_plugins
