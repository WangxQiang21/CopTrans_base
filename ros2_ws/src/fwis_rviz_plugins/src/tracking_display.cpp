#include "tracking_hud.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QVariant>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreTextureUnitState.h>
#include <OgreViewport.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>

#include "nav_msgs/msg/odometry.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_default_plugins/displays/marker/marker_common.hpp"
#include "rviz_rendering/render_window.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

// render_system.hpp pulls in X11, whose macros can conflict with Qt headers.
#include "rviz_rendering/render_system.hpp"

namespace fwis_rviz_plugins
{
namespace
{

constexpr auto update_period = std::chrono::milliseconds(100);
constexpr double sample_period = 0.10;
constexpr std::size_t status_queue_depth = 512;
// Keep the full default long-route window at the 10 Hz sampling rate.
constexpr std::size_t max_chart_samples = 9000;
// Nominal display footprint for the default cargo_wedge formation.
constexpr double cargo_length = 1.40;
constexpr double cargo_width = 2.18;
constexpr double cargo_center_forward = 0.50;
constexpr double cargo_center_height = 0.463;
constexpr double cargo_thickness = 0.05;
const std::array<QString, 3> modes = {"Ackermann", "Parallel", "Spin"};
const std::array<std::string, track_count> prefixes = {
  "/ranger1", "/ranger2", "/ranger3", "/cargo"};

class OverlayStrip
{
public:
  explicit OverlayStrip(const std::string & name)
  : name_(name)
  {
    auto & overlays = Ogre::OverlayManager::getSingleton();
    overlay_ = overlays.create(name_);
    overlay_->setZOrder(500);
    panel_ = static_cast<Ogre::PanelOverlayElement *>(
      overlays.createOverlayElement("Panel", name_ + "/Panel"));
    panel_->setMetricsMode(Ogre::GMM_PIXELS);
    overlay_->add2D(panel_);

    material_ = Ogre::MaterialManager::getSingleton().create(
      name_ + "/Material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    auto * pass = material_->getTechnique(0)->getPass(0);
    // QPainter writes premultiplied RGBA pixels.
    pass->setSceneBlending(Ogre::SBF_ONE, Ogre::SBF_ONE_MINUS_SOURCE_ALPHA);
    pass->setDepthCheckEnabled(false);
    pass->setDepthWriteEnabled(false);
    pass->setLightingEnabled(false);
    unit_ = pass->createTextureUnitState();
    unit_->setTextureFiltering(Ogre::TFO_NONE);
    panel_->setMaterialName(material_->getName());
  }

  ~OverlayStrip()
  {
    auto & overlays = Ogre::OverlayManager::getSingleton();
    overlay_->remove2D(panel_);
    overlays.destroyOverlayElement(panel_);
    overlays.destroy(overlay_);
    Ogre::MaterialManager::getSingleton().remove(material_->getHandle());
    material_.reset();
    if (texture_) {
      Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
    }
  }

  void draw(const QImage & pixels, int y)
  {
    auto & textures = Ogre::TextureManager::getSingleton();
    if (size_ != pixels.size()) {
      if (texture_) {
        textures.remove(texture_->getHandle());
      }
      texture_ = textures.createManual(
        name_ + "/Texture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, pixels.width(), pixels.height(), 0, Ogre::PF_BYTE_RGBA,
        Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
      unit_->setTexture(texture_);
      size_ = pixels.size();
      panel_->setDimensions(size_.width(), size_.height());
    }
    const Ogre::PixelBox source(
      pixels.width(), pixels.height(), 1, Ogre::PF_BYTE_RGBA,
      const_cast<uchar *>(pixels.constBits()));
    texture_->getBuffer()->blitFromMemory(source);
    panel_->setPosition(0, y);
    overlay_->show();
  }

  void hide() {overlay_->hide();}

private:
  std::string name_;
  QSize size_;
  Ogre::Overlay * overlay_;
  Ogre::PanelOverlayElement * panel_;
  Ogre::MaterialPtr material_;
  Ogre::TextureUnitState * unit_;
  Ogre::TexturePtr texture_;
};

struct TimedMode
{
  int value;
  double stamp;
};

struct ErrorHistory
{
  double plan_id = -1;
  qint64 trajectory_stamp_ns = 0;
  std::deque<QPointF> points;
  QString phase;
  double last_error_stamp = 0;
};

struct Telemetry
{
  struct Track
  {
    std::optional<Feedback> odom;
    std::optional<TimedMode> mode;
    ErrorHistory history;
  };

  std::array<Track, track_count> tracks;
  std::optional<QPointF> constraint;  // (stamp, metres)
};

bool fresh(double now, double stamp)
{
  return now - stamp >= -0.05 && now - stamp <= feedback_timeout;
}

std::optional<Feedback> readOdometry(const nav_msgs::msg::Odometry & message)
{
  const auto & p = message.pose.pose.position;
  const auto & q = message.pose.pose.orientation;
  const auto & twist = message.twist.twist;
  const double stamp = message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec;
  const double q_length = std::hypot(std::hypot(q.w, q.x), std::hypot(q.y, q.z));
  if (message.header.frame_id != "map" || !std::isfinite(q_length) || q_length < 1e-6 ||
    !std::isfinite(p.x) || !std::isfinite(p.y) ||
    !std::isfinite(std::hypot(twist.linear.x, twist.linear.y)) || !std::isfinite(twist.angular.z))
  {
    return std::nullopt;
  }
  const double x = q.x / q_length;
  const double y = q.y / q_length;
  const double z = q.z / q_length;
  const double w = q.w / q_length;
  const double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return Feedback{stamp, p.x, p.y, yaw, twist.linear.x, twist.linear.y, twist.angular.z};
}

void recordError(const std::string & message, ErrorHistory & history)
{
  const auto row = QJsonDocument::fromJson(QByteArray::fromStdString(message)).object();
  const auto plan = row.value("plan_id");
  const auto trajectory = row.value("trajectory_stamp_ns");
  const auto stamp = row.value("stamp");
  if (row.value("kind").toString() != "execution" ||
    !plan.isDouble() || !trajectory.isDouble() || !stamp.isDouble())
  {
    return;
  }
  const double plan_id = plan.toDouble();
  const qint64 trajectory_stamp_ns = trajectory.toVariant().toLongLong();
  const double now = stamp.toDouble();
  const double elapsed = now - 1e-9 * trajectory_stamp_ns;
  const auto phase = row.value("phase").toString();
  const auto error = row.value("body_error").toArray();

  if (history.plan_id != plan_id || history.trajectory_stamp_ns != trajectory_stamp_ns) {
    history = ErrorHistory{};
    history.plan_id = plan_id;
    history.trajectory_stamp_ns = trajectory_stamp_ns;
  }
  if (history.phase == "COMPLETE" && phase == "COMPLETE") {
    return;
  }
  history.phase = phase;
  if (phase == "STOPPED" || !std::isfinite(elapsed) || elapsed < 0 ||
    error.size() < 2 || !error.at(0).isDouble() || !error.at(1).isDouble())
  {
    return;
  }
  const double cm = 100 * std::hypot(error.at(0).toDouble(), error.at(1).toDouble());
  if (!std::isfinite(cm)) {
    return;
  }
  if (!history.points.empty()) {
    const double dt = elapsed - history.points.back().x();
    if (dt <= 0 || (dt < sample_period - 1e-6 && phase != "COMPLETE")) {
      return;
    }
  }
  history.points.emplace_back(elapsed, cm);
  history.last_error_stamp = now;
  if (history.points.size() > max_chart_samples) {
    history.points.pop_front();
  }
}

Dashboard snapshot(const Telemetry & data, double now)
{
  Dashboard dashboard;
  for (std::size_t index = 0; index < track_count; ++index) {
    const auto & source = data.tracks[index];
    auto & view = dashboard[index];
    if (source.odom && fresh(now, source.odom->stamp)) {
      view.feedback = source.odom;
    }
    if (view.feedback && source.mode && fresh(now, source.mode->stamp)) {
      view.mode = modes[source.mode->value];
    }
    view.phase = source.history.phase;
    view.errors = source.history.points;
  }
  auto & cargo = dashboard[cargo_index];
  const auto & history = data.tracks[cargo_index].history;
  if (cargo.feedback && data.constraint && fresh(now, data.constraint->x())) {
    cargo.constraint_metres = data.constraint->y();
  }
  if (cargo.feedback && !cargo.errors.empty() &&
    (history.phase == "COMPLETE" ||
    (history.phase != "STOPPED" && fresh(now, history.last_error_stamp))))
  {
    cargo.error_cm = cargo.errors.back().y();
  }
  return dashboard;
}

}  // namespace

class TrackingDisplay final : public rviz_common::Display
{
public:
  TrackingDisplay()
  : show_cargo_marker_(new rviz_common::properties::BoolProperty(
        "Show Cargo Marker", true, "Draw the default cargo geometry from /cargo/odom.", this))
  {}

  void reset() override
  {
    Display::reset();
    telemetry_ = {};
    if (cargo_marker_) {
      cargo_marker_->clearMarkers();
      last_cargo_stamp_.reset();
    }
  }

protected:
  void onInitialize() override
  {
    rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
    cargo_marker_ = std::make_unique<rviz_default_plugins::displays::MarkerCommon>(this);
    cargo_marker_->initialize(context_, scene_node_);
  }

  void onEnable() override
  {
    if (!Ogre::OverlayManager::getSingletonPtr()) {
      setStatus(rviz_common::properties::StatusProperty::Error, "HUD", "Ogre overlays unavailable");
      return;
    }
    if (!status_strip_) {
      static unsigned next_id = 0;
      const std::string name = "CopTransTracking/" + std::to_string(next_id++);
      status_strip_ = std::make_unique<OverlayStrip>(name + "/Status");
      chart_strip_ = std::make_unique<OverlayStrip>(name + "/Charts");
    }

    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    clock_ = node->get_clock();
    // Drain telemetry in batches on the GUI thread. RViz's normal executor
    // visits each subscription once per frame and can fall behind the controller.
    callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = node->get_node_base_interface()->get_context();
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_callback_group(callback_group_, node->get_node_base_interface());
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    const auto sensor_qos = rclcpp::SensorDataQoS();
    for (std::size_t index = 0; index < track_count; ++index) {
      const auto & prefix = prefixes[index];
      subscriptions_.push_back(node->create_subscription<nav_msgs::msg::Odometry>(
        prefix + "/odom", sensor_qos,
        [this, index](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          telemetry_.tracks[index].odom = readOdometry(*msg);
        }, options));
      // Keep the beginning of the trace while RViz loads the vehicle meshes.
      subscriptions_.push_back(node->create_subscription<std_msgs::msg::String>(
        prefix + "/controller_status", status_queue_depth,
        [this, index](std_msgs::msg::String::ConstSharedPtr msg) {
          recordError(msg->data, telemetry_.tracks[index].history);
        }, options));
      if (index != cargo_index) {
        subscriptions_.push_back(node->create_subscription<std_msgs::msg::Int32>(
          prefix + "/motion_state", sensor_qos,
          [this, index](std_msgs::msg::Int32::ConstSharedPtr msg) {
            auto & sample = telemetry_.tracks[index].mode;
            if (msg->data >= 0 && static_cast<std::size_t>(msg->data) < modes.size()) {
              sample = TimedMode{msg->data, clock_->now().seconds()};
            } else {
              sample.reset();
            }
          }, options));
      }
    }
    subscriptions_.push_back(node->create_subscription<std_msgs::msg::Float64>(
      "/cargo/constraint_error", sensor_qos,
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        if (std::isfinite(msg->data) && msg->data >= 0) {
          telemetry_.constraint = QPointF(clock_->now().seconds(), msg->data);
        } else {
          telemetry_.constraint.reset();
        }
      }, options));
    next_update_ = {};
  }

  void onDisable() override
  {
    executor_.reset();
    subscriptions_.clear();
    callback_group_.reset();
    reset();
    if (status_strip_) {
      status_strip_->hide();
      chart_strip_->hide();
    }
    context_->queueRender();
  }

  void update(float, float) override
  {
    if (!executor_) {
      return;
    }
    auto * window = context_->getViewManager()->getRenderPanel()->getRenderWindow();
    auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(window);
    if (!viewport) {
      return;
    }
    const QSize size(viewport->getActualWidth(), viewport->getActualHeight());
    const auto now = std::chrono::steady_clock::now();
    if (size == viewport_size_ && now < next_update_) {
      return;
    }
    viewport_size_ = size;
    next_update_ = now + update_period;
    executor_->spin_all(std::chrono::milliseconds(4));
    const auto dashboard = snapshot(telemetry_, clock_->now().seconds());
    updateCargoMarker(show_cargo_marker_->getBool() ? dashboard[cargo_index].feedback : std::nullopt);
    if (size.width() >= 180 && size.height() >= 150) {
      const double scale = dashboardScale(size.width(), size.height(), window->devicePixelRatio());
      status_strip_->draw(drawStatusCards(dashboard, size.width(), scale), 0);
      const auto charts = drawErrorCharts(dashboard, size.width(), scale);
      chart_strip_->draw(charts, size.height() - charts.height());
    } else {
      status_strip_->hide();
      chart_strip_->hide();
    }
    context_->queueRender();
  }

private:
  void updateCargoMarker(const std::optional<Feedback> & feedback)
  {
    if (!feedback) {
      if (last_cargo_stamp_) {
        cargo_marker_->deleteMarker({"cargo", 0});
        last_cargo_stamp_.reset();
      }
      return;
    }
    if (last_cargo_stamp_ == feedback->stamp) {
      return;
    }

    auto marker = std::make_shared<visualization_msgs::msg::Marker>();
    marker->header.frame_id = "map";
    marker->ns = "cargo";
    marker->type = visualization_msgs::msg::Marker::CUBE;
    marker->pose.position.x = feedback->x + cargo_center_forward * std::cos(feedback->yaw);
    marker->pose.position.y = feedback->y + cargo_center_forward * std::sin(feedback->yaw);
    marker->pose.position.z = cargo_center_height;
    marker->pose.orientation.z = std::sin(feedback->yaw / 2);
    marker->pose.orientation.w = std::cos(feedback->yaw / 2);
    marker->scale.x = cargo_length;
    marker->scale.y = cargo_width;
    marker->scale.z = cargo_thickness;
    marker->color.r = 0.34;
    marker->color.g = 0.68;
    marker->color.b = 0.79;
    marker->color.a = 0.28;
    cargo_marker_->processMessage(marker);
    last_cargo_stamp_ = feedback->stamp;
  }

  std::unique_ptr<OverlayStrip> status_strip_;
  std::unique_ptr<OverlayStrip> chart_strip_;
  rviz_common::properties::BoolProperty * show_cargo_marker_;
  std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> cargo_marker_;
  std::optional<double> last_cargo_stamp_;
  QSize viewport_size_;
  std::chrono::steady_clock::time_point next_update_;
  rclcpp::Clock::SharedPtr clock_;
  Telemetry telemetry_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

}  // namespace fwis_rviz_plugins

PLUGINLIB_EXPORT_CLASS(fwis_rviz_plugins::TrackingDisplay, rviz_common::Display)
