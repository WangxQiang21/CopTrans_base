#include "fwis_control/tracking_core.hpp"
#include "control_timing.hpp"
#include "hybrid_tracking.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fwis_v3_msgs/msg/formation_execution_permit.hpp>
#include <fwis_v3_msgs/msg/hybrid_execution_permit.hpp>
#include <fwis_v3_msgs/msg/timed_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

namespace fwis_control
{
namespace
{

bool timing_directory_configured() noexcept
{
  const char * value = std::getenv("COPTRANS_TIMING_DIR");
  return value && value[0] != '\0';
}

using fwis_v3_msgs::msg::FormationExecutionPermit;
using fwis_v3_msgs::msg::TimedTrajectory;
constexpr int64_t ns_per_second = 1000000000;

template<typename Time>
int64_t stamp_ns(const Time & stamp) noexcept
{
  return stamp.nanosec < ns_per_second ? stamp.sec * ns_per_second + stamp.nanosec : -1;
}

double seconds(int64_t stamp) noexcept
{
  return static_cast<double>(stamp) * 1e-9;
}

void load_fields(
  const YAML::Node & node, std::initializer_list<std::pair<const char *, double *>> fields,
  bool all_required = false)
{
  if (!node.IsMap()) {
    throw std::invalid_argument("expected a mapping for controller profile");
  }
  for (const auto & field : node) {
    const auto key = field.first.as<std::string>();
    const auto found = std::find_if(fields.begin(), fields.end(), [&key](const auto & item) {
        return key == item.first;
      });
    if (found == fields.end()) {
      throw std::invalid_argument("unknown controller profile key: " + key);
    }
    *found->second = field.second.as<double>();
  }
  if (all_required && node.size() != fields.size()) {
    throw std::invalid_argument("controller profile is missing required fields");
  }
}

PIDGains load_pid(const YAML::Node & node)
{
  PIDGains gains;
  load_fields(node, {{"kp", &gains.kp}, {"ki", &gains.ki}, {"kd", &gains.kd},
    {"integral_limit", &gains.integral_limit}, {"derivative_tau", &gains.derivative_tau}}, true);
  return gains;
}

Configuration load_configuration(const std::string & path)
{
  const auto data = YAML::LoadFile(path);
  Configuration cfg;
  auto & l = cfg.limits;
  load_fields(data["limits"], {
    {"max_speed", &l.max_speed}, {"max_yaw_rate", &l.max_yaw_rate},
    {"max_accel", &l.max_accel}, {"max_yaw_accel", &l.max_yaw_accel},
    {"mechanical_radius", &l.mechanical_radius}, {"switch_radius", &l.switch_radius},
    {"radius_margin", &l.radius_margin}, {"parallel_min_vy", &l.parallel_min_vy}});
  auto & t = cfg.timing;
  load_fields(data["timing"], {
    {"frequency", &t.frequency}, {"state_timeout", &t.state_timeout},
    {"mode_timeout", &t.mode_timeout}, {"plan_admission_timeout", &t.plan_admission_timeout},
    {"clock_tolerance", &t.clock_tolerance}, {"approach_time", &t.approach_time},
    {"stable_time", &t.stable_time}, {"switch_timeout", &t.switch_timeout},
    {"settle_time", &t.settle_time}, {"stop_speed", &t.stop_speed},
    {"stop_yaw_rate", &t.stop_yaw_rate}, {"final_stop_timeout", &t.final_stop_timeout}});
  auto & p = cfg.preflight;
  load_fields(data["preflight"], {
    {"min_step", &p.min_step}, {"max_step", &p.max_step},
    {"stationary_position", &p.stationary_position}, {"stationary_yaw", &p.stationary_yaw},
    {"stop_declared_speed", &p.stop_declared_speed}, {"velocity_tolerance", &p.velocity_tolerance},
    {"oblique_yaw_tolerance", &p.oblique_yaw_tolerance},
    {"spin_position_tolerance", &p.spin_position_tolerance},
    {"declared_speed_warn", &p.declared_speed_warn},
    {"declared_speed_tolerance", &p.declared_speed_tolerance},
    {"acceleration_tolerance", &p.acceleration_tolerance}});
  auto & n = cfg.nonlinear;
  load_fields(data["nonlinear"], {
    {"ack_x", &n.ack_x}, {"ack_y", &n.ack_y}, {"ack_yaw", &n.ack_yaw},
    {"oblique_xy", &n.oblique_xy}, {"spin_yaw", &n.spin_yaw}});
  const auto pid = data["pid"];
  if (!pid.IsMap() || pid.size() != 5) {
    throw std::invalid_argument("missing or unknown mode-specific PID gains");
  }
  cfg.ack_x = load_pid(pid["ack_x"]);
  cfg.ack_y = load_pid(pid["ack_y"]);
  cfg.ack_yaw = load_pid(pid["ack_yaw"]);
  cfg.oblique = load_pid(pid["oblique"]);
  cfg.spin = load_pid(pid["spin"]);
  cfg.validate();
  return cfg;
}

// One producer and one consumer. The control thread never waits for space.
template<typename Value, std::size_t Size>
class Queue
{
public:
  bool has_room(std::size_t count = 1) const noexcept
  {
    return write_.load(std::memory_order_relaxed) - read_.load(std::memory_order_acquire)
           <= Size - count;
  }

  bool push(const Value & value) noexcept
  {
    if (!has_room()) {
      return false;
    }
    const auto index = write_.load(std::memory_order_relaxed);
    values_[index % Size] = value;
    write_.store(index + 1, std::memory_order_release);
    return true;
  }

  bool pop(Value & value) noexcept
  {
    const auto index = read_.load(std::memory_order_relaxed);
    if (index == write_.load(std::memory_order_acquire)) {
      return false;
    }
    value = values_[index % Size];
    read_.store(index + 1, std::memory_order_release);
    return true;
  }

private:
  std::array<Value, Size> values_{};
  std::atomic<std::size_t> write_{}, read_{};
};

struct Status
{
  uint32_t plan_id{};
  int64_t trajectory_stamp_ns{};
  std::string_view model_sha256;
  bool native_ready_to_start{};
  double now{};
  std::string_view admission;
  const Preflight * checked{};
  Output output;
  std::optional<Mode> feedback_mode;
  std::optional<std::array<double, 3>> body_error;
  double compute_s{};
  double callback_interval_s{};
  std::optional<ControlTiming> timing;
};

void json_number(std::ostream & stream, double value)
{
  if (std::isfinite(value)) {
    stream << value;
  } else {
    stream << "null";
  }
}

void json_vec(std::ostream & stream, const std::array<double, 3> & items)
{
  stream << '[';
  for (std::size_t i = 0; i < items.size(); ++i) {
    if (i != 0) {
      stream << ',';
    }
    json_number(stream, items[i]);
  }
  stream << ']';
}

void json_mode(std::ostream & stream, std::optional<Mode> mode)
{
  if (mode) {
    stream << static_cast<int>(*mode);
  } else {
    stream << "null";
  }
}

std_msgs::msg::String status_message(const Status & row, const std::string & algorithm)
{
  // Status string fields are closed sets of literals, never ROS/user input.
  // This formatter runs on the background executor.
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(17) << "{\"plan_id\":" << row.plan_id
         << ",\"trajectory_stamp_ns\":" << row.trajectory_stamp_ns << ",\"stamp\":" << row.now;
  if (!row.model_sha256.empty()) stream << ",\"model_sha256\":\"" << row.model_sha256 << '\"';
  if (!row.admission.empty()) {
    stream << ",\"kind\":\"admission\",\"result\":\"" << row.admission << '"';
    if (row.checked) {
      stream << ",\"phase\":\"" << name(row.output.phase) << "\",\"issues\":[";
      bool comma = false;
      for (const auto & issue : row.checked->issues) {
        if (comma) {
          stream << ',';
        }
        comma = true;
        stream << "{\"level\":\"" << (issue.warning ? "WARN" : "FAIL")
               << "\",\"code\":\"" << issue.code << "\",\"start\":" << issue.start
               << ",\"end\":" << issue.end << ",\"value\":";
        json_number(stream, issue.value);
        stream << ",\"limit\":";
        json_number(stream, issue.limit);
        stream << '}';
      }
      stream << ']';
    }
  } else {
    const auto & out = row.output;
    if (!row.model_sha256.empty()) stream << ",\"preflight_accepted\":" << (row.native_ready_to_start ? "true" : "false");
    stream << ",\"kind\":\"execution\",\"phase\":\"" << name(out.phase)
           << "\",\"algorithm\":\"" << algorithm << "\",\"reference_time\":";
    if (out.reference) {
      json_number(stream, out.reference->time);
    } else {
      stream << "null";
    }
    stream << ",\"planned_mode\":";
    json_mode(stream, out.reference ? std::optional<Mode>(out.reference->mode) : std::nullopt);
    stream << ",\"feedback_mode\":";
    json_mode(stream, row.feedback_mode);
    stream << ",\"requested_mode\":";
    json_mode(stream, out.requested_mode);
    stream << ",\"body_error\":";
    if (row.body_error) {
      json_vec(stream, *row.body_error);
    } else {
      stream << "null";
    }
    stream << ",\"raw\":";
    json_vec(stream, {out.raw.vx, out.raw.vy, out.raw.wz});
    stream << ",\"command\":";
    json_vec(stream, {out.command.vx, out.command.vy, out.command.wz});
    stream << ",\"reasons\":[";
    for (std::size_t i = 0; i < out.reasons.count; ++i) {
      if (i != 0) {
        stream << ',';
      }
      stream << '"' << out.reasons.codes[i] << '"';
    }
    stream << "],\"compute_s\":" << row.compute_s
           << ",\"callback_interval_s\":" << row.callback_interval_s;
    if (row.timing) {
      const auto & t = *row.timing;
      stream << ",\"control_timing\":{\"timer_due_clock_ns\":" << t.timer.due_clock_ns
             << ",\"timer_called_clock_ns\":" << t.timer.called_clock_ns
             << ",\"timer_called_steady_ns\":" << t.timer.called_steady_ns
             << ",\"timer_period_ns\":" << t.timer.period_ns
             << ",\"callback_started_steady_ns\":" << t.started_steady_ns
             << ",\"callback_ended_steady_ns\":" << t.ended_steady_ns
             << ",\"callback_ended_clock_ns\":" << t.ended_clock_ns
             << ",\"core_ns\":" << t.core_ns
             << ",\"command_publish_steady_ns\":" << t.command_publish_steady_ns
             << ",\"command_published_steady_ns\":" << t.command_published_steady_ns
             << ",\"thread_cpu_ns\":" << t.thread_at_entry.cpu_ns
             << ",\"thread_voluntary_switches\":" << t.thread_at_entry.voluntary_switches
             << ",\"thread_involuntary_switches\":" << t.thread_at_entry.involuntary_switches
             << ",\"odom_stamp_ns\":" << t.odom.odom_stamp_ns
             << ",\"odom_source_realtime_ns\":" << t.odom.source_realtime_ns
             << ",\"odom_received_realtime_ns\":" << t.odom.received_realtime_ns
             << ",\"odom_callback_realtime_ns\":" << t.odom.callback_realtime_ns
             << ",\"odom_callback_steady_ns\":" << t.odom.callback_steady_ns
             << ",\"status_drops\":" << t.status_drops << '}';
    }
  }
  stream << '}';
  std_msgs::msg::String message;
  message.data = stream.str();
  return message;
}

class TrackingNode : public rclcpp::Node
{
public:
  TrackingNode()
  : Node("tracking_controller"),
    config_(load_configuration(declare_parameter(
          "config_file", ament_index_cpp::get_package_share_directory("fwis_control") +
          "/config/ranger_tracking.yaml"))),
    algorithm_(declare_parameter("algorithm", std::string("nonlinear"))),
    permit_topic_(declare_parameter("permit_topic", std::string{})),
    future_window_(declare_parameter("future_start_window_s", 0.)),
    timing_enabled_(declare_parameter("timing_telemetry", timing_directory_configured())),
    engine_(config_, algorithm_ == "pid")
  {
    if (algorithm_ != "nonlinear" && algorithm_ != "pid") {
      throw std::invalid_argument("unknown tracking algorithm: " + algorithm_);
    }
    if (!permit_topic_.empty() && !(std::isfinite(future_window_) && future_window_ > 0.)) {
      throw std::invalid_argument("coordinated tracking requires a positive future start window");
    }
    expected_frame_ = get_namespace();
    if (!expected_frame_.empty() && expected_frame_.front() == '/') {
      expected_frame_.erase(0, 1);
    }
    expected_frame_ += "/base_link";

    auto topic = declare_parameter("trajectory_topic", std::string("/plan/timed_trajectory"));
    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions control_options;
    control_options.callback_group = control_group_;

    command_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    mode_request_ = create_publisher<std_msgs::msg::Int32>("sim_request_mode", 1);
    status_ = create_publisher<std_msgs::msg::String>("controller_status", 10);

    if (declare_parameter("hybrid_joint", false)) {
      vehicle_id_ = declare_parameter("vehicle_id", 1);
      if (vehicle_id_ < 1 || vehicle_id_ > 3 || permit_topic_.empty()) {
        throw std::invalid_argument("native model requires a vehicle ID and a group permit");
      }
      grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map",hybrid_ros::durable(),
        [this](const nav_msgs::msg::OccupancyGrid & msg) {
          try {
            auto next=hybrid_ros::grid_from(msg);
            if (hybrid_grid_ && next->hash()!=hybrid_grid_->hash()) invalidate_native();
            hybrid_grid_=std::move(next);
            if(deferred_model_){auto model=std::move(deferred_model_);on_model(*model);}
          } catch(const std::exception & e) {
            hybrid_grid_.reset(); invalidate_native(); RCLCPP_WARN(get_logger(),"%s",e.what());
          }
        });
      model_sub_=create_subscription<hybrid_ros::Envelope>(
        declare_parameter("hybrid_plan_topic",std::string(hybrid_ros::envelope_topic)),hybrid_ros::durable(),
        [this](hybrid_ros::Envelope::ConstSharedPtr m){
          if(!hybrid_grid_){deferred_model_=std::move(m);return;}
          on_model(*m);
        });
    } else trajectory_sub_ = create_subscription<TimedTrajectory>(
      topic, rclcpp::QoS(1).transient_local(),
      [this](const TimedTrajectory & message) {on_trajectory(message);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 5,
      [this](const nav_msgs::msg::Odometry & msg, const rclcpp::MessageInfo & info) {
        on_odom(msg, info);
      }, control_options);
    mode_sub_ = create_subscription<std_msgs::msg::Int32>(
      "motion_state", 5, [this](const std_msgs::msg::Int32 & msg) {on_mode(msg.data);},
      control_options);
    if (!permit_topic_.empty()) {
      if(model_sub_){
        native_permit_sub_=create_subscription<fwis_v3_msgs::msg::HybridExecutionPermit>(
          permit_topic_,1,[this](const fwis_v3_msgs::msg::HybridExecutionPermit& msg){
            FormationExecutionPermit copy;copy.plan_id=msg.plan_id;copy.stamp=msg.stamp;
            copy.start=msg.start;copy.enabled=msg.enabled;
            permit_=copy;
            native_permit_hash_.fill(0);
            if(msg.model_sha256.size()==64)std::copy_n(msg.model_sha256.begin(),64,native_permit_hash_.begin());
          },control_options);
      } else permit_sub_ = create_subscription<FormationExecutionPermit>(
        permit_topic_, 1, [this](const FormationExecutionPermit & msg) {permit_ = msg;},
        control_options);
    }
    const auto period = rclcpp::Duration::from_seconds(config_.timing.period());
    if (timing_enabled_) {
      control_timer_ = std::make_shared<TimingTimer>(
        get_clock(), period.to_chrono<std::chrono::nanoseconds>(),
        get_node_base_interface()->get_context(), [this](TimerDispatch dispatch) {step(dispatch);});
      get_node_timers_interface()->add_timer(control_timer_, control_group_);
    } else {
      control_timer_ = rclcpp::create_timer(
        this, get_clock(), period, [this]() {step();}, control_group_);
    }
    watchdog_ = create_wall_timer(
      std::chrono::duration<double>(config_.timing.period()), [this]() {watchdog();},
      control_group_);
    status_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {drain_status();});
  }

  ~TrackingNode() override
  {
    // Executors have finished and the raw pointers no longer cross threads.
    auto * deferred = static_cast<PreparedPlan *>(nullptr);
    while (retired_.pop(deferred)) {
      delete deferred;
    }
    delete incoming_.exchange(nullptr);
    delete pending_;
    delete plan_;
  }

  const rclcpp::CallbackGroup::SharedPtr & control_group() const noexcept
  {
    return control_group_;
  }

  void stop_motion()
  {
    publish_command({}, {});
    drain_status();
  }

private:
  void invalidate_native()
  {
    auto cleared=std::make_unique<PreparedPlan>();
    cleared->trajectory.plan_id=native_id_;
    cleared->trajectory.stamp_ns=native_epoch_;
    cleared->model_sha256=native_hash_;
    delete incoming_.exchange(cleared.release());
  }

  void on_model(const hybrid_ros::Envelope & m)
  {
    try {
      auto data=std::make_shared<hybrid_ros::Decoded>(hybrid_ros::decode(m,hybrid_grid_));
      if(m.plan_id==native_id_){
        if(m.model_sha256!=native_hash_)throw std::invalid_argument("model changed for existing plan ID");
        return;
      }
      if(m.plan_id<native_id_)throw std::invalid_argument("stale native plan ID");
      native_id_=m.plan_id;native_epoch_=hybrid_ros::ns(m.header.stamp);native_hash_=m.model_sha256;
      auto checked=std::make_unique<PreparedPlan>(prepare_hybrid(data,vehicle_id_,native_epoch_,native_hash_,config_));
      delete incoming_.exchange(checked.release());
    } catch(const std::exception & e) {
      invalidate_native();
      RCLCPP_WARN(get_logger(),"Rejected hybrid plan: %s",e.what());
    }
  }

  void on_trajectory(const TimedTrajectory & message)
  {
    const auto stamp = stamp_ns(message.header.stamp);
    const auto key = std::make_tuple(message.plan_id, stamp, !message.points.empty());
    if (last_key_ == key) {
      return;
    }
    last_key_ = key;
    Trajectory trajectory{stamp, message.header.frame_id, message.plan_id, {}};
    trajectory.points.reserve(message.points.size());
    for (const auto & p : message.points) {
      trajectory.points.push_back({stamp_ns(p.time_from_start),
        {p.center_pose.x, p.center_pose.y, p.center_pose.theta}, p.v, p.mode});
    }
    auto prepared = std::make_unique<PreparedPlan>(
      prepare(std::move(trajectory), config_, !permit_topic_.empty()));
    delete incoming_.exchange(prepared.release(), std::memory_order_acq_rel);
  }

  void on_odom(const nav_msgs::msg::Odometry & msg, const rclcpp::MessageInfo & message_info)
  {
    const auto stamp = stamp_ns(msg.header.stamp);
    if (timing_enabled_) {
      const auto & middleware = message_info.get_rmw_message_info();
      odometry_timing_ = {stamp, middleware.source_timestamp, middleware.received_timestamp,
        realtime_ns(), steady_ns()};
    }
    odom_.reset();
    if (stamp < 0 || msg.header.frame_id != "map" || msg.child_frame_id != expected_frame_) {
      return;
    }
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (norm == 0. || !std::isfinite(norm) || !std::isfinite(p.x) || !std::isfinite(p.y)) {
      return;
    }
    const double x = q.x / norm;
    const double y = q.y / norm;
    const double z = q.z / norm;
    const double w = q.w / norm;
    const auto & t = msg.twist.twist;
    odom_ = Observation{{p.x, p.y, std::atan2(2. * (w * z + x * y), 1. - 2. * (y * y + z * z))},
      {t.linear.x, t.linear.y, t.angular.z}, seconds(stamp), {}, 0.};
  }

  void on_mode(int raw_mode)
  {
    if (valid_mode(raw_mode)) {
      mode_ = std::make_pair(static_cast<Mode>(raw_mode), now().seconds());
    } else {
      mode_.reset();
    }
  }

  bool permit_invalid(double ros_now) const noexcept
  {
    const bool matches = permit_ && permit_->plan_id == plan_->trajectory.plan_id &&
      stamp_ns(permit_->start) == plan_->trajectory.stamp_ns &&
      (!model_sub_ || std::string_view(native_permit_hash_.data(),64) == plan_->model_sha256);
    if (matches && !permit_->enabled) {
      return true;
    }
    if (ros_now < engine_.start()) {
      return false;
    }
    if (!matches) {
      return true;
    }
    const double age = ros_now - seconds(stamp_ns(permit_->stamp));
    return age < -config_.timing.clock_tolerance || age > config_.timing.state_timeout;
  }

  void publish_command(
    const Twist & twist, std::optional<Mode> mode, ControlTiming * timing = nullptr)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = twist.vx;
    msg.linear.y = twist.vy;
    msg.angular.z = twist.wz;
    if (timing) {
      timing->command_publish_steady_ns = steady_ns();
    }
    command_->publish(msg);
    if (timing) {
      timing->command_published_steady_ns = steady_ns();
    }
    if (mode || mode_request_pending_) {
      std_msgs::msg::Int32 request;
      request.data = mode ? static_cast<int>(*mode) : -1;
      mode_request_->publish(request);
      mode_request_pending_ = mode.has_value();
    }
  }

  void take_pending() noexcept
  {
    if (pending_ && !retired_.has_room()) {
      return;
    }
    if (auto * fresh = incoming_.exchange(nullptr, std::memory_order_acq_rel)) {
      if (pending_) {
        retired_.push(pending_);
      }
      pending_ = fresh;
    }
  }

  void queue_admission(
    std::string_view result, double ros_now, const Preflight * check = nullptr) noexcept
  {
    Status row;
    row.plan_id = plan_->trajectory.plan_id;
    row.trajectory_stamp_ns = plan_->trajectory.stamp_ns;
    row.model_sha256 = plan_->model_sha256;
    row.now = ros_now;
    row.admission = result;
    row.checked = check;
    row.output.phase = engine_.phase();
    status_rows_.push(row);
  }

  void queue_execution(
    const Output & output, const std::optional<Observation> & state, double ros_now,
    WallClock::time_point begin, double callback_interval = 0.,
    std::optional<ControlTiming> timing = {})
  {
    if (!plan_) {
      return;
    }
    Status row;
    row.plan_id = plan_->trajectory.plan_id;
    row.trajectory_stamp_ns = plan_->trajectory.stamp_ns;
    row.model_sha256 = plan_->model_sha256;
    row.now = ros_now;
    row.output = output;
    // Re-announce successful preflight during the stationary, pre-epoch wait.
    // DDS discovery may complete after this tracker's one-shot admission event.
    row.native_ready_to_start = plan_->model_sample && engine_.active() && output.phase == Phase::WaitStart;
    if (state) {
      row.feedback_mode = state->mode;
      if (output.reference) {
        row.body_error = state->pose.error_to(output.reference->pose);
      }
    }
    if (timing) {
      timing->ended_clock_ns = now().nanoseconds();
      timing->ended_steady_ns = steady_ns();
      timing->status_drops = status_drops_;
      row.timing = timing;
    }
    row.compute_s = std::chrono::duration<double>(WallClock::now() - begin).count();
    row.callback_interval_s = callback_interval;
    if (!status_rows_.push(row)) {
      ++status_drops_;
    }
  }

  void activate_pending() noexcept
  {
    if (plan_) {
      retired_.push(plan_);
    }
    plan_ = std::exchange(pending_, nullptr);
  }

  void step(std::optional<TimerDispatch> dispatch = {})
  {
    const auto begin = WallClock::now();
    std::optional<ControlTiming> timing;
    if (dispatch) {
      timing.emplace();
      timing->timer = *dispatch;
      timing->odom = odometry_timing_;
      timing->started_steady_ns = steady_ns(begin);
      timing->thread_at_entry = thread_counters();
    }
    const double callback_interval = last_tick_wall_ ?
      std::chrono::duration<double>(begin - *last_tick_wall_).count() : 0.;
    last_tick_wall_ = begin;
    const double ros_now = now().seconds();
    auto state = mode_ ? odom_ : std::nullopt;
    if (state) {
      state->mode = mode_->first;
      state->mode_stamp = mode_->second;
    }
    take_pending();
    if (pending_ && (!plan_ || retired_.has_room()) && status_rows_.has_room(2)) {
      if (pending_->trajectory.points.empty()) {
        engine_.abort("planner_cleared_trajectory");
        activate_pending();
        queue_admission("CLEARED", ros_now);
      } else if (engine_.active()) {
        const auto stopped = engine_.abort("trajectory_replaced_during_execution");
        publish_command({}, {}, timing ? &*timing : nullptr);
        queue_execution(stopped, {}, ros_now, begin, callback_interval, timing);
        return;
      } else if (engine_.feedback_is_fresh(state, ros_now) &&
        state->twist.speed() <= config_.timing.stop_speed &&
        std::abs(state->twist.wz) <= config_.timing.stop_yaw_rate)
      {
        activate_pending();
        if (plan_->model_sample) {
          const auto error = state->pose.error_to(plan_->trajectory.points.front().pose);
          if (std::hypot(error[0],error[1]) > .10 || std::abs(wrap(error[2])) > .10) {
            engine_.abort("model_start_differs_from_feedback");
            queue_admission("REJECTED",ros_now);
            publish_command({}, {});
            return;
          }
        }
        const auto & pre = engine_.admit(*plan_, state->mode, ros_now,
          permit_topic_.empty() ? 0. : future_window_);
        queue_admission(pre.result(), ros_now, &pre);
      } else if (ros_now - seconds(pending_->trajectory.stamp_ns) >
        config_.timing.plan_admission_timeout)
      {
        activate_pending();
        engine_.abort("trajectory_expired_waiting_for_stopped_feedback");
        queue_admission("EXPIRED", ros_now);
      }
    }

    const bool invalid_permit =
      !permit_topic_.empty() && engine_.active() && permit_invalid(ros_now);
    const auto core_started = timing ? steady_ns() : 0;
    const auto result = invalid_permit ?
      engine_.abort("formation_permit_missing_or_revoked") : engine_.tick(ros_now, state);
    if (timing) {
      timing->core_ns = steady_ns() - core_started;
    }
    publish_command(result.command, result.requested_mode, timing ? &*timing : nullptr);
    queue_execution(result, state, ros_now, begin, callback_interval, timing);
  }

  void watchdog()
  {
    const auto begin = WallClock::now();
    if (!engine_.active() || !last_tick_wall_ ||
      std::chrono::duration<double>(begin - *last_tick_wall_).count() <= config_.timing.state_timeout)
    {
      return;
    }
    const auto result = engine_.abort("control_clock_jump_or_overrun");
    publish_command({}, {});
    queue_execution(result, {}, now().seconds(), begin);
  }

  void drain_status()
  {
    // Synchronize with retirement before reading statuses. No new status may
    // refer to a plan that the control thread has already retired.
    std::array<std::unique_ptr<PreparedPlan>, 32> ready;
    for (auto & item : ready) {
      PreparedPlan * retired;
      if (!retired_.pop(retired)) {
        break;
      }
      item.reset(retired);
    }
    Status row;
    for (std::size_t i = 0; i < 128 && status_rows_.pop(row); ++i) {
      status_->publish(status_message(row, algorithm_));
    }
  }

  const Configuration config_;
  const std::string algorithm_, permit_topic_;
  const double future_window_;
  const bool timing_enabled_;
  std::string expected_frame_;
  Execution engine_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_request_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<TimedTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<hybrid_ros::Envelope>::SharedPtr model_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  hybrid_ros::Envelope::ConstSharedPtr deferred_model_;
  std::shared_ptr<const hybrid_ros::hj::GridSnapshot> hybrid_grid_;
  int vehicle_id_{};
  uint32_t native_id_{};
  int64_t native_epoch_{};
  std::string native_hash_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
  rclcpp::Subscription<FormationExecutionPermit>::SharedPtr permit_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_, watchdog_, status_timer_;

  std::optional<std::tuple<uint32_t, int64_t, bool>> last_key_;
  std::atomic<PreparedPlan *> incoming_{};
  Queue<PreparedPlan *, 32> retired_;
  Queue<Status, 128> status_rows_;
  PreparedPlan * plan_{};
  PreparedPlan * pending_{};
  std::optional<Observation> odom_;
  OdometryTiming odometry_timing_;
  uint64_t status_drops_{};
  std::optional<std::pair<Mode, double>> mode_;
  std::optional<FormationExecutionPermit> permit_;
  rclcpp::Subscription<fwis_v3_msgs::msg::HybridExecutionPermit>::SharedPtr native_permit_sub_;
  std::array<char,64> native_permit_hash_{};
  std::optional<WallClock::time_point> last_tick_wall_;
  bool mode_request_pending_{};
};

volatile std::sig_atomic_t stopping = 0;

void stop(int) noexcept
{
  stopping = 1;
}

}  // namespace
}  // namespace fwis_control

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, fwis_control::stop);
  std::signal(SIGTERM, fwis_control::stop);
  try {
    auto node = std::make_shared<fwis_control::TrackingNode>();
    rclcpp::executors::SingleThreadedExecutor background;
    rclcpp::executors::SingleThreadedExecutor control;
    background.add_node(node);
    control.add_callback_group(node->control_group(), node->get_node_base_interface());
    std::exception_ptr control_error;
    std::atomic<bool> control_failed{false};
    std::atomic<bool> control_stop{false};
    std::thread worker([&]() {
        try {
          while (rclcpp::ok() && !control_stop.load(std::memory_order_relaxed)) {
            control.spin_once(100ms);
          }
        } catch (...) {
          control_error = std::current_exception();
          control_failed.store(true, std::memory_order_release);
        }
      });
    std::exception_ptr background_error;
    try {
      while (rclcpp::ok() && !fwis_control::stopping &&
        !control_failed.load(std::memory_order_acquire))
      {
        background.spin_once(100ms);
      }
    } catch (...) {
      background_error = std::current_exception();
    }
    control_stop.store(true, std::memory_order_relaxed);
    control.cancel();
    worker.join();
    if (rclcpp::ok()) {
      node->stop_motion();
    }
    if (control_error) {
      std::rethrow_exception(control_error);
    }
    if (background_error) {
      std::rethrow_exception(background_error);
    }
  } catch (const std::exception & error) {
    std::cerr << "C++ tracking controller failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
