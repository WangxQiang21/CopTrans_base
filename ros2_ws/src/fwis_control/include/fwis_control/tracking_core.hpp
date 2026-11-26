#ifndef FWIS_CONTROL_TRACKING_CORE_HPP_
#define FWIS_CONTROL_TRACKING_CORE_HPP_

#include <array>
#include <functional>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace fwis_control
{

// Poses use the map frame. Twist and error vectors use the vehicle body frame.
// Linear dimensions use metres, heading uses radians, and times use seconds
// unless the field is explicitly marked in nanoseconds.
enum class Mode : uint8_t {Ackermann, Oblique, Spin};
bool valid_mode(int mode) noexcept;
double wrap(double angle) noexcept;

struct Pose
{
  double x{}, y{}, yaw{};
  std::array<double, 3> error_to(const Pose & reference) const noexcept;
};

struct Twist
{
  double vx{}, vy{}, wz{};
  double speed() const noexcept;
  bool zero() const noexcept;
  bool finite() const noexcept;
  Twist interpolate(const Twist & other, double fraction) const noexcept;
};

Pose advance(const Pose & pose, const Twist & twist, double dt) noexcept;
Twist between(const Pose & first, const Pose & last, double dt) noexcept;

struct Point
{
  int64_t time_ns{};
  Pose pose;
  double speed{};
  int mode{};
  double time() const noexcept;
};

struct Trajectory
{
  int64_t stamp_ns{};
  std::string frame;
  uint32_t plan_id{};
  std::vector<Point> points;
};

struct Reference
{
  double time{};
  Pose pose;
  Twist twist;
  Mode mode{};
};

struct Observation
{
  Pose pose;
  Twist twist;
  double stamp{};
  Mode mode{};
  double mode_stamp{};
};

struct Limits
{
  double max_speed{.28}, max_yaw_rate{.55}, max_accel{.45}, max_yaw_accel{.70};
  double mechanical_radius{.810}, switch_radius{.47644}, radius_margin{.02};
  double parallel_min_vy{.00001};
  double high_steer_speed{0.}, high_steer_angle{.3490658503988659};
  double wheelbase{.494}, track{.364};
  double guard_radius() const noexcept;
  void validate() const;
};

struct Timing
{
  double frequency{25.}, state_timeout{.20}, mode_timeout{.20};
  double plan_admission_timeout{.70}, clock_tolerance{.04}, approach_time{.35};
  double stable_time{.08}, switch_timeout{.30}, settle_time{.08};
  double stop_speed{.012}, stop_yaw_rate{.020}, final_stop_timeout{1.5};
  double period() const noexcept;
  double minimum_hold() const noexcept;
  void validate() const;
};

struct PreflightSettings
{
  double min_step{.0005}, max_step{.25}, stationary_position{1e-8}, stationary_yaw{1e-8};
  double stop_declared_speed{.005}, velocity_tolerance{.0003}, oblique_yaw_tolerance{.003};
  double spin_position_tolerance{.003}, declared_speed_warn{.01}, declared_speed_tolerance{.025};
  double acceleration_tolerance{.035};
  void validate() const;
};

struct NonlinearGains
{
  double ack_x{1.35}, ack_y{9.0}, ack_yaw{6.0}, oblique_xy{1.65}, spin_yaw{2.0};
  void validate() const;
};

struct PIDGains
{
  double kp{}, ki{}, kd{}, integral_limit{}, derivative_tau{};
  void validate() const;
};

struct Configuration
{
  Limits limits;
  Timing timing;
  PreflightSettings preflight;
  NonlinearGains nonlinear;
  PIDGains ack_x{1.20, .22, .10, .05, .11};
  PIDGains ack_y{1.35, .12, .12, .08, .11};
  PIDGains ack_yaw{1.35, .10, .09, .08, .11};
  PIDGains oblique{1.40, .22, .12, .08, .11};
  PIDGains spin{1.80, .20, .10, .08, .11};
  void validate() const;
};

// Strings are static diagnostic literals. Storage is fixed for the control loop.
struct Reasons
{
  std::array<std::string_view, 8> codes{};
  std::size_t count{};
  void add(std::string_view code) noexcept;
};

class SafetyFilter
{
public:
  explicit SafetyFilter(Limits limits) : limits_(limits) {}
  struct Result {Twist command; Reasons reasons;};
  void reset() noexcept;
  const Twist & previous() const noexcept {return previous_;}
  Result apply(
    Twist raw, Mode mode, double dt, double direction = 0.,
    std::optional<double> oblique_reference_vy = {}, bool continuous_oblique = false) noexcept;

private:
  Limits limits_;
  Twist previous_;
  Mode mode_{Mode::Ackermann};
};

class Feedback
{
public:
  Feedback(const Configuration & configuration, bool pid);
  void reset() noexcept;
  Twist update(
    const Reference & reference, const Observation & state, double dt,
    double direction) noexcept;
  void accept(const Twist & applied) noexcept;

private:
  struct Axis
  {
    PIDGains gains;
    bool angular{};
    double integral{}, previous_integral{}, derivative{};
    std::optional<double> previous_error;
    double update(double error, double dt) noexcept;
    void accept(double saturation) noexcept;
    void reset() noexcept;
  };

  NonlinearGains nonlinear_;
  bool pid_;
  std::array<Axis, 6> axes_;
  std::optional<Mode> last_mode_;
  double last_direction_{};
  Twist raw_;
};

struct Issue
{
  std::string_view code;
  double start{}, end{}, value{}, limit{};
  bool warning{};
};

struct Switch
{
  double time{}, resume{};
  Mode mode{};
};

struct Interval
{
  double start{}, end{};
  Mode mode{};
  Twist velocity;
  bool stationary{};
  double middle() const noexcept;
};

struct Preflight
{
  std::vector<Issue> issues;
  std::vector<Switch> switches;
  std::vector<Interval> intervals;
  bool valid{};
  std::string_view result() const noexcept;
};

Preflight preflight(
  const Trajectory & trajectory, const Limits & limits, const PreflightSettings & settings,
  double switch_budget, Mode initial_mode, bool allow_axial_oblique = false);

Reference sample(
  const Trajectory & trajectory, const Preflight & checked, double seconds) noexcept;

struct PreparedPlan
{
  Trajectory trajectory;
  Preflight matching_mode;
  Preflight different_mode;
  std::string model_sha256;
  std::function<Reference(double)> model_sample;
  const Preflight & checked(Mode initial_mode) const noexcept;
};

PreparedPlan prepare(Trajectory trajectory, const Configuration & config, bool allow_axial_oblique);

enum class Phase : uint8_t {
  Idle, WaitStart, Tracking, ApproachSwitch, Stopping, RequestModeSwitch,
  WaitModeConfirmation, Settle, FinalStop, Complete, Stopped, Rejected, StaleTrajectory
};
std::string_view name(Phase phase) noexcept;

struct Output
{
  Phase phase{Phase::Idle};
  Twist command;
  Twist raw;
  std::optional<Reference> reference;
  std::optional<Mode> requested_mode;
  Reasons reasons;
};

// Each Execution belongs to one control thread. The caller owns immutable plans
// through completion or abort, and prepares them outside that thread.
class Execution
{
public:
  Execution(const Configuration & config, bool pid);
  const Preflight & admit(
    const PreparedPlan & plan, Mode initial_mode, double now, double future_window = 0.) noexcept;
  Output abort(std::string_view reason, std::optional<Reference> reference = {}) noexcept;
  bool feedback_is_fresh(const std::optional<Observation> & observation, double now) const noexcept;
  Output tick(double now, const std::optional<Observation> & observation) noexcept;
  bool active() const noexcept {return plan_ != nullptr;}
  double start() const noexcept {return start_;}
  Phase phase() const noexcept {return phase_;}

private:
  bool stopped(const Observation & observation) const noexcept;
  Output brake(
    Phase phase, std::optional<Reference> reference, Mode mode, double dt,
    std::optional<Mode> requested = {}) noexcept;
  std::optional<Output> transition(
    const Switch & event, double now, double elapsed, const Observation & observation,
    const Reference & reference, double dt) noexcept;

  Timing timing_;
  Feedback controller_;
  SafetyFilter filter_;
  const PreparedPlan * plan_{};
  const Preflight * checked_{};
  Phase phase_{Phase::Idle};
  Mode expected_mode_{Mode::Ackermann};
  std::size_t switch_index_{};
  double start_{}, direction_{};
  std::optional<double> last_tick_, still_since_, request_at_, settle_from_;
};

}  // namespace fwis_control
#endif  // FWIS_CONTROL_TRACKING_CORE_HPP_
