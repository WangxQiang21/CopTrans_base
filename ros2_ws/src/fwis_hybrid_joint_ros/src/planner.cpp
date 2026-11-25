#include "fwis_hybrid_joint_ros/inputs.hpp"
#include <nav_msgs/msg/path.hpp>
#include <atomic>
#include <future>

namespace hybrid_ros
{
class Planner final : public Inputs
{
public:
  Planner()
  : Inputs("hybrid_joint_planner")
  {
    output = create_publisher<Envelope>(model_topic(), durable());
    if (settings.multi) {
      state = create_publisher<std_msgs::msg::String>("/formation/optimizer_status", durable());
      progress = create_publisher<std_msgs::msg::String>(
        "/formation/uniform_progress", rclcpp::QoS(
          10).transient_local());
      job = create_publisher<std_msgs::msg::String>("/formation/job_state", durable());
    } else {
      state = create_publisher<std_msgs::msg::String>("/plan/status", durable());
      search = create_publisher<std_msgs::msg::String>("/plan/search_status", durable());
      single = create_publisher<Timed>("/plan/timed_trajectory", durable());
    }
    for (size_t i = 0; i < (settings.multi ? 3u : 1u); ++i) {
      paths.push_back(
        create_publisher<nav_msgs::msg::Path>(
          settings.multi ? "/formation/path/vehicle_" +
          std::to_string(i + 1) : "/plan/path", durable()));
    }
    timer = create_wall_timer(std::chrono::milliseconds(5), [this] {poll();});
  }
  ~Planner() {cancel.store(true);if (worker.valid()) {worker.wait();}}

private:
  struct Work {Context context;Specs specs;};
  std::optional<Work> pending, current;
  std::atomic<bool> cancel{false};
  std::future<hj::Result> worker;
  bool obsolete{};
  rclcpp::Publisher<Envelope>::SharedPtr output;
  rclcpp::Publisher<Timed>::SharedPtr single;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state, search, job, progress;
  hj::Clock::time_point started, last_progress;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> paths;
  rclcpp::TimerBase::SharedPtr timer;
  void single_status(const std::string & s) {state->publish(message(s));search->publish(message(s));}
  void formation_status(
    uint32_t id, const std::string & phase, const std::string & reason,
    double seconds = 0., const Json::Value & stages = Json::Value())
  {
    Json::Value j;
    j["backend"] = "hybrid_joint";
    j["plan_id"] = id;
    j["node_state"] = phase;
    j["model_schema"] = schema;
    j["solver_build"] = build_digest();
    j["legacy_fallback_used"] = false;
    j["speed_profile"] = settings.profile;
    j["reason"] = reason;
    j["wall_time_s"] = seconds;
    if (!stages.isNull()) {j["planning_stages"] = stages;}
    state->publish(message(json(j)));
  }
  void progress_message(uint32_t id, const char * stage, const std::string & reason = "")
  {
    if (!progress) {return;}
    const auto wall = hj::Clock::now();
    Json::Value j;
    j["plan_id"] = id;
    j["stage"] = stage;
    j["actual_backend"] = settings.multi ? "formation_hybrid_joint_cpp" : "single_hybrid_joint_cpp";
    j["model_schema"] = schema;
    j["solver_build"] = build_digest();
    j["legacy_fallback_used"] = false;
    j["speed_profile"] = settings.profile;
    j["reason"] = reason;
    j["wall_time_s"] = std::chrono::duration<double>(wall - started).count();
    j["budget_s"] = settings.budget;
    progress->publish(message(json(j)));
    last_progress = wall;
  }
  void map_ready() override
  {
    if (!settings.multi) {
      single_status("map_ready " + std::to_string(grid->width()) + "x" + std::to_string(grid->height()));
      return;
    }
    Json::Value j;
    j["backend"] = "hybrid_joint";
    j["map_ready"] = true;
    j["map_frame_id"] = "map";
    job->publish(message(json(j)));
  }
  void invalidate() override {cancel.store(true);obsolete = true;pending.reset();}
  void failure(uint32_t id, const std::string & reason) override
  {
    if (settings.multi) {
      formation_status(id, "FAILED", reason);
      progress_message(id, "failed", reason);
    } else {
      single_status("rejected id=" + std::to_string(id) + " reason=" + reason);
    }
  }
  void requested(Context c, Specs s) override
  {
    if (settings.multi) {formation_status(c.id, "SOLVING", "");} else {
      single_status("search_pending id=" + std::to_string(c.id));
    }
    pending = Work{std::move(c), std::move(s)};
    poll();
  }
  void publish(const hj::Result & r)
  {
    auto & c = current->context;
    const auto pack_start = hj::Clock::now();
    const auto publication_ros_ns = now().nanoseconds();
    const auto epoch = publication_ros_ns + std::llround(settings.lead * 1e9);
    auto envelope =
      encode(
      c, *r.plan, epoch,
      std::chrono::duration_cast<std::chrono::nanoseconds>(hj::Clock::now().time_since_epoch()).count(),
      publication_ros_ns);
    const auto encoded = hj::Clock::now();
    display_samples(envelope, c, *r.plan);
    envelope.trajectory.vehicles = current->specs;
    const auto publish_begin = hj::Clock::now();
    output->publish(envelope);
    const auto submitted = hj::Clock::now();
    Json::Value stages;
    stages["policy_status"] = r.status;
    stages["policy_reason"] = r.reason;
    stages["first_certified_s"] = r.first_certified_s;
    if (r.status == "CERTIFIED_POLICY_COMPLIANT") {
      stages["first_policy_compliant_s"] = r.first_policy_compliant_s;
    }
    stages["seed_s"] = r.seed_seconds;
    stages["edit_s"] = r.edit_seconds;
    double build = 0., solve = 0., audit = 0.;
    for (const auto & attempt : r.attempts) {
      build += attempt.nlp_build_s;
      solve += attempt.nlp_solve_s;
      audit += attempt.certify_s;
    }
    stages["attempts"] = Json::UInt64(r.attempts.size());
    stages["nlp_build_s"] = build;
    stages["nlp_solve_s"] = solve;
    stages["certify_s"] = audit;
    stages["encode_s"] = std::chrono::duration<double>(encoded - pack_start).count();
    stages["sample_s"] = std::chrono::duration<double>(publish_begin - encoded).count();
    stages["publish_call_s"] = std::chrono::duration<double>(submitted - publish_begin).count();
    for (size_t i = 0; i < paths.size(); ++i) {
      nav_msgs::msg::Path path;
      path.header = envelope.header;
      for (const auto & v:envelope.trajectory.trajectories[i].points) {
        auto & p = path.poses.emplace_back();
        p.header = path.header;
        p.pose.position.x = v.center_pose.x;
        p.pose.position.y = v.center_pose.y;
        p.pose.orientation.z = std::sin(.5 * v.center_pose.theta);
        p.pose.orientation.w = std::cos(.5 * v.center_pose.theta);
      }
      paths[i]->publish(path);
    }
    if (settings.multi) {
      formation_status(c.id, "PUBLISHED", r.status, std::chrono::duration<double>(submitted - started).count(), stages);
      progress_message(c.id, r.status == "FEASIBLE_POLICY_DEGRADED" ? "policy_degraded" : "certified", r.reason);
      return;
    }
    Timed t;
    t.header = envelope.header;
    t.plan_id = c.id;
    for (const auto & v:envelope.trajectory.trajectories[0].points) {
      auto & p = t.points.emplace_back();
      p.time_from_start = v.time_from_start;
      p.center_pose = v.center_pose;
      p.mode = v.mode;
      p.v = v.center_speed_mps;
    }
    single->publish(t);
    single_status(
      "success id=" + std::to_string(
        c.id) + " source=hybrid_joint schema=" + schema + " solver_build=" + build_digest() + " legacy_fallback=false speed_profile=" +
      settings.profile + " stages=" + json(stages));
  }
  void poll()
  {
    if (worker.valid()) {
      if (worker.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        if (!obsolete && hj::Clock::now() - last_progress > std::chrono::milliseconds(250)) {
          progress_message(current->context.id, "solve");
        }
        return;
      }
      try {
        auto result = worker.get();
        if (!obsolete) {
          if (result.plan) {
            publish(result);
          } else {
            failure(current->context.id, result.status + " " + result.reason);
          }
        }
      } catch (const std::exception & e) {if (!obsolete) {failure(current->context.id, e.what());}}
      current.reset();
    }
    if (!pending) {return;}
    current = std::move(pending);
    pending.reset();
    obsolete = false;
    cancel.store(false);
    started = hj::Clock::now();
    progress_message(current->context.id, "solve");
    auto deadline = started +
      std::chrono::duration_cast<hj::Clock::duration>(std::chrono::duration<double>(settings.budget));
    worker = std::async(
      std::launch::async, [this, deadline] {
        return hj::solve(current->context.request, deadline, [this] {return cancel.load();});
      });
  }
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hybrid_ros::Planner>());
  rclcpp::shutdown();
}
