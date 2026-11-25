#include "fwis_hybrid_joint_ros/inputs.hpp"
#include <fwis_v3_msgs/msg/hybrid_execution_permit.hpp>
#include <algorithm>
#include <future>
#include <atomic>
namespace hybrid_ros
{
using Permit = fwis_v3_msgs::msg::HybridExecutionPermit;
class Coordinator final : public Inputs
{
public:
  Coordinator()
  : Inputs("hybrid_joint_executor")
  {
    permits = create_publisher<Permit>(permit_topic, 1);
    result = create_publisher<std_msgs::msg::String>("/hybrid_joint/admission", durable());
    readiness = create_publisher<std_msgs::msg::String>("/hybrid_joint/readiness", durable());
    sub = create_subscription<Envelope>(model_topic(), durable(), [this](const Envelope & m) {receive(m);});
    for (unsigned i = 0; i < (settings.multi ? 3u : 1u); ++i) {
      feedback.push_back(
        create_subscription<std_msgs::msg::String>(
          "/robot" + std::to_string(i + 1) + "/controller_status", 30,
          [this, i](const std_msgs::msg::String & m) {status(i, m.data);}));
    }
    cancel_sub =
      create_subscription<std_msgs::msg::String>(
      "/hybrid_joint/cancel", 1,
      [this](const std_msgs::msg::String &) {stop("external_stop");expected.reset();});
    timer = create_wall_timer(std::chrono::milliseconds(20), [this] {tick();});
  }

private:
  rclcpp::Publisher<Permit>::SharedPtr permits;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result, readiness;
  struct Proof {
    std::string hash;
    std::shared_ptr<std::atomic_bool> cancelled;
    std::future<hj::Audit> outcome;
  };
  std::vector<Proof> proofs;
  bool proven{};
  void map_ready() override {
    readiness->publish(message(grid->hash()));
  }
  void collect_proofs() {
    for (auto it = proofs.begin(); it != proofs.end();) {
      if (it->outcome.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        ++it;
        continue;
      }
      const bool current = key && !ended && key->model_sha256 == it->hash && !it->cancelled->load();
      try {
        const auto audit = it->outcome.get();
        if (current && audit.executable) {
          proven = true;
          diagnostic("PRECHECK", "");
        } else if (current) {
          stop("receiver_certificate:" + audit.reason);
        }
      } catch (const std::exception & e) {
        if (current) stop(e.what());
      }
      it = proofs.erase(it);
    }
  }
  rclcpp::Subscription<Envelope>::SharedPtr sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_sub;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> feedback;
  rclcpp::TimerBase::SharedPtr timer;
  std::optional<Context> expected;
  std::optional<Envelope> early;
  std::optional<Permit> key;
  std::array<bool, 3> accepted{}, complete{};
  std::array<std::string, 3> early_admissions;
  std::array<hj::Clock::time_point, 3> fresh{};
  std::array<double, 3> source_stamp{};
  bool enabled{}, ended{};
  hj::Clock::time_point began;
  int64_t last_ros{}, publication_steady_ns{};
  void diagnostic(const std::string & phase, const std::string & reason)
  {
    Json::Value j;
    j["phase"] = phase;
    j["reason"] = reason;
    j["plan_id"] = key ? key->plan_id : (expected ? expected->id : 0);
    j["model_sha256"] = key ? key->model_sha256 : "";
    if (key) {
      j["start_ns"] = Json::Int64(ns(key->start));
      j["certificate_complete"] = proven;
      j["accepted_ids"] = Json::Value(Json::arrayValue);
      j["tracker_age_s"] = Json::Value(Json::arrayValue);
      const auto stamp = now().seconds();
      for (size_t i=0; i<feedback.size(); ++i) {
        if (accepted[i]) j["accepted_ids"].append(static_cast<unsigned>(i+1));
        j["tracker_age_s"].append(source_stamp[i]>0 ? Json::Value(stamp-source_stamp[i]) : Json::Value{});
      }
    }
    if (key && phase == "AUTHORIZED") {
      const auto current = hj::Clock::now();
      j["admission_wall_s"] = std::chrono::duration<double>(current - began).count();
      const auto released =
        std::chrono::duration_cast<std::chrono::nanoseconds>(current.time_since_epoch()).count();
      if (publication_steady_ns > 0 && released >= publication_steady_ns) {
        j["delivery_and_admission_wall_s"] = (released - publication_steady_ns) * 1e-9;
      }
    }
    result->publish(message(json(j)));
  }
  void publish(bool allow)
  {
    if (!key) {return;}
    key->stamp = now();
    key->enabled = allow;
    permits->publish(*key);
  }
  void stop(const std::string & reason)
  {
    for (auto & work : proofs) work.cancelled->store(true);
    if (key && !ended) {
      publish(false);
      ended = true;
      diagnostic("STOPPED", reason);
    }
    enabled = false;
  }
  void invalidate()override
  {
    stop("request_or_map_replaced");
    key.reset();
    expected.reset();
    early_admissions = {};
  }
  void failure(uint32_t id, const std::string & reason) override {
    if (key) {
      stop(reason);
    } else {
      Json::Value j;
      j["phase"] = "STOPPED";
      j["plan_id"] = id;
      j["reason"] = reason;
      result->publish(message(json(j)));
    }
  }
  void requested(Context c, Specs)override
  {
    expected = std::move(c);
    diagnostic("REGISTERED", "");
    if (early) {auto e = std::move(*early);early.reset();if (e.plan_id == expected->id) {receive(e);}}
  }
  void receive(const Envelope & m)
  {
    if (!expected) {early = m;return;}
    if (key) {
      if (m.plan_id != key->plan_id || key->model_sha256 != m.model_sha256 ||
        digest(m.model_json) != m.model_sha256 || ns(key->start) != ns(m.header.stamp))
      {
        stop("model_changed");
      }
      return;
    }
    started(m);
    try {
      auto decoded = decode(m, grid, &*expected);
      publication_steady_ns = parse(m.model_json)["publication_steady_ns"].asInt64();
      if (ns(m.header.stamp) <= now().nanoseconds() ||
        ns(m.header.stamp) - now().nanoseconds() > int64_t((settings.lead + .10) * 1e9))
      {
        throw std::invalid_argument("invalid future epoch");
      }
      // Recompute the safety proof using the locally observed map, request and limits.
      auto deadline = hj::Clock::now() + std::chrono::milliseconds(
        static_cast<int64_t>(std::clamp((ns(m.header.stamp) - now().nanoseconds()) * 1e-6 - 80., 0., 400.)));
      collect_proofs();
      if (proofs.size() >= 2) throw std::runtime_error("previous_certificate_cancellation_pending");
      auto cancelled = std::make_shared<std::atomic_bool>(false);
      auto proof = std::async(std::launch::async,
        [request=expected->request, plan=std::move(decoded.plan), cancelled, deadline] {
          return hj::certify(request, plan, deadline, [cancelled]{return cancelled->load();});
        });
      proofs.push_back({m.model_sha256, std::move(cancelled), std::move(proof)});
      for (unsigned i = 0; i < early_admissions.size(); ++i) {
        if (!early_admissions[i].empty()) {
          status(i, early_admissions[i]);
        }
      }
    } catch (const std::exception & e) {stop(e.what());}
  }
  void started(const Envelope & m)
  {
    key.emplace();
    key->plan_id = m.plan_id;
    key->model_sha256 = m.model_sha256;
    key->start = m.header.stamp;
    publication_steady_ns = 0;
    proven = false;
    enabled = false;
    ended = false;
    accepted = {};
    complete = {};
    fresh = {};
    source_stamp = {};
    began = hj::Clock::now();
    last_ros = now().nanoseconds();
  }
  void status(unsigned i, const std::string & text)
  {
    try {
      if (!key) {
        if (expected) {
          auto row = parse(text);
          if (row["plan_id"].asUInt() == expected->id && row["kind"] == "admission") {
            early_admissions[i] = text;
          }
        }
        return;
      }
      if (ended) {return;}
      auto row = parse(text);
      if (row["plan_id"].asUInt() != key->plan_id || row["trajectory_stamp_ns"].asInt64() != ns(key->start)) {
        return;
      }
      if (row["model_sha256"] != key->model_sha256) {stop("tracker_model_mismatch");return;}
      auto phase = row["phase"].asString();
      if (phase == "STOPPED" || phase == "REJECTED" || phase == "STALE_TRAJECTORY") {
        stop("tracker_" + phase);
        return;
      }
      if (row["kind"] == "admission") {
        if ((row["result"] != "PASS" && row["result"] != "WARN") ||
          (phase != "WAIT_START" && phase != "TRACKING")) {stop("preflight_rejected");return;}
        accepted[i] = true;
      }
      if (row["kind"]=="execution" && phase=="WAIT_START" &&
          row["preflight_accepted"].isBool() && row["preflight_accepted"].asBool()) {
        // The enclosing ID, hash and epoch have already matched this request.
        // Only an active tracker with completed native preflight sets this bit.
        accepted[i] = true;
      }
      if (phase == "COMPLETE") {complete[i] = true;}
      fresh[i] = hj::Clock::now();
      source_stamp[i] = row["stamp"].asDouble();
    } catch (const std::exception &) {stop("malformed_tracker_feedback");}
  }
  void tick()
  {
    collect_proofs();
    const auto stamp = now().nanoseconds();
    if (!key && early && stamp >= ns(early->header.stamp) - 80000000) {
      const auto id = early->plan_id;
      early.reset();
      failure(id,"request_missing_at_model_epoch");
    }
    if (!key || ended) return;
    const auto wall = hj::Clock::now();
    if (stamp < last_ros) {stop("clock_rollback");return;} last_ros = stamp;
    if (!enabled && stamp >= ns(key->start)-80000000) {
      stop("admission_deadline");
      return;
    }
    for (unsigned i = 0; i < feedback.size(); ++i) {
      if (accepted[i] && !complete[i] &&
        (wall - fresh[i] > std::chrono::milliseconds(200) || stamp * 1e-9 - source_stamp[i] > .2))
      {
        // While stationary before authorization, wait for a fresh sample.
        // After authorization any stale member immediately stops the whole group.
        if (enabled) stop("stale_tracker_feedback");
        return;
      }
    }
    if (!enabled) {
      if (!proven) return;
      if (!std::all_of(accepted.begin(), accepted.begin() + feedback.size(), [](bool v) {return v;})) {
        return;
      }
      enabled = true;
      diagnostic("AUTHORIZED", "");
    }
    if (std::all_of(complete.begin(), complete.begin() + feedback.size(), [](bool v) {return v;})) {
      ended = true;
      publish(false);
      diagnostic("COMPLETE", "");
      return;
    }
    publish(true);
  }
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hybrid_ros::Coordinator>());
  rclcpp::shutdown();
}
