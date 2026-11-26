#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <QApplication>
#include <QClipboard>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStringList>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>

namespace fwis_rviz_plugins
{
namespace
{

QString phaseName(const QString & phase)
{
  if (phase == "waiting_start") {return QStringLiteral("选择编队起点");}
  if (phase == "placing") {return QStringLiteral("同步 Gazebo 起点");}
  if (phase == "waiting_goal") {return QStringLiteral("选择编队终点");}
  if (phase == "assembly_planning") {return QStringLiteral("集结规划");}
  if (phase == "assembly_executing") {return QStringLiteral("三车驶向挂点");}
  if (phase == "docking") {return QStringLiteral("核对挂点与停车反馈");}
  if (phase == "transport_planning") {return QStringLiteral("CTS 运输规划");}
  if (phase == "transport_executing") {return QStringLiteral("CTS 运输中");}
  if (phase == "terminal_alignment") {return QStringLiteral("终点车身对齐");}
  if (phase == "planning") {return QStringLiteral("规划中");}
  if (phase == "verification") {return QStringLiteral("验证轨迹模型");}
  if (phase == "arming") {return QStringLiteral("等待三车准入");}
  if (phase == "running") {return QStringLiteral("编队执行中");}
  if (phase == "finished") {return QStringLiteral("闭环完成");}
  if (phase == "failed") {return QStringLiteral("试验失败");}
  return QStringLiteral("等待仿真就绪");
}

QString stageName(const QString & stage)
{
  if (stage == "check_endpoints") {return QStringLiteral("核对起终点");}
  if (stage == "corridor_search") {return QStringLiteral("凸走廊引导搜索");}
  if (stage == "joint_search") {return QStringLiteral("A/O/S 联合搜索");}
  if (stage == "terminal_audit") {return QStringLiteral("终点候选独立认证");}
  if (stage == "common_timing") {return QStringLiteral("求解共同时间轴");}
  if (stage == "solve") {return QStringLiteral("C++ 联合优化");}
  if (stage == "certified") {return QStringLiteral("独立认证通过");}
  if (stage == "publication") {return QStringLiteral("校验轨迹信封");}
  return QStringLiteral("等待求解器");
}

}  // namespace

class PlanningPanel final : public rviz_common::Panel
{
public:
  explicit PlanningPanel(QWidget * parent = nullptr)
  : rviz_common::Panel(parent)
  {
    auto * layout = new QVBoxLayout(this);
    layout->setSpacing(9);
    auto * identity = new QHBoxLayout;
    experiment_label_ = new QLabel;
    experiment_label_->setTextFormat(Qt::PlainText);
    experiment_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    identity->addWidget(experiment_label_, 1);
    copy_id_ = new QPushButton(QStringLiteral("复制编号"));
    identity->addWidget(copy_id_);
    layout->addLayout(identity);
    connect(copy_id_, &QPushButton::clicked, this, [this]() {
      QApplication::clipboard()->setText(experiment_id_);
    });
    title_ = makeLabel(layout);
    QFont title_font = title_->font();
    title_font.setBold(true);
    title_font.setPointSize(title_font.pointSize() + 3);
    title_->setFont(title_font);
    solving_ = makeLabel(layout);
    identity_ = makeLabel(layout);
    progress_ = makeLabel(layout);
    timing_ = makeLabel(layout);
    stage_times_ = makeLabel(layout);
    admission_ = makeLabel(layout);
    details_ = makeLabel(layout);
    layout->addStretch(1);
    setMinimumWidth(235);
    auto * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {render();});
    timer->start(85);
    render();
  }

  void onInitialize() override
  {
    const auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    const auto qos = rclcpp::QoS(1).transient_local().reliable();
    execution_sub_ = node->create_subscription<std_msgs::msg::String>(
      "/formation/interactive_status", qos,
      [state = state_](std_msgs::msg::String::ConstSharedPtr message) {
        receive(*state, message->data, false);
      });
    solver_sub_ = node->create_subscription<std_msgs::msg::String>(
      "/formation/uniform_progress", qos,
      [state = state_](std_msgs::msg::String::ConstSharedPtr message) {
        receive(*state, message->data, true);
      });
  }

private:
  using WallClock = std::chrono::steady_clock;

  struct Telemetry
  {
    std::mutex mutex;
    QJsonObject solver_progress, execution;
    WallClock::time_point started{};
  };

  static QLabel * makeLabel(QVBoxLayout * layout)
  {
    auto * label = new QLabel;
    label->setTextFormat(Qt::PlainText);
    label->setWordWrap(true);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(label);
    return label;
  }

  static void receive(Telemetry & state, const std::string & message, bool solver)
  {
    const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(message));
    if (!document.isObject()) {return;}
    const auto payload = document.object();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (solver) {
      state.solver_progress = payload;
    } else {
      if (payload.value("solver_outcome").toString() == "running" &&
        state.execution.value("solver_outcome").toString() != "running")
      {
        state.started = WallClock::now();
      }
      state.execution = payload;
    }
  }

  void render()
  {
    QJsonObject state, progress;
    WallClock::time_point started;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      state = state_->execution;
      progress = state_->solver_progress;
      started = state_->started;
    }
    const auto phase = state.value("phase").toString();
    const auto outcome = state.value("solver_outcome").toString();
    experiment_id_ = state.value("experiment_id").toString();
    experiment_label_->setText(experiment_id_.isEmpty() ? QStringLiteral("实验编号 —") :
      QStringLiteral("实验编号\n%1").arg(experiment_id_));
    copy_id_->setEnabled(!experiment_id_.isEmpty());
    if (progress.value("plan_id") != state.value("plan_id")) {progress = {};}
    title_->setText(phaseName(phase));
    title_->setStyleSheet(phase == "failed" ? "color: #c94f41;" :
      (phase == "finished" ? "color: #25825c;" : ""));

    QString solver = QStringLiteral("待开始");
    if (outcome == "success") {solver = QStringLiteral("已认证");}
    if (outcome == "degraded") {solver = QStringLiteral("已认证 · 模式偏好降级");}
    if (outcome == "failed") {solver = QStringLiteral("失败");}
    if (outcome == "running") {
      solver = phase == "assembly_planning" ? QStringLiteral("分配挂点与协调避碰") :
        stageName(progress.value("stage").toString());
    }
    solving_->setText(QStringLiteral("求解  %1").arg(solver));

    const auto backend = progress.value("actual_backend").toString();
    identity_->setVisible(!backend.isEmpty());
    if (!backend.isEmpty()) {
      auto label = QStringLiteral("%1 · %2\n%3 · 构建 %4")
        .arg(backend, progress.value("speed_profile").toString(),
             progress.value("model_schema").toString(), progress.value("solver_build").toString().left(10));
      const auto fallback = progress.value("legacy_fallback_used");
      if (fallback.isBool()) {
        label += fallback.toBool() ? QStringLiteral("\n已回退至旧算法") : QStringLiteral("\n未回退");
      }
      identity_->setText(label);
    }
    const auto expanded = progress.value("expansions");
    progress_->setVisible(expanded.isDouble());
    if (expanded.isDouble()) {
      progress_->setText(QStringLiteral("已扩展 %1 / %2 · 候选 %3\n终点审计 %4 次")
        .arg(expanded.toInt()).arg(progress.value("expansion_budget").toInt())
        .arg(progress.value("candidates").toInt())
        .arg(progress.value("terminal_audits").toInt()));
    }
    double elapsed = state.value("solver_elapsed_s").toDouble(-1.0);
    if (outcome == "running" && started != WallClock::time_point{}) {
      elapsed = std::chrono::duration<double>(WallClock::now() - started).count();
    }
    timing_->setText(elapsed >= 0.0 ? QStringLiteral("求解耗时  %1 s").arg(elapsed, 0, 'f', 3) :
      QStringLiteral("求解耗时  —"));
    const auto assembly_elapsed = state.value("assembly_elapsed_s");
    const auto transport_elapsed = state.value("transport_elapsed_s");
    QStringList durations;
    if (assembly_elapsed.isDouble()) {
      durations << QStringLiteral("集结 %1 s").arg(assembly_elapsed.toDouble(), 0, 'f', 2);
    }
    if (transport_elapsed.isDouble()) {
      durations << QStringLiteral("运输 %1 s").arg(transport_elapsed.toDouble(), 0, 'f', 2);
    }
    stage_times_->setVisible(!durations.isEmpty());
    stage_times_->setText(durations.join(QStringLiteral(" · ")));

    const int count = state.value("vehicle_count").toInt(3);
    const auto docked = state.value("docked");
    admission_->setText(docked.isDouble() && (phase.startsWith("assembly_") || phase == "docking") ?
      QStringLiteral("到位 %1/%2").arg(docked.toInt()).arg(count) :
      QStringLiteral("准入 %1/%2 · 完成 %3/%2")
      .arg(state.value("admitted").toInt()).arg(count).arg(state.value("complete").toInt()));
    const auto reason = state.value("reason").toString();
    const auto detail = state.value("detail").toString(QStringLiteral(
      "使用 2D Pose Estimate 选起点，使用 2D Goal Pose 选终点"));
    details_->setText(reason.isEmpty() ? detail : reason + QStringLiteral("\n") + detail);
  }

  QLabel * experiment_label_;
  QPushButton * copy_id_;
  QString experiment_id_;
  QLabel * title_;
  QLabel * solving_;
  QLabel * identity_;
  QLabel * progress_;
  QLabel * timing_;
  QLabel * stage_times_;
  QLabel * admission_;
  QLabel * details_;
  std::shared_ptr<Telemetry> state_ = std::make_shared<Telemetry>();
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr solver_sub_, execution_sub_;
};

}  // namespace fwis_rviz_plugins

PLUGINLIB_EXPORT_CLASS(fwis_rviz_plugins::PlanningPanel, rviz_common::Panel)
