#include "openarmx_head_joint_slider_panel/head_joint_slider_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QScrollArea>
#include <QStringList>
#include <QTimer>
#include <QVBoxLayout>
#include <QFontDatabase>
#include <QFontMetrics>
#include <QDir>
#include <kdl_parser/kdl_parser.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/parameter_client.hpp>
#include <QSettings>
#include <urdf/model.h>

using namespace std::chrono_literals;

namespace openarmx_head_joint_slider_panel {

namespace {
QString recordTargetsSettingsPath() {
  return QDir::homePath() + "/.config/OpenArmX/HeadJointSliderPanel.conf";
}

bool loadRecordTargetsAutoSave(QString *records) {
  if (!records) {
    return false;
  }
  QSettings settings(recordTargetsSettingsPath(), QSettings::IniFormat);
  if (!settings.value("General/auto_saved", false).toBool()) {
    return false;
  }
  *records = settings.value("General/record_targets").toString();
  return true;
}

QString loadRecordTargetsFallback() {
  const QString settings_path = recordTargetsSettingsPath();
  QSettings settings(settings_path, QSettings::IniFormat);
  return settings.value("General/record_targets").toString();
}

void saveRecordTargetsAutoSave(const QString &records) {
  QSettings settings(recordTargetsSettingsPath(), QSettings::IniFormat);
  settings.setValue("General/record_targets", records);
  settings.setValue("General/auto_saved", true);
  settings.sync();
}

void clearLayout(QLayout *layout) {
  if (!layout) {
    return;
  }
  while (auto *item = layout->takeAt(0)) {
    if (auto *child_layout = item->layout()) {
      clearLayout(child_layout);
    }
    if (auto *widget = item->widget()) {
      widget->deleteLater();
    }
    delete item;
  }
}
}  // namespace

HeadJointSliderPanel::HeadJointSliderPanel(QWidget *parent) : rviz_common::Panel(parent) {
  setupUi();
}

HeadJointSliderPanel::~HeadJointSliderPanel() {
  stopCommandWorker();
}

void HeadJointSliderPanel::setupUi() {
  auto *root_layout = new QVBoxLayout;

  auto *title = new QLabel("<h3>OpenArmX 头部关节滑块面板</h3>");
  title->setAlignment(Qt::AlignCenter);
  root_layout->addWidget(title);

  auto *head_group = new QGroupBox("头部 (2关节: 偏航 + 俯仰)");
  auto *head_layout = new QVBoxLayout;
  for (size_t i = 0; i < HEAD_DOF; ++i) {
    joint_sliders_[i] = createJointSliderRow(head_joint_labels_[i], head_layout, 0.0);
  }
  head_group->setLayout(head_layout);
  root_layout->addWidget(head_group);

  auto *button_row = new QHBoxLayout;
  sync_button_ = new QPushButton("从 /joint_states 同步");
  home_button_ = new QPushButton("Home 回零");
  add_record_button_ = new QPushButton("新增记录");

  button_row->addWidget(sync_button_);
  button_row->addWidget(home_button_);
  button_row->addWidget(add_record_button_);
  root_layout->addLayout(button_row);

  record_group_ = new QGroupBox("记录点");
  record_list_layout_ = new QVBoxLayout;
  record_group_->setLayout(record_list_layout_);
  root_layout->addWidget(record_group_);

  auto *joint_step_row = new QHBoxLayout;
  joint_step_row->addWidget(new QLabel("关节步长:"));
  joint_step_slider_ = new QSlider(Qt::Horizontal);
  joint_step_slider_->setRange(1, 200);  // 0.001~0.200 rad per cycle
  joint_step_slider_->setValue(joint_step_mrad_);
  joint_step_slider_->setTickInterval(10);
  joint_step_slider_->setTickPosition(QSlider::TicksBelow);
  joint_step_value_label_ = new QLabel(QString("%1 mrad").arg(joint_step_mrad_));
  joint_step_value_label_->setMinimumWidth(90);
  joint_step_value_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  joint_step_row->addWidget(joint_step_slider_, 1);
  joint_step_row->addWidget(joint_step_value_label_);
  root_layout->addLayout(joint_step_row);

  status_label_ = new QLabel("状态: 等待初始化...");
  status_label_->setWordWrap(true);
  status_label_->setStyleSheet("background-color: #f0f0f0; padding: 6px; border-radius: 4px;");
  root_layout->addWidget(status_label_);

  root_layout->addStretch();
  setLayout(root_layout);

  connect(sync_button_, &QPushButton::clicked, this, &HeadJointSliderPanel::onSyncFromRobotClicked);
  connect(home_button_, &QPushButton::clicked, this, &HeadJointSliderPanel::onHomeClicked);
  connect(add_record_button_, &QPushButton::clicked, this, &HeadJointSliderPanel::onAddRecordClicked);
  connect(joint_step_slider_, &QSlider::valueChanged, this, &HeadJointSliderPanel::onJointStepSliderChanged);
  rebuildRecordList();
}

HeadJointSliderPanel::SliderBinding HeadJointSliderPanel::createJointSliderRow(
    const QString &title, QVBoxLayout *parent_layout, double init_value) {
  auto *row_layout = new QHBoxLayout;
  auto *name_label = new QLabel(title);
  name_label->setMinimumWidth(140);

  auto *slider = new QSlider(Qt::Horizontal);
  slider->setTickPosition(QSlider::TicksBelow);
  slider->setRange(kSliderMin, kSliderMax);
  slider->setTickInterval(300);
  slider->setValue(static_cast<int>(std::round(init_value * kSliderScale)));

  auto *value_label = new QLabel;
  value_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  value_label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);

  auto mono_font = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  value_label->setFont(mono_font);
  QFontMetrics fm(mono_font);
  const int width = fm.horizontalAdvance("+000.0°") + 14;
  value_label->setFixedWidth(width);

  row_layout->addWidget(name_label);
  row_layout->addWidget(slider);
  row_layout->addWidget(value_label);
  parent_layout->addLayout(row_layout);

  SliderBinding binding;
  binding.slider = slider;
  binding.value_label = value_label;

  connect(slider, &QSlider::valueChanged, this, [this, binding](int) {
    updateSliderLabel(binding);
    if (!suppress_slider_events_) {
      updateDesiredTargetFromSliders();
    }
  });

  updateSliderLabel(binding);
  return binding;
}

void HeadJointSliderPanel::updateSliderLabel(const SliderBinding &binding) {
  if (!binding.slider || !binding.value_label) {
    return;
  }

  const double rad = static_cast<double>(binding.slider->value()) / static_cast<double>(kSliderScale);
  const double deg = rad * 57.29577951308232;
  binding.value_label->setText(QString::asprintf("%+0.1f°", deg));
}

void HeadJointSliderPanel::applyDefaultSliderRanges() {
  joint_sliders_[0].slider->setRange(kSliderMin, kSliderMax);
  joint_sliders_[1].slider->setRange(kPitchSliderMin, kPitchSliderMax);
  for (auto &binding : joint_sliders_) {
    updateSliderLabel(binding);
  }
}

bool HeadJointSliderPanel::applyUrdfJointLimits(const std::string &urdf_xml) {
  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    return false;
  }

  bool all_ok = true;
  for (size_t i = 0; i < HEAD_DOF; ++i) {
    auto joint = model.getJoint(head_joint_names_[i]);
    if (!joint || !joint->limits) {
      all_ok = false;
      continue;
    }
    const double lower = joint->limits->lower;
    const double upper = joint->limits->upper;
    int min_raw = static_cast<int>(std::lround(lower * kSliderScale));
    int max_raw = static_cast<int>(std::lround(upper * kSliderScale));
    if (min_raw > max_raw) {
      std::swap(min_raw, max_raw);
    }
    if (min_raw == max_raw) {
      max_raw += 1;
    }
    joint_sliders_[i].slider->setRange(min_raw, max_raw);
    updateSliderLabel(joint_sliders_[i]);
  }

  return all_ok;
}

bool HeadJointSliderPanel::loadJointLimitsFromRobotDescription() {
  std::vector<std::string> candidate_nodes = {
      "/controller_manager", "/robot_state_publisher", "/move_group"};

  if (node_) {
    std::set<std::string> candidate_set(candidate_nodes.begin(), candidate_nodes.end());
    const auto graph_nodes = node_->get_node_graph_interface()->get_node_names_and_namespaces();
    for (const auto &entry : graph_nodes) {
      const std::string &name = entry.first;
      const std::string &ns = entry.second;
      if (name.find("controller_manager") == std::string::npos &&
          name.find("robot_state_publisher") == std::string::npos &&
          name.find("move_group") == std::string::npos) {
        continue;
      }

      std::string full_name = ns;
      if (full_name.empty()) {
        full_name = "/";
      }
      if (full_name.back() != '/') {
        full_name.push_back('/');
      }
      full_name += name;
      candidate_set.insert(full_name);
    }
    candidate_nodes.assign(candidate_set.begin(), candidate_set.end());
  }

  for (const auto &node_name : candidate_nodes) {
    try {
      auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node_, node_name);
      if (!param_client->wait_for_service(700ms)) {
        continue;
      }

      const auto params = param_client->get_parameters({"robot_description"});
      if (params.empty()) {
        continue;
      }
      if (params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        continue;
      }

      const std::string urdf_xml = params[0].as_string();
      if (urdf_xml.empty()) {
        continue;
      }

      setupPreviewModel(urdf_xml);

      if (applyUrdfJointLimits(urdf_xml)) {
        joint_limits_loaded_ = true;
        joint_limits_source_node_ = node_name;
        return true;
      }
    } catch (const std::exception &) {
      continue;
    }
  }
  return false;
}

void HeadJointSliderPanel::onInitialize() {
  node_ = std::make_shared<rclcpp::Node>("openarmx_head_joint_slider_panel");

  head_forward_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/head_forward_position_controller/commands", 10);

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 30,
      std::bind(&HeadJointSliderPanel::jointStateCallback, this, std::placeholders::_1));

  rclcpp::QoS description_qos(rclcpp::KeepLast(1));
  description_qos.reliable();
  description_qos.transient_local();
  robot_description_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/robot_description", description_qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!msg || msg->data.empty()) {
          return;
        }

        setupPreviewModel(msg->data);

        if (applyUrdfJointLimits(msg->data)) {
          joint_limits_loaded_ = true;
          joint_limits_source_node_ = "/robot_description(topic)";
        }
      });

  ros_spin_timer_ = new QTimer(this);
  connect(ros_spin_timer_, &QTimer::timeout, this, [this]() { rclcpp::spin_some(node_); });
  ros_spin_timer_->start(50);

  status_timer_ = new QTimer(this);
  connect(status_timer_, &QTimer::timeout, this, &HeadJointSliderPanel::updateStatusText);
  status_timer_->start(1000);

  preview_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  preview_send_timer_ = new QTimer(this);
  preview_send_timer_->setSingleShot(true);
  connect(preview_send_timer_, &QTimer::timeout, this, &HeadJointSliderPanel::onPreviewClicked);

  preview_publish_timer_ = new QTimer(this);
  connect(preview_publish_timer_, &QTimer::timeout, this, &HeadJointSliderPanel::onPreviewPublishTimer);
  preview_publish_timer_->start(50);

  live_preview_enabled_ = true;
  last_preview_send_time_ = std::chrono::steady_clock::now();
  preview_model_retry_time_ = std::chrono::steady_clock::now();

  applyDefaultSliderRanges();
  if (!loadJointLimitsFromRobotDescription()) {
    joint_limits_loaded_ = false;
    joint_limits_source_node_.clear();
  }

  updateDesiredTargetFromSliders();
  startCommandWorker();
  updateStatusText();
}

void HeadJointSliderPanel::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  latest_joint_state_map_.clear();
  const size_t n = std::min(msg->name.size(), msg->position.size());
  for (size_t i = 0; i < n; ++i) {
    latest_joint_state_map_[msg->name[i]] = msg->position[i];
  }
  has_joint_state_ = (n > 0);

  if (hasAllTargetJointStates()) {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (!command_state_initialized_) {
        const TargetState current_state = targetStateFromLatestJointStates();
        command_target_ = current_state;
        desired_target_ = current_state;
        command_state_initialized_ = true;
      }
    }
    command_cv_.notify_all();
  }

  if (!auto_sync_done_ && hasAllTargetJointStates()) {
    if (applyJointStateToSliders()) {
      auto_sync_done_ = true;
      const TargetState synced = collectTargetStateFromSliders();
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        desired_target_ = synced;
        command_target_ = synced;
        command_state_initialized_ = true;
      }
      command_cv_.notify_all();
      setStatus("已自动从 /joint_states 同步头部初始姿态", "#E3F2FD");
      scheduleLivePreview();
    }
  }
}

bool HeadJointSliderPanel::hasAllTargetJointStates() const {
  for (const auto &j : head_joint_names_) {
    if (latest_joint_state_map_.find(j) == latest_joint_state_map_.end()) {
      return false;
    }
  }
  return true;
}

HeadJointSliderPanel::TargetState HeadJointSliderPanel::targetStateFromLatestJointStates() const {
  TargetState target;
  target.joints.reserve(HEAD_DOF);

  for (const auto &name : head_joint_names_) {
    target.joints.push_back(latest_joint_state_map_.at(name));
  }
  return target;
}

bool HeadJointSliderPanel::applyJointStateToSliders() {
  if (!has_joint_state_ || !hasAllTargetJointStates()) {
    return false;
  }

  applyTargetToSliders(targetStateFromLatestJointStates());
  return true;
}

void HeadJointSliderPanel::applyTargetToSliders(const TargetState &target) {
  const TargetState clamped = clampTargetToSliderRanges(target);
  suppress_slider_events_ = true;

  for (size_t i = 0; i < HEAD_DOF; ++i) {
    const double rad = i < clamped.joints.size() ? clamped.joints[i] : 0.0;
    const int raw = static_cast<int>(std::lround(rad * kSliderScale));
    joint_sliders_[i].slider->setValue(std::clamp(
        raw, joint_sliders_[i].slider->minimum(), joint_sliders_[i].slider->maximum()));
  }

  suppress_slider_events_ = false;
}

HeadJointSliderPanel::TargetState HeadJointSliderPanel::clampTargetToSliderRanges(
    const TargetState &target) const {
  TargetState clamped = target;
  clamped.joints.resize(HEAD_DOF, 0.0);
  for (size_t i = 0; i < HEAD_DOF; ++i) {
    const int raw = static_cast<int>(std::lround(clamped.joints[i] * kSliderScale));
    clamped.joints[i] = static_cast<double>(std::clamp(
        raw, joint_sliders_[i].slider->minimum(), joint_sliders_[i].slider->maximum())) /
        kSliderScale;
  }
  return clamped;
}

void HeadJointSliderPanel::onSyncFromRobotClicked() {
  if (applyJointStateToSliders()) {
    const TargetState synced = collectTargetStateFromSliders();
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      desired_target_ = synced;
      command_target_ = synced;
      command_state_initialized_ = true;
    }
    command_cv_.notify_all();
    setStatus("已从 /joint_states 同步头部滑块", "#E3F2FD");
    scheduleLivePreview();
  } else {
    setStatus("同步失败: /joint_states 缺少头部关节", "#FFF3CD");
  }
}

void HeadJointSliderPanel::onAddRecordClicked() {
  if (!has_joint_state_ || !hasAllTargetJointStates()) {
    setStatus("新增记录失败: /joint_states 缺少头部关节", "#FFF3CD");
    return;
  }

  record_targets_.push_back(targetStateFromLatestJointStates());
  saveRecordTargetsAutoSave(serializeRecords());
  rebuildRecordList();
  setStatus(QString("已新增头部记录点 %1").arg(record_targets_.size()), "#E3F2FD");
}

void HeadJointSliderPanel::onJointStepSliderChanged(int value) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  joint_step_mrad_ = std::clamp(value, 1, 200);
  if (joint_step_value_label_) {
    joint_step_value_label_->setText(QString("%1 mrad").arg(joint_step_mrad_));
  }
  command_cv_.notify_all();
}

double HeadJointSliderPanel::jointStepRad() const {
  return static_cast<double>(std::clamp(joint_step_mrad_, 1, 200)) / 1000.0;
}

void HeadJointSliderPanel::setHomeSliders() {
  suppress_slider_events_ = true;

  for (auto &binding : joint_sliders_) {
    binding.slider->setValue(std::clamp(0, binding.slider->minimum(), binding.slider->maximum()));
  }

  suppress_slider_events_ = false;
}

bool HeadJointSliderPanel::startSteppedMotionFromCurrentState(
    const TargetState &target, const QString &target_name) {
  if (!has_joint_state_ || !hasAllTargetJointStates()) {
    setStatus("执行失败: /joint_states 缺少头部关节", "#FFF3CD");
    return false;
  }

  const TargetState clamped_target = clampTargetToSliderRanges(target);
  applyTargetToSliders(clamped_target);

  const TargetState current = targetStateFromLatestJointStates();
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_target_ = current;
    desired_target_ = clamped_target;
    command_state_initialized_ = true;
  }
  command_cv_.notify_all();
  scheduleLivePreview();
  setStatus(QString("已确认运动到 %1，正在按步长分段执行").arg(target_name), "#E3F2FD");
  return true;
}

void HeadJointSliderPanel::onHomeClicked() {
  if (!node_) {
    setStatus("面板尚未初始化", "#FFCDD2");
    return;
  }

  const auto result = QMessageBox::question(
      this, "确认回零",
      "确认让头部运动到 Home 位置？\n将从当前 /joint_states 开始按步长分段执行。",
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
  if (result != QMessageBox::Yes) {
    setStatus("已取消头部 Home 回零", "#F0F0F0");
    return;
  }

  TargetState home;
  home.joints.assign(HEAD_DOF, 0.0);
  startSteppedMotionFromCurrentState(home, "Home");
}

void HeadJointSliderPanel::rebuildRecordList() {
  if (!record_list_layout_) {
    return;
  }

  clearLayout(record_list_layout_);

  if (record_targets_.empty()) {
    auto *empty_label = new QLabel("暂无记录点");
    empty_label->setStyleSheet("color: #777;");
    record_list_layout_->addWidget(empty_label);
    return;
  }

  for (size_t i = 0; i < record_targets_.size(); ++i) {
    auto *row = new QHBoxLayout;
    auto *label = new QLabel(QString("记录 %1  %2").arg(i + 1).arg(targetSummary(record_targets_[i])));
    label->setWordWrap(true);
    auto *move_button = new QPushButton("运动到此点");
    auto *delete_button = new QPushButton("删除");

    row->addWidget(label, 1);
    row->addWidget(move_button);
    row->addWidget(delete_button);
    record_list_layout_->addLayout(row);

    connect(move_button, &QPushButton::clicked, this, [this, i]() { onRecordMoveClicked(i); });
    connect(delete_button, &QPushButton::clicked, this, [this, i]() { onRecordDeleteClicked(i); });
  }
}

void HeadJointSliderPanel::onRecordMoveClicked(size_t index) {
  if (index >= record_targets_.size()) {
    return;
  }

  const QString name = QString("记录点 %1").arg(index + 1);
  const auto result = QMessageBox::question(
      this, "确认运动",
      QString("确认头部运动到 %1？\n%2\n将从当前 /joint_states 开始按步长分段执行。")
          .arg(name, targetSummary(record_targets_[index])),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
  if (result != QMessageBox::Yes) {
    setStatus(QString("已取消运动到 %1").arg(name), "#F0F0F0");
    return;
  }

  startSteppedMotionFromCurrentState(record_targets_[index], name);
}

void HeadJointSliderPanel::onRecordDeleteClicked(size_t index) {
  if (index >= record_targets_.size()) {
    return;
  }

  const auto result = QMessageBox::question(
      this, "确认删除", QString("删除头部记录点 %1？").arg(index + 1),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
  if (result != QMessageBox::Yes) {
    return;
  }

  record_targets_.erase(record_targets_.begin() + static_cast<std::ptrdiff_t>(index));
  saveRecordTargetsAutoSave(serializeRecords());
  rebuildRecordList();
  setStatus("已删除头部记录点", "#E3F2FD");
}

QString HeadJointSliderPanel::targetSummary(const TargetState &target) const {
  auto rad_to_deg = [](double rad) { return rad * 57.29577951308232; };
  const double yaw = target.joints.empty() ? 0.0 : rad_to_deg(target.joints[0]);
  const double pitch = target.joints.size() < 2 ? 0.0 : rad_to_deg(target.joints[1]);
  return QString("Yaw %1°  Pitch %2°").arg(yaw, 0, 'f', 1).arg(pitch, 0, 'f', 1);
}

QString HeadJointSliderPanel::serializeRecords() const {
  QStringList lines;
  for (const auto &target : record_targets_) {
    QStringList values;
    for (double value : target.joints) {
      values << QString::number(value, 'g', 17);
    }
    lines << values.join(",");
  }
  return lines.join(";");
}

void HeadJointSliderPanel::deserializeRecords(const QString &serialized) {
  record_targets_.clear();
  for (const QString &line : serialized.split(";", Qt::SkipEmptyParts)) {
    const QStringList parts = line.split(",", Qt::SkipEmptyParts);
    if (parts.size() != static_cast<int>(HEAD_DOF)) {
      continue;
    }

    TargetState target;
    target.joints.reserve(HEAD_DOF);
    bool ok = true;
    for (int i = 0; i < static_cast<int>(HEAD_DOF); ++i) {
      target.joints.push_back(parts[i].toDouble(&ok));
      if (!ok) break;
    }
    if (ok) {
      record_targets_.push_back(target);
    }
  }
  rebuildRecordList();
}

void HeadJointSliderPanel::scheduleLivePreview() {
  if (!live_preview_enabled_ || !preview_send_timer_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - last_preview_send_time_ > 40ms && !preview_send_timer_->isActive()) {
    onPreviewClicked();
    return;
  }

  preview_send_timer_->start(40);
}

void HeadJointSliderPanel::onPreviewClicked() {
  publishPreviewTransforms();
}

bool HeadJointSliderPanel::setupPreviewModel(const std::string &urdf_xml) {
  if (urdf_xml.empty()) {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(), "setupPreviewModel skipped: empty URDF");
    }
    return false;
  }
  if (!kdl_parser::treeFromString(urdf_xml, preview_kdl_tree_)) {
    preview_model_ready_ = false;
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "setupPreviewModel failed: kdl_parser::treeFromString failed");
    }
    return false;
  }

  const auto root_it = preview_kdl_tree_.getRootSegment();
  if (root_it == preview_kdl_tree_.getSegments().end()) {
    preview_model_ready_ = false;
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "setupPreviewModel failed: root segment not found");
    }
    return false;
  }
  preview_root_link_ = root_it->first;
  preview_anchor_frame_ = preview_root_link_;

  preview_mimic_map_.clear();
  urdf::Model model;
  if (model.initString(urdf_xml)) {
    for (const auto &entry : model.joints_) {
      const auto &joint = entry.second;
      if (!joint || !joint->mimic) {
        continue;
      }
      preview_mimic_map_[entry.first] = {
          joint->mimic->joint_name,
          {joint->mimic->multiplier, joint->mimic->offset}};
    }
  }

  preview_model_ready_ = true;
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "setupPreviewModel ok: root=%s, segments=%u",
                preview_root_link_.c_str(), preview_kdl_tree_.getNrOfSegments());
  }
  return true;
}

void HeadJointSliderPanel::collectPreviewTransforms(
    const KDL::SegmentMap::const_iterator &segment_it,
    const std::map<std::string, double> &joint_positions, const rclcpp::Time &stamp,
    const std::string &frame_prefix, const std::string &frame_separator,
    std::vector<geometry_msgs::msg::TransformStamped> &out) const {
  for (const auto &child_pair : segment_it->second.children) {
    const auto &child_it = child_pair;
    const std::string &child_name = child_it->first;
    const KDL::Segment &segment = child_it->second.segment;
    const KDL::Joint &joint = segment.getJoint();

    double q = 0.0;
    if (joint.getType() != KDL::Joint::None) {
      const auto pos_it = joint_positions.find(joint.getName());
      if (pos_it != joint_positions.end()) {
        q = pos_it->second;
      }
    }

    const KDL::Frame edge = segment.pose(q);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = frame_prefix + frame_separator + segment_it->first;
    tf.child_frame_id = frame_prefix + frame_separator + child_name;
    tf.transform.translation.x = edge.p.x();
    tf.transform.translation.y = edge.p.y();
    tf.transform.translation.z = edge.p.z();
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    edge.M.GetQuaternion(qx, qy, qz, qw);
    tf.transform.rotation.x = qx;
    tf.transform.rotation.y = qy;
    tf.transform.rotation.z = qz;
    tf.transform.rotation.w = qw;
    out.push_back(tf);

    collectPreviewTransforms(child_it, joint_positions, stamp, frame_prefix, frame_separator, out);
  }
}

void HeadJointSliderPanel::publishPreviewTransforms() {
  if (!node_ || !live_preview_enabled_ || !preview_tf_broadcaster_ || !preview_model_ready_) {
    return;
  }

  const auto root_it = preview_kdl_tree_.getRootSegment();
  if (root_it == preview_kdl_tree_.getSegments().end()) {
    return;
  }

  const TargetState preview_target = collectTargetStateFromSliders();
  std::map<std::string, double> joint_positions;

  for (size_t i = 0; i < head_joint_names_.size() && i < preview_target.joints.size(); ++i) {
    joint_positions[head_joint_names_[i]] = preview_target.joints[i];
  }

  for (const auto &entry : preview_mimic_map_) {
    const auto source_it = joint_positions.find(entry.second.first);
    if (source_it == joint_positions.end()) {
      continue;
    }
    const double multiplier = entry.second.second.first;
    const double offset = entry.second.second.second;
    joint_positions[entry.first] = source_it->second * multiplier + offset;
  }

  const rclcpp::Time stamp = node_->get_clock()->now();
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve((preview_kdl_tree_.getNrOfSegments() + 1) * 2);

  auto append_preview_set = [&](const std::string &prefix, const std::string &separator) {
    geometry_msgs::msg::TransformStamped anchor_tf;
    anchor_tf.header.stamp = stamp;
    anchor_tf.header.frame_id = "world";
    anchor_tf.child_frame_id = prefix + separator + preview_root_link_;
    anchor_tf.transform.translation.x = 0.0;
    anchor_tf.transform.translation.y = 0.0;
    anchor_tf.transform.translation.z = 0.0;
    anchor_tf.transform.rotation.x = 0.0;
    anchor_tf.transform.rotation.y = 0.0;
    anchor_tf.transform.rotation.z = 0.0;
    anchor_tf.transform.rotation.w = 1.0;
    transforms.push_back(anchor_tf);

    collectPreviewTransforms(root_it, joint_positions, stamp, prefix, separator, transforms);
  };

  append_preview_set("preview", "_");
  append_preview_set("preview_", "/");

  if (!transforms.empty()) {
    preview_tf_broadcaster_->sendTransform(transforms);
    last_preview_send_time_ = std::chrono::steady_clock::now();
  }
}

void HeadJointSliderPanel::onPreviewPublishTimer() {
  if (!live_preview_enabled_) {
    return;
  }

  if (preview_model_ready_) {
    publishPreviewTransforms();
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now < preview_model_retry_time_) {
    return;
  }
  preview_model_retry_time_ = now + 2s;
  loadJointLimitsFromRobotDescription();
}

HeadJointSliderPanel::TargetState HeadJointSliderPanel::collectTargetStateFromSliders() const {
  TargetState target;
  target.joints.reserve(HEAD_DOF);

  for (const auto &binding : joint_sliders_) {
    target.joints.push_back(static_cast<double>(binding.slider->value()) / kSliderScale);
  }

  return target;
}

void HeadJointSliderPanel::updateDesiredTargetFromSliders() {
  const TargetState target = collectTargetStateFromSliders();
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    desired_target_ = target;
  }
  command_cv_.notify_all();
  scheduleLivePreview();
}

void HeadJointSliderPanel::startCommandWorker() {
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (command_worker_running_) {
    return;
  }
  command_worker_running_ = true;
  command_state_initialized_ = false;
  command_worker_ = std::thread(&HeadJointSliderPanel::commandWorkerLoop, this);
}

void HeadJointSliderPanel::stopCommandWorker() {
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!command_worker_running_) {
      return;
    }
    command_worker_running_ = false;
  }
  command_cv_.notify_all();
  if (command_worker_.joinable()) {
    command_worker_.join();
  }
}

bool HeadJointSliderPanel::stepTowardsTarget(TargetState &current, const TargetState &target,
                                              double arm_step_rad) const {
  auto step_scalar = [](double &value, double target_value, double step) {
    const double delta = target_value - value;
    if (std::abs(delta) <= step) {
      value = target_value;
      return std::abs(delta) > 1e-9;
    }
    value += (delta > 0.0 ? step : -step);
    return true;
  };

  bool moved = false;
  for (size_t i = 0; i < current.joints.size() && i < target.joints.size(); ++i) {
    moved = step_scalar(current.joints[i], target.joints[i], arm_step_rad) || moved;
  }
  return moved;
}

void HeadJointSliderPanel::commandWorkerLoop() {
  using clock = std::chrono::steady_clock;
  auto next_tick = clock::now();

  while (true) {
    TargetState desired;
    TargetState command;
    double arm_step = 0.02;
    bool initialized = false;
    bool running = false;

    {
      std::unique_lock<std::mutex> lock(command_mutex_);
      command_cv_.wait_until(lock, next_tick);
      running = command_worker_running_;
      if (!running) {
        break;
      }

      desired = desired_target_;
      command = command_target_;
      arm_step = jointStepRad();
      initialized = command_state_initialized_;
    }

    bool moved = false;
    if (initialized) {
      moved = stepTowardsTarget(command, desired, arm_step);
    }

    if (moved) {
      sendWithForwardController(command);
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_target_ = command;
    }

    next_tick = clock::now() + 20ms;
  }
}

bool HeadJointSliderPanel::sendWithForwardController(const TargetState &target) {
  if (!head_forward_pub_) {
    return false;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data = target.joints;

  head_forward_pub_->publish(msg);

  return true;
}

void HeadJointSliderPanel::setStatus(const QString &text, const QString &color_hex) {
  if (!status_label_) {
    return;
  }
  status_label_->setText(text);
  status_label_->setStyleSheet(
      QString("background-color: %1; padding: 6px; border-radius: 4px;").arg(color_hex));
}

void HeadJointSliderPanel::updateStatusText() {
  if (!node_) {
    return;
  }

  const QString joint_state_str = has_joint_state_ ? "joint_states: 正常" : "joint_states: 等待中";
  const QString limits_str =
      joint_limits_loaded_
          ? QString("限位: 来自 %1 的 URDF").arg(QString::fromStdString(joint_limits_source_node_))
          : "限位: 使用默认兜底值 (Yaw ±90°, Pitch -62.2°~+23.1°)";
  const QString step_str = QString("步长: %1 mrad").arg(joint_step_mrad_);
  setStatus(QString("控制: 滑块直控(分段执行) | %1 | %2 | %3 | 预览: 实时")
                .arg(joint_state_str, limits_str, step_str),
            "#F0F0F0");
}

void HeadJointSliderPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);

  int joint_step_mrad = joint_step_mrad_;
  if (config.mapGetInt("joint_step_mrad", &joint_step_mrad)) {
    joint_step_mrad_ = std::clamp(joint_step_mrad, 1, 200);
    if (joint_step_slider_) {
      joint_step_slider_->setValue(joint_step_mrad_);
    }
    if (joint_step_value_label_) {
      joint_step_value_label_->setText(QString("%1 mrad").arg(joint_step_mrad_));
    }
  }

  live_preview_enabled_ = true;

  QString records;
  if (!loadRecordTargetsAutoSave(&records)) {
    const bool has_rviz_records =
        config.mapGetString("record_targets", &records) && !records.trimmed().isEmpty();
    if (!has_rviz_records) {
      records = loadRecordTargetsFallback();
    }
  }
  deserializeRecords(records);
}

void HeadJointSliderPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("joint_step_mrad", joint_step_mrad_);
  config.mapSetValue("record_targets", serializeRecords());
  saveRecordTargetsAutoSave(serializeRecords());
}

}  // namespace openarmx_head_joint_slider_panel

PLUGINLIB_EXPORT_CLASS(openarmx_head_joint_slider_panel::HeadJointSliderPanel, rviz_common::Panel)
