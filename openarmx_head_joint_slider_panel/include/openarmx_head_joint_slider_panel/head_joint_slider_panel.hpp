#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <QLabel>
#include <QPushButton>
#include <QGroupBox>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kdl/tree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace openarmx_head_joint_slider_panel {

class HeadJointSliderPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit HeadJointSliderPanel(QWidget *parent = nullptr);
  ~HeadJointSliderPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config &config) override;
  void save(rviz_common::Config config) const override;

 private Q_SLOTS:
	  void onSyncFromRobotClicked();
	  void onAddRecordClicked();
	  void onPreviewClicked();
	  void onHomeClicked();
	  void onJointStepSliderChanged(int value);
  void updateStatusText();

 private:
  static constexpr size_t HEAD_DOF = 2;

  struct SliderBinding {
    QSlider *slider{nullptr};
    QLabel *value_label{nullptr};
  };

  struct TargetState {
    std::vector<double> joints;  // [yaw, pitch]
  };

  void setupUi();
  SliderBinding createJointSliderRow(const QString &title, QVBoxLayout *parent_layout,
                                     double init_value);
  void updateSliderLabel(const SliderBinding &binding);
  void setStatus(const QString &text, const QString &color_hex);
  void applyDefaultSliderRanges();
  bool loadJointLimitsFromRobotDescription();
  bool applyUrdfJointLimits(const std::string &urdf_xml);
	  void scheduleLivePreview();
	  void updateDesiredTargetFromSliders();
	  void setHomeSliders();
	  void applyTargetToSliders(const TargetState &target);
	  TargetState clampTargetToSliderRanges(const TargetState &target) const;
	  bool startSteppedMotionFromCurrentState(const TargetState &target, const QString &target_name);
	  double jointStepRad() const;
	  void rebuildRecordList();
	  void onRecordMoveClicked(size_t index);
	  void onRecordDeleteClicked(size_t index);
	  QString targetSummary(const TargetState &target) const;
	  QString serializeRecords() const;
	  void deserializeRecords(const QString &serialized);
	  void publishPreviewTransforms();
	  void onPreviewPublishTimer();
  void startCommandWorker();
  void stopCommandWorker();
  void commandWorkerLoop();
  bool stepTowardsTarget(TargetState &current, const TargetState &target,
                         double arm_step_rad) const;
  bool setupPreviewModel(const std::string &urdf_xml);
  void collectPreviewTransforms(const KDL::SegmentMap::const_iterator &segment_it,
                                const std::map<std::string, double> &joint_positions,
                                const rclcpp::Time &stamp, const std::string &frame_prefix,
                                const std::string &frame_separator,
                                std::vector<geometry_msgs::msg::TransformStamped> &out) const;

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool hasAllTargetJointStates() const;
  TargetState targetStateFromLatestJointStates() const;
  bool applyJointStateToSliders();

  TargetState collectTargetStateFromSliders() const;

  bool sendWithForwardController(const TargetState &target);

  static constexpr int kSliderScale = 1000;       // slider unit: 0.001 rad
  static constexpr int kSliderMin = -1571;        // yaw fallback: -1.571 rad
  static constexpr int kSliderMax = 1571;         // yaw fallback: +1.571 rad
  static constexpr int kPitchSliderMin = -1086;   // pitch fallback: -1.086 rad
  static constexpr int kPitchSliderMax = 403;     // pitch fallback: +0.403 rad

  std::array<SliderBinding, HEAD_DOF> joint_sliders_;

	  QPushButton *sync_button_{nullptr};
	  QPushButton *home_button_{nullptr};
	  QPushButton *add_record_button_{nullptr};
	  QGroupBox *record_group_{nullptr};
	  QVBoxLayout *record_list_layout_{nullptr};
	  QLabel *status_label_{nullptr};
  QSlider *joint_step_slider_{nullptr};
  QLabel *joint_step_value_label_{nullptr};

  QTimer *ros_spin_timer_{nullptr};
  QTimer *status_timer_{nullptr};
  QTimer *preview_send_timer_{nullptr};
  QTimer *preview_publish_timer_{nullptr};

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_forward_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> preview_tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  std::map<std::string, double> latest_joint_state_map_;
  bool has_joint_state_{false};
  bool auto_sync_done_{false};

  bool suppress_slider_events_{false};
  bool joint_limits_loaded_{false};
  std::string joint_limits_source_node_;

  std::mutex command_mutex_;
  std::condition_variable command_cv_;
  std::thread command_worker_;
  bool command_worker_running_{false};
  bool command_state_initialized_{false};
	  TargetState desired_target_;
	  TargetState command_target_;
	  std::vector<TargetState> record_targets_;
	  int joint_step_mrad_{10};         // 0.010 rad per cycle
  bool live_preview_enabled_{true};
  std::chrono::steady_clock::time_point last_preview_send_time_;
  std::chrono::steady_clock::time_point preview_model_retry_time_;
  KDL::Tree preview_kdl_tree_;
  std::string preview_root_link_;
  std::string preview_anchor_frame_;
  bool preview_model_ready_{false};
  std::map<std::string, std::pair<std::string, std::pair<double, double>>> preview_mimic_map_;

  const std::vector<std::string> head_joint_names_ = {
      "openarmx_head_yaw_joint",
      "openarmx_head_pitch_joint"
  };

  const std::vector<QString> head_joint_labels_ = {
      "偏航 Yaw (左右)",
      "俯仰 Pitch (上下)"
  };
};

}  // namespace openarmx_head_joint_slider_panel
