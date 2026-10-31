#pragma once

#include <fstream>
#include <memory>
#include <mutex>
#include <string>

#include <QImage>
#include <QObject>
#include <QString>
#include <QTimer>
#include <QVariantList>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "visualizer_qt/map_panel.h"
#include "visualizer_qt/position_log_watcher.h"
#include "visualizer_qt/rosout_log.h"
#include "visualizer_qt/status_strip.h"

namespace visualizer_qt
{

// QML-facing owner of the rclcpp node and every backend watcher class
// (reused unchanged from visualizer/ — they're all framework-agnostic).
// A single QTimer drives rclcpp::spin_some + image refresh every tick;
// the heavier QVariantList rebuilds (status/position-log/rosout) only
// happen every kListRebuildEveryNTicks ticks, since those don't need
// per-frame freshness and rebuilding them is the actual expensive part
// (PRD §8 resource budget — this app must not compete with inference
// workloads on the Orin for CPU).
class RosBridge : public QObject
{
  Q_OBJECT
  Q_PROPERTY(QImage cameraImage READ cameraImage NOTIFY camerasChanged)
  Q_PROPERTY(QImage depthImage READ depthImage NOTIFY camerasChanged)
  Q_PROPERTY(QImage annotatedImage READ annotatedImage NOTIFY camerasChanged)
  Q_PROPERTY(QImage mapImage READ mapImage NOTIFY mapChanged)
  Q_PROPERTY(bool cameraImageValid READ cameraImageValid NOTIFY camerasChanged)
  Q_PROPERTY(bool mapImageValid READ mapImageValid NOTIFY mapChanged)
  Q_PROPERTY(QVariantList statusBadges READ statusBadges NOTIFY statusChanged)
  // Redesign stage 2 — header's "ROS2 connected" chip was a hardcoded dot,
  // never reflecting real graph state. Count of topics in the ROS graph,
  // excluding /rosout and /parameter_events (always present, not useful
  // signal). Rebuilt on the same cadence as statusBadges.
  Q_PROPERTY(int rosTopicCount READ rosTopicCount NOTIFY statusChanged)
  Q_PROPERTY(QVariantList positionLogRows READ positionLogRows NOTIFY positionLogChanged)
  Q_PROPERTY(bool positionLogActive READ positionLogActive NOTIFY positionLogChanged)
  Q_PROPERTY(QString positionLogPath READ positionLogPath NOTIFY positionLogChanged)
  Q_PROPERTY(QVariantList rosoutLines READ rosoutLines NOTIFY rosoutChanged)
  Q_PROPERTY(QVariantList processOutputLines READ processOutputLines NOTIFY processOutputChanged)

public:
  explicit RosBridge(QObject * parent = nullptr);
  ~RosBridge() override;

  // PRD §5.9 — shells out to the project's own already-field-tested
  // start_sensors.sh/run.sh (never reimplements launch logic), detached
  // (setsid) so the pipeline survives this app closing, output tailed from
  // a logfile into processOutputLines. repo_root must be set via the
  // "repo_root" ROS parameter (launch file) — no safe way to auto-derive
  // it from an installed colcon package.
  Q_INVOKABLE void startSensors();
  // env, if non-empty, is prepended as a shell env-var assignment ahead of
  // run.sh (e.g. "LOCALIZATION=true") — lets a mode's panel pick which of
  // run.sh's own built-in toggles to set without run.sh itself changing.
  Q_INVOKABLE void startPipeline(const QString & mode, const QString & env = QString());

  QImage cameraImage() const {return camera_image_;}
  QImage depthImage() const {return depth_image_;}
  QImage annotatedImage() const {return annotated_image_;}
  QImage mapImage() const {return map_image_;}
  bool cameraImageValid() const {return !camera_image_.isNull();}
  bool mapImageValid() const {return !map_image_.isNull();}
  QVariantList statusBadges() const {return status_badges_;}
  int rosTopicCount() const {return ros_topic_count_;}
  QVariantList positionLogRows() const {return position_log_rows_;}
  bool positionLogActive() const {return position_log_active_;}
  QString positionLogPath() const {return position_log_path_;}
  QVariantList rosoutLines() const {return rosout_lines_;}
  QVariantList processOutputLines() const {return process_output_lines_;}

signals:
  void camerasChanged();
  void mapChanged();
  void statusChanged();
  void positionLogChanged();
  void rosoutChanged();
  void processOutputChanged();

private slots:
  void onTick();

private:
  static QImage matToQImage(const cv::Mat & bgr);
  void appendProcessLine(const QString & tag, const QString & line);
  void tailProcessLog(const QString & path, std::ifstream & file, std::streamoff & offset, const QString & tag);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr annotated_sub_;

  QTimer timer_;
  int tick_count_ = 0;

  std::mutex frame_mutex_;
  cv::Mat latest_camera_frame_;
  cv::Mat latest_depth_frame_;
  cv::Mat latest_annotated_frame_;

  QImage camera_image_;
  QImage depth_image_;
  QImage annotated_image_;
  QImage map_image_;

  PositionLogWatcher position_log_;
  bool position_log_active_ = false;
  QString position_log_path_;
  QVariantList position_log_rows_;

  std::unique_ptr<StatusStrip> status_strip_;
  QVariantList status_badges_;
  int ros_topic_count_ = 0;

  std::unique_ptr<RosoutLog> rosout_log_;
  QVariantList rosout_lines_;

  std::unique_ptr<MapPanel> map_panel_;

  std::string repo_root_;
  QVariantList process_output_lines_;
  std::ifstream sensors_log_file_;
  std::streamoff sensors_log_offset_ = 0;
  std::ifstream pipeline_log_file_;
  std::streamoff pipeline_log_offset_ = 0;
};

}  // namespace visualizer_qt
