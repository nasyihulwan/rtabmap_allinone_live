#include "visualizer_qt/ros_bridge.h"

#include <algorithm>
#include <regex>
#include <sstream>

#include <QCoreApplication>
#include <QFile>
#include <QProcess>

#include <opencv2/imgproc.hpp>

namespace visualizer_qt
{

namespace
{

constexpr size_t kMaxProcessOutputLines = 300;
constexpr const char * kSensorsLogPath = "/tmp/visualizer_qt_start_sensors.log";
constexpr const char * kPipelineLogPath = "/tmp/visualizer_qt_run_pipeline.log";

// start_sensors.sh/run.sh color their own terminal output with raw ANSI SGR
// escapes (${CYAN}/${GREEN}/... bash vars) — fine in a real terminal, but
// QML Text doesn't interpret them, so they showed up as literal garbage
// ("^[[0;36m...") in the log panel. Strip them; the log panel already
// colors lines by source tag (process output vs /rosout severity), so
// there's no need to reparse and reapply the shell's own per-phrase colors.
std::string stripAnsi(const std::string & s)
{
  static const std::regex kAnsiSgr("\x1B\\[[0-9;]*m");
  return std::regex_replace(s, kAnsiSgr, "");
}

// Ported from visualizer/src/main.cpp's colorizeDepth — same clip range,
// same TURBO colormap, same zero-depth-stays-black behavior.
constexpr double kDepthMinMm = 300.0;
constexpr double kDepthMaxMm = 6000.0;

// Wraps the ROS Image message's raw buffer directly instead of going
// through cv_bridge::toCvCopy/toCvShare. cv_bridge.so is a prebuilt ROS
// binary linked against the system's apt OpenCV; on this Orin a second,
// differently-versioned OpenCV build also exists (CUDA/TensorRT build for
// YOLO) and gets picked up by our own find_package(OpenCV). Neither build
// uses OpenCV's ABI-versioned inline namespace here, so cv::Mat::create/
// clone resolve to a single, arbitrarily-chosen symbol process-wide — a
// cv::Mat built by cv_bridge's OpenCV and touched by our OpenCV (or vice
// versa) reads/writes the wrong struct layout and crashes
// (cv::Mat::create "Assertion failed: s >= 0"). Building the cv::Mat
// ourselves from the raw buffer means it's only ever touched by our own
// linked OpenCV, so the two builds never have to agree on ABI.
cv::Mat toBgrMat(const sensor_msgs::msg::Image & msg)
{
  if (msg.encoding != "rgb8" && msg.encoding != "bgr8") {
    return cv::Mat();
  }
  cv::Mat raw(
    static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC3,
    const_cast<uint8_t *>(msg.data.data()), msg.step);
  if (msg.encoding == "bgr8") {
    return raw.clone();
  }
  cv::Mat bgr;
  cv::cvtColor(raw, bgr, cv::COLOR_RGB2BGR);
  return bgr;
}

cv::Mat toDepthMat16(const sensor_msgs::msg::Image & msg)
{
  if (msg.encoding != "16UC1" && msg.encoding != "mono16") {
    return cv::Mat();
  }
  cv::Mat raw(
    static_cast<int>(msg.height), static_cast<int>(msg.width), CV_16UC1,
    const_cast<uint8_t *>(msg.data.data()), msg.step);
  return raw.clone();
}

cv::Mat colorizeDepth(const cv::Mat & depth16)
{
  if (depth16.empty() || depth16.type() != CV_16UC1) {
    return cv::Mat();
  }
  cv::Mat valid_mask = depth16 > 0;
  cv::Mat clipped;
  depth16.convertTo(clipped, CV_64F);
  cv::min(clipped, kDepthMaxMm, clipped);
  cv::max(clipped, kDepthMinMm, clipped);
  cv::Mat normalized;
  const double scale = 255.0 / (kDepthMaxMm - kDepthMinMm);
  clipped.convertTo(normalized, CV_8U, scale, -scale * kDepthMinMm);
  cv::Mat colorized;
  cv::applyColorMap(normalized, colorized, cv::COLORMAP_TURBO);
  colorized.setTo(cv::Scalar(0, 0, 0), ~valid_mask);
  return colorized;
}

// List properties are only sent to QML every Nth tick (~1 Hz at the default
// 10 Hz timer) — rebuilding these every frame would be wasted CPU the
// camera/map image refresh doesn't need and the Orin can't spare (PRD §8).
constexpr int kListRebuildEveryNTicks = 10;
constexpr size_t kMaxPositionLogRowsShown = 50;
constexpr size_t kMaxRosoutLinesShown = 150;

}  // namespace

QImage RosBridge::matToQImage(const cv::Mat & bgr)
{
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    return QImage();
  }
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return QImage(
    rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
    QImage::Format_RGB888).copy();
}

RosBridge::RosBridge(QObject * parent)
: QObject(parent)
{
  node_ = std::make_shared<rclcpp::Node>("visualizer_qt_node");
  repo_root_ = node_->declare_parameter("repo_root", std::string(""));

  camera_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/camera/camera/color/image_raw", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
      cv::Mat bgr = toBgrMat(*msg);
      if (bgr.empty()) {
        RCLCPP_WARN(node_->get_logger(), "unsupported camera color encoding: %s", msg->encoding.c_str());
        return;
      }
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_camera_frame_ = bgr;
    });

  depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/camera/camera/aligned_depth_to_color/image_raw", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
      cv::Mat depth16 = toDepthMat16(*msg);
      if (depth16.empty()) {
        RCLCPP_WARN(node_->get_logger(), "unsupported depth encoding: %s", msg->encoding.c_str());
        return;
      }
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_depth_frame_ = depth16;
    });

  annotated_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/semantic/image_annotated", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
      cv::Mat bgr = toBgrMat(*msg);
      if (bgr.empty()) {
        RCLCPP_WARN(node_->get_logger(), "unsupported annotated-image encoding: %s", msg->encoding.c_str());
        return;
      }
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_annotated_frame_ = bgr;
    });

  status_strip_ = std::make_unique<StatusStrip>(node_);

  rosout_log_ = std::make_unique<RosoutLog>(node_);
  map_panel_ = std::make_unique<MapPanel>(node_);

  connect(&timer_, &QTimer::timeout, this, &RosBridge::onTick);
  timer_.start(100);  // 10 Hz — image refresh rate, PRD §8 display-rate cap
}

RosBridge::~RosBridge() = default;

void RosBridge::onTick()
{
  // rclcpp installs its own SIGINT/SIGTERM handler that calls
  // rclcpp::shutdown() directly, independent of Qt's event loop — without
  // this check, the next timer tick after a signal calls spin_some() on an
  // already-invalidated rcl context and crashes (guard_condition error).
  // Mirrors the ImGui app's `while (... && rclcpp::ok())` loop-exit check,
  // just via a timer tick instead of a while loop.
  if (!rclcpp::ok()) {
    QCoreApplication::quit();
    return;
  }
  // A single spin_some() call drains at most ~1 message per subscription
  // (confirmed empirically: Scan/Odom Hz badges plateaued at ~9.9 Hz, i.e.
  // this timer's own 100ms tick rate, regardless of the real ~40/48 Hz
  // topic rate). Loop it so one tick keeps up with the several sensor
  // messages that actually arrive between UI-refresh ticks, instead of
  // silently undersampling and reporting a fake Hz.
  for (int i = 0; i < 10 && rclcpp::ok(); ++i) {
    rclcpp::spin_some(node_);
  }
  ++tick_count_;

  tailProcessLog(kSensorsLogPath, sensors_log_file_, sensors_log_offset_, "start_sensors.sh");
  tailProcessLog(kPipelineLogPath, pipeline_log_file_, pipeline_log_offset_, "run.sh");

  cv::Mat cam, depth, annotated;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    cam = latest_camera_frame_;
    depth = latest_depth_frame_;
    annotated = latest_annotated_frame_;
  }

  bool cameras_dirty = false;
  if (!cam.empty()) {
    camera_image_ = matToQImage(cam);
    cameras_dirty = true;
  }
  if (!depth.empty()) {
    cv::Mat colorized = colorizeDepth(depth);
    if (!colorized.empty()) {
      depth_image_ = matToQImage(colorized);
      cameras_dirty = true;
    }
  }
  if (!annotated.empty()) {
    annotated_image_ = matToQImage(annotated);
    cameras_dirty = true;
  }
  if (cameras_dirty) {
    emit camerasChanged();
  }

  cv::Mat map_mat = map_panel_->render();
  if (!map_mat.empty()) {
    map_image_ = matToQImage(map_mat);
    emit mapChanged();
  }

  // Cheap watchers (self-throttled internally) can update every tick; only
  // the QVariantList rebuild+emit below is gated to kListRebuildEveryNTicks.
  position_log_.update();
  status_strip_->update();

  if (tick_count_ % kListRebuildEveryNTicks != 0) {
    return;
  }

  position_log_active_ = position_log_.hasActiveFile();
  position_log_path_ = QString::fromStdString(position_log_.currentPath());
  QVariantList rows;
  const auto & all_rows = position_log_.rows();
  size_t start = all_rows.size() > kMaxPositionLogRowsShown ?
    all_rows.size() - kMaxPositionLogRowsShown : 0;
  for (size_t i = start; i < all_rows.size(); ++i) {
    const auto & row = all_rows[i];
    QVariantMap m;
    m["frameSeq"] = QString::fromStdString(row.frame_seq);
    m["className"] = QString::fromStdString(row.class_name);
    m["trackerId"] = QString::fromStdString(row.tracker_id);
    m["x"] = QString::fromStdString(row.x);
    m["y"] = QString::fromStdString(row.y);
    m["confidence"] = QString::fromStdString(row.confidence);
    rows.append(m);
  }
  position_log_rows_ = rows;
  emit positionLogChanged();

  int topic_count = 0;
  for (const auto & kv : node_->get_topic_names_and_types()) {
    if (kv.first != "/rosout" && kv.first != "/parameter_events") {
      ++topic_count;
    }
  }
  ros_topic_count_ = topic_count;

  QVariantList badges;
  for (const auto & b : status_strip_->badges()) {
    QVariantMap m;
    m["label"] = QString::fromStdString(b.label);
    m["detail"] = QString::fromStdString(b.detail);
    switch (b.level) {
      case BadgeLevel::kGreen: m["level"] = "green"; break;
      case BadgeLevel::kAmber: m["level"] = "amber"; break;
      case BadgeLevel::kRed: m["level"] = "red"; break;
      default: m["level"] = "advisory"; break;
    }
    badges.append(m);
  }
  status_badges_ = badges;
  emit statusChanged();

  QVariantList lines;
  const auto & all_entries = rosout_log_->entries();
  size_t rstart = all_entries.size() > kMaxRosoutLinesShown ?
    all_entries.size() - kMaxRosoutLinesShown : 0;
  for (size_t i = rstart; i < all_entries.size(); ++i) {
    const auto & e = all_entries[i];
    QVariantMap m;
    m["stamp"] = e.stamp.seconds();
    m["level"] = static_cast<int>(e.level);
    m["node"] = QString::fromStdString(e.node_name);
    m["message"] = QString::fromStdString(e.message);
    lines.append(m);
  }
  rosout_lines_ = lines;
  emit rosoutChanged();
}

void RosBridge::appendProcessLine(const QString & tag, const QString & line)
{
  QVariantMap m;
  m["tag"] = tag;
  m["line"] = line;
  process_output_lines_.append(m);
  if (static_cast<size_t>(process_output_lines_.size()) > kMaxProcessOutputLines) {
    process_output_lines_.removeFirst();
  }
  emit processOutputChanged();
}

void RosBridge::tailProcessLog(
  const QString & path, std::ifstream & file, std::streamoff & offset, const QString & tag)
{
  if (!file.is_open()) {
    file.open(path.toStdString());
    if (!file.is_open()) {
      return;  // not created yet — nothing started this session
    }
    offset = 0;
  }

  file.clear();
  file.seekg(offset);
  std::ostringstream buf;
  buf << file.rdbuf();
  const std::string chunk = buf.str();
  if (chunk.empty()) {
    return;
  }

  size_t pos = 0;
  size_t consumed = 0;
  while (true) {
    size_t nl = chunk.find('\n', pos);
    if (nl == std::string::npos) {
      break;
    }
    std::string line = chunk.substr(pos, nl - pos);
    if (!line.empty()) {
      appendProcessLine(tag, QString::fromStdString(stripAnsi(line)));
    }
    pos = nl + 1;
    consumed = pos;
  }
  offset += static_cast<std::streamoff>(consumed);
}

void RosBridge::startSensors()
{
  if (repo_root_.empty()) {
    appendProcessLine(
      "start_sensors.sh",
      "ERROR: repo_root parameter not set — pass via launch file "
      "(repo_root:=/path/to/rtabmap_allinone_live)");
    return;
  }

  // Reset the tailer so the new run's output starts from byte 0, not
  // wherever the previous invocation's log happened to end.
  sensors_log_file_.close();
  sensors_log_offset_ = 0;
  QFile::remove(QString(kSensorsLogPath));

  QString script = QString::fromStdString(repo_root_) + "/start_sensors.sh";
  QString cmd = script + " > " + kSensorsLogPath + " 2>&1 < /dev/null";
  appendProcessLine("start_sensors.sh", "launching: " + script);
  // setsid detaches fully — the sensor stack must keep running even if
  // this viewer is closed, PRD §5.9.
  QProcess::startDetached("setsid", {"bash", "-c", cmd});
}

void RosBridge::startPipeline(const QString & mode, const QString & env)
{
  if (repo_root_.empty()) {
    appendProcessLine(
      "run.sh",
      "ERROR: repo_root parameter not set — pass via launch file "
      "(repo_root:=/path/to/rtabmap_allinone_live)");
    return;
  }

  pipeline_log_file_.close();
  pipeline_log_offset_ = 0;
  QFile::remove(QString(kPipelineLogPath));

  QString envPrefix = env.isEmpty() ? QString() : (env + " ");
  QString script = QString::fromStdString(repo_root_) + "/run.sh";
  QString cmd = envPrefix + script + " " + mode + " > " + kPipelineLogPath + " 2>&1 < /dev/null";
  appendProcessLine("run.sh", "launching: " + envPrefix + script + " " + mode);
  appendProcessLine(
    "run.sh",
    "note: run.sh opens a terminal (gnome-terminal/xterm) for the actual "
    "rtabmap process — confirm one is installed on this machine, PRD §5.9.");
  QProcess::startDetached("setsid", {"bash", "-c", cmd});
}

}  // namespace visualizer_qt
