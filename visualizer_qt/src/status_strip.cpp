#include "visualizer_qt/status_strip.h"

#include <algorithm>
#include <cstdio>

#include <tf2/time.h>

#include "visualizer_qt/frame_names.h"

namespace visualizer_qt
{

namespace
{

constexpr double kScanMinHz = 8.0;
constexpr double kOdomMinHz = 45.0;
constexpr double kCameraMinHz = 25.0;  // RealSense color/depth stream at ~30 fps
constexpr double kStaleSecs = 2.0;
constexpr size_t kHzWindow = 30;

void pushStamp(std::deque<rclcpp::Time> & stamps, const rclcpp::Time & t)
{
  stamps.push_back(t);
  while (stamps.size() > kHzWindow) {
    stamps.pop_front();
  }
}

}  // namespace

StatusStrip::StatusStrip(const rclcpp::Node::SharedPtr & node)
: node_(node),
  tf_buffer_(node_->get_clock()),
  tf_listener_(tf_buffer_, node_)
{
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/a200_1060/scan", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
      pushStamp(scan_stamps_, rclcpp::Time(msg->header.stamp));
    });

  camera_color_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/camera/camera/color/image_raw", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
      pushStamp(camera_color_stamps_, rclcpp::Time(msg->header.stamp));
    });

  camera_depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/camera/camera/aligned_depth_to_color/image_raw", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
      pushStamp(camera_depth_stamps_, rclcpp::Time(msg->header.stamp));
    });

  // The real battery_state_estimator publisher is BEST_EFFORT (confirmed
  // via `ros2 topic info --verbose`) — a RELIABLE subscriber is QoS-
  // incompatible with it and silently never connects. SensorDataQoS matches.
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/a200_1060/platform/bms/state", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::BatteryState::ConstSharedPtr msg) {
      battery_percentage_ = msg->percentage;
      battery_voltage_ = msg->voltage;
      battery_received_ = true;
    });

  reloc_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_, "semantic_map_exporter");
}

void StatusStrip::update()
{
  auto now = std::chrono::steady_clock::now();
  if (now - last_refresh_ < std::chrono::milliseconds(1000)) {
    return;
  }
  last_refresh_ = now;
  refresh();
}

void StatusStrip::refresh()
{
  badges_.clear();
  const rclcpp::Time now = node_->now();

  badges_.push_back(tfBadge("TF map->base", kMapFrame, kBaseFrame));
  badges_.push_back(tfBadge("TF base->camera", kBaseFrame, kCameraFrame));
  badges_.push_back(tfBadge("TF base->laser", kBaseAliasFrame, kLaserFrame));

  badges_.push_back(hzBadge("Scan", scan_stamps_, now, kScanMinHz));
  ensureOdomSubscription();
  badges_.push_back(hzBadge(odom_label_, odom_stamps_, now, kOdomMinHz));

  badges_.push_back(hzBadge("Camera color", camera_color_stamps_, now, kCameraMinHz));
  badges_.push_back(hzBadge("Camera depth", camera_depth_stamps_, now, kCameraMinHz));

  badges_.push_back(batteryBadge());

  auto names = nodeBaseNames();
  badges_.push_back(nodeBadge("rtabmap", names, true));
  badges_.push_back(nodeBadge("yolo_detector", names, false));
  badges_.push_back(nodeBadge("semantic_projector", names, false));

  pollRelocalizationParam();
  badges_.push_back(relocalizationBadge());

  session_meta_.update();
  appendSessionBadges();
}

void StatusStrip::ensureOdomSubscription()
{
  if (odom_sub_) {
    return;
  }
  // Mirrors precheck()'s odom-topic selection in run.sh: prefer the filtered
  // EKF output if the robot publishes it, fall back to raw /odom otherwise.
  auto topics = node_->get_topic_names_and_types();
  std::string chosen;
  if (topics.count("/a200_1060/platform/odom/filtered") > 0) {
    chosen = "/a200_1060/platform/odom/filtered";
  } else if (topics.count("/odom") > 0) {
    chosen = "/odom";
  } else {
    return;  // neither exists yet — retry next refresh tick
  }

  odom_label_ = "Odom (" + chosen + ")";
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    chosen, rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      pushStamp(odom_stamps_, rclcpp::Time(msg->header.stamp));
    });
}

void StatusStrip::pollRelocalizationParam()
{
  if (reloc_param_in_flight_) {
    return;
  }
  if (!reloc_param_client_->service_is_ready()) {
    relocalization_ok_.reset();
    return;
  }
  reloc_param_in_flight_ = true;
  reloc_param_client_->get_parameters(
    {"relocalization_ok"},
    [this](std::shared_future<std::vector<rclcpp::Parameter>> result) {
      reloc_param_in_flight_ = false;
      auto params = result.get();
      if (!params.empty() &&
        params.front().get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        relocalization_ok_ = params.front().as_bool();
      }
    });
}

std::vector<std::string> StatusStrip::nodeBaseNames()
{
  return node_->get_node_names();
}

Badge StatusStrip::tfBadge(
  const std::string & label, const std::string & parent, const std::string & child)
{
  std::string err;
  bool ok = tf_buffer_.canTransform(parent, child, tf2::TimePointZero, &err);
  Badge b;
  b.label = label;
  if (ok) {
    b.level = BadgeLevel::kGreen;
    b.detail = parent + " <-> " + child;
  } else {
    b.level = BadgeLevel::kRed;
    b.detail = "missing: " + err;
  }
  return b;
}

Badge StatusStrip::hzBadge(
  const std::string & label, const std::deque<rclcpp::Time> & stamps,
  const rclcpp::Time & now, double min_hz)
{
  Badge b;
  b.label = label;

  if (stamps.empty()) {
    b.level = BadgeLevel::kRed;
    b.detail = "no data";
    return b;
  }

  double age = (now - stamps.back()).seconds();
  if (age > kStaleSecs) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "stale (%.1fs ago)", age);
    b.level = BadgeLevel::kRed;
    b.detail = buf;
    return b;
  }

  if (stamps.size() < 2) {
    b.level = BadgeLevel::kAmber;
    b.detail = "measuring...";
    return b;
  }

  double span = (stamps.back() - stamps.front()).seconds();
  double hz = span > 0.0 ? (static_cast<double>(stamps.size() - 1) / span) : 0.0;
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.1f Hz (target %.0f)", hz, min_hz);
  b.detail = buf;
  b.level = (hz >= min_hz) ? BadgeLevel::kGreen : BadgeLevel::kAmber;
  return b;
}

Badge StatusStrip::nodeBadge(
  const std::string & name, const std::vector<std::string> & present_names, bool critical)
{
  bool present = std::find(present_names.begin(), present_names.end(), "/" + name) !=
    present_names.end();
  Badge b;
  b.label = name;
  if (present) {
    b.level = BadgeLevel::kGreen;
    b.detail = "running";
  } else if (critical) {
    b.level = BadgeLevel::kRed;
    b.detail = "not running";
  } else {
    b.level = BadgeLevel::kAmber;
    b.detail = "not running (expected if this isn't a semantic session)";
  }
  return b;
}

void StatusStrip::appendSessionBadges()
{
  // PRD §5.6: session/mode awareness, sourced from meta_<session_id>.yaml —
  // informational, not a pass/fail check, so these use the neutral/advisory
  // color rather than red/amber/green (relocalization_ok already has its
  // own live-parameter badge above; this is session *metadata*, not a
  // second copy of that check).
  if (!session_meta_.hasActiveSession()) {
    Badge b;
    b.label = "Session";
    b.level = BadgeLevel::kAdvisory;
    b.detail = "no active logged session (enable_position_logger not set, or none started yet)";
    badges_.push_back(b);
    return;
  }

  const SessionMeta & m = session_meta_.meta();

  Badge session_id_badge;
  session_id_badge.label = "Session ID";
  session_id_badge.level = BadgeLevel::kAdvisory;
  session_id_badge.detail = m.session_id;
  badges_.push_back(session_id_badge);

  Badge phase_badge;
  phase_badge.label = "Phase";
  phase_badge.level = BadgeLevel::kAdvisory;
  phase_badge.detail = m.phase.empty() ? "(unknown)" : m.phase;
  badges_.push_back(phase_badge);

  Badge objects_badge;
  objects_badge.label = "Objects present";
  objects_badge.level = BadgeLevel::kAdvisory;
  if (m.objects_present.empty()) {
    objects_badge.detail = "(none)";
  } else {
    std::string joined;
    for (size_t i = 0; i < m.objects_present.size(); ++i) {
      if (i > 0) {joined += ", ";}
      joined += m.objects_present[i];
    }
    objects_badge.detail = joined + " (" + std::to_string(m.objects_present.size()) + ")";
  }
  badges_.push_back(objects_badge);
}

Badge StatusStrip::relocalizationBadge()
{
  Badge b;
  b.label = "relocalization_ok";
  if (!relocalization_ok_.has_value()) {
    b.level = BadgeLevel::kAdvisory;
    b.detail = "n/a (semantic_map_exporter not running / param not read yet)";
  } else if (*relocalization_ok_) {
    b.level = BadgeLevel::kGreen;
    b.detail = "true";
  } else {
    b.level = BadgeLevel::kRed;
    b.detail = "false";
  }
  return b;
}

Badge StatusStrip::batteryBadge()
{
  Badge b;
  b.label = "Battery";
  if (!battery_received_) {
    b.level = BadgeLevel::kRed;
    b.detail = "no data";
    return b;
  }
  char buf[64];
  std::snprintf(
    buf, sizeof(buf), "%.0f%% (%.1fV)",
    static_cast<double>(battery_percentage_) * 100.0, static_cast<double>(battery_voltage_));
  b.detail = buf;
  if (battery_percentage_ < 0.15f) {
    b.level = BadgeLevel::kRed;
  } else if (battery_percentage_ < 0.30f) {
    b.level = BadgeLevel::kAmber;
  } else {
    b.level = BadgeLevel::kGreen;
  }
  return b;
}

}  // namespace visualizer_qt
