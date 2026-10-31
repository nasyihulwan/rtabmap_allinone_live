#pragma once

#include <chrono>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "visualizer_qt/session_meta.h"

namespace visualizer_qt
{

// FAA 14 CFR 25.1322 alerting convention (PRD §5.5a): red = critical/broken,
// amber = caution/degraded, green = normal, advisory = neutral info.
enum class BadgeLevel { kGreen, kAmber, kRed, kAdvisory };

struct Badge
{
  std::string label;
  BadgeLevel level = BadgeLevel::kAdvisory;
  std::string detail;
};

// Drives the PRD §5.5a status strip: TF connectivity, sensor Hz/liveness,
// node liveness, and the semantic_map_exporter relocalization_ok parameter.
// Internally throttled to ~1 Hz; call update() every render frame.
class StatusStrip
{
public:
  explicit StatusStrip(const rclcpp::Node::SharedPtr & node);

  void update();
  const std::vector<Badge> & badges() const {return badges_;}

private:
  void refresh();
  void ensureOdomSubscription();
  void pollRelocalizationParam();
  std::vector<std::string> nodeBaseNames();

  Badge tfBadge(const std::string & label, const std::string & parent, const std::string & child);
  Badge hzBadge(
    const std::string & label, const std::deque<rclcpp::Time> & stamps,
    const rclcpp::Time & now, double min_hz);
  Badge nodeBadge(const std::string & name, const std::vector<std::string> & present_names, bool critical);
  Badge relocalizationBadge();
  Badge batteryBadge();
  void appendSessionBadges();

  rclcpp::Node::SharedPtr node_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  std::string odom_label_ = "Odom";
  std::deque<rclcpp::Time> scan_stamps_;
  std::deque<rclcpp::Time> odom_stamps_;
  std::deque<rclcpp::Time> camera_color_stamps_;
  std::deque<rclcpp::Time> camera_depth_stamps_;

  // BMS state has no meaningful header.stamp (driver publishes it zeroed) —
  // track "did we ever receive one" ourselves instead of using the
  // timestamp-deque Hz machinery the other sensors use.
  bool battery_received_ = false;
  float battery_percentage_ = 0.0f;
  float battery_voltage_ = 0.0f;

  rclcpp::AsyncParametersClient::SharedPtr reloc_param_client_;
  bool reloc_param_in_flight_ = false;
  std::optional<bool> relocalization_ok_;

  SessionMetaWatcher session_meta_;

  // Throttles recompute to ~1 Hz; pure UI pacing, not a staleness check, so
  // wall-clock steady_clock is correct here even under use_sim_time (§5.7) —
  // the actual data-staleness math below uses node_->now() / message stamps.
  std::chrono::steady_clock::time_point last_refresh_{};
  std::vector<Badge> badges_;
};

}  // namespace visualizer_qt
