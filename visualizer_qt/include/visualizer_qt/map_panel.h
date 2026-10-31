#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace visualizer_qt
{

// Renders the PRD §5.4 map+pose panel: /map (OccupancyGrid) as a 2D
// top-down grid with the robot pose (map->base TF) drawn as a triangle on
// top. Works unchanged whether the grid is actively growing (mapping,
// Mem/IncrementalMemory=true) or static with a moving pose (K1/K2
// localization) — same topic, same render path, no mode-specific code.
class MapPanel
{
public:
  explicit MapPanel(const rclcpp::Node::SharedPtr & node);

  // Call once per frame. Returns an empty Mat if no /map has arrived yet.
  cv::Mat render();

private:
  void onMap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  cv::Point2f worldToPixel(double wx, double wy) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  cv::Mat base_image_;  // occupancy grid colorized; rebuilt only when /map arrives
  double resolution_ = 0.0;
  int width_ = 0;
  int height_ = 0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_yaw_ = 0.0;
  bool has_map_ = false;
};

}  // namespace visualizer_qt
