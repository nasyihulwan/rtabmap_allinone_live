#include "visualizer_qt/map_panel.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/exceptions.h>

#include "visualizer_qt/frame_names.h"

namespace visualizer_qt
{

namespace
{
// Pose triangle size, in meters — independent of grid resolution so it
// stays visible on both coarse and fine-resolution maps.
constexpr double kArrowMinLenM = 0.3;
constexpr double kArrowResMultiplier = 6.0;
constexpr double kArrowHalfWidthRatio = 0.35;
// Back-corner sweep angle (radians) from the forward direction — ~137.5°,
// giving a classic arrowhead silhouette (apex forward, corners swept back).
constexpr double kArrowBackSweepRad = 2.4;
}  // namespace

MapPanel::MapPanel(const rclcpp::Node::SharedPtr & node)
: node_(node),
  tf_buffer_(node_->get_clock()),
  tf_listener_(tf_buffer_, node_)
{
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {onMap(msg);});
}

void MapPanel::onMap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  width_ = static_cast<int>(msg->info.width);
  height_ = static_cast<int>(msg->info.height);
  resolution_ = msg->info.resolution;
  origin_x_ = msg->info.origin.position.x;
  origin_y_ = msg->info.origin.position.y;

  // Planar yaw from quaternion (ground robot — roll/pitch assumed 0, true
  // for every real map origin seen on this project so far).
  double qz = msg->info.origin.orientation.z;
  double qw = msg->info.origin.orientation.w;
  origin_yaw_ = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);

  if (width_ <= 0 || height_ <= 0 ||
    msg->data.size() != static_cast<size_t>(width_) * static_cast<size_t>(height_))
  {
    return;
  }

  // RViz-style coloring: free=white, occupied=black (linear in between),
  // unknown (-1)=mid-gray, distinct from the occupancy gradient.
  cv::Mat img(height_, width_, CV_8UC3);
  for (int y = 0; y < height_; ++y) {
    // OccupancyGrid row 0 is the "bottom" in map-frame +y; image row 0 is
    // the top, so read grid rows back-to-front to orient the image sanely.
    int src_row = (height_ - 1) - y;
    const int8_t * row = &msg->data[static_cast<size_t>(src_row) * width_];
    cv::Vec3b * out_row = img.ptr<cv::Vec3b>(y);
    for (int x = 0; x < width_; ++x) {
      int v = static_cast<int>(row[x]);
      uint8_t gray;
      if (v < 0) {
        gray = 128;
      } else {
        v = std::min(v, 100);
        gray = static_cast<uint8_t>(255 - (v * 255 / 100));
      }
      out_row[x] = cv::Vec3b(gray, gray, gray);
    }
  }
  base_image_ = img;
  has_map_ = true;
}

cv::Point2f MapPanel::worldToPixel(double wx, double wy) const
{
  double dx = wx - origin_x_;
  double dy = wy - origin_y_;
  double cos_yaw = std::cos(-origin_yaw_);
  double sin_yaw = std::sin(-origin_yaw_);
  double local_x = cos_yaw * dx - sin_yaw * dy;
  double local_y = sin_yaw * dx + cos_yaw * dy;
  double mx = local_x / resolution_;
  double my = local_y / resolution_;
  return cv::Point2f(
    static_cast<float>(mx),
    static_cast<float>((height_ - 1) - my));
}

cv::Mat MapPanel::render()
{
  if (!has_map_) {
    return cv::Mat();
  }
  cv::Mat img = base_image_.clone();

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(kMapFrame, kBaseFrame, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return img;  // map without a pose arrow is still useful (PRD §8 idle state)
  }

  double rx = tf.transform.translation.x;
  double ry = tf.transform.translation.y;
  double rqz = tf.transform.rotation.z;
  double rqw = tf.transform.rotation.w;
  double ryaw = std::atan2(2.0 * rqw * rqz, 1.0 - 2.0 * rqz * rqz);

  double arrow_len_m = std::max(resolution_ * kArrowResMultiplier, kArrowMinLenM);
  double half_w_m = arrow_len_m * kArrowHalfWidthRatio;

  cv::Point2f tip = worldToPixel(
    rx + arrow_len_m * std::cos(ryaw), ry + arrow_len_m * std::sin(ryaw));
  cv::Point2f left = worldToPixel(
    rx + half_w_m * std::cos(ryaw + kArrowBackSweepRad),
    ry + half_w_m * std::sin(ryaw + kArrowBackSweepRad));
  cv::Point2f right = worldToPixel(
    rx + half_w_m * std::cos(ryaw - kArrowBackSweepRad),
    ry + half_w_m * std::sin(ryaw - kArrowBackSweepRad));

  std::vector<cv::Point> tri = {cv::Point(tip), cv::Point(left), cv::Point(right)};
  cv::fillConvexPoly(img, tri, cv::Scalar(0, 60, 255));  // orange-red, stands out on grayscale grid
  return img;
}

}  // namespace visualizer_qt
