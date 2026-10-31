#include "visualizer_qt/rosout_log.h"

namespace visualizer_qt
{

RosoutLog::RosoutLog(const rclcpp::Node::SharedPtr & node)
{
  sub_ = node->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", rclcpp::QoS(1000),
    [this](rcl_interfaces::msg::Log::ConstSharedPtr msg) {
      entries_.push_back(
        RosoutEntry{rclcpp::Time(msg->stamp), msg->level, msg->name, msg->msg});
      if (entries_.size() > kMaxEntries) {
        entries_.pop_front();
      }
    });
}

}  // namespace visualizer_qt
