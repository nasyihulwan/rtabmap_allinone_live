#pragma once

#include <cstdint>
#include <deque>
#include <string>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>

namespace visualizer_qt
{

struct RosoutEntry
{
  rclcpp::Time stamp;
  uint8_t level = 0;
  std::string node_name;
  std::string message;
};

// Tails /rosout for the PRD §5.5b log panel — pipeline-wide output, not just
// this app's own logs. Capped ring buffer; oldest entries drop silently.
// Callback and reader both run on the same thread (main's spin_some/render
// loop), so no locking is needed here.
class RosoutLog
{
public:
  explicit RosoutLog(const rclcpp::Node::SharedPtr & node);

  const std::deque<RosoutEntry> & entries() const {return entries_;}

private:
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr sub_;
  std::deque<RosoutEntry> entries_;
  static constexpr size_t kMaxEntries = 2000;
};

}  // namespace visualizer_qt
