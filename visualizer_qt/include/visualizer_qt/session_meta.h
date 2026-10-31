#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace visualizer_qt
{

struct SessionMeta
{
  std::string session_id;
  std::string phase;
  std::optional<bool> relocalization_ok;
  std::vector<std::string> objects_present;
};

// Watches ~/.ros/session_meta/ for the newest meta_<session_id>.yaml (PRD
// §5.6) — same "watch for the newest file" pattern as PositionLogWatcher,
// since this shares the same session_id as the position-log CSV and only
// exists when enable_position_logger=true for the running session.
// semantic_map_exporter.py rewrites the same file in place at session end
// (end=true), so the newest file is reloaded every poll tick, not just once.
class SessionMetaWatcher
{
public:
  void update();
  bool hasActiveSession() const {return has_meta_;}
  const SessionMeta & meta() const {return meta_;}

private:
  void pollForNewestFile();
  void loadFile(const std::string & path);

  bool has_meta_ = false;
  SessionMeta meta_;
  std::chrono::steady_clock::time_point last_poll_{};
};

}  // namespace visualizer_qt
