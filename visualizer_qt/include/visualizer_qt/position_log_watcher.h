#pragma once

#include <chrono>
#include <fstream>
#include <string>
#include <vector>

namespace visualizer_qt
{

// One row of the live table (PRD §5.3 — a display subset of
// POSITION_LOG_FIELDS from semantic_map_exporter.py).
struct PositionLogRow
{
  std::string frame_seq;
  std::string class_name;
  std::string tracker_id;
  std::string x;
  std::string y;
  std::string confidence;
};

// Watches ~/.ros/position_logs/ for the newest det_<session_id>.csv and
// tails it as semantic_map_exporter.py appends rows. The session_id isn't
// known at app startup (PRD §5.3), so this always follows whichever file
// was most recently written, switching over if a new session starts.
class PositionLogWatcher
{
public:
  void update();

  const std::vector<PositionLogRow> & rows() const {return rows_;}
  bool hasActiveFile() const {return file_.is_open();}
  const std::string & currentPath() const {return current_path_;}

private:
  void pollForNewestFile();
  void switchToFile(const std::string & path);
  void readNewLines();
  void parseDataLine(const std::string & line);

  std::string current_path_;
  std::ifstream file_;
  std::streamoff read_offset_ = 0;
  std::chrono::steady_clock::time_point last_poll_{};

  int idx_frame_seq_ = -1;
  int idx_class_name_ = -1;
  int idx_tracker_id_ = -1;
  int idx_x_ = -1;
  int idx_y_ = -1;
  int idx_confidence_ = -1;

  std::vector<PositionLogRow> rows_;
};

}  // namespace visualizer_qt
