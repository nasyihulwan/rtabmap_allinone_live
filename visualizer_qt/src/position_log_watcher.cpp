#include "visualizer_qt/position_log_watcher.h"

#include <cstdlib>
#include <filesystem>
#include <sstream>

namespace fs = std::filesystem;

namespace visualizer_qt
{

namespace
{

// semantic_map_exporter.py writes via csv.DictWriter (QUOTE_MINIMAL) and
// every field it logs is numeric or a plain class name — none contain
// commas — so a plain split is safe without a full RFC4180 parser.
std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

std::string positionLogsDir()
{
  const char * home = std::getenv("HOME");
  return std::string(home != nullptr ? home : "") + "/.ros/position_logs";
}

constexpr auto kPollInterval = std::chrono::milliseconds(500);
constexpr size_t kMaxRows = 500;

}  // namespace

void PositionLogWatcher::update()
{
  auto now = std::chrono::steady_clock::now();
  if (now - last_poll_ >= kPollInterval) {
    last_poll_ = now;
    pollForNewestFile();
  }
  readNewLines();
}

void PositionLogWatcher::pollForNewestFile()
{
  std::error_code ec;
  fs::path dir(positionLogsDir());
  if (!fs::exists(dir, ec)) {
    return;
  }

  fs::path newest_path;
  fs::file_time_type newest_time{};
  for (const auto & entry : fs::directory_iterator(dir, ec)) {
    if (ec || !entry.is_regular_file()) {
      continue;
    }
    const std::string name = entry.path().filename().string();
    if (name.rfind("det_", 0) != 0 || entry.path().extension() != ".csv") {
      continue;
    }
    if (entry.file_size(ec) == 0) {
      continue;  // writer just created it, header not flushed yet
    }
    auto mtime = entry.last_write_time(ec);
    if (ec) {
      continue;
    }
    if (newest_path.empty() || mtime > newest_time) {
      newest_path = entry.path();
      newest_time = mtime;
    }
  }

  if (!newest_path.empty() && newest_path.string() != current_path_) {
    switchToFile(newest_path.string());
  }
}

void PositionLogWatcher::switchToFile(const std::string & path)
{
  file_.close();
  file_.open(path);
  rows_.clear();
  current_path_.clear();
  read_offset_ = 0;
  idx_frame_seq_ = idx_class_name_ = idx_tracker_id_ = idx_x_ = idx_y_ = idx_confidence_ = -1;

  if (!file_.is_open()) {
    return;
  }

  std::string header_line;
  if (!std::getline(file_, header_line)) {
    file_.close();
    return;
  }

  std::vector<std::string> columns = splitCsvLine(header_line);
  for (size_t i = 0; i < columns.size(); ++i) {
    const std::string & col = columns[i];
    if (col == "frame_seq") {idx_frame_seq_ = static_cast<int>(i);}
    else if (col == "class_name") {idx_class_name_ = static_cast<int>(i);}
    else if (col == "tracker_id") {idx_tracker_id_ = static_cast<int>(i);}
    else if (col == "x") {idx_x_ = static_cast<int>(i);}
    else if (col == "y") {idx_y_ = static_cast<int>(i);}
    else if (col == "confidence") {idx_confidence_ = static_cast<int>(i);}
  }

  current_path_ = path;
  read_offset_ = file_.tellg();
  readNewLines();
}

void PositionLogWatcher::readNewLines()
{
  if (!file_.is_open()) {
    return;
  }

  file_.clear();
  file_.seekg(read_offset_);
  std::ostringstream buf;
  buf << file_.rdbuf();
  const std::string chunk = buf.str();
  if (chunk.empty()) {
    return;
  }

  size_t pos = 0;
  size_t consumed = 0;
  while (true) {
    size_t nl = chunk.find('\n', pos);
    if (nl == std::string::npos) {
      break;  // incomplete trailing line — re-read it on the next poll
    }
    parseDataLine(chunk.substr(pos, nl - pos));
    pos = nl + 1;
    consumed = pos;
  }
  read_offset_ += static_cast<std::streamoff>(consumed);
}

void PositionLogWatcher::parseDataLine(const std::string & line)
{
  if (line.empty()) {
    return;
  }
  std::vector<std::string> fields = splitCsvLine(line);
  auto field = [&fields](int idx) -> std::string {
      return (idx >= 0 && static_cast<size_t>(idx) < fields.size()) ? fields[idx] : "";
    };

  PositionLogRow row;
  row.frame_seq = field(idx_frame_seq_);
  row.class_name = field(idx_class_name_);
  row.tracker_id = field(idx_tracker_id_);
  row.x = field(idx_x_);
  row.y = field(idx_y_);
  row.confidence = field(idx_confidence_);
  rows_.push_back(std::move(row));

  if (rows_.size() > kMaxRows) {
    rows_.erase(rows_.begin(), rows_.begin() + (rows_.size() - kMaxRows));
  }
}

}  // namespace visualizer_qt
