#include "visualizer_qt/session_meta.h"

#include <cstdlib>
#include <filesystem>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace visualizer_qt
{

namespace
{

constexpr auto kPollInterval = std::chrono::milliseconds(1000);

std::string sessionMetaDir()
{
  const char * home = std::getenv("HOME");
  return std::string(home != nullptr ? home : "") + "/.ros/session_meta";
}

}  // namespace

void SessionMetaWatcher::update()
{
  auto now = std::chrono::steady_clock::now();
  if (now - last_poll_ < kPollInterval) {
    return;
  }
  last_poll_ = now;
  pollForNewestFile();
}

void SessionMetaWatcher::pollForNewestFile()
{
  std::error_code ec;
  fs::path dir(sessionMetaDir());
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
    if (name.rfind("meta_", 0) != 0 || entry.path().extension() != ".yaml") {
      continue;
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

  if (!newest_path.empty()) {
    loadFile(newest_path.string());
  }
}

void SessionMetaWatcher::loadFile(const std::string & path)
{
  try {
    YAML::Node root = YAML::LoadFile(path);
    SessionMeta m;
    m.session_id = root["session_id"] ? root["session_id"].as<std::string>() : "";
    m.phase = (root["phase"] && !root["phase"].IsNull()) ? root["phase"].as<std::string>() : "";
    if (root["relocalization_ok"] && !root["relocalization_ok"].IsNull()) {
      m.relocalization_ok = root["relocalization_ok"].as<bool>();
    }
    if (root["objects_present"] && root["objects_present"].IsSequence()) {
      for (const auto & item : root["objects_present"]) {
        m.objects_present.push_back(item.as<std::string>());
      }
    }
    meta_ = std::move(m);
    has_meta_ = true;
  } catch (const YAML::Exception &) {
    // semantic_map_exporter.py doesn't write this file atomically — a
    // mid-write read can land here. Just retry on the next poll tick.
  }
}

}  // namespace visualizer_qt
