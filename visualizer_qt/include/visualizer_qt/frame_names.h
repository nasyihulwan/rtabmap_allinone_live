#pragma once

namespace visualizer_qt
{

// Mirrors ROBOT_BASE_FRAME/MAP_FRAME in launch/live/rtabmap_live.launch.py
// and the static TF tree in launch/live/static_tf_live.launch.py as of
// 2026-10-29. PRD §5.4 flags this as a moving target — re-check against the
// live launch files if TF-dependent checks/panels start failing.
constexpr const char * kMapFrame = "map";
constexpr const char * kBaseFrame = "a200_1060/base_link";  // rtabmap's actual base_link_frame
constexpr const char * kBaseAliasFrame = "base_link";  // bridge alias; hokuyo_bringup parents laser off this
constexpr const char * kCameraFrame = "camera_link";
constexpr const char * kLaserFrame = "laser_frame";

}  // namespace visualizer_qt
