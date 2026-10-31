#!/bin/bash
# Launcher used by visualizer_qt.desktop (and callable directly from a terminal).
# Resolves the workspace root from this script's own location so the same
# file works unchanged after the repo is cloned onto a different machine
# at a different path. repo_root is passed so the Start-sensors/
# Start-pipeline buttons (PRD §5.9) can find start_sensors.sh/run.sh.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# Distro differs per machine (humble on the Orin, jazzy on the dev box).
# A .desktop launch has no shell profile, so ROS_DISTRO isn't set either.
for distro in "${ROS_DISTRO:-}" jazzy humble; do
  if [ -n "${distro}" ] && [ -f "/opt/ros/${distro}/setup.bash" ]; then
    source "/opt/ros/${distro}/setup.bash"
    break
  fi
done

# Dev box builds into ~/ros2_ws; the Orin builds into the repo's own install/.
if [ -f "${HOME}/ros2_ws/install/visualizer_qt/share/visualizer_qt/package.xml" ]; then
  source "${HOME}/ros2_ws/install/setup.bash"
else
  source "${WORKSPACE_ROOT}/install/setup.bash"
fi

exec ros2 launch visualizer_qt visualizer_qt.launch.py repo_root:="${WORKSPACE_ROOT}" "$@"
