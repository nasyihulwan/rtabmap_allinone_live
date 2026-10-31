#!/bin/bash
# Installs a user-level .desktop launcher (~/.local/share/applications) so
# the Qt visualizer shows up in any XDG-compliant app launcher. No sudo.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VISUALIZER_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

DEST_DIR="${HOME}/.local/share/applications"
mkdir -p "${DEST_DIR}"

sed "s|__RUN_SCRIPT__|${SCRIPT_DIR}/run_visualizer_qt.sh|" \
  "${VISUALIZER_DIR}/visualizer_qt.desktop.in" > "${DEST_DIR}/visualizer_qt.desktop"

chmod +x "${DEST_DIR}/visualizer_qt.desktop"

if command -v update-desktop-database >/dev/null; then
  update-desktop-database "${DEST_DIR}"
fi

echo "Installed: ${DEST_DIR}/visualizer_qt.desktop"
