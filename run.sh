#!/bin/bash
# ╔══════════════════════════════════════════════════════════════════╗
# ║  run.sh — RTABMap Live Mode (AGX Orin)                         ║
# ║                                                                  ║
# ║  Usage:                                                          ║
# ║    ./run.sh              live mode, mapping + semantic           ║
# ║    ./run.sh output       live + capture visualisasi             ║
# ║    ./run.sh localize     live mode, localization                ║
# ║    ./run.sh diagnose     pre-check + tests, tanpa launch        ║
# ╚══════════════════════════════════════════════════════════════════╝

CYAN='\033[0;36m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m'; BLUE='\033[0;34m'; BOLD='\033[1m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Konfigurasi ───────────────────────────────────────────────────
ROS_WS="/home/kmp-orin/ros2_ws"
DB_PATH="/home/kmp-orin/.ros/rtabmap.db"
MODEL_ENGINE="$SCRIPT_DIR/models/best_brin_yolo11m_v1.engine"
MODEL_PT="$SCRIPT_DIR/models/best_brin_yolo11m_v1_20260517_0724.pt"
ODOM_TOPIC="/a200_1060/platform/odom/filtered"
HZ_SAMPLE_SECS=3

# ── Parse argumen ─────────────────────────────────────────────────
ARG="${1:-}"   # output | localize | diagnose | help | (kosong = mapping)

print_banner() {
    local label="$1"
    echo -e "${BLUE}${BOLD}"
    echo "  ╔══════════════════════════════════════════╗"
    echo "  ║   RTABMap Live Mode — AGX Orin           ║"
    echo "  ║   VLP-32C + D455 | $label"
    echo "  ╚══════════════════════════════════════════╝"
    echo -e "${NC}"
}

# ══════════════════════════════════════════════════════════════════
# PRE-CHECK
# ══════════════════════════════════════════════════════════════════
precheck() {
    local fail=0

    echo -e "${CYAN}${BOLD}=== Pre-check ===\n${NC}"

    # ROS workspace
    if [ ! -f "$ROS_WS/install/setup.bash" ]; then
        echo -e "  ${RED}✗ ROS workspace tidak ditemukan: $ROS_WS${NC}"
        echo -e "  ${YELLOW}  Sesuaikan ROS_WS di script${NC}"
        fail=1
    else
        echo -e "  ${GREEN}✓ ROS workspace: $ROS_WS${NC}"
    fi

    # Model YOLO
    if [ -f "$MODEL_ENGINE" ]; then
        MODEL_PATH="$MODEL_ENGINE"
        echo -e "  ${GREEN}✓ Model: $(basename $MODEL_ENGINE) (TRT engine)${NC}"
    elif [ -f "$MODEL_PT" ]; then
        MODEL_PATH="$MODEL_PT"
        echo -e "  ${YELLOW}⚠ Model: $(basename $MODEL_PT) (fallback .pt — lebih lambat)${NC}"
    else
        echo -e "  ${RED}✗ Model tidak ditemukan di $SCRIPT_DIR/models/${NC}"
        echo -e "  ${YELLOW}  Diharapkan: best_brin_yolo11m_v1.engine atau .pt${NC}"
        fail=1
    fi
    export MODEL_PATH

    # Semantic files
    local sem_ok=1
    for f in \
        "$SCRIPT_DIR/semantic/yolo_detector.py" \
        "$SCRIPT_DIR/semantic/semantic_projector.py" \
        "$SCRIPT_DIR/semantic/semantic_grid_node.py" \
        "$SCRIPT_DIR/decompress_depth_fix.py"; do
        [ ! -f "$f" ] && \
            echo -e "  ${RED}✗ $(basename $f) tidak ditemukan${NC}" && \
            sem_ok=0 && fail=1
    done
    [ "$sem_ok" = "1" ] && echo -e "  ${GREEN}✓ Semantic files OK${NC}"

    # ROS packages
    source /opt/ros/humble/setup.bash 2>/dev/null
    [ -f "$ROS_WS/install/setup.bash" ] && source "$ROS_WS/install/setup.bash" 2>/dev/null
    local pkg_ok=1
    for pkg in rtabmap_slam rtabmap_odom rtabmap_viz topic_tools \
               robot_localization rviz2; do
        if ! ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
            echo -e "  ${RED}✗ ROS pkg: $pkg${NC}"
            pkg_ok=0; fail=1
        fi
    done
    [ "$pkg_ok" = "1" ] && echo -e "  ${GREEN}✓ ROS packages OK${NC}"

    # Sensor topics + Hz
    echo -e "\n  ${CYAN}Sensor topics (sampling ${HZ_SAMPLE_SECS}s)...${NC}"
    local topics_ok=1

    check_topic_hz() {
        local topic="$1"
        local min_hz="$2"
        local buf
        buf=$(timeout $((HZ_SAMPLE_SECS + 5)) \
              ros2 topic hz "$topic" --window 10 2>/dev/null || true)
        local avg
        avg=$(echo "$buf" | grep -oP 'average rate: \K[0-9.]+' | tail -1)
        if [ -z "$avg" ]; then
            echo -e "    ${YELLOW}? $topic (tidak muncul)${NC}"
            topics_ok=0
            return
        fi
        local ok
        ok=$(echo "$avg $min_hz" | awk '{print ($1 >= $2) ? "1" : "0"}')
        if [ "$ok" = "1" ]; then
            echo -e "    ${GREEN}✓ $topic @ ${avg} Hz (target ≥ ${min_hz})${NC}"
        else
            echo -e "    ${RED}✗ $topic @ ${avg} Hz (target ≥ ${min_hz})${NC}"
            topics_ok=0
        fi
    }

    check_topic_hz "/velodyne_points"                       8
    check_topic_hz "/camera/camera/color/image_raw"         15
    check_topic_hz "/camera/camera/depth/image_rect_raw"    15
    if ros2 topic list 2>/dev/null | grep -q "^/a200_1060/platform/odom/filtered$"; then
        check_topic_hz "/a200_1060/platform/odom/filtered"  45
    else
        check_topic_hz "/odom"                              45
    fi
    [ "$topics_ok" = "0" ] && fail=1

    echo ""
    return $fail
}

# ══════════════════════════════════════════════════════════════════
# SPAWN TERMINAL
# ══════════════════════════════════════════════════════════════════
spawn_terminal() {
    local title="$1"
    local cmd="$2"
    if command -v gnome-terminal &>/dev/null; then
        gnome-terminal --title="$title" -- bash -c "$cmd; exec bash" &
    elif command -v xterm &>/dev/null; then
        xterm -title "$title" -e "bash -c '$cmd; exec bash'" &
    else
        echo -e "${RED}✗ gnome-terminal dan xterm tidak ditemukan${NC}"
        echo -e "${YELLOW}  Jalankan manual: $cmd${NC}"
    fi
}

# ══════════════════════════════════════════════════════════════════
# LAUNCH LIVE
# ══════════════════════════════════════════════════════════════════
launch_live() {
    local localize="$1"
    local capture="$2"

    local mode_label
    [ "$localize" = "true" ] && mode_label="LOCALIZATION" || mode_label="MAPPING"

    print_banner "$mode_label                  "
    echo -e "${CYAN}Mode    : $mode_label${NC}"
    echo -e "${CYAN}Odom    : $ODOM_TOPIC${NC}"
    echo -e "${CYAN}DB      : $DB_PATH${NC}"
    [ "$capture" = "true" ] && echo -e "${CYAN}Capture : aktif${NC}"
    echo ""

    if ! precheck; then
        echo -e "${RED}✗ Pre-check gagal. Perbaiki masalah di atas lalu ulangi.${NC}"
        exit 1
    fi

    # OS buffer untuk UDP LiDAR
    sudo sysctl -w net.core.rmem_max=33554432 2>/dev/null || true
    sudo sysctl -w net.core.rmem_default=33554432 2>/dev/null || true

    # Static TF di background
    echo -e "${CYAN}→ Static TF (background)...${NC}"
    source /opt/ros/humble/setup.bash 2>/dev/null
    [ -f "$ROS_WS/install/setup.bash" ] && source "$ROS_WS/install/setup.bash" 2>/dev/null
    ros2 launch "$SCRIPT_DIR/launch/live/static_tf_live.launch.py" &
    TF_PID=$!
    sleep 1

    # Terminal 1 — RTABMap
    echo -e "${CYAN}→ Terminal 1: RTABMap...${NC}"
    spawn_terminal "RTABMap Live ($mode_label)" \
        "source /opt/ros/humble/setup.bash; \
         [ -f '$ROS_WS/install/setup.bash' ] && source '$ROS_WS/install/setup.bash'; \
         ros2 launch '$SCRIPT_DIR/launch/live/rtabmap_live.launch.py' \
             db_path:='$DB_PATH' \
             odom_topic:='$ODOM_TOPIC' \
             localization:=$localize \
             use_ekf:=false \
             rviz:=true"
    sleep 4

    # Terminal 2 — Semantic nodes
    echo -e "${CYAN}→ Terminal 2: Semantic nodes...${NC}"
    spawn_terminal "Semantic Nodes" \
        "source /opt/ros/humble/setup.bash; \
         [ -f '$ROS_WS/install/setup.bash' ] && source '$ROS_WS/install/setup.bash'; \
         ros2 launch '$SCRIPT_DIR/launch/semantic_only.launch.py' \
             model_path:='$MODEL_PATH' \
             device:=cuda \
             confidence:=0.5"

    # Terminal 3 — Capture (opsional)
    if [ "$capture" = "true" ]; then
        echo -e "${CYAN}→ Terminal 3: Capture visualisasi...${NC}"
        sleep 8
        spawn_terminal "Capture" \
            "source /opt/ros/humble/setup.bash; \
             [ -f '$ROS_WS/install/setup.bash' ] && source '$ROS_WS/install/setup.bash'; \
             python3 '$SCRIPT_DIR/capture_semantic_map.py'"
    fi

    echo -e "\n${GREEN}✓ Semua node launched.${NC}"
    echo -e "${YELLOW}  Tekan Ctrl+C untuk stop static TF background.${NC}"
    wait $TF_PID
}

# ══════════════════════════════════════════════════════════════════
# DIAGNOSE
# ══════════════════════════════════════════════════════════════════
diagnose_all() {
    print_banner "Diagnose                  "
    source /opt/ros/humble/setup.bash 2>/dev/null
    [ -f "$ROS_WS/install/setup.bash" ] && source "$ROS_WS/install/setup.bash" 2>/dev/null

    precheck || true

    echo -e "${CYAN}${BOLD}=== Tests ===\n${NC}"

    # Launch static TF di background khusus untuk test_tf.py
    echo -e "${CYAN}→ Launch static TF untuk diagnose...${NC}"
    ros2 launch "$SCRIPT_DIR/launch/live/static_tf_live.launch.py" &
    TF_DIAG_PID=$!
    sleep 3

    for test_file in \
        "$SCRIPT_DIR/tests/test_cleanup.py" \
        "$SCRIPT_DIR/tests/test_semantic_nodes.py" \
        "$SCRIPT_DIR/tests/test_models.py" \
        "$SCRIPT_DIR/tests/test_topics.py" \
        "$SCRIPT_DIR/tests/test_tf.py"; do
        if [ -f "$test_file" ]; then
            echo -e "${CYAN}→ $(basename $test_file)${NC}"
            python3 "$test_file" && \
                echo -e "  ${GREEN}✓ PASS${NC}" || \
                echo -e "  ${RED}✗ FAIL${NC}"
            echo ""
        fi
    done

    # Kill static TF setelah semua test selesai
    kill $TF_DIAG_PID 2>/dev/null || true
}

# ══════════════════════════════════════════════════════════════════
# ENTRY POINT
# ══════════════════════════════════════════════════════════════════
case "$ARG" in
    ""|mapping)
        launch_live false false
        ;;
    output)
        launch_live false true
        ;;
    localize)
        launch_live true false
        ;;
    diagnose)
        diagnose_all
        ;;
    help|-h|--help)
        echo ""
        echo -e "${CYAN}Usage: ./run.sh [arg]${NC}"
        echo ""
        echo "  ./run.sh              live mode, mapping + semantic"
        echo "  ./run.sh output       live + capture visualisasi"
        echo "  ./run.sh localize     live mode, localization"
        echo "  ./run.sh diagnose     pre-check + tests, tanpa launch"
        echo ""
        ;;
    *)
        echo -e "${RED}✗ Argumen tidak dikenal: $ARG${NC}"
        echo "Usage: ./run.sh [output | localize | diagnose | help]"
        exit 1
        ;;
esac
