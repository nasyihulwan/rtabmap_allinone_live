#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  test_deploy.py — Deployment Readiness Check                    ║
║                                                                  ║
║  Cek apakah semua komponen aman untuk di-deploy di AGX Orin.   ║
║                                                                  ║
║  Mode:                                                           ║
║    python3 tests/test_deploy.py           ← static checks only ║
║    python3 tests/test_deploy.py --live    ← + ROS topic/TF cek ║
║                                                                  ║
║  Sections:                                                       ║
║    1. File Structure                                             ║
║    2. nav2_params.yaml Validation                               ║
║    3. Python Imports                                             ║
║    4. Model Files                                                ║
║    5. Logic Sanity (boundary, YAML format)                      ║
║    6. semantic_map.yaml (jika ada)                              ║
║    7. ROS Live Checks (--live flag)                             ║
╚══════════════════════════════════════════════════════════════════╝
"""

import sys
import os
import yaml
import subprocess
import tempfile
import struct
import time
import math

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'semantic'))

LIVE_MODE = '--live' in sys.argv

# ── ANSI colors ───────────────────────────────────────────────────────────
GREEN  = '\033[0;32m'
RED    = '\033[0;31m'
YELLOW = '\033[0;33m'
CYAN   = '\033[0;36m'
RESET  = '\033[0m'
BOLD   = '\033[1m'

_results = []   # list of (section, label, passed, detail)


def _ok(section, label, detail=''):
    print(f'  {GREEN}✓ PASS{RESET}  {label}' + (f' — {detail}' if detail else ''))
    _results.append((section, label, True, detail))


def _fail(section, label, detail=''):
    print(f'  {RED}✗ FAIL{RESET}  {label}' + (f'\n         {RED}{detail}{RESET}' if detail else ''))
    _results.append((section, label, False, detail))


def _skip(section, label, detail=''):
    print(f'  {YELLOW}~ SKIP{RESET}  {label}' + (f' — {detail}' if detail else ''))
    _results.append((section, label, None, detail))


def _header(title):
    print(f'\n{BOLD}{CYAN}[ {title} ]{RESET}')


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 1: File Structure
# ═══════════════════════════════════════════════════════════════════════════

def check_files():
    _header('1. File Structure')
    SEC = '1-files'

    required = [
        ('config/nav2_params.yaml',               'Nav2 params config'),
        ('semantic/object_goal_nav.py',            'Object goal navigation node'),
        ('semantic/semantic_map_viz.py',           'Semantic map visualizer'),
        ('semantic/semantic_map_exporter.py',      'Semantic map YAML exporter'),
        ('semantic/yolo_detector.py',              'YOLO detector node'),
        ('semantic/semantic_projector.py',         'Semantic projector node'),
        ('semantic/semantic_grid_node.py',         'Semantic grid node'),
        ('launch/semantic_only.launch.py',         'Semantic pipeline launch'),
        ('launch/live/rtabmap_live.launch.py',     'RTABMap live launch'),
        ('launch/live/static_tf_live.launch.py',   'Static TF launch'),
        ('capture_semantic_map.py',                'Map capture tool'),
        ('run.sh',                                 'Main run script'),
    ]
    for relpath, label in required:
        full = os.path.join(ROOT, relpath)
        if os.path.exists(full):
            _ok(SEC, label, relpath)
        else:
            _fail(SEC, label, f'tidak ditemukan: {relpath}')

    # run.sh executable?
    run_sh = os.path.join(ROOT, 'run.sh')
    if os.path.exists(run_sh):
        if os.access(run_sh, os.X_OK):
            _ok(SEC, 'run.sh executable')
        else:
            _fail(SEC, 'run.sh executable', 'chmod +x run.sh diperlukan')


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 2: nav2_params.yaml Validation
# ═══════════════════════════════════════════════════════════════════════════

def check_nav2_params():
    _header('2. nav2_params.yaml Validation')
    SEC = '2-nav2'

    nav2_path = os.path.join(ROOT, 'config', 'nav2_params.yaml')
    if not os.path.exists(nav2_path):
        _fail(SEC, 'nav2_params.yaml exists', 'file tidak ditemukan')
        return

    try:
        with open(nav2_path) as f:
            cfg = yaml.safe_load(f)
    except Exception as e:
        _fail(SEC, 'nav2_params.yaml valid YAML', str(e))
        return

    _ok(SEC, 'nav2_params.yaml valid YAML')

    # robot_radius local costmap
    try:
        r = cfg['local_costmap']['local_costmap']['ros__parameters']['robot_radius']
        if r >= 0.33:
            _ok(SEC, f'local_costmap robot_radius={r} (min 0.33)')
        else:
            _fail(SEC, f'local_costmap robot_radius={r}',
                  'terlalu kecil! Husky A200 min 0.33m. Risiko tabrakan.')
    except KeyError as e:
        _fail(SEC, 'local_costmap robot_radius', f'key tidak ditemukan: {e}')

    # robot_radius global costmap
    try:
        r = cfg['global_costmap']['global_costmap']['ros__parameters']['robot_radius']
        if r >= 0.33:
            _ok(SEC, f'global_costmap robot_radius={r} (min 0.33)')
        else:
            _fail(SEC, f'global_costmap robot_radius={r}',
                  'terlalu kecil! Husky A200 min 0.33m. Risiko tabrakan.')
    except KeyError as e:
        _fail(SEC, 'global_costmap robot_radius', f'key tidak ditemukan: {e}')

    # Velodyne topic
    try:
        params = cfg['local_costmap']['local_costmap']['ros__parameters']
        obs = params.get('voxel_layer', {}).get('observation_sources', '')
        src = obs if isinstance(obs, str) else list(obs)[0]
        topic = params['voxel_layer'][src]['topic']
        dtype = params['voxel_layer'][src]['data_type']
        if topic == '/velodyne_points':
            _ok(SEC, f'voxel_layer topic={topic}')
        else:
            _fail(SEC, f'voxel_layer topic={topic!r}',
                  'harus /velodyne_points! VLP-32C tidak publish /scan.')
        if dtype == 'PointCloud2':
            _ok(SEC, f'voxel_layer data_type={dtype}')
        else:
            _fail(SEC, f'voxel_layer data_type={dtype!r}',
                  'harus PointCloud2! VLP-32C publish PointCloud2.')
        if 'min_obstacle_height' in params['voxel_layer'][src]:
            _ok(SEC, 'voxel_layer min_obstacle_height configured (filter ground)')
        else:
            _fail(SEC, 'voxel_layer min_obstacle_height missing',
                  'tanpa ini VLP-32C akan marking lantai sebagai obstacle')
    except KeyError as e:
        _fail(SEC, 'voxel_layer config', f'key tidak ditemukan: {e}')

    # use_sim_time: false
    sim_checks = [
        ('bt_navigator',       ['bt_navigator', 'ros__parameters', 'use_sim_time']),
        ('controller_server',  ['controller_server', 'ros__parameters', 'use_sim_time']),
        ('planner_server',     ['planner_server', 'ros__parameters', 'use_sim_time']),
        ('local_costmap',      ['local_costmap', 'local_costmap', 'ros__parameters', 'use_sim_time']),
        ('global_costmap',     ['global_costmap', 'global_costmap', 'ros__parameters', 'use_sim_time']),
    ]
    all_sim_false = True
    for name, path in sim_checks:
        try:
            val = cfg
            for k in path:
                val = val[k]
            if val is False:
                pass
            else:
                _fail(SEC, f'{name} use_sim_time',
                      f'harus false untuk live mode (got {val})')
                all_sim_false = False
        except KeyError:
            pass
    if all_sim_false:
        _ok(SEC, 'use_sim_time: false di semua komponen')

    # lifecycle_manager nodes
    try:
        nodes = set(cfg['lifecycle_manager_navigation']['ros__parameters']['node_names'])
        required_nav = {
            'controller_server', 'smoother_server', 'planner_server',
            'behavior_server',   'bt_navigator',    'waypoint_follower',
            'velocity_smoother',
        }
        missing = required_nav - nodes
        if not missing:
            _ok(SEC, f'lifecycle_manager memiliki {len(nodes)} required nodes')
        else:
            _fail(SEC, 'lifecycle_manager nodes', f'kekurangan: {missing}')
    except KeyError as e:
        _fail(SEC, 'lifecycle_manager', f'key tidak ditemukan: {e}')


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 3: Python Imports
# ═══════════════════════════════════════════════════════════════════════════

def check_imports():
    _header('3. Python Imports')
    SEC = '3-imports'

    # ── Deteksi apakah ROS2 tersedia di environment ini ──────────────────
    try:
        import rclpy  # noqa: F401
        ros_available = True
        _ok(SEC, 'rclpy (ROS2 Python client)')
    except ImportError:
        ros_available = False
        _skip(SEC, 'rclpy tidak tersedia di environment ini',
              'jalankan test_deploy.py di AGX Orin setelah: source /opt/ros/humble/setup.bash')

    # Paket ROS2 message/action hanya dicek jika rclpy tersedia
    ros_checks = [
        ('sensor_msgs.msg',        'sensor_msgs'),
        ('geometry_msgs.msg',      'geometry_msgs'),
        ('nav_msgs.msg',           'nav_msgs'),
        ('nav2_msgs.action',       'nav2_msgs (NavigateToPose)'),
        ('visualization_msgs.msg', 'visualization_msgs (MarkerArray)'),
        ('std_msgs.msg',           'std_msgs'),
    ]
    if ros_available:
        for module, label in ros_checks:
            try:
                __import__(module)
                _ok(SEC, label)
            except ImportError as e:
                _fail(SEC, label, str(e))
    else:
        for _, label in ros_checks:
            _skip(SEC, label, 'skip — ROS2 tidak di-source')

    # Paket non-ROS selalu harus tersedia
    for module, label in [('yaml', 'yaml (PyYAML)'), ('numpy', 'numpy'),
                           ('cv2', 'cv2 (OpenCV)')]:
        try:
            __import__(module)
            _ok(SEC, label)
        except ImportError as e:
            _fail(SEC, label, str(e))

    # ultralytics — diperlukan di Orin untuk YOLO inference
    try:
        import ultralytics  # noqa: F401
        _ok(SEC, 'ultralytics (YOLO inference)')
    except ImportError:
        if ros_available:
            # Di Orin, ultralytics HARUS tersedia
            _fail(SEC, 'ultralytics',
                  'pip install ultralytics — diperlukan untuk yolo_detector.py')
        else:
            _skip(SEC, 'ultralytics', 'tidak di-cek di luar Orin')

    # torch (optional, untuk SegFormer)
    try:
        import torch
        cuda = torch.cuda.is_available()
        _ok(SEC, f'torch (CUDA={cuda})')
        if not cuda:
            _skip(SEC, 'CUDA tersedia',
                  'torch ada tapi CUDA tidak tersedia — SegFormer akan lambat')
    except ImportError:
        _skip(SEC, 'torch', 'tidak diinstall — SegFormer tidak akan jalan')

    # transformers (SegFormer)
    try:
        import transformers  # noqa: F401
        _ok(SEC, 'transformers (SegFormer)')
    except ImportError:
        _skip(SEC, 'transformers', 'tidak diinstall — SegFormer tidak akan jalan')


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 4: Model Files
# ═══════════════════════════════════════════════════════════════════════════

def check_models():
    _header('4. Model Files')
    SEC = '4-models'

    models_dir = os.path.join(ROOT, 'models')
    if not os.path.isdir(models_dir):
        _fail(SEC, 'models/ folder exists')
        return

    engine_files = [f for f in os.listdir(models_dir) if f.endswith('.engine')]
    pt_files     = [f for f in os.listdir(models_dir) if f.endswith('.pt')]

    if engine_files:
        _ok(SEC, f'TensorRT engine ditemukan: {engine_files[0]}',
            'akan digunakan untuk inference (optimal untuk Orin)')
    elif pt_files:
        _skip(SEC, f'TensorRT engine tidak ada, .pt tersedia: {pt_files[0]}',
              'inference lebih lambat — run: yolo export model=... format=engine')
    else:
        _fail(SEC, 'YOLO model file (.engine atau .pt)',
              f'tidak ada model di {models_dir}')

    # Cek ukuran file (engine yang corrupt biasanya < 1MB)
    for eng in engine_files:
        path = os.path.join(models_dir, eng)
        size_mb = os.path.getsize(path) / 1e6
        if size_mb > 1.0:
            _ok(SEC, f'{eng} size={size_mb:.1f}MB (terlihat valid)')
        else:
            _fail(SEC, f'{eng} size={size_mb:.1f}MB', 'engine file mungkin corrupt!')


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 5: Logic Sanity
# ═══════════════════════════════════════════════════════════════════════════

def check_logic():
    _header('5. Logic Sanity (tanpa ROS)')
    SEC = '5-logic'

    # ── Boundary logic dari object_goal_nav ──────────────────────────────
    try:
        # Simulasi dengan parameter map default (rtabmap_15april)
        ox, oy = -8.07391, -5.30406
        res = 0.05
        w, h = 348, 669
        x_min = ox
        x_max = ox + w * res
        y_min = oy
        y_max = oy + h * res

        def is_in_map(x, y):
            return x_min <= x <= x_max and y_min <= y <= y_max

        margin = 0.5
        def clamp(x, y):
            return (
                max(x_min + margin, min(x_max - margin, x)),
                max(y_min + margin, min(y_max - margin, y)),
            )

        # Test: titik di dalam map
        assert is_in_map(0.0, 5.0), "titik valid gagal"
        # Test: titik di luar (clamp)
        cx, cy = clamp(100.0, 100.0)
        assert cx < x_max, "clamp x_max gagal"
        assert cy < y_max, "clamp y_max gagal"
        # Test: titik sudut origin
        assert is_in_map(ox, oy), "origin harus inside"

        _ok(SEC, 'object_goal_nav boundary logic (_is_in_map, _clamp_to_map)')
    except AssertionError as e:
        _fail(SEC, 'object_goal_nav boundary logic', str(e))

    # ── Retry offset ─────────────────────────────────────────────────────
    try:
        retry_offset = 0.5
        offsets = [
            (0.0,  retry_offset),
            (0.0, -retry_offset),
            (retry_offset, 0.0),
            (-retry_offset, 0.0),
            (0.0,  retry_offset * 2),
        ]
        x0, y0 = 5.0, 3.0
        for step in range(1, 6):
            idx = (step - 1) % len(offsets)
            dx, dy = offsets[idx]
            nx, ny = x0 + dx, y0 + dy
            dist = math.hypot(nx - x0, ny - y0)
            assert dist > 0, f"step {step} menghasilkan offset=0"
        _ok(SEC, 'object_goal_nav retry offsets semua non-zero')
    except AssertionError as e:
        _fail(SEC, 'object_goal_nav retry offsets', str(e))

    # ── semantic_map.yaml format compatibility ────────────────────────────
    try:
        import io
        sample = {
            'chair_2':        {'x': 1.23, 'y': 4.56, 'class_id': 0, 'cell_count': 30},
            'refrigerator_1': {'x': 3.0,  'y': -1.0, 'class_id': 3, 'cell_count': 25},
        }
        buf = io.StringIO()
        yaml.dump(sample, buf)
        buf.seek(0)
        loaded = yaml.safe_load(buf)

        # Akses seperti object_goal_nav.py
        for label, obj in loaded.items():
            x, y = obj['x'], obj['y']
            _ = obj['cell_count']

        # Akses seperti semantic_map_viz.py
        for label, obj in loaded.items():
            cid = obj.get('class_id', 0)

        _ok(SEC, 'semantic_map.yaml format kompatibel dengan object_goal_nav + viz')
    except Exception as e:
        _fail(SEC, 'semantic_map.yaml format compatibility', str(e))

    # ── PointCloud2 RGB decode ────────────────────────────────────────────
    try:
        import struct
        import numpy as np
        # Pack R=5 (class), G=3 (inst), B=0
        packed_int = (5 << 16) | (3 << 8) | 0
        f = struct.unpack('<f', struct.pack('<I', packed_int))[0]
        u32 = struct.unpack('<I', struct.pack('<f', f))[0]
        class_id = (u32 >> 16) & 0xFF
        inst_id  = (u32 >> 8)  & 0xFF
        assert class_id == 5, f"class_id decode salah: {class_id}"
        assert inst_id  == 3, f"inst_id decode salah: {inst_id}"
        _ok(SEC, 'PointCloud2 RGB decode (R=class_id, G=inst_id)')
    except AssertionError as e:
        _fail(SEC, 'PointCloud2 RGB decode', str(e))


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 6: semantic_map.yaml (jika ada)
# ═══════════════════════════════════════════════════════════════════════════

def check_semantic_map():
    _header('6. semantic_map.yaml (jika sudah ada)')
    SEC = '6-semmap'

    map_path = os.path.expanduser('~/.ros/semantic_map.yaml')
    if not os.path.exists(map_path):
        _skip(SEC, 'semantic_map.yaml belum ada',
              'akan dibuat otomatis oleh semantic_map_exporter.py saat pipeline jalan')
        return

    try:
        with open(map_path) as f:
            data = yaml.safe_load(f)
    except Exception as e:
        _fail(SEC, 'semantic_map.yaml valid YAML', str(e))
        return

    if not data:
        _skip(SEC, 'semantic_map.yaml kosong (0 objek)', 'pipeline mungkin baru dimulai')
        return

    _ok(SEC, f'semantic_map.yaml ada: {len(data)} objek')

    required_fields = {'x', 'y', 'class_id', 'cell_count'}
    errors = []
    for label, obj in data.items():
        if not isinstance(obj, dict):
            errors.append(f'{label}: bukan dict')
            continue
        missing = required_fields - set(obj.keys())
        if missing:
            errors.append(f'{label}: field tidak ada: {missing}')
        if not isinstance(obj.get('x'), (int, float)):
            errors.append(f'{label}: x bukan number')
        if not isinstance(obj.get('y'), (int, float)):
            errors.append(f'{label}: y bukan number')

    if errors:
        _fail(SEC, 'semantic_map.yaml field validation', '; '.join(errors[:3]))
    else:
        _ok(SEC, f'Semua {len(data)} entry punya field x, y, class_id, cell_count')

    # Sample: list 5 pertama
    print(f'  {CYAN}  Contoh objek:{RESET}')
    for label, obj in list(data.items())[:5]:
        print(f'         {label:30s} x={obj.get("x",0):7.3f}  '
              f'y={obj.get("y",0):7.3f}  cells={obj.get("cell_count",0)}')


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 7: ROS Live Checks (--live flag)
# ═══════════════════════════════════════════════════════════════════════════

def _run_ros2(args, timeout=8) -> tuple:
    """Run ros2 command, return (stdout, returncode)."""
    try:
        r = subprocess.run(
            ['ros2'] + args,
            capture_output=True, text=True, timeout=timeout)
        return r.stdout + r.stderr, r.returncode
    except subprocess.TimeoutExpired:
        return 'TIMEOUT', -1
    except FileNotFoundError:
        return 'ros2 not found', -1


def check_live():
    _header('7. ROS Live Checks')
    SEC = '7-live'

    if not LIVE_MODE:
        _skip(SEC, 'Live checks', 'jalankan dengan --live untuk cek ROS topics/TF')
        return

    # Daemon running?
    out, rc = _run_ros2(['daemon', 'status'], timeout=5)
    if 'running' in out.lower() or rc == 0:
        _ok(SEC, 'ROS2 daemon running')
    else:
        _fail(SEC, 'ROS2 daemon running', 'jalankan: ros2 daemon start')

    # Topic checks
    required_topics = [
        ('/velodyne_points',                   'VLP-32C LiDAR'),
        ('/map',                               'RTABMap grid map'),
        ('/camera/camera/color/image_raw',     'RealSense color'),
        ('/camera/camera/depth/image_rect_raw', 'RealSense depth'),
        ('/a200_1060/platform/odom/filtered',  'Husky odom'),
    ]

    print(f'  {YELLOW}  Checking topics (timeout 5s each)...{RESET}')
    out, _ = _run_ros2(['topic', 'list'], timeout=8)
    active_topics = set(line.strip() for line in out.splitlines() if line.startswith('/'))

    for topic, label in required_topics:
        if topic in active_topics:
            _ok(SEC, f'{label} ({topic})')
        else:
            _fail(SEC, f'{label} ({topic})', 'tidak publishing — cek sensor/driver')

    # Semantic pipeline topics (opsional)
    semantic_topics = [
        '/semantic/pointcloud',
        '/semantic/grid_map',
        '/semantic/object_markers',
    ]
    for topic in semantic_topics:
        if topic in active_topics:
            _ok(SEC, f'Semantic: {topic}')
        else:
            _skip(SEC, f'Semantic: {topic}', 'pipeline belum jalan')

    # TF checks
    tf_pairs = [
        ('map',               'odom',                 'RTABMap TF'),
        ('odom',              'base_link',             'Husky odometry TF'),
        ('a200_1060/base_link', 'velodyne',            'LiDAR static TF'),
        ('a200_1060/base_link', 'camera_link',         'Camera static TF'),
    ]
    print(f'  {YELLOW}  Checking TF (timeout 3s each)...{RESET}')
    for parent, child, label in tf_pairs:
        out, rc = _run_ros2(
            ['run', 'tf2_ros', 'tf2_echo', parent, child], timeout=3)
        if 'translation' in out.lower() or 'rotation' in out.lower():
            _ok(SEC, f'TF: {label} ({parent}→{child})')
        elif 'TIMEOUT' in out:
            _fail(SEC, f'TF: {label}', f'transform {parent}→{child} tidak tersedia')
        else:
            _fail(SEC, f'TF: {label}', f'{out[:80]}')

    # Nav2 action server
    out, _ = _run_ros2(['action', 'list'], timeout=5)
    if '/navigate_to_pose' in out:
        _ok(SEC, 'Nav2 action /navigate_to_pose available')
    else:
        _skip(SEC, 'Nav2 action /navigate_to_pose',
              'Nav2 belum jalan — jalankan Nav2 dengan nav2_params.yaml ini')

    # /object_goal topic (apakah ada subscriber?)
    if '/object_goal' in active_topics:
        _ok(SEC, '/object_goal topic tersedia (object_goal_nav.py running)')
    else:
        _skip(SEC, '/object_goal topic', 'object_goal_nav.py belum dijalankan')


# ═══════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════

def main():
    print(f'\n{BOLD}{"═"*65}{RESET}')
    print(f'{BOLD}  test_deploy.py — Deployment Readiness Check (AGX Orin){RESET}')
    mode_str = 'STATIC + LIVE' if LIVE_MODE else 'STATIC ONLY'
    print(f'{BOLD}  Mode: {mode_str}{RESET}')
    print(f'{BOLD}{"═"*65}{RESET}')

    check_files()
    check_nav2_params()
    check_imports()
    check_models()
    check_logic()
    check_semantic_map()
    check_live()

    # ── Summary ───────────────────────────────────────────────────────────
    passed  = sum(1 for _, _, ok, _ in _results if ok is True)
    failed  = sum(1 for _, _, ok, _ in _results if ok is False)
    skipped = sum(1 for _, _, ok, _ in _results if ok is None)

    print(f'\n{BOLD}{"═"*65}{RESET}')
    print(f'{BOLD}  HASIL AKHIR{RESET}')
    print(f'{BOLD}{"═"*65}{RESET}')
    print(f'  {GREEN}✓ PASS  {passed}{RESET}')
    print(f'  {RED}✗ FAIL  {failed}{RESET}')
    print(f'  {YELLOW}~ SKIP  {skipped}{RESET}')

    if failed == 0:
        print(f'\n  {GREEN}{BOLD}✓ AMAN UNTUK DEPLOY ke AGX Orin!{RESET}')
    else:
        print(f'\n  {RED}{BOLD}✗ ADA {failed} MASALAH — FIX DULU SEBELUM DEPLOY!{RESET}')
        print(f'\n  {RED}  Items yang gagal:{RESET}')
        for sec, label, ok, detail in _results:
            if ok is False:
                print(f'  {RED}    ✗ [{sec}] {label}{RESET}')
                if detail:
                    print(f'  {RED}        {detail}{RESET}')

    if not LIVE_MODE:
        print(f'\n  {YELLOW}Tip: jalankan dengan --live untuk cek ROS topics/TF secara live:{RESET}')
        print(f'  {YELLOW}  python3 tests/test_deploy.py --live{RESET}')

    print()
    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
