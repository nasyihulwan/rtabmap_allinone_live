#!/usr/bin/env python3

# ═══════════════════════════════════════════════════════════════════════
# semantic_only.launch.py
#
# Semantic pipeline nodes (Terminal 2).
# Script Python dijalankan langsung via ExecuteProcess.
#
# PENTING: Pastikan topik berikut sudah tersedia sebelum launch ini
# (disediakan oleh RTABMap launch + RealSense driver di live mode):
#
# Topik yang HARUS sudah tersedia:
#   /camera/camera/color/image_raw                        ← RealSense driver
#   /camera/camera/aligned_depth_to_color/image_raw      ← RealSense driver
#   /camera/camera/color/camera_info                     ← RealSense driver
#   /map                                                  ← RTABMap
#
# Parameter Map-level IoU (semantic_grid_node):
#   map_iou_threshold  — minimum IoU untuk dianggap objek sama (default: 0.1)
#   min_cells_to_check — minimum cell sebelum instance dievaluasi (default: 20)
# ═══════════════════════════════════════════════════════════════════════

import datetime
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    script_dir   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    semantic_dir = os.path.join(script_dir, 'semantic')
    model_default = os.path.join(script_dir, 'models', 'best_brin_yolo11m_v1.engine')

    segformer_default = os.path.join(script_dir, 'models', 'segformer_b0_ade')

    # [J1] Dibuat SEKALI di sini (bukan di dalam node) — string Python biasa,
    # dihitung saat generate_launch_description() dipanggil (waktu launch
    # sesungguhnya, bukan waktu import). Dipakai identik untuk session_id
    # exporter DAN nama folder rosbag, supaya keduanya bisa dikorelasikan
    # lintas file tanpa harus menebak-nebak timestamp mana yang cocok.
    default_session_id = datetime.datetime.now().strftime('S%Y%m%d_%H%M')

    # ── Argumen YOLO ──────────────────────────────────────────────────────
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value=model_default)
    segformer_model_arg = DeclareLaunchArgument(
        'segformer_model', default_value=segformer_default,
        description='Path ke direktori model SegFormer (local) atau HuggingFace model ID')
    device_arg = DeclareLaunchArgument('device', default_value='0')
    confidence_arg = DeclareLaunchArgument('confidence', default_value='0.15')
    # [D3] Sebelumnya implisit (default Ultralytics: iou=0.7, imgsz=640),
    # sekarang eksplisit + direkam ke metadata sesi (exporter, untuk meta yaml).
    iou_nms_arg = DeclareLaunchArgument('iou_nms', default_value='0.7',
        description='NMS IoU threshold YOLO inference (D3 — harus konsisten antar sesi)')
    imgsz_arg = DeclareLaunchArgument('imgsz', default_value='640',
        description='Ukuran input YOLO inference (D3)')

    # ── Argumen Map-level IoU ─────────────────────────────────────────────
    iou_threshold_arg = DeclareLaunchArgument(
        'map_iou_threshold',
        default_value='0.1',
        description='Minimum IoU antar footprint cell untuk dianggap objek sama')
    min_cells_arg = DeclareLaunchArgument(
        'min_cells_to_check',
        default_value='20',
        description='Minimum cell yang harus terakumulasi sebelum instance dievaluasi')

    # ── Argumen fusi LiDAR (semantic_projector) ───────────────────────────
    depth_source_arg = DeclareLaunchArgument(
        'depth_source', default_value='camera',
        description="'camera' (default, tidak berubah) atau 'lidar' — Z per-piksel "
                    'dari range Velodyne yang diproyeksikan, fallback ke depth kamera')
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic', default_value='/velodyne_points')
    lidar_dilate_px_arg = DeclareLaunchArgument(
        'lidar_dilate_px', default_value='5',
        description='Ukuran kernel min-filter untuk menyebarkan titik LiDAR yang jarang')

    model_path       = LaunchConfiguration('model_path')
    device           = LaunchConfiguration('device')
    confidence       = LaunchConfiguration('confidence')
    iou_nms          = LaunchConfiguration('iou_nms')
    imgsz            = LaunchConfiguration('imgsz')
    iou_threshold    = LaunchConfiguration('map_iou_threshold')
    min_cells        = LaunchConfiguration('min_cells_to_check')
    segformer_model  = LaunchConfiguration('segformer_model')
    depth_source     = LaunchConfiguration('depth_source')
    lidar_topic      = LaunchConfiguration('lidar_topic')
    lidar_dilate_px  = LaunchConfiguration('lidar_dilate_px')

    # ── Node 1: YOLO Detector ─────────────────────────────────────────────
    yolo_detector = ExecuteProcess(
        cmd=[
            'python3', os.path.join(semantic_dir, 'yolo_detector.py'),
            '--ros-args',
            '-p', ['model_path:=', model_path],
            '-p', ["device:='", device, "'"],
            '-p', ['confidence:=', confidence],
            '-p', ['iou_nms:=', iou_nms],
            '-p', ['imgsz:=', imgsz],
            '-p', 'process_every_n:=1',
        ],
        output='screen',
    )

    # ── Node 2: Semantic Projector (delay 2s) ────────────────────────────
    semantic_projector = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3', os.path.join(semantic_dir, 'semantic_projector.py'),
                    '--ros-args',
                    '-p', ['depth_source:=', depth_source],
                    '-p', ['lidar_topic:=', lidar_topic],
                    '-p', ['lidar_dilate_px:=', lidar_dilate_px],
                ],
                output='screen',
            )
        ]
    )

    # ── Node 3: Semantic Grid (delay 3s) ─────────────────────────────────
    semantic_grid = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3', os.path.join(semantic_dir, 'semantic_grid_node.py'),
                    '--ros-args',
                    '-p', ['map_iou_threshold:=', iou_threshold],
                    '-p', ['min_cells_to_check:=', min_cells],
                    '-p', ['model_path:=', model_path],
                ],
                output='screen',
            )
        ]
    )

    # ── Node 4: SegFormer (delay 1s — YOLO must be up first) ─────────────
    segformer_node = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3', os.path.join(semantic_dir, 'segformer_node.py'),
                    '--ros-args',
                    '-p', ['model_name:=', segformer_model],
                    '-p', 'device:=cuda',
                    '-p', 'input_size:=512',
                    '-p', 'process_every_n:=1',
                ],
                output='screen',
            )
        ]
    )

    # ── Argumen Semantic Map Exporter ────────────────────────────────────
    model_path_export_arg = DeclareLaunchArgument(
        'export_model_path', default_value=model_default,
        description='YOLO model path untuk class names di semantic_map_exporter')
    export_model_path = LaunchConfiguration('export_model_path')
    # [clear] Kalau true, exporter mulai dari kosong, abaikan YAML lama.
    clear_on_start_arg = DeclareLaunchArgument(
        'clear_on_start', default_value='false',
        description='Kalau true, semantic_map_exporter abaikan YAML lama saat startup')
    clear_on_start = LaunchConfiguration('clear_on_start')
    # [B1] Matikan re-id sepenuhnya — baseline naif untuk eksperimen K1/K2.
    # Nama param persis 'enable_reid' (checklist B1).
    enable_reid_arg = DeclareLaunchArgument(
        'enable_reid', default_value='true',
        description='Kalau false, setiap deteksi selalu jadi instance baru (baseline naif)')
    enable_reid = LaunchConfiguration('enable_reid')
    # [logger] Opt-in position + reid-timing logger untuk sesi K1/K2.
    enable_position_logger_arg = DeclareLaunchArgument(
        'enable_position_logger', default_value='false',
        description='Kalau true, tulis CSV per deteksi per frame + biaya komputasi re-id')
    enable_position_logger = LaunchConfiguration('enable_position_logger')
    # [A4] Direktori, bukan path file — satu file per sesi dibuat otomatis
    # di dalamnya (nama mengandung session_id).
    position_log_dir_arg = DeclareLaunchArgument(
        'position_log_dir', default_value=os.path.expanduser('~/.ros/position_logs'),
        description='Direktori output CSV position logger (satu file per sesi)')
    position_log_dir = LaunchConfiguration('position_log_dir')
    reid_timing_log_dir_arg = DeclareLaunchArgument(
        'reid_timing_log_dir', default_value=os.path.expanduser('~/.ros/reid_timing_logs'),
        description='Direktori output CSV biaya komputasi re-id (satu file per sesi)')
    reid_timing_log_dir = LaunchConfiguration('reid_timing_log_dir')
    min_free_disk_gb_arg = DeclareLaunchArgument(
        'min_free_disk_gb', default_value='2.0',
        description='[checklist I] Ambang peringatan disk penuh saat sesi eksperimen mulai')
    min_free_disk_gb = LaunchConfiguration('min_free_disk_gb')

    # [C2/C3] Metadata sesi — diisi lewat argumen launch (checklist C3:
    # "gampang diisi tanpa edit file manual"), bukan prompt interaktif
    # (node ROS tidak punya stdin interaktif yang praktis saat di-launch).
    session_phase_arg = DeclareLaunchArgument(
        'session_phase', default_value='',
        description='K1 / K2a / K2b / K2c / K2d — dicatat di meta_<session_id>.yaml')
    session_phase = LaunchConfiguration('session_phase')
    objects_present_arg = DeclareLaunchArgument(
        'objects_present', default_value='',
        description='Daftar object_uid yang terpasang sesi ini, pisah koma, mis. "CONE_A,CONE_B"')
    objects_present = LaunchConfiguration('objects_present')
    environment_note_arg = DeclareLaunchArgument(
        'environment_note', default_value='',
        description='Catatan bebas status lingkungan sesi ini')
    environment_note = LaunchConfiguration('environment_note')
    operator_note_arg = DeclareLaunchArgument(
        'operator_note', default_value='',
        description='Catatan bebas operator')
    operator_note = LaunchConfiguration('operator_note')
    relocalization_ok_arg = DeclareLaunchArgument(
        'relocalization_ok', default_value='true',
        description='Apakah relokalisasi berhasil sesi ini — bisa diubah live via '
                    "'ros2 param set /semantic_map_exporter relocalization_ok false'")
    relocalization_ok = LaunchConfiguration('relocalization_ok')

    # [J1] session_id dibagi bareng exporter (CSV) & folder rosbag —
    # default auto-generate (default_session_id di atas), override manual
    # kalau perlu.
    session_id_arg = DeclareLaunchArgument(
        'session_id', default_value=default_session_id,
        description='[checklist C1/J1] Dipakai sama persis oleh CSV logger & folder rosbag')
    session_id = LaunchConfiguration('session_id')

    # [J1/J3] rosbag record — opt-in terpisah dari enable_position_logger
    # (kadang mau CSV saja tanpa video, mis. sesi K1 yang cuma butuh posisi).
    record_bag_arg = DeclareLaunchArgument(
        'record_bag', default_value='false',
        description='[checklist J1] Nyalakan ros2 bag record bareng sesi ini')
    record_bag = LaunchConfiguration('record_bag')
    bag_output_dir_arg = DeclareLaunchArgument(
        'bag_output_dir', default_value='/media/kmp-orin/external_hdd/rosbags',
        description='[checklist J1] Path HDD EKSTERNAL (bukan SSD internal Jetson) — '
                    'SESUAIKAN dengan mount point HDD sesungguhnya di robot sebelum dipakai')
    bag_output_dir = LaunchConfiguration('bag_output_dir')

    # [J3] Topik yang direkam: RGB (latar bbox), deteksi mentah (bbox/class/
    # confidence/tracker_id — /yolo/detections, checklist J2), TF (trayektori
    # + transform ke map frame), /map (occupancy grid untuk figure trayektori).
    # SENGAJA TIDAK termasuk /velodyne_points — ukuran jauh lebih besar dan
    # tidak diperlukan untuk kebutuhan visual ini (occupancy grid cukup).
    bag_record = ExecuteProcess(
        condition=IfCondition(record_bag),
        cmd=[
            'ros2', 'bag', 'record',
            '-o', [bag_output_dir, '/bag_', session_id],
            '/camera/camera/color/image_raw',
            '/yolo/detections',
            '/tf', '/tf_static',
            '/map',
        ],
        output='screen',
    )

    # ── Node 5: Semantic Map Exporter (delay 5s — grid node harus up) ────
    semantic_map_exporter = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3', os.path.join(semantic_dir, 'semantic_map_exporter.py'),
                    '--ros-args',
                    '-p', ['model_path:=', export_model_path],
                    '-p', 'min_points:=20',
                    '-p', 'export_rate:=5.0',
                    '-p', 'merge_dist:=1.5',
                    '-p', ['clear_on_start:=', clear_on_start],
                    '-p', ['enable_reid:=', enable_reid],
                    '-p', ['enable_position_logger:=', enable_position_logger],
                    '-p', ['position_log_dir:=', position_log_dir],
                    '-p', ['reid_timing_log_dir:=', reid_timing_log_dir],
                    '-p', ['min_free_disk_gb:=', min_free_disk_gb],
                    '-p', ['session_phase:=', session_phase],
                    '-p', ["objects_present:='", objects_present, "'"],
                    '-p', ["environment_note:='", environment_note, "'"],
                    '-p', ["operator_note:='", operator_note, "'"],
                    '-p', ['relocalization_ok:=', relocalization_ok],
                    '-p', ['session_id_override:=', session_id],
                    # [D3] Nilai YOLO ini TIDAK dipakai secara fungsional oleh
                    # exporter — cuma direkam ke meta_<session_id>.yaml supaya
                    # pengaturan inferensi sesi ini tercatat di satu tempat.
                    '-p', ['yolo_confidence_meta:=', confidence],
                    '-p', ['yolo_iou_nms_meta:=', iou_nms],
                    '-p', ['yolo_imgsz_meta:=', imgsz],
                ],
                output='screen',
            )
        ]
    )

    # ── Node 6: Semantic Map Viz (delay 6s — exporter harus up dulu) ─────
    semantic_map_viz = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3', os.path.join(semantic_dir, 'semantic_map_viz.py'),
                    '--ros-args',
                ],
                output='screen',
            )
        ]
    )

    # ── Node 7: Lidar Obstacle Stop (safety, langsung dari awal) ─────────
    # Pakai FastDDS sama dengan node lain supaya bisa subscribe /velodyne_points.
    # Mekanisme stop: publish zero Twist ke /cmd_vel_nav (bukan emergency_stop Bool).
    obstacle_stop = ExecuteProcess(
        cmd=[
            'python3', os.path.join(semantic_dir, 'obstacle_stop_node.py'),
            '--ros-args',
            '-p', 'fwd_distance:=1.0',
            '-p', 'side_distance:=0.6',
            '-p', 'robot_width:=0.9',
            '-p', 'sensor_height:=0.7',
            '-p', 'stop_hysteresis:=2',
            '-p', 'clear_hysteresis:=10',
        ],
        output='screen',
    )

    return LaunchDescription([
        clear_on_start_arg,
        model_path_arg,
        segformer_model_arg,
        device_arg,
        confidence_arg,
        iou_nms_arg,
        imgsz_arg,
        iou_threshold_arg,
        min_cells_arg,
        depth_source_arg,
        lidar_topic_arg,
        lidar_dilate_px_arg,
        model_path_export_arg,
        enable_reid_arg,
        enable_position_logger_arg,
        position_log_dir_arg,
        reid_timing_log_dir_arg,
        min_free_disk_gb_arg,
        session_phase_arg,
        objects_present_arg,
        environment_note_arg,
        operator_note_arg,
        relocalization_ok_arg,
        session_id_arg,
        record_bag_arg,
        bag_output_dir_arg,
        yolo_detector,
        semantic_projector,
        semantic_grid,
        segformer_node,
        semantic_map_exporter,
        semantic_map_viz,
        obstacle_stop,
        bag_record,
    ])
