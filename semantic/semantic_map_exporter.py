#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  semantic_map_exporter.py — Semantic Map YAML Exporter          ║
║                                                                  ║
║  Subscribe:                                                      ║
║    /semantic/pointcloud  (PointCloud2, R=class_id, G=inst_id,  ║
║                           B=round(confidence*255))              ║
║                                                                  ║
║  Accumulate posisi per instance → tulis ke:                     ║
║    ~/.ros/semantic_map.yaml                                      ║
║                                                                  ║
║  Format YAML:                                                    ║
║    chair_2:                                                      ║
║      x: 1.234                                                    ║
║      y: 5.678                                                    ║
║      class_id: 0                                                 ║
║      cell_count: 42                                              ║
║                                                                  ║
║  Changelog:                                                      ║
║    [fix1] class_id >= 0 — class_id=0 sebelumnya difilter salah  ║
║    [fix2] key dict jadi (class_id, inst_id) — hindari collision ║
║    [fix3] decode vectorized numpy, tanpa Python loop            ║
║    [fix4] TF2 transform xyz ke map frame sebelum accumulate     ║
║    [fix5] load existing YAML saat startup — data mapping tidak  ║
║           hilang saat restart di localize mode                  ║
║    [reid] instance re-identification berdasarkan posisi peta:   ║
║           instance baru dari sesi baru di-merge ke instance     ║
║           lama jika posisinya berdekatan (< merge_dist meter)   ║
║    [logger] position logger opt-in: satu baris CSV per deteksi  ║
║           per frame, ditulis SEBELUM keputusan merge — dipakai  ║
║           untuk kalibrasi merge_dist (replay offline)           ║
║    [timing] reid_timing_log opt-in: ms/frame mekanisme re-id     ║
║           (proposal Tujuan 6)                                   ║
║    [cov] subscribe /rtabmap/localization_pose (kalau ada) untuk  ║
║           isi robot_cov_xx/yy/yaw di position logger            ║
║    [E2] state akumulasi + gating dipindah ke reid_core.py —      ║
║           dipakai bareng skrip replay position_log_analyzer.py, ║
║           SATU implementasi bukan dua yang "seharusnya sama"    ║
║    [C2/D2/F1/F2] meta_<session_id>.yaml — metadata sesi (phase,  ║
║           objects_present, hash model, versi software, dump     ║
║           parameter rtabmap) ditulis otomatis saat startup dan  ║
║           di-update saat shutdown bersih                        ║
╚══════════════════════════════════════════════════════════════════╝
"""

import csv
import datetime
import hashlib
import os
import shutil
import subprocess
import time
import yaml
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros

from reid_core import ReidAccumulator

SEMANTIC_MAP_PATH = os.path.expanduser('~/.ros/semantic_map.yaml')
# [A4] SATU FILE PER SESI, nama file mengandung session_id (checklist A4) —
# bukan lagi satu file yang di-append lintas sesi. Direktori, bukan path
# file langsung; nama file sesungguhnya dibentuk saat session_id sudah tahu.
POSITION_LOG_DIR    = os.path.expanduser('~/.ros/position_logs')
REID_TIMING_LOG_DIR = os.path.expanduser('~/.ros/reid_timing_logs')
META_LOG_DIR         = os.path.expanduser('~/.ros/session_meta')

# [A3] Kolom wajib checklist: session_id, frame_seq, stamp_ros, stamp_wall,
# class_name, confidence, x, y, z, n_points, tracker_id, det_idx,
# robot_x/y/yaw. class_id & robot_cov_* ditambahkan sebagai ekstra (tidak
# mengganggu kolom wajib, tetap berguna untuk analisis).
# depth_valid_ratio/loop_closure_count/mask_area_px (kolom opsional
# checklist) BELUM diimplementasikan — butuh instrumentasi tambahan di
# yolo_detector.py/semantic_projector.py (mask_area_px) atau subscribe
# rtabmap_msgs/Info yang belum diverifikasi tersedia (loop_closure_count).
POSITION_LOG_FIELDS = [
    'session_id', 'frame_seq', 'det_idx', 'stamp_ros', 'stamp_wall',
    'class_id', 'class_name', 'tracker_id',
    'x', 'y', 'z', 'n_points', 'confidence',
    'robot_x', 'robot_y', 'robot_yaw',
    'robot_cov_xx', 'robot_cov_yy', 'robot_cov_yaw',
]

# [timing] Satu baris per frame — biaya komputasi mekanisme re-id
# (proposal subbab 3.6, Tujuan 6). reid_ms diukur bahkan saat n_checked=0
# supaya distribusi ms/frame yang tercatat mencerminkan biaya nyata di
# setiap frame, bukan hanya frame yang menghasilkan keputusan merge.
REID_TIMING_FIELDS = [
    'session_id', 'frame_seq', 'stamp_wall',
    'reid_ms', 'n_checked', 'n_confirmed_instances',
    'enable_reid', 'merge_dist',
]


def _new_session_id() -> str:
    """[C1] Format S<YYYYMMDD>_<HHMM> — kalau file dengan nama ini SUDAH
    ada (dua sesi mulai di menit yang sama), tambahkan sufiks _2, _3, dst
    supaya sesi baru TIDAK PERNAH menimpa sesi lama (checklist I)."""
    base = datetime.datetime.now().strftime('S%Y%m%d_%H%M')
    return base


def _sha256_file(path: str, chunk_size: int = 1 << 20):
    """[D2] Hash SHA-256 file model — kalau di-retrain di tengah eksperimen
    tanpa disadari, hash ini yang membuktikannya. None kalau file tidak ada."""
    if not path or not os.path.exists(path):
        return None
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(chunk_size), b''):
            h.update(chunk)
    return h.hexdigest()


def _run_cmd(cmd: list, timeout: float = 3.0):
    """Jalankan command, return stdout ter-strip atau None kalau gagal/timeout.
    Dipakai untuk query versi software — semuanya best-effort, tidak boleh
    membuat node gagal start hanya karena satu tool CLI tidak ada."""
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        out = result.stdout.strip()
        return out if out else None
    except Exception:
        return None


def _gather_software_versions(pipeline_repo_dir: str) -> dict:
    """
    [F2] Versi software untuk Bagian 3.1 naskah — dikumpulkan saat runtime,
    bukan diingat/dikarang belakangan. Best-effort: field yang gagal
    di-query diisi None, TIDAK menghentikan node.
    """
    git_status = _run_cmd(['git', '-C', pipeline_repo_dir, 'status', '--porcelain'])
    versions = {
        'ros_distro': os.environ.get('ROS_DISTRO'),
        'pipeline_git_commit': _run_cmd(
            ['git', '-C', pipeline_repo_dir, 'rev-parse', 'HEAD']),
        'pipeline_git_dirty': bool(git_status),
    }
    for pkg, import_name in [('ultralytics', 'ultralytics'), ('torch', 'torch'),
                              ('torchvision', 'torchvision'), ('opencv', 'cv2')]:
        try:
            mod = __import__(import_name)
            versions[pkg] = getattr(mod, '__version__', 'unknown')
        except Exception:
            versions[pkg] = None
    # rtabmap & rtabmap_ros — query via dpkg (paket ROS Debian di Jetson/Ubuntu)
    versions['rtabmap_dpkg'] = _run_cmd(
        ['bash', '-c', "dpkg-query -W -f='${Version}\\n' 'ros-*-rtabmap*' 2>/dev/null | head -1"])
    # JetPack / L4T (khusus Jetson) — file version standar L4T
    l4t = None
    if os.path.exists('/etc/nv_tegra_release'):
        l4t = _run_cmd(['cat', '/etc/nv_tegra_release'])
    versions['l4t_release'] = l4t
    # CUDA
    versions['cuda_version'] = _run_cmd(['nvcc', '--version'])
    return versions


def _dump_rtabmap_params(output_path: str) -> bool:
    """
    [F1] Best-effort `ros2 param dump /rtabmap`. `ros2 param dump` menulis
    ke `rtabmap.yaml` di working directory (bukan stdout, bukan
    --output-dir — tidak konsisten antar distro ROS2), jadi dijalankan
    dengan cwd=direktori tujuan lalu file hasilnya di-rename ke
    output_path. Return True kalau berhasil. Node rtabmap mungkin
    belum/tidak aktif saat exporter start — kegagalan di sini TIDAK
    menghentikan exporter, cuma dicatat di log oleh caller.
    """
    out_dir = os.path.dirname(output_path)
    os.makedirs(out_dir, exist_ok=True)
    try:
        subprocess.run(['ros2', 'param', 'dump', '/rtabmap'],
                        capture_output=True, text=True, timeout=10.0, cwd=out_dir)
        produced = os.path.join(out_dir, 'rtabmap.yaml')
        if os.path.exists(produced):
            os.replace(produced, output_path)
            return True
        return False
    except Exception:
        return False


# TF target frame — confirmed dari view_frames: map → odom → base_link → ... → camera_depth_optical_frame
MAP_FRAME = 'map'


def _load_class_names(model_path: str) -> dict:
    """Try to load class names from YOLO model. Returns {} on failure."""
    if not model_path:
        return {}
    try:
        from ultralytics import YOLO   # noqa: PLC0415
        model = YOLO(model_path)
        return {int(k): str(v) for k, v in model.names.items()}
    except Exception:
        return {}


def _quat_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """
    Convert quaternion (x, y, z, w) to 3x3 rotation matrix.
    No scipy dependency — pure numpy.
    """
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def decode_pointcloud2_xyz_rgb(msg: PointCloud2):
    """
    Decode PointCloud2 → (N,3) float32 array [x,y,z] dan (N,) uint32 array [rgb].

    [fix3] Fully vectorized — tidak ada Python loop.
    RGB disimpan sebagai float32 tapi isinya packed bytes (bit-reinterpretation),
    bukan numeric cast. Gunakan .view(np.uint32) agar byte pattern terjaga.

    Returns (xyz, rgb_u32) atau (None, None) on error.
    """
    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ('x', 'y', 'z', 'rgb')):
        return None, None

    n_points = msg.width * msg.height
    if n_points == 0:
        return None, None

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if len(raw) < n_points * msg.point_step:
        return None, None

    # Reshape ke (N, point_step) agar bisa slice per field
    arr = raw.reshape(n_points, msg.point_step)

    ox   = field_map['x'].offset
    oy   = field_map['y'].offset
    oz   = field_map['z'].offset
    orgb = field_map['rgb'].offset

    x_arr   = arr[:, ox:ox+4].copy().view(np.float32).reshape(-1)
    y_arr   = arr[:, oy:oy+4].copy().view(np.float32).reshape(-1)
    z_arr   = arr[:, oz:oz+4].copy().view(np.float32).reshape(-1)

    # [fix3] .view(np.uint32) — reinterpret bytes, BUKAN numeric cast
    rgb_u32 = arr[:, orgb:orgb+4].copy().view(np.uint32).reshape(-1)

    xyz = np.stack([x_arr, y_arr, z_arr], axis=1)
    return xyz, rgb_u32


# ─────────────────────────────────────────────────────────────────
# [logger] Pure helper — group titik valid dalam SATU pointcloud
# message per (class_id, tracker_id). Dipanggil SEBELUM merge/re-id,
# satu baris output = satu deteksi pada frame ini (bukan akumulasi
# lintas waktu). Tidak bergantung pada Node → bisa diuji langsung.
# ─────────────────────────────────────────────────────────────────
def _group_points_by_detection(xs: np.ndarray, ys: np.ndarray, zs: np.ndarray,
                                cids: np.ndarray, iids: np.ndarray,
                                confs: np.ndarray) -> list:
    """Returns list of dict: {class_id, tracker_id, x, y, z, n_points, confidence}.

    confidence = mean confidence (0..1) titik-titik deteksi ini pada frame
    ini — di-decode dari channel B pointcloud (round(conf*255)/255).
    """
    if len(cids) == 0:
        return []
    keys = np.stack([cids, iids], axis=1)
    uniq_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
    out = []
    for i, (cid, iid) in enumerate(uniq_keys):
        m = inverse == i
        out.append({
            'class_id':   int(cid),
            'tracker_id': int(iid),
            'x':          float(xs[m].mean()),
            'y':          float(ys[m].mean()),
            'z':          float(zs[m].mean()),
            'n_points':   int(m.sum()),
            'confidence': float(confs[m].mean()),
        })
    return out


# ─────────────────────────────────────────────────────────────────
# [logger][A4] Satu file CSV per sesi — pure I/O helper, tidak bergantung
# pada Node, bisa diuji langsung.
# ─────────────────────────────────────────────────────────────────
def _open_session_csv(directory: str, filename: str, fields: list):
    """
    Buka file CSV BARU untuk sesi ini (bukan append ke file lama —
    checklist A4: satu file per sesi). Kalau filename sudah ada (dua sesi
    mulai di menit yang sama, session_id collide), tambah sufiks _2, _3,
    dst — sesi baru TIDAK PERNAH menimpa sesi lama (checklist I).

    Returns (path, file_handle, csv_writer).
    """
    os.makedirs(directory, exist_ok=True)
    stem, ext = os.path.splitext(filename)
    path = os.path.join(directory, filename)
    n = 2
    while os.path.exists(path):
        path = os.path.join(directory, f'{stem}_{n}{ext}')
        n += 1

    f = open(path, 'x', newline='')   # mode 'x' — gagal keras kalau race condition
    writer = csv.writer(f)
    writer.writerow(fields)
    f.flush()
    return path, f, writer


def _check_disk_space(path: str, min_free_gb: float) -> tuple:
    """
    [I] Cek ruang disk tersisa di partisi tempat `path` berada. Dipanggil
    saat startup — kehabisan disk di tengah sesi K1/K2 (mis. hari ke-10)
    berarti mengulang dari nol tanpa bisa dipercepat.

    Returns (free_gb, ok) — ok=False kalau free_gb < min_free_gb.
    """
    os.makedirs(path, exist_ok=True)
    free_bytes = shutil.disk_usage(path).free
    free_gb = free_bytes / (1024 ** 3)
    return free_gb, free_gb >= min_free_gb


class SemanticMapExporter(Node):
    def __init__(self):
        super().__init__('semantic_map_exporter')

        self.declare_parameter('semantic_map_path', SEMANTIC_MAP_PATH)
        self.declare_parameter('model_path',        '')
        self.declare_parameter('min_points',        20)
        self.declare_parameter('export_rate',       5.0)
        # [reid] Jarak maksimum (meter) antara dua instance same-class
        # agar dianggap objek yang sama dan di-merge.
        self.declare_parameter('merge_dist',        1.0)
        self.declare_parameter('clear_on_start',     False)
        # [B1] Matikan re-id sepenuhnya — dipakai untuk baseline naif pada
        # eksperimen K1/K2 (setiap deteksi jadi instance baru). Nama param
        # persis 'enable_reid' sesuai checklist (bukan 'reid_enabled').
        self.declare_parameter('enable_reid',        True)
        # [logger] Opt-in — off by default supaya tidak mengubah perilaku
        # runtime produksi. Nyalakan saat sesi pengumpulan data K1/K2.
        # Satu flag yang sama menyalakan position logger DAN reid timing
        # logger — keduanya cuma relevan saat sesi eksperimen berjalan.
        self.declare_parameter('enable_position_logger', False)
        self.declare_parameter('position_log_dir',     POSITION_LOG_DIR)
        self.declare_parameter('reid_timing_log_dir',  REID_TIMING_LOG_DIR)
        self.declare_parameter('meta_log_dir',         META_LOG_DIR)
        # [I] Peringatkan (bukan menghentikan node — jangan menambah risiko
        # baru) kalau disk di bawah ambang ini saat sesi eksperimen mulai.
        self.declare_parameter('min_free_disk_gb',     2.0)
        # [J1] Kalau diisi, PAKAI nilai ini sebagai session_id (bukan
        # auto-generate sendiri) — supaya sinkron persis dengan nama folder
        # rosbag yang dibuat launch file di waktu yang sama.
        self.declare_parameter('session_id_override',  '')

        # [C2/C3] Metadata sesi — diisi lewat parameter (launch arg), bukan
        # prompt interaktif. relocalization_ok SENGAJA dibiarkan mutable
        # (bisa diubah live lewat `ros2 param set` kalau relokalisasi
        # ternyata gagal di tengah sesi) — dibaca ulang saat shutdown,
        # bukan cuma sekali di sini.
        self.declare_parameter('session_phase',      '')
        self.declare_parameter('objects_present',    '')
        self.declare_parameter('environment_note',   '')
        self.declare_parameter('operator_note',      '')
        self.declare_parameter('relocalization_ok',  True)
        # [D3] Nilai info-only, TIDAK dipakai secara fungsional di sini —
        # cuma direkam ke meta_<session_id>.yaml. Nilai sesungguhnya yang
        # DIPAKAI untuk inferensi ada di parameter yolo_detector.py sendiri;
        # operator wajib memastikan angka yang dikirim ke sini sama persis
        # (launch file sudah reuse LaunchConfiguration yang sama).
        self.declare_parameter('yolo_confidence_meta', -1.0)
        self.declare_parameter('yolo_iou_nms_meta',    -1.0)
        self.declare_parameter('yolo_imgsz_meta',      -1)

        self.map_path    = self.get_parameter('semantic_map_path').value
        model_path       = self.get_parameter('model_path').value
        self.min_points  = self.get_parameter('min_points').value
        export_rate      = self.get_parameter('export_rate').value
        self.merge_dist  = self.get_parameter('merge_dist').value
        clear_on_start   = self.get_parameter('clear_on_start').value
        self.enable_reid = self.get_parameter('enable_reid').value
        self.enable_position_logger = self.get_parameter('enable_position_logger').value
        self.position_log_dir      = self.get_parameter('position_log_dir').value
        self.reid_timing_log_dir   = self.get_parameter('reid_timing_log_dir').value
        self.meta_log_dir          = self.get_parameter('meta_log_dir').value
        self.min_free_disk_gb      = self.get_parameter('min_free_disk_gb').value
        self.model_path             = model_path
        self.session_phase          = self.get_parameter('session_phase').value
        self.objects_present        = self.get_parameter('objects_present').value
        self.environment_note       = self.get_parameter('environment_note').value
        self.operator_note          = self.get_parameter('operator_note').value
        self.yolo_confidence_meta   = self.get_parameter('yolo_confidence_meta').value
        self.yolo_iou_nms_meta      = self.get_parameter('yolo_iou_nms_meta').value
        self.yolo_imgsz_meta        = self.get_parameter('yolo_imgsz_meta').value

        self.class_names = _load_class_names(model_path)
        if self.class_names:
            self.get_logger().info(
                f'Class names loaded: {len(self.class_names)} classes dari {model_path}')
        else:
            self.get_logger().info(
                'Class names tidak di-load — label akan berformat class{id}_{iid}. '
                'Set param model_path untuk nama yang lebih readable.')

        # [fix4] TF2 buffer + listener untuk transform ke map frame
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # [fix2] Key: (class_id, instance_id) — bukan instance_id saja
        # [fix5] Load existing YAML saat startup supaya data mapping tidak
        #        hilang ketika exporter restart di localize mode.
        #        Observasi baru akan di-merge ke data yang sudah ada.
        # [E2] Seluruh state akumulasi + gating ada di ReidAccumulator
        #      (reid_core.py) — modul yang sama dipakai skrip replay.
        self._lock = threading.Lock()
        self._accumulator = ReidAccumulator(
            self.merge_dist, self.min_points, self.enable_reid)
        if clear_on_start:
            self.get_logger().info('[clear] clear_on_start=true — mulai dari kosong, YAML lama diabaikan.')
        else:
            self._load_existing_yaml()

        # [logger][C1] session_id dari wall-clock start, format S<tanggal>_<jam menit>
        # — satu-satunya cara membedakan sesi sekarang (tidak ada ID eksplisit
        # lain di pipeline). frame_seq dicatat eksplisit per pointcloud
        # message, TIDAK boleh diasumsikan dari urutan baris CSV.
        session_id_override = self.get_parameter('session_id_override').value
        self.session_id = session_id_override if session_id_override else _new_session_id()
        self.frame_seq  = 0
        self._session_start_wall = datetime.datetime.now().isoformat()

        self._log_file, self._log_writer = None, None
        self._timing_file, self._timing_writer = None, None
        self.position_log_path = None
        self.reid_timing_log_path = None
        self._rtabmap_param_dump_path = None
        self._software_versions = None
        if self.enable_position_logger:
            # [I] Cek disk SEBELUM buka file — kehabisan disk di tengah
            # sesi K1/K2 = ulang dari nol, tidak bisa dipercepat.
            free_gb, disk_ok = _check_disk_space(self.position_log_dir, self.min_free_disk_gb)
            if not disk_ok:
                self.get_logger().error(
                    f'[I] DISK HAMPIR PENUH: {free_gb:.2f} GB tersisa di {self.position_log_dir} '
                    f'(ambang {self.min_free_disk_gb} GB) — logger tetap jalan, tapi SEGERA '
                    f'pindahkan data lama / tambah storage sebelum sesi berikutnya.')
            else:
                self.get_logger().info(f'[I] Disk tersisa: {free_gb:.2f} GB — aman.')

            self.position_log_path, self._log_file, self._log_writer = \
                _open_session_csv(self.position_log_dir,
                                        f'det_{self.session_id}.csv', POSITION_LOG_FIELDS)
            self.reid_timing_log_path, self._timing_file, self._timing_writer = \
                _open_session_csv(self.reid_timing_log_dir,
                                        f'timing_{self.session_id}.csv', REID_TIMING_FIELDS)
            self.get_logger().info(
                f'[logger] Position + timing logger AKTIF | session_id={self.session_id} | '
                f'det={self.position_log_path} | timing={self.reid_timing_log_path}')

            # [D2/F1/F2] Best-effort, dijalankan sekali saat startup — bisa
            # makan beberapa detik (hash model + subprocess query versi +
            # ros2 param dump), sengaja hanya terjadi saat mode eksperimen.
            pipeline_repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self._software_versions = _gather_software_versions(pipeline_repo_dir)

            rtabmap_dump_path = os.path.join(self.meta_log_dir, f'rtabmap_params_{self.session_id}.yaml')
            if _dump_rtabmap_params(rtabmap_dump_path):
                self._rtabmap_param_dump_path = rtabmap_dump_path
                self.get_logger().info(f'[F1] rtabmap param dump → {rtabmap_dump_path}')
            else:
                self.get_logger().warn(
                    '[F1] Gagal dump /rtabmap param (node rtabmap belum aktif / ros2 CLI '
                    'tidak tersedia?) — rtabmap_param_dump_path kosong di metadata.')

            self._write_session_metadata(end=False)
            self.get_logger().info(f'[C2] meta_{self.session_id}.yaml ditulis di {self.meta_log_dir}')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(
            PointCloud2, '/semantic/pointcloud', self._cloud_cb, qos)

        # [cov] RTAB-Map publish PoseWithCovarianceStamped di topic ini
        # secara default (nama bawaan node 'rtabmap', tidak di-remap di
        # rtabmap_live.launch.py) — belum terverifikasi di sistem live,
        # cek log startup: kalau topic ini tidak pernah publish,
        # robot_cov_* di position logger akan tetap kosong (graceful).
        self._latest_cov = None
        qos_pose = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            PoseWithCovarianceStamped, '/rtabmap/localization_pose',
            self._pose_cov_cb, qos_pose)

        self.create_timer(export_rate, self._export_yaml)

        self.get_logger().info(
            f'SemanticMapExporter ready | '
            f'min_points={self.min_points} | '
            f'export_rate={export_rate}s | '
            f'merge_dist={self.merge_dist}m | '
            f'enable_reid={self.enable_reid} | '
            f'output={self.map_path} | '
            f'target_frame={MAP_FRAME}')

    # ─────────────────────────────────────────────────────────────
    # [fix5] LOAD EXISTING YAML
    # ─────────────────────────────────────────────────────────────
    def _load_existing_yaml(self) -> None:
        """
        Load semantic_map.yaml yang sudah ada, seed ke self._accumulator.

        YAML format:
          chair_2:
            x: 1.234
            y: 5.678
            class_id: 0
            cell_count: 42

        Setiap objek di-reconstruct dengan cell_count synthetic points di
        posisi mean-nya (ReidAccumulator.seed_instance) — mean tetap
        identik, dan instance ini langsung confirmed (bukan provisional).
        """
        if not os.path.exists(self.map_path):
            self.get_logger().info('[fix5] Tidak ada YAML existing — mulai dari kosong.')
            return

        try:
            with open(self.map_path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warn(f'[fix5] Gagal load YAML: {e} — mulai dari kosong.')
            return

        n_loaded = 0
        for label, obj in data.items():
            try:
                cid        = int(obj['class_id'])
                x_mean     = float(obj['x'])
                y_mean     = float(obj['y'])
                cell_count = int(obj.get('cell_count', 1))

                # [bugfix seed key] TIDAK memakai inst_id mentah dari label
                # (mis. "chair_2" -> 2) sebagai key — lihat docstring
                # ReidAccumulator.seed_instance soal kenapa itu berbahaya
                # (collision dengan tracker_id live yang reset tiap restart).
                # Key baru dibuat otomatis (auto negative id), label lama
                # hanya dipakai untuk logging, tidak untuk keying internal.
                self._accumulator.seed_instance(cid, x_mean, y_mean, cell_count)
                n_loaded += 1
            except (KeyError, ValueError, IndexError) as e:
                self.get_logger().warn(f'[fix5] Skip entry "{label}": {e}')
                continue

        self.get_logger().info(
            f'[fix5] Loaded {n_loaded} objek dari existing YAML: {self.map_path}')

    # ─────────────────────────────────────────────────────────────
    # TF TRANSFORM HELPER
    # ─────────────────────────────────────────────────────────────
    def _transform_xyz_to_map(self, xyz: np.ndarray, source_frame: str, stamp) -> np.ndarray | None:
        """
        [fix4] Transform (N,3) xyz array dari source_frame ke MAP_FRAME.

        Chain TF yang dipakai (confirmed dari view_frames):
          camera_depth_optical_frame → camera_depth_frame → camera_link
          → a200_1060/base_link → base_link → odom → map

        Returns transformed (N,3) array, atau None jika TF lookup gagal.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                source_frame,
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.1))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'[TF] lookup {source_frame} → {MAP_FRAME} gagal: {e}',
                throttle_duration_sec=5.0)
            return None

        tr = t.transform.translation
        ro = t.transform.rotation

        R = _quat_to_rotation_matrix(ro.x, ro.y, ro.z, ro.w)
        T = np.array([tr.x, tr.y, tr.z], dtype=np.float64)

        # (N,3) @ (3,3).T + (3,) — broadcast translation
        xyz_map = (R @ xyz.astype(np.float64).T).T + T
        return xyz_map.astype(np.float32)

    # ─────────────────────────────────────────────────────────────
    # [C2] METADATA SESI
    # ─────────────────────────────────────────────────────────────
    def _build_session_metadata(self, end: bool) -> dict:
        return {
            'session_id':         self.session_id,
            'phase':               self.session_phase or None,
            'start_wall':          self._session_start_wall,
            'end_wall':             datetime.datetime.now().isoformat() if end else None,
            'enable_reid':          self.enable_reid,
            'merge_dist':           self.merge_dist,
            'min_points':           self.min_points,
            # [C2] Dibaca ULANG di sini (bukan dari cache __init__) supaya
            # perubahan live lewat `ros2 param set ... relocalization_ok
            # false` di tengah sesi ikut tercatat di meta_end.
            'relocalization_ok':    self.get_parameter('relocalization_ok').value,
            'objects_present':      [s.strip() for s in self.objects_present.split(',') if s.strip()],
            'environment_note':     self.environment_note or None,
            'operator_note':        self.operator_note or None,
            'position_log_path':     self.position_log_path,
            'reid_timing_log_path':  self.reid_timing_log_path,
            'yolo_model_path':       self.model_path,
            'yolo_model_sha256':     _sha256_file(self.model_path),
            'yolo_confidence':       self.yolo_confidence_meta if self.yolo_confidence_meta >= 0 else None,
            'yolo_iou_nms':          self.yolo_iou_nms_meta if self.yolo_iou_nms_meta >= 0 else None,
            'yolo_imgsz':            self.yolo_imgsz_meta if self.yolo_imgsz_meta >= 0 else None,
            'rtabmap_param_dump_path': self._rtabmap_param_dump_path,
            'software_versions':      self._software_versions,
        }

    def _write_session_metadata(self, end: bool):
        if not self.enable_position_logger:
            return
        os.makedirs(self.meta_log_dir, exist_ok=True)
        path = os.path.join(self.meta_log_dir, f'meta_{self.session_id}.yaml')
        meta = self._build_session_metadata(end=end)
        with open(path, 'w') as f:
            yaml.dump(meta, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

    # ─────────────────────────────────────────────────────────────
    # [logger] POSITION LOGGER — level-deteksi, sebelum merge
    # ─────────────────────────────────────────────────────────────
    def _pose_cov_cb(self, msg: PoseWithCovarianceStamped):
        """
        [cov] Cache covariance terbaru dari /rtabmap/localization_pose.
        Row-major 6x6 (x,y,z,roll,pitch,yaw) — ambil diagonal x,y,yaw saja.
        TIDAK di-sync ke stamp pointcloud (topic ini bukan TF, tidak ada
        historical lookup semudah TF buffer) — "terbaru yang diterima" adalah
        aproksimasi yang disengaja, bukan nilai pada stamp yang presisi sama.
        """
        c = msg.pose.covariance
        self._latest_cov = (float(c[0]), float(c[7]), float(c[35]))

    def _lookup_robot_pose(self, stamp):
        """
        Best-effort lookup posisi robot (base_link) di map frame, pada
        stamp yang sama dengan pointcloud (TF, presisi per-stamp).
        Covariance-nya sendiri datang dari sumber terpisah (_pose_cov_cb)
        karena TF tidak membawa uncertainty — lihat catatan di sana.

        Returns (x, y, yaw) atau None jika lookup gagal.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                MAP_FRAME, 'base_link', stamp,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

        tr = t.transform.translation
        ro = t.transform.rotation
        yaw = float(np.arctan2(
            2*(ro.w*ro.z + ro.x*ro.y),
            1 - 2*(ro.y*ro.y + ro.z*ro.z)))
        return float(tr.x), float(tr.y), yaw

    def _log_frame_detections(self, detections: list, stamp):
        """
        Tulis satu baris CSV per deteksi pada frame ini — dipanggil
        SEBELUM grouping-by-key/merge di bawah, jadi tiap baris adalah
        observasi mentah (raw), bukan posisi rata-rata akumulasi.

        stamp: header.stamp pesan PointCloud2 (builtin_interfaces/Time) —
        sumber stamp_ros. stamp_wall (ISO-8601) dari jam sistem saat baris
        ini ditulis, TIDAK sama dengan stamp_ros (lihat A3: dua kolom
        waktu berbeda tujuan — sinkronisasi antar-topic vs analisis lintas
        hari/korelasi pencahayaan).
        """
        if not detections or self._log_writer is None:
            return

        robot_pose = self._lookup_robot_pose(stamp)
        rx, ry, ryaw = robot_pose if robot_pose is not None else ('', '', '')
        cov_xx, cov_yy, cov_yaw = self._latest_cov if self._latest_cov is not None else ('', '', '')
        stamp_ros = stamp.sec + stamp.nanosec * 1e-9
        stamp_wall = datetime.datetime.now().isoformat()

        for det_idx, d in enumerate(detections):
            name = self.class_names.get(d['class_id'], f"class{d['class_id']}")
            self._log_writer.writerow([
                self.session_id, self.frame_seq, det_idx,
                f'{stamp_ros:.6f}', stamp_wall,
                d['class_id'], name, d['tracker_id'],
                round(d['x'], 4), round(d['y'], 4), round(d['z'], 4),
                d['n_points'], round(d['confidence'], 4),
                rx, ry, ryaw,
                cov_xx, cov_yy, cov_yaw,
            ])
        self._log_file.flush()

    def _log_reid_timing(self, reid_ms: float, n_checked: int, n_confirmed: int):
        """
        [timing] Satu baris per frame — biaya komputasi mekanisme re-id
        (proposal Tujuan 6 / Tabel 3.8). enable_reid & merge_dist dicatat
        ulang di setiap baris (bukan cuma sekali di header) supaya CSV dari
        beberapa sesi dengan setting berbeda bisa digabung lalu dibedakan
        tanpa perlu menelusuri log startup node secara terpisah.
        """
        if self._timing_writer is None:
            return
        stamp_wall = datetime.datetime.now().isoformat()
        self._timing_writer.writerow([
            self.session_id, self.frame_seq, stamp_wall,
            round(reid_ms, 4), n_checked, n_confirmed,
            self.enable_reid, self.merge_dist,
        ])
        self._timing_file.flush()

    # ─────────────────────────────────────────────────────────────
    # CLOUD CALLBACK
    # ─────────────────────────────────────────────────────────────
    def _cloud_cb(self, msg: PointCloud2):
        self.frame_seq += 1
        n_raw = msg.width * msg.height
        self.get_logger().info(
            f'[DEBUG] cloud_cb dipanggil, points={n_raw}, frame={msg.header.frame_id}')

        xyz, rgb_u32 = decode_pointcloud2_xyz_rgb(msg)
        if xyz is None:
            self.get_logger().warn('[DEBUG] decode_pointcloud2_xyz_rgb returned None')
            return

        # [fix4] Transform ke map frame sebelum accumulate
        xyz_map = self._transform_xyz_to_map(xyz, msg.header.frame_id, msg.header.stamp)
        if xyz_map is None:
            return  # TF belum ready atau lookup gagal — skip frame ini

        # Decode class_id, instance_id, confidence dari packed RGB
        class_ids   = ((rgb_u32 >> 16) & 0xFF).astype(np.int32)
        inst_ids    = ((rgb_u32 >>  8) & 0xFF).astype(np.int32)
        confidences = (rgb_u32 & 0xFF).astype(np.float32) / 255.0

        # [DEBUG2] Sample nilai sebelum filter
        sample_n = min(5, n_raw)
        self.get_logger().info(
            f'[DEBUG2] sample class_ids={class_ids[:sample_n].tolist()} '
            f'inst_ids={inst_ids[:sample_n].tolist()} '
            f'map_xy={xyz_map[:sample_n, :2].tolist()}')

        # [fix1] class_id >= 0 — class_id=0 valid (class pertama YOLO)
        valid = (inst_ids > 0) & (class_ids >= 0)

        n_valid = int(valid.sum())
        self.get_logger().info(
            f'[DEBUG2] valid={n_valid}/{n_raw}',
            throttle_duration_sec=2.0)

        if not valid.any():
            return

        xs    = xyz_map[valid, 0]
        ys    = xyz_map[valid, 1]
        zs    = xyz_map[valid, 2]
        cids  = class_ids[valid]
        iids  = inst_ids[valid]
        confs = confidences[valid]

        # [E2] Selalu dihitung per-deteksi (bukan cuma saat logger aktif) —
        # dipakai ReidAccumulator.add_detection() DAN position logger. Ini
        # granularitas yang sama persis dengan yang dibaca skrip replay dari
        # CSV, sehingga live dan replay memproses unit observasi yang identik.
        detections = _group_points_by_detection(xs, ys, zs, cids, iids, confs)

        # [logger] Level-deteksi, SEBELUM merge/remap — dicatat pakai key
        # mentah (class_id, tracker_id) dari frame ini, bukan hasil remap.
        if self.enable_position_logger:
            self._log_frame_detections(detections, msg.header.stamp)

        with self._lock:
            for d in detections:
                self._accumulator.add_detection(
                    d['class_id'], d['tracker_id'], d['x'], d['y'], d['n_points'])

            # [timing] Diukur di sini saja (bukan loop akumulasi di atas) —
            # ini bagian yang jadi objek eksperimen biaya komputasi (Tujuan 6),
            # bukan overhead umum yang tetap ada terlepas dari mekanisme re-id.
            t0 = time.perf_counter()
            events = self._accumulator.promote_ready()
            reid_ms = (time.perf_counter() - t0) * 1000.0
            n_checked = len(events)
            n_confirmed = self._accumulator.n_confirmed

        for (cid, iid), ev in events.items():
            if ev['merged']:
                self.get_logger().info(
                    f'[ReID] ({cid},{iid}) → merged ke {ev["target"]} '
                    f'(dist={ev["dist"]:.3f}m)',
                    throttle_duration_sec=5.0)
            else:
                self.get_logger().debug(
                    f'[ReID] ({cid},{iid}) — tidak ada kandidat dalam {self.merge_dist}m '
                    f'→ instance baru confirmed')

        if self.enable_position_logger:
            self._log_reid_timing(reid_ms, n_checked, n_confirmed)

    # ─────────────────────────────────────────────────────────────
    # YAML EXPORT
    # ─────────────────────────────────────────────────────────────
    def _make_label(self, class_id: int, instance_id: int) -> str:
        name = self.class_names.get(class_id, f'class{class_id}')
        return f'{name}_{instance_id}'

    def _export_yaml(self):
        with self._lock:
            snap = self._accumulator.snapshot()

        out = {}
        for (cid, iid), entry in snap.items():
            label = self._make_label(cid, iid)
            out[label] = {
                'x':          round(entry['x'], 4),
                'y':          round(entry['y'], 4),
                'class_id':   entry['class_id'],
                'cell_count': entry['cell_count'],
            }

        if not out:
            return

        os.makedirs(os.path.dirname(self.map_path), exist_ok=True)
        with open(self.map_path, 'w') as f:
            yaml.dump(out, f, default_flow_style=False, allow_unicode=True)

        self.get_logger().info(
            f'[Export] {len(out)} objek → {self.map_path}',
            throttle_duration_sec=10.0)

    def destroy_node(self):
        # [C2] Update meta dengan end_wall + relocalization_ok final — HANYA
        # kalau shutdown bersih (SIGKILL/power loss tidak akan sampai sini,
        # tapi meta_start yang ditulis di __init__ tetap ada — lebih baik
        # daripada tidak ada metadata sama sekali, lihat checklist H4).
        self._write_session_metadata(end=True)
        if self._log_file is not None:
            self._log_file.close()
        if self._timing_file is not None:
            self._timing_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
