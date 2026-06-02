# Launch & Tooling Reference
**Proyek:** Semantic Mapping — Husky A200 + Velodyne VLP-32C + RealSense D455
**Platform:** ROS 2 Humble | Middleware: FastDDS
**Repo:** https://github.com/nasyihulwan/rtabmap_allinone

---

## Cara Pakai Cepat

```bash
# Terminal 1 — RTAB-Map + Rosbag
./run.sh

# Terminal 2 — Semantic Pipeline (setelah Terminal 1 stabil)
./run_semantic.sh

# Terminal 3 — Capture hasil ke PNG (Ctrl+C untuk simpan)
python3 capture_semantic_map.py
```

---

## 1. `run.sh` → `rtabmap_rosbag.launch.py`

Entry point utama. Menjalankan rosbag replay + seluruh pipeline RTAB-Map.

### Cara pakai

```bash
./run.sh [rate] [mode] [use_ekf] [odom_topic]
```

| Perintah | Keterangan |
|---|---|
| `./run.sh` | Default: rate 0.5, mapping mode, EKF off |
| `./run.sh 0.3` | Replay lebih lambat (lebih stabil untuk debug) |
| `./run.sh 0.5 localize` | Localization mode (peta sudah ada) |
| `./run.sh 0.5 mapping true` | EKF aktif (untuk bag lama tanpa odom/filtered) |
| `./run.sh setup` | Setup OS network buffer (jalankan sekali setelah boot) |
| `./run.sh diagnose` | Cek package dan bag tersedia |
| `./run.sh help` | Tampilkan help |

### Parameter launch

| Parameter | Default | Keterangan |
|---|---|---|
| `bag_path` | `rosbag2_2026_04_15-08_39_23` | Path ke folder rosbag |
| `rate` | `0.5` | Kecepatan replay (1.0 = realtime, 0.5 = setengah) |
| `rviz` | `true` | Tampilkan RViz2 otomatis |
| `db_path` | `/root/.ros/rtabmap.db` | Path database RTAB-Map |
| `use_ekf` | `false` | `false` = pakai odom dari bag langsung |
| `odom_topic` | `/a200_1060/platform/odom/filtered` | Topic odometri ke RTAB-Map |
| `localization` | `false` | `true` = localization mode (tidak buat peta baru) |

> **Catatan bag 2026-04-15:** `use_ekf=false` karena `/a200_1060/platform/odom/filtered` sudah ada di dalam bag. EKF hanya perlu diaktifkan untuk bag lama.

### Urutan startup (timing)

| Waktu | Yang dijalankan |
|---|---|
| T+0s | `ros2 bag play` — rosbag mulai diputar |
| T+2s | Relay TF, Static TF, Decompress color & depth |
| T+3s | EKF node (hanya jika `use_ekf=true`) |
| T+6s | RTAB-Map SLAM + rtabmap_viz |
| T+8s | RViz2 |

### Topik yang di-relay

| Dari (bag) | Ke (sistem) | Keterangan |
|---|---|---|
| `/a200_1060/tf` | `/tf` | TF utama robot |
| `/a200_1060/tf_static` | `/tf_static` | TF statis |
| `/a200_1060/platform/odom` | `/odom` | Raw wheel odometry |
| `/a200_1060/sensors/imu_0/data` | `/imu/data` | IMU (hanya jika EKF aktif) |

### Static TF yang dipasang

| Frame dari | Frame ke | Offset |
|---|---|---|
| `a200_1060/base_link` | `camera_link` | x=0.2m, z=0.25m (bracket kamera) |
| `camera_link` | `camera_color_optical_frame` | Rotasi -90° Y, -90° Z |
| `camera_link` | `camera_depth_optical_frame` | Rotasi -90° Y, -90° Z |
| `base_link` | `a200_1060/base_link` | Identity (bridge frame) |

### Parameter RTAB-Map penting

**Memory:**
| Parameter | Nilai | Keterangan |
|---|---|---|
| `Mem/STMSize` | `30` | Short-term memory size |
| `Mem/RehearsalSimilarity` | `0.45` | Threshold kesamaan node untuk rehearsal |
| `Kp/MaxFeatures` | `600` | Maksimum fitur visual per frame |
| `Rtabmap/DetectionRate` | `1.0` | Rate deteksi loop closure (Hz) |

**Grid (Occupancy Map):**
| Parameter | Nilai | Keterangan |
|---|---|---|
| `Grid/Sensor` | `2` | Sumber: LiDAR (2) |
| `Grid/3D` | `false` | 2D occupancy grid |
| `Grid/MaxObstacleHeight` | `2.0` | Tinggi maksimum obstacle |
| `Grid/FootprintLength` | `0.8` | Panjang footprint Husky |
| `Grid/FootprintWidth` | `0.5` | Lebar footprint Husky |
| `Grid/RayTracing` | `true` | Ray tracing untuk free space |

**ICP (LiDAR registration):**
| Parameter | Nilai | Keterangan |
|---|---|---|
| `Icp/MaxCorrespondenceDistance` | `0.15` | Maksimum jarak korespondensi |
| `Icp/VoxelSize` | `0.1` | Downsampling voxel |
| `Icp/MaxTranslation` | `1.5` | Batas translasi per step |
| `Reg/Force3DoF` | `true` | Paksa 2D (x, y, yaw) |

**Loop Closure:**
| Parameter | Nilai | Keterangan |
|---|---|---|
| `RGBD/NeighborLinkRefining` | `true` | Refine odometry dengan ICP |
| `RGBD/ProximityBySpace` | `true` | Cek loop closure berdasarkan posisi |
| `RGBD/AngularUpdate` | `0.01` | Minimal rotasi untuk update node |
| `RGBD/LinearUpdate` | `0.01` | Minimal translasi untuk update node |

### Topik output RTAB-Map

| Topik | Tipe | Keterangan |
|---|---|---|
| `/map` | `OccupancyGrid` | Peta 2D LiDAR |
| `/rtabmap/cloud_map` | `PointCloud2` | Point cloud peta |
| `/rtabmap/grid_map` | `OccupancyGrid` | Grid dari RTAB-Map |
| `/tf` | `TFMessage` | Transformasi map→odom→base_link |

---

## 2. `run_semantic.sh` → `semantic_only.launch.py`

Pipeline semantik: YOLO deteksi → proyeksi ke 3D → akumulasi ke peta global.

### Cara pakai

```bash
./run_semantic.sh              # default: cpu, confidence 0.5
./run_semantic.sh cuda         # pakai GPU
./run_semantic.sh cpu 0.4      # confidence threshold 0.4
```

> **PENTING:** Jalankan ini **setelah** `run.sh` sudah stabil dan RTAB-Map sudah mulai publish `/map`.

### Parameter launch

| Parameter | Default | Keterangan |
|---|---|---|
| `model_path` | `models/yolov8n-seg_coco.pt` | Path model YOLOv8 |
| `device` | `cpu` | Device inferensi: `cpu` atau `cuda` |
| `confidence` | `0.5` | Confidence threshold YOLO (0.0–1.0) |
| `map_iou_threshold` | `0.1` | Minimum IoU footprint untuk dedup instance di peta |
| `min_cells_to_check` | `20` | Minimum cell akumulasi sebelum instance dievaluasi |

### Node 1: `yolo_detector.py` (T+0s)

**Subscribe:**
- `/camera/color/image_raw` — frame kamera RGB (dari decompress)

**Publish:**
- `/semantic/image_annotated` — frame dengan overlay mask + label
- `/semantic/label_image` — mono8: class_id+1 per pixel
- `/semantic/instance_image` — mono8: instance_id per pixel

**Parameter internal:**

| Parameter | Default | Keterangan |
|---|---|---|
| `model_path` | — | Path model YOLO |
| `confidence` | `0.5` | Confidence threshold |
| `device` | `cpu` | CPU/GPU |
| `process_every_n` | `3` | Proses 1 dari N frame (hemat CPU) |
| `iou_threshold` | `0.3` | IoU tracker antar frame (short-term) |
| `max_track_age` | `10` | Frame maksimum sebelum track expired |

> **IoU Tracker** di node ini menjaga konsistensi instance ID **antar frame berurutan** (short-term). Berbeda dengan Map-level IoU di grid node yang menangani duplikat **saat robot revisit** (long-term).

### Node 2: `semantic_projector.py` (T+2s)

**Subscribe:**
- `/semantic/label_image` — class ID per pixel
- `/semantic/instance_image` — instance ID per pixel
- `/camera/depth/image_rect_raw` — depth 16UC1
- `/camera/color/camera_info` — intrinsik kamera

**Publish:**
- `/semantic/pointcloud` — PointCloud2 dengan encoding RGB: R=class_id, G=instance_id, B=0

**Parameter internal:**

| Parameter | Default | Keterangan |
|---|---|---|
| `max_depth` | `8.0` | Batas jauh depth yang dipakai (meter) |
| `min_depth` | `0.2` | Batas dekat depth (meter) |
| `sample_step` | `4` | Sampling pixel: proses 1 dari setiap 4 pixel |
| `sync_slop` | `0.5` | Toleransi timestamp sync antar topic (detik) |

### Node 3: `semantic_grid_node.py` (T+3s)

**Subscribe:**
- `/semantic/pointcloud` — dari semantic projector

**Publish:**
- `/semantic/grid_map` — OccupancyGrid dengan instance_id per cell
- `/semantic/grid_colored` — PointCloud2 berwarna per instance

**Parameter internal:**

| Parameter | Default | Keterangan |
|---|---|---|
| `map_frame` | `map` | Reference frame peta |
| `resolution` | `0.05` | Resolusi grid (meter/cell) |
| `min_height` | `0.1` | Filter tinggi minimum objek (meter) |
| `max_height` | `2.0` | Filter tinggi maksimum objek (meter) |
| `publish_rate` | `1.0` | Rate publish peta semantik (Hz) |
| `decay_time` | `0.0` | Waktu decay cell (0 = tidak decay) |
| `map_iou_threshold` | `0.1` | Minimum IoU untuk merge instance duplikat |
| `min_cells_to_check` | `20` | Minimum cell sebelum instance dievaluasi |

**Logika deduplication (Map-level IoU):**

Instance baru masuk ke `pending` dulu. Setelah akumulasi ≥ `min_cells_to_check` cell, footprint-nya dibandingkan dengan semua instance confirmed sesama class menggunakan IoU. Kalau IoU ≥ `map_iou_threshold` → merge ke instance lama. Kalau tidak → confirm sebagai instance baru.

```
Contoh log terminal:
[Dedup]   refrigerator_18 → merge ke refrigerator_6 (IoU=0.23)
[Confirm] chair_21 confirmed (best IoU=0.04, cells=22)
```

**Topik RViz yang perlu ditambahkan:**

| Topik | Tipe | Keterangan |
|---|---|---|
| `/semantic/grid_map` | OccupancyGrid | Instance ID per cell |
| `/semantic/grid_colored` | PointCloud2 | Visualisasi warna per instance |

---

## 3. `capture_semantic_map.py`

Tool capture standalone. Simpan snapshot peta semantik ke 4 file PNG sekaligus.

### Cara pakai

```bash
# Terminal 3 (setelah Terminal 1 dan 2 jalan)
python3 capture_semantic_map.py

# Custom output directory
python3 capture_semantic_map.py /path/ke/folder/output
```

Tekan **Ctrl+C** kapanpun untuk trigger capture dan simpan 4 PNG.

### Subscribe

| Topik | Tipe | Keterangan |
|---|---|---|
| `/map` | OccupancyGrid | Base map LiDAR dari RTAB-Map |
| `/semantic/grid_colored` | PointCloud2 | Warna instance dari grid node |
| `/semantic/pointcloud` | PointCloud2 | Raw pointcloud untuk decode class+instance |

### Output — 4 file PNG per capture

Semua file disimpan di `semantic_results/` dengan timestamp yang sama.

| Suffix | Keterangan |
|---|---|
| `_a_class.png` | Warna per **class** — semua kursi warna sama, semua kulkas warna sama |
| `_b_outline.png` | **Kontur** per instance — outline polygon tiap objek |
| `_c_centroid.png` | **Centroid** per instance — kotak kecil di titik tengah tiap objek |
| `_d_dense.png` | **Dense fill** per instance — warna berbeda per instance |

### Status display (setiap 5 detik)

```
[INFO] Base: ✓ | Semantic: ✓ (1842 pts) | Instances: 5 [chair_2, chair_5, refrigerator_6...]
       → Ctrl+C untuk capture
```

---

## Troubleshooting Umum

| Gejala | Kemungkinan penyebab | Solusi |
|---|---|---|
| Semantic tidak muncul di peta | Terminal 1 belum stabil saat Terminal 2 dijalankan | Tunggu log RTAB-Map sudah ada output, baru jalankan Terminal 2 |
| TF fail berulang di semantic_grid_node | `map` frame belum publish | Pastikan bag sudah diputar dan RTAB-Map sudah running |
| Instance terus bertambah (banyak duplikat) | `map_iou_threshold` terlalu kecil | Naikkan ke 0.15–0.2 di launch file |
| Instance terlalu cepat di-merge | `map_iou_threshold` terlalu besar | Turunkan ke 0.05–0.08 |
| YOLO lambat | Model terlalu besar atau CPU bottleneck | Pakai `./run_semantic.sh cuda` atau turunkan resolusi kamera |
| Bag tidak ditemukan | Path salah di `run.sh` | Edit variabel `BAG_PATH` di `run.sh` |
