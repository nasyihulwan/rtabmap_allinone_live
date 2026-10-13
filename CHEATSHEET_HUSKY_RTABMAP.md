# Cheat Sheet — `husky_rtabmap` (RTABMap-only, toggle Lidar/Kamera)

Tool baru: RTABMap **tanpa** YOLO/SegFormer, dengan sensor (lidar/kamera) yang bisa
dimatikan salah satunya. Dibuat karena Velodyne kadang dipakai project lain di
kmp-orin, jadi perlu cara jalanin SLAM kamera-saja tanpa nunggu lidar available.

Sumber kode: `launch/live/husky_rtabmap.launch.py` (+ mode baru `./run.sh husky_rtabmap`).
Juga di-deploy sebagai folder mandiri di Orin: `~/anas/husky_rtabmap/` (tidak
ganggu clone repo utama di `~/anas/rtabmap_allinone_live` yang mungkin dipakai
project lain).

---

## 0. Prasyarat sensor (jalankan SEBELUM husky_rtabmap)

Cek dulu, jangan langsung restart driver (kmp-orin shared machine):

```bash
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/anas/rtabmap_allinone_live/config/fastdds_udp.xml
timeout 5 ros2 topic hz /velodyne_points 2>&1 | tail -2
timeout 3 ros2 topic echo /camera/camera/color/image_raw --once --qos-reliability best_effort \
  >/dev/null 2>&1 && echo "camera OK" || echo "camera NO DATA"
```

- **Kamera (RealSense D455)** — kalau belum jalan:
  ```bash
  ros2 launch realsense2_camera rs_launch.py \
      enable_color:=true enable_depth:=true align_depth.enable:=true \
      rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30
  ```
- **Lidar (Velodyne VLP-32C)** — driver mentah ada di PC Husky, transform+laserscan
  di kmp-orin (arsitektur split, CPU PC Husky cuma 4-core). Kalau `/velodyne_points`
  kosong dan lidar memang lagi dipakai project lain → **jangan restart**, pakai
  mode `USE_LIDAR=false` di bawah.

FastDDS profile (`fastdds_udp.xml`) **wajib** di-export di semua terminal — tanpa
ini data lidar drop total. Jangan pernah edit `interfaceWhiteList`-nya.

---

## 1. Cara jalanin — dua pilihan

**A) Dari repo penuh** (di laptop dev, atau di Orin `~/anas/rtabmap_allinone_live` setelah `git pull`):
```bash
cd ~/anas/rtabmap_allinone_live
./run.sh husky_rtabmap
```

**B) Folder mandiri di Orin** (tidak butuh repo penuh, tidak sentuh clone bersama):
```bash
cd ~/anas/husky_rtabmap
./run_husky_rtabmap.sh
```

Kedua cara pakai env var toggle yang sama (lihat tabel di bawah). Perbedaan:
opsi A pakai `precheck` penuh punya `run.sh` (cek model YOLO dll — akan warning
tapi tidak menghalangi karena mode ini tidak butuh YOLO) dan buka terminal baru;
opsi B lebih ringan, jalan langsung di terminal yang sama (Ctrl+C untuk stop).

---

## 2. Kombinasi mode (mapping / localize × lidar / kamera)

| Skenario | Command (opsi A, dari repo) | Command (opsi B, folder mandiri) |
|---|---|---|
| **Mapping, lidar+kamera** (default, paling akurat) | `./run.sh husky_rtabmap` | `./run_husky_rtabmap.sh` |
| **Mapping, kamera saja** (lidar dipakai project lain) | `USE_LIDAR=false ./run.sh husky_rtabmap` | `USE_LIDAR=false ./run_husky_rtabmap.sh` |
| **Mapping, lidar saja** (kamera mati/rusak) | `USE_CAMERA=false ./run.sh husky_rtabmap` | `USE_CAMERA=false ./run_husky_rtabmap.sh` |
| **Localization, lidar+kamera** (pakai DB yang sudah ada) | `LOCALIZATION=true ./run.sh husky_rtabmap` | `LOCALIZATION=true ./run_husky_rtabmap.sh` |
| **Localization, kamera saja** | `USE_LIDAR=false LOCALIZATION=true ./run.sh husky_rtabmap` | `USE_LIDAR=false LOCALIZATION=true ./run_husky_rtabmap.sh` |
| **Localization, lidar saja** | `USE_CAMERA=false LOCALIZATION=true ./run.sh husky_rtabmap` | `USE_CAMERA=false LOCALIZATION=true ./run_husky_rtabmap.sh` |

`USE_LIDAR=false` **dan** `USE_CAMERA=false` bersamaan = ditolak (tidak ada
sensor untuk SLAM sama sekali), script langsung exit dengan pesan error.

⚠️ **Penting — pakai DB terpisah tiap kombinasi sensor.** Peta yang dibangun
kamera-saja dan lidar-saja punya karakteristik geometri berbeda (lihat tabel
parameter di bawah) — mencampur keduanya di satu database yang sama bisa
merusak loop closure. Override lewat `DB_PATH`:
```bash
DB_PATH=~/.ros/husky_rtabmap_camera_only.db USE_LIDAR=false ./run.sh husky_rtabmap
```
(Opsi B / `run_husky_rtabmap.sh` sudah default ke DB terpisah:
`~/.ros/husky_rtabmap.db`, tidak menimpa `~/.ros/rtabmap.db` utama.)

---

## 3. Parameter kunci yang dipakai (RTABMap)

Parameter berikut **otomatis berubah** tergantung kombinasi `use_lidar`/`use_camera`:

| Parameter | Lidar+Kamera | Kamera saja | Lidar saja | Arti |
|---|---|---|---|---|
| `subscribe_scan_cloud` | `true` | `false` | `true` | subscribe `/velodyne_points` |
| `subscribe_rgb` / `subscribe_depth` | `true` | `true` | `false` | subscribe RGB-D dari D455 |
| `Reg/Strategy` | `2` (Visual+ICP) | `0` (Visual only) | `1` (ICP only) | metode registrasi antar-frame |
| `Grid/Sensor` | `2` (Scan+Depth) | `1` (Depth) | `0` (Laser scan) | sumber data occupancy grid |

Parameter yang **sama** di semua mode:
| Parameter | Nilai | Arti |
|---|---|---|
| `frame_id` | `a200_1060/base_link` | frame robot |
| `odom_topic` | `/a200_1060/platform/odom/filtered` | odometry Husky (override via `ODOM_TOPIC`) |
| `Mem/IncrementalMemory` | `true` (mapping) / `false` (localize) | tambah node baru ke peta atau tidak |
| `publish_tf` | `true` (kalau `use_amcl=false`) | RTABMap publish TF `map→odom` sendiri |
| `Kp/MaxFeatures` | `600` | jumlah fitur visual maksimum per frame |
| `Vis/MaxDepth` | `10.0` m | jarak maksimum depth yang dipakai utk visual odom |
| `Icp/MaxCorrespondenceDistance` | `0.15` m | toleransi matching ICP |
| `RGBD/CreateOccupancyGrid` | `true` | selalu build occupancy grid 2D utk Nav2 |
| `approx_sync_max_interval` | `6.0` s | toleransi sinkronisasi antar-topic sensor |

Topic remapping (tetap, tidak berubah antar mode):
- `rgb/image` → `/camera/camera/color/image_raw`
- `depth/image` → `/camera/camera/depth/image_rect_raw`
- `scan_cloud` → `/velodyne_points`
- `map` → `/map`, `cloud_map` → `/rtabmap/cloud_map`, `grid_map` → `/rtabmap/grid_map`

---

## 4. Verifikasi setelah jalan

```bash
ros2 node list | grep rtabmap          # node 'rtabmap' harus muncul
ros2 topic hz /map                     # occupancy grid ke-publish
ros2 topic echo /rtabmap/info --once   # cek loop closure / WM size
```
Di RViz (`rviz:=true`, default): tambah display `Map` (`/map`), `TF`, dan
`PointCloud2` (`/rtabmap/cloud_map`) untuk lihat progres mapping live.

---

## 5. RTABMap Viewer (buka hasil mapping setelah selesai)

Sudah terinstall di Orin (dicek 2026-08-31, `ros-humble-rtabmap` 0.22.1) —
tidak perlu install apa pun tambahan:
```bash
which rtabmap rtabmap-databaseViewer
# /opt/ros/humble/bin/rtabmap
# /opt/ros/humble/bin/rtabmap-databaseViewer
```

Buka database hasil mapping (offline, tanpa perlu robot/sensor nyala):
```bash
source /opt/ros/humble/setup.bash
rtabmap-databaseViewer ~/.ros/husky_rtabmap.db     # hasil opsi B (folder mandiri)
# atau: rtabmap-databaseViewer ~/.ros/rtabmap.db   # hasil opsi A (default run.sh)
```
Database Viewer bisa tampilkan point cloud 3D, graph pose, dan replay frame-by-frame
— cocok untuk demo hasil ke pembimbing tanpa perlu robot fisik nyala lagi.

---

## 6. Argumen launch lengkap (`ros2 launch ... --show-args`)

```
rviz              (default: true)     — buka RViz otomatis
use_rtabmap_viz   (default: false)    — rtabmap_viz (berat, biasanya skip)
db_path           (default: ~/.ros/rtabmap.db)
use_ekf           (default: false)    — pakai EKF custom vs odom filtered langsung
odom_topic        (default: /a200_1060/platform/odom/filtered)
localization      (default: false)    — false=mapping, true=localize
use_amcl          (default: false)    — true kalau digabung dengan AMCL/Nav2
use_lidar         (default: true)
use_camera        (default: true)
```
