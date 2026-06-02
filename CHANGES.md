# CHANGES — Bag Baru 2026-04-15

Dokumen ini merangkum semua perubahan yang dilakukan untuk menyesuaikan
pipeline dengan rosbag baru `rosbag2_2026_04_15-08_39_23`.

---

## Ringkasan Perbedaan Bag

| Aspek              | Bag Lama (2026-04-10)              | Bag Baru (2026-04-15)                                         |
|--------------------|------------------------------------|---------------------------------------------------------------|
| Odometry           | tidak ada `/odom/filtered` di bag  | `/a200_1060/platform/odom/filtered` sudah terekam             |
| Color image        | `/camera/color/image_raw` (raw)    | `/camera/camera/color/image_raw/compressed`                   |
| Depth image        | `/camera/depth/image_rect_raw`     | `/camera/camera/aligned_depth_to_color/image_raw/compressedDepth` |
| Camera info color  | `/camera/color/camera_info`        | `/camera/camera/color/camera_info`                            |
| Camera info depth  | `/camera/depth/camera_info`        | `/camera/camera/aligned_depth_to_color/camera_info`           |
| IMU                | belum tentu terekam                | `/a200_1060/sensors/imu_0/data`                               |
| EKF                | perlu dijalankan live              | sudah ada di bag, tidak perlu run live                        |

---

## File yang Diubah

### 1. `run.sh`

- **BAG_PATH** diubah ke path bag baru
- **Default ODOM_TOPIC** diubah ke `/a200_1060/platform/odom/filtered`
- **Default USE_EKF** diubah ke `false` (odom/filtered sudah di bag)

Cara jalankan bag baru (cukup default):
```bash
./run.sh
./run.sh 0.3        # rate lebih lambat
./run.sh 0.5 localize  # mode localization
```

### 2. `launch/rtabmap_rosbag.launch.py`

Perubahan utama:

**A. Tambah 4 node baru (decompress + relay):**
- `decompress_color` — republish compressed color → raw
- `decompress_depth` — republish compressedDepth aligned → raw
- `relay_color_info` — relay camera_info dari namespace baru ke namespace lama
- `relay_depth_info` — relay depth camera_info
- `relay_odom` — relay odom filtered ke `/odom` (opsional)

**B. Default argumen diupdate:**
- `use_ekf` default → `false`
- `odom_topic` default → `/a200_1060/platform/odom/filtered`
- `bag_path` default → path bag baru

**C. Topic di `--topics` bag play diupdate:**
- Menambahkan semua topic baru dari bag 2026-04-15
- Menghapus topic yang tidak ada di bag baru

### 3. `launch/semantic_only.launch.py`

Tidak ada perubahan topic di file ini — karena semantic nodes
subscribe ke topik yang sudah menjadi output dari decompress nodes
di Terminal 1 (`/camera/color/image_raw`, `/camera/depth/image_rect_raw`).

---

## Cara Jalankan

```bash
# Terminal 1 — RTABMap + rosbag
./run.sh

# Tunggu stabil (~10-15 detik, lihat map terbentuk di RViz)

# Terminal 2 — Semantic nodes
./run_semantic.sh

# Terminal 3 — Capture hasil
python3 capture_semantic_map.py
```

---

## Sanity Check

Setelah Terminal 1 jalan, verifikasi dengan:

```bash
# Cek decompress berhasil
ros2 topic hz /camera/color/image_raw          # ~30 Hz
ros2 topic hz /camera/depth/image_rect_raw     # ~30 Hz

# Cek odom tersedia
ros2 topic hz /a200_1060/platform/odom/filtered  # ~50 Hz

# Cek semua topic semantic tersedia
ros2 topic list | grep semantic
```

Atau jalankan script sanity check:
```bash
python3 sanity_check.py
```

---

## Catatan Teknis

### Kenapa `use_ekf=false`?
Bag baru merekam langsung dari robot saat EKF sudah aktif. Artinya
`/a200_1060/platform/odom/filtered` sudah berisi odometri yang
sudah di-fuse antara wheel encoder + IMU. Tidak perlu menjalankan
EKF ulang.

### Kenapa `aligned_depth_to_color`?
Bag baru sudah menyimpan depth yang sudah di-align ke frame kamera
warna. Ini lebih akurat dan menghindari kebutuhan alignment manual.
Ukurannya lebih kecil dari depth raw (3378 frame vs 6495 frame)
karena sudah di-downsample ke resolusi color.

### ICP Odometry
Di launch file baru, `icp_odometry` node di-comment. Ini karena
kita sudah punya odom yang bagus dari bag. Kalau ingin eksperimen
dengan ICP odom dari LiDAR, uncomment blok tersebut di
`rtabmap_rosbag.launch.py`.
