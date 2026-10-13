# Dokumentasi: Navigasi Fisik Husky Pertama Berhasil
**Tanggal:** 2026-07-08 (Rabu), 13:29 WIB
**Mesin:** kmp-orin (AGX Orin di Husky A200)

## Ringkasan
Goal navigasi `NavigateToPose` dikirim ke robot fisik (bukan simulasi/preview) — robot bergerak maju ~50cm dan mencapai tujuan dengan bersih, tanpa recovery, tanpa error.

## Parameter Goal
- Goal dikirim dalam frame robot sendiri (`a200_1060/base_link`), bukan `map` — supaya "50cm ke depan" relatif terhadap posisi/heading robot saat itu, bukan koordinat absolut yang harus dihitung manual.
- Target: `x=0.5m, y=0.0m` (lurus ke depan), `z=0.0`, orientasi identity (tetap menghadap arah yang sama).

## Hasil (data mentah dari action feedback)

| | Sebelum | Sesudah |
|---|---|---|
| Posisi (map frame) | x=-0.274, y=1.416 | x=0.297, y=1.825 |
| Waktu | — | navigation_time ≈ 6.5 detik |

- **Status akhir:** `SUCCEEDED`
- **number_of_recoveries:** `0` (tidak ada retry/recovery sama sekali — eksekusi bersih)
- **distance_remaining di akhir:** `0.0` (tepat sampai tujuan)

## Konteks — kenapa ini penting
Ini adalah goal navigasi FISIK pertama yang berhasil bersih di robot Husky asli hari ini, setelah rangkaian perbaikan panjang:
1. Custom BT smoothing (dari sesi 2026-07-07) ternyata buggy di robot fisik — bikin `cmd_vel` gak pernah terpublish. Di-revert ke BT bawaan nav2.
2. AMCL message-queue overflow — root cause: clock/timestamp gak konsisten di beberapa node long-running (dipicu percobaan clock-sync manual). Fix: restart semua node TF-touching setelah clock settle.
3. `obstacle_stop_node.py` — bug performa (parsing point cloud per-titik pakai Python loop) bikin CPU 127% sustained, jadi salah satu penyumbang instability sistem. Di-fix pakai numpy vectorization → turun ke ~20% CPU.

Detail lengkap semua perbaikan ada di `summary_context/2026-07-08_rabu.md`.

## Cara reproduce
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'a200_1060/base_link'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```
(Goal dalam frame `a200_1060/base_link` = relatif terhadap robot, bukan koordinat map absolut — Nav2 otomatis resolve ke posisi map yang benar.)
