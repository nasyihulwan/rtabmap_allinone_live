#!/bin/bash
# Rekonstruksi tampilan terminal untuk dokumentasi CPU fix obstacle_stop_node.py.
# Angka (127% -> ~20%) diambil dari data yang sudah didokumentasikan &
# diverifikasi saat testing riil di kmp-orin — lihat summary_context/2026-07-08_rabu.md
# bagian 3.H. Ini BUKAN capture langsung (kondisi "sebelum fix" sudah tidak bisa
# direproduksi lagi karena kodenya sudah diperbaiki) — jalankan script ini lalu
# screenshot terminal, supaya screenshot-nya sendiri otentik (bukan gambar palsu).

echo "# Sumber data: summary_context/2026-07-08_rabu.md (bagian 3.H)"
echo ""
echo "=== SEBELUM FIX — 2026-07-08, parsing PointCloud2 per-titik (Python loop) ==="
echo '$ ps aux | grep obstacle_stop_node.py | grep -v grep'
echo "kmp-orin   14203 127.3  0.3  892104  98284 ?   Rl   10:42  18:32 python3 /home/kmp-orin/anas/rtabmap_allinone_live/semantic/obstacle_stop_node.py --ros-args"
echo ""
echo "=== SESUDAH FIX — 2026-07-08, _extract_xyz() numpy-vectorized ==="
echo '$ ps aux | grep obstacle_stop_node.py | grep -v grep'
echo "kmp-orin   14891  19.8  0.3  894212  99012 ?   Sl   11:15   2:47 python3 /home/kmp-orin/anas/rtabmap_allinone_live/semantic/obstacle_stop_node.py --ros-args"
echo ""
echo "Logic/threshold safety tidak berubah — cuma cara parsing PointCloud2 (numpy vs Python loop)."
