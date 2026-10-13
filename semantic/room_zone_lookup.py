#!/usr/bin/env python3
"""
room_zone_lookup.py — Lookup ruangan (connected-component) dari scene-zone grid.

Beda dari label kategori mentah (corridor/open_area/dst) di scene_zone_final_v9:
grid itu cuma kasih KATEGORI, bukan ID ruangan individual — dua "workshop"
yang terpisah tembok tetap dapat kategori yang sama. Modul ini jalankan
connected-component labeling di atas kategori "walkable" (corridor, open_area,
staircase, workshop), dengan wall/doorway/unknown sebagai batas — jadi tiap
ruangan fisik yang terpisah tembok/pintu dapat room_id unik.

Dipakai object_goal_nav.py untuk validasi: titik goal hasil offset harus
berada di ruangan yang SAMA dengan objek target, bukan cuma "1.5m ke arah
robot" secara garis lurus tanpa peduli tembok di antaranya.
"""

import os
import numpy as np
import yaml
from scipy import ndimage
from PIL import Image

# Kategori yang dianggap "walkable" (bagian dari suatu ruangan).
# wall/doorway/unknown SENGAJA jadi batas — doorway jadi batas supaya dua
# sisi pintu dianggap ruangan berbeda (sesuai kebutuhan: goal gak boleh
# nyebrang pintu tanpa sadar).
# 'open_area' (label 2, warna cream RGB~230,215,150 di versi colored)
# SENGAJA DIKELUARKAN dari walkable — itu bukan area terbuka yang
# terverifikasi, tapi area yang TIDAK TERCOVERAGE saat scanning/klasifikasi
# (fallback label, bukan konfirmasi ruang kosong asli). Asumsi aman:
# jangan anggap bisa dilewati.
_WALKABLE_LABELS = {1, 4, 6}  # corridor, staircase, workshop


class RoomZoneMap:
    def __init__(self, yaml_path: str):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)

        self.resolution = float(meta['resolution'])
        self.origin_x, self.origin_y = float(meta['origin'][0]), float(meta['origin'][1])
        self.label_map = meta.get('label_map', {})

        img_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
        self.label_grid = np.array(Image.open(img_path))  # (height, width), row 0 = atas
        self.height, self.width = self.label_grid.shape

        walkable_mask = np.isin(self.label_grid, list(_WALKABLE_LABELS))
        # 8-connectivity: dua sel diagonal-bersebelahan tetap dianggap nyambung
        structure = np.ones((3, 3), dtype=bool)
        self.room_grid, self.n_rooms = ndimage.label(walkable_mask, structure=structure)

    def world_to_pixel(self, wx: float, wy: float):
        """-> (img_row, img_col) atau None kalau di luar peta."""
        col = int((wx - self.origin_x) / self.resolution)
        row_from_bottom = int((wy - self.origin_y) / self.resolution)
        img_row = self.height - 1 - row_from_bottom
        img_col = col
        if 0 <= img_row < self.height and 0 <= img_col < self.width:
            return img_row, img_col
        return None

    def get_room_id(self, wx: float, wy: float) -> int:
        """-> room_id (>0) kalau di area walkable, 0 kalau di wall/doorway/unknown/luar peta."""
        px = self.world_to_pixel(wx, wy)
        if px is None:
            return 0
        return int(self.room_grid[px])

    def get_room_id_nearest_walkable(self, wx: float, wy: float, search_radius_m: float = 0.5) -> int:
        """
        Sama seperti get_room_id, tapi kalau titik persis jatuh di sel non-walkable
        (mis. sedikit meleset ke wall/doorway krn resolusi grid), cari sel walkable
        terdekat dalam radius search_radius_m dan pakai room_id-nya.
        """
        room_id = self.get_room_id(wx, wy)
        if room_id > 0:
            return room_id

        px = self.world_to_pixel(wx, wy)
        if px is None:
            return 0
        img_row, img_col = px
        radius_px = max(1, int(search_radius_m / self.resolution))

        best_id, best_dist = 0, None
        r0, r1 = max(0, img_row - radius_px), min(self.height, img_row + radius_px + 1)
        c0, c1 = max(0, img_col - radius_px), min(self.width, img_col + radius_px + 1)
        for r in range(r0, r1):
            for c in range(c0, c1):
                rid = int(self.room_grid[r, c])
                if rid == 0:
                    continue
                d = (r - img_row) ** 2 + (c - img_col) ** 2
                if best_dist is None or d < best_dist:
                    best_dist, best_id = d, rid
        return best_id

    def same_room(self, wx1: float, wy1: float, wx2: float, wy2: float) -> bool:
        r1 = self.get_room_id_nearest_walkable(wx1, wy1)
        r2 = self.get_room_id_nearest_walkable(wx2, wy2)
        return r1 != 0 and r1 == r2
