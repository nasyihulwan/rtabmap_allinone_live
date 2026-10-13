#!/usr/bin/env python3
"""
scene_zone_to_keepout_mask.py — Konversi scene-zone grid (label kategori) jadi
mask keepout format map_server (PGM+YAML), siap dipakai nav2_costmap_2d::KeepoutFilter.

Kenapa perlu: costmap Nav2 (static_layer dari /map RTABMap) sama sekali tidak tahu
soal kategori scene_zone (corridor/open_area/dst) — dia cuma tahu occupied/free/
unknown dari lidar. Robot bisa "aneh" merencanakan jalur lewat area cream
(open_area, label 2) yang sebenarnya cuma UNCOVERED/belum terverifikasi saat
scanning, bukan ruang kosong asli. Mask ini menandai area itu (+wall+unknown)
sebagai KEEPOUT (occupied) supaya planner_server tidak pernah lewat situ.

Walkable (jadi FREE di mask): corridor(1), doorway(3), staircase(4), workshop(6).
Doorway WAJIB free di sini (beda dari room_zone_lookup.py yang justru exclude
doorway supaya jadi batas ruangan) — robot harus bisa fisik lewat pintu.
Keepout (jadi OCCUPIED di mask): unknown(0), open_area/cream(2), wall(5).

Output: PGM (P5 binary) + YAML format map_server standar (origin/resolution
sama persis dgn input scene_zone yaml, sudah divalidasi align dgn lab_map.yaml)
— dipakai langsung oleh keepout_mask_server (lihat launch/nav2_husky.launch.py).
Plus PNG berwarna (hijau=free/walkable, merah=keepout) buat review manual —
PGM P5 binary gak semua image viewer bisa buka langsung.
"""

import argparse
import os
import numpy as np
import yaml
from PIL import Image

_FREE_LABELS = {1, 3, 4, 6}  # corridor, doorway, staircase, workshop


def convert(zone_yaml_path: str, out_path: str):
    with open(zone_yaml_path) as f:
        meta = yaml.safe_load(f)

    img_path = os.path.join(os.path.dirname(zone_yaml_path), meta['image'])
    label_grid = np.array(Image.open(img_path))

    # map_server PGM: 255=free, 0=occupied (dgn negate:0, occupied_thresh:0.65,
    # free_thresh:0.25 default) — row 0 = atas, sama persis orientasi PNG scene_zone.
    mask = np.where(np.isin(label_grid, list(_FREE_LABELS)), 255, 0).astype(np.uint8)

    out_pgm = out_path + '.pgm'
    out_yaml = out_path + '.yaml'

    height, width = mask.shape
    with open(out_pgm, 'wb') as f:
        f.write(f"P5\n{width} {height}\n255\n".encode('ascii'))
        f.write(mask.tobytes())

    yaml_out = {
        'image': os.path.basename(out_pgm),
        'resolution': float(meta['resolution']),
        'origin': list(meta['origin']),
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
        'mode': 'trinary',
    }
    with open(out_yaml, 'w') as f:
        yaml.safe_dump(yaml_out, f, default_flow_style=False)

    # Preview PNG berwarna — bukan dipakai runtime (KeepoutFilter baca .pgm/
    # .yaml di atas), murni buat review visual manusia lebih gampang drpd PGM.
    out_png = out_path + '.png'
    preview = np.zeros((height, width, 3), dtype=np.uint8)
    preview[mask == 255] = (60, 179, 75)    # hijau: free/walkable
    preview[mask == 0] = (220, 50, 50)      # merah: keepout/occupied
    Image.fromarray(preview, mode='RGB').save(out_png)

    n_free = int(np.count_nonzero(mask == 255))
    n_occ = int(np.count_nonzero(mask == 0))
    print(f"[scene_zone_to_keepout_mask] {out_pgm}")
    print(f"[scene_zone_to_keepout_mask] {out_yaml}")
    print(f"[scene_zone_to_keepout_mask] {out_png}")
    print(f"free={n_free} occupied={n_occ} ({100.0*n_occ/(n_free+n_occ):.1f}% keepout)")


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--zone-yaml', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'output', 'scene_zone_test', 'scene_zone_final_v9_final.yaml'))
    ap.add_argument('--out', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'output', 'scene_zone_test', 'keepout_mask'))
    args = ap.parse_args()
    convert(args.zone_yaml, args.out)
