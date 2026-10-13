#!/usr/bin/env python3
"""
room_label_tool.py — Render overlay nomor room_id utk labeling manual.

Bukan tool interaktif/GUI — jumlah ruangan kecil (~10-30), gak sepadan bikin
labeling tool. Cukup 1 gambar statis: nomor room_id ditulis di centroid tiap
ruangan, ditumpuk di atas <..._colored.png> yang sudah ada (visualisasi
kategori corridor/workshop/dst dari --mode geometry-final). User buka gambar
ini di image viewer, lalu tulis tangan file room_labels_<versi>.yaml:
{room_id: "nama ruangan", ...} — room_type_classifier.py join ke sini via
room_id, tidak butuh format/tool baru.

Reuse: RoomZoneMap (room_zone_lookup.py) utk room_grid per ruangan.
"""

import os
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from room_zone_lookup import RoomZoneMap


def render_room_id_overlay(zone_final_yaml_path: str, colored_png_path: str,
                            out_png_path: str) -> int:
    """-> jumlah room yang diberi label angka."""
    room_map = RoomZoneMap(zone_final_yaml_path)
    base = Image.open(colored_png_path).convert('RGB')
    if base.size != (room_map.width, room_map.height):
        raise ValueError(
            f'Ukuran {colored_png_path} ({base.size}) tidak cocok dgn '
            f'{zone_final_yaml_path} ({room_map.width}x{room_map.height}) — '
            f'pastikan _colored.png dan _final.yaml dari versi yang sama.')

    draw = ImageDraw.Draw(base)
    room_ids = sorted(int(rid) for rid in np.unique(room_map.room_grid) if rid > 0)

    for room_id in room_ids:
        row, col = ndimage.center_of_mass(room_map.room_grid == room_id)
        text = str(room_id)
        # outline hitam + isi putih supaya kebaca di background warna apapun
        for dx, dy in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
            draw.text((col + dx, row + dy), text, fill=(0, 0, 0))
        draw.text((col, row), text, fill=(255, 255, 255))

    os.makedirs(os.path.dirname(out_png_path) or '.', exist_ok=True)
    base.save(out_png_path)
    print(f'[RoomLabelTool] {len(room_ids)} room diberi label -> {out_png_path}')
    return len(room_ids)


def main(args=None):
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--zone-final-yaml', required=True,
                     help='mis. output/scene_zone_test/scene_zone_final_v9_final.yaml')
    ap.add_argument('--colored-png', required=True,
                     help='pasangan visual dari --zone-final-yaml, '
                          'mis. scene_zone_final_v9_colored.png')
    ap.add_argument('--out', default='room_id_overlay.png')
    args = ap.parse_args(args=args)
    render_room_id_overlay(args.zone_final_yaml, args.colored_png, args.out)


if __name__ == '__main__':
    main()
