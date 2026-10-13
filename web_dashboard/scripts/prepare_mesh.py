#!/usr/bin/env python3
"""
prepare_mesh.py — Ekstrak MESH 3D (bukan point cloud sparse) dari
sementara/rtabmap.db utk layer "peta 3D" di web dashboard.

Menggantikan prepare_pointcloud.py: point cloud (voxel-filtered, titik
terpisah) kelihatan "dikit-dikit"/berlubang di browser krn spacing antar
titik (voxel 0.1m) lebih lebar drpd ukuran titik yang wajar dirender. Mesh
(triangulasi permukaan) ngisi celah itu jadi permukaan solid kontinu —
jauh lebih mirip rekonstruksi asli.

Sama seperti prepare_pointcloud.py: pakai CLI resmi `rtabmap-export --mesh`
(bukan hand-parse binary internal RTABMap). Proses ini JAUH lebih berat
drpd --cloud (rekonstruksi mesh per-keyframe + merge + color transfer,
~5-6 menit utk db lab BRIN 241 keyframe) — jalankan di background/expect
nunggu lama, JANGAN diulang-ulang tanpa perlu.

Usage:
    python3 scripts/prepare_mesh.py
        (baca sementara/rtabmap.db, tulis public/map/mesh.bin +
         mesh_meta.json)
"""
import json
import os
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SEMENTARA = os.path.join(HERE, '..', 'sementara')
OUT_DIR = os.path.join(HERE, '..', 'public', 'map')
DB_PATH = os.path.join(SEMENTARA, 'rtabmap.db')
RTABMAP_EXPORT = '/opt/ros/humble/bin/rtabmap-export'

# voxel 0.05m (depth decimation 4) -> ~138rb vertex / ~187rb face dari db lab
# BRIN (divalidasi manual) — solid tapi tetap ringan (~4-5MB binary). Voxel
# lebih kecil = mesh lebih detail TAPI proses jauh lebih lama (voxel 0.03
# sempat timeout >2 menit) dan file lebih besar.
#
# [2026-07-14] Sempat dicoba --noise_radius/--noise_k (filter outlier point
# sebelum meshing) buat hilangin "shredded paper"/pecah2 di tepi dinding —
# HASIL: dengan radius/k moderat (0.1/8) nyaris gak ada bedanya scr visual;
# dengan radius/k lebih agresif (0.15/10 + voxel 0.1 lebih kasar) JUSTRU
# BUANG SEBAGIAN BESAR mesh (188rb->34rb face, sisa cuma "sliver" tipis).
# KESIMPULAN: jaggedness ini kemungkinan besar keterbatasan data sumber
# (depth noise RealSense + registrasi antar-frame gak sempurna), BUKAN
# sesuatu yg bisa dibersihkan lewat flag CLI rtabmap-export — sesuai
# temuan sesi sebelumnya soal coverage kamera yg terbatas dibanding lidar.
# --min_cluster -1 (buang cluster kecil selain yg terbesar) DIPERTAHANKAN
# krn gak merusak apa2 (efeknya minimal tapi aman), noise_radius/noise_k
# SENGAJA TIDAK dipakai lagi (default 0/disabled).
VOXEL_SIZE = 0.05
DECIMATION = 4
MAX_RANGE = 6.0
MIN_CLUSTER = -1  # -1 = keep only biggest contiguous surface


def run_rtabmap_export_mesh(tmpdir: str) -> str:
    if not os.path.exists(RTABMAP_EXPORT):
        sys.exit(f"rtabmap-export tidak ditemukan di {RTABMAP_EXPORT} — perlu ros-humble-rtabmap-slam.")
    cmd = [
        RTABMAP_EXPORT, '--mesh',
        '--voxel', str(VOXEL_SIZE),
        '--decimation', str(DECIMATION),
        '--max_range', str(MAX_RANGE),
        '--min_cluster', str(MIN_CLUSTER),
        '--ascii',
        '--output', 'mesh',
        '--output_dir', tmpdir,
        DB_PATH,
    ]
    print('[prepare_mesh]', ' '.join(cmd))
    print('[prepare_mesh] Proses mesh reconstruction ~5-6 menit, mohon tunggu...')
    subprocess.run(cmd, check=True)
    ply_path = os.path.join(tmpdir, 'mesh_mesh.ply')
    if not os.path.exists(ply_path):
        sys.exit(f"rtabmap-export tidak menghasilkan {ply_path} (cek log di atas).")
    return ply_path


def parse_ascii_ply_mesh(path: str):
    """Parse element vertex (x,y,z,red,green,blue,...) + element face (list uchar int vertex_indices)."""
    with open(path, 'r') as f:
        n_vertex = None
        n_face = None
        vertex_props = []
        cur_element = None
        while True:
            line = f.readline().strip()
            if line == 'end_header':
                break
            if line.startswith('element'):
                parts = line.split()
                cur_element = parts[1]
                if cur_element == 'vertex':
                    n_vertex = int(parts[2])
                elif cur_element == 'face':
                    n_face = int(parts[2])
            elif line.startswith('property') and cur_element == 'vertex':
                vertex_props.append(line.split()[-1])

        idx = {name: i for i, name in enumerate(vertex_props)}
        for required in ('x', 'y', 'z', 'red', 'green', 'blue'):
            if required not in idx:
                sys.exit(f"PLY tidak punya field '{required}' — cek versi rtabmap-export.")

        xs = [0.0] * n_vertex
        ys = [0.0] * n_vertex
        zs = [0.0] * n_vertex
        rs = [0] * n_vertex
        gs = [0] * n_vertex
        bs = [0] * n_vertex
        for v in range(n_vertex):
            parts = f.readline().split()
            xs[v] = float(parts[idx['x']])
            ys[v] = float(parts[idx['y']])
            zs[v] = float(parts[idx['z']])
            rs[v] = int(parts[idx['red']])
            gs[v] = int(parts[idx['green']])
            bs[v] = int(parts[idx['blue']])

        faces = []
        for _ in range(n_face):
            parts = f.readline().split()
            count = int(parts[0])
            if count != 3:
                continue  # skip non-triangle (jarang/gak ada di output rtabmap-export, tapi jaga-jaga)
            faces.append((int(parts[1]), int(parts[2]), int(parts[3])))

    return xs, ys, zs, rs, gs, bs, faces


def write_binary(out_path: str, xs, ys, zs, rs, gs, bs, faces):
    n_vertex = len(xs)
    n_face = len(faces)
    with open(out_path, 'wb') as f:
        f.write(struct.pack('<II', n_vertex, n_face))
        for i in range(n_vertex):
            f.write(struct.pack('<fff', xs[i], ys[i], zs[i]))
        for i in range(n_vertex):
            f.write(struct.pack('<BBB', rs[i], gs[i], bs[i]))
        for (a, b, c) in faces:
            f.write(struct.pack('<III', a, b, c))


def main():
    if not os.path.exists(DB_PATH):
        sys.exit(f"Tidak ditemukan: {DB_PATH}")
    os.makedirs(OUT_DIR, exist_ok=True)

    with tempfile.TemporaryDirectory() as tmpdir:
        ply_path = run_rtabmap_export_mesh(tmpdir)
        xs, ys, zs, rs, gs, bs, faces = parse_ascii_ply_mesh(ply_path)

    n_vertex = len(xs)
    n_face = len(faces)
    out_bin = os.path.join(OUT_DIR, 'mesh.bin')
    write_binary(out_bin, xs, ys, zs, rs, gs, bs, faces)

    meta = {
        'demo': True,
        'note': ('Mesh 3D (triangulasi, permukaan solid) dari sementara/rtabmap.db '
                 f'(voxel {VOXEL_SIZE}m). Frame "map" ROS (z=atas), SAMA dgn pose '
                 'robot/objek lain di dashboard — konversi ke Three.js (rosToThree) '
                 'dilakukan di viewer. Gantikan pointcloud.bin (sparse points) yg '
                 'sebelumnya kelihatan "dikit-dikit" krn spacing antar titik.'),
        'vertexCount': n_vertex,
        'faceCount': n_face,
        'voxelSize': VOXEL_SIZE,
        'bounds': {
            'minX': min(xs), 'maxX': max(xs),
            'minY': min(ys), 'maxY': max(ys),
            'minZ': min(zs), 'maxZ': max(zs),
        },
        'format': ('uint32 vertexCount, uint32 faceCount, then vertexCount*float32[3] xyz, '
                    'then vertexCount*uint8[3] rgb, then faceCount*uint32[3] triangle indices '
                    '(all little-endian)'),
    }
    with open(os.path.join(OUT_DIR, 'mesh_meta.json'), 'w') as f:
        json.dump(meta, f, indent=2)

    size_mb = os.path.getsize(out_bin) / 1e6
    print(f'[prepare_mesh] {n_vertex} vertex, {n_face} face -> {out_bin} ({size_mb:.1f} MB)')
    print(f'[prepare_mesh] bounds: {meta["bounds"]}')


if __name__ == '__main__':
    main()
