#!/usr/bin/env python3
"""
reid_core.py — Logika akumulasi posisi & re-identifikasi (gating + merge).

[E2] SATU-SATUNYA implementasi mekanisme ini di seluruh pipeline. Dipakai
oleh node ROS (semantic_map_exporter.py, live) DAN skrip replay offline
(position_log_analyzer.py, dari CSV) — TANPA modul ini, keduanya akan
menulis ulang logika yang "seharusnya sama" secara terpisah, dan cepat
atau lambat berbeda perilaku tanpa disadari. Kesetiaan replay (Uji Terima
H2) bergantung mutlak pada dua pemakai ini memanggil kode yang sama.

Tidak ada dependency ROS sama sekali di modul ini — murni numpy.

Kesetaraan matematis granularitas titik vs deteksi:
  mean(semua titik individual suatu instance)
    == mean-berbobot(mean setiap deteksi, bobot = n_points deteksi itu)
  (linearitas rata-rata: sum(titik) = sum atas deteksi dari (mean_deteksi *
  n_points_deteksi)). Karena itu, add_detection() cukup diberi (x_mean,
  y_mean, n_points) SATU KALI per deteksi per frame — hasil akhirnya
  identik dengan mengakumulasi tiap titik individual satu-satu. Ini yang
  membuat replay dari position_log.csv (satu baris = satu deteksi, sudah
  dirata-ratakan) sah mereproduksi hasil live yang aslinya mengakumulasi
  per-titik.
"""

import numpy as np


def find_reid_candidate(new_x: float, new_y: float, cid: int,
                         instances: dict, provisional_keys: set,
                         merge_dist: float, exclude_key,
                         reid_enabled: bool = True):
    """
    Cari instance confirmed same-class terdekat dalam merge_dist.

    instances: {key: {'class_id': int, 'xs': [...], 'ys': [...]}}
    provisional_keys: key yang belum cukup data untuk jadi kandidat —
        termasuk instance baru lain di frame yang sama, supaya dua
        deteksi baru yang kebetulan berdekatan tidak saling merge.

    Returns (best_key, best_dist) — best_key None jika tidak ada kandidat
    (atau reid_enabled=False, dipakai untuk mode baseline-naif).
    """
    if not reid_enabled:
        return None, float('inf')

    best_key, best_dist = None, float('inf')
    for existing_key, ex_entry in instances.items():
        if existing_key == exclude_key:
            continue
        if ex_entry['class_id'] != cid:
            continue
        if existing_key in provisional_keys:
            continue

        ex_x = float(np.mean(ex_entry['xs']))
        ex_y = float(np.mean(ex_entry['ys']))
        dist = float(np.hypot(new_x - ex_x, new_y - ex_y))

        if dist < merge_dist and dist < best_dist:
            best_dist = dist
            best_key  = existing_key

    return best_key, best_dist


class ReidAccumulator:
    """
    Akumulasi posisi per instance + keputusan merge/re-id berbasis posisi
    world-frame. Granularitas: satu add_detection() per deteksi per frame
    (bukan per titik individual, bukan per instance/sesi — lihat A2/A3
    checklist).

    Pemakaian tipikal (live maupun replay):
        acc = ReidAccumulator(merge_dist=1.5, min_points=20, reid_enabled=True)
        for det in detections_pada_frame_ini:
            acc.add_detection(det.class_id, det.tracker_id, det.x, det.y, det.n_points)
        events = acc.promote_ready()   # dipanggil SEKALI per frame, bukan per deteksi
    """

    def __init__(self, merge_dist: float, min_points: int, reid_enabled: bool = True):
        self.merge_dist    = merge_dist
        self.min_points    = min_points
        self.reid_enabled  = reid_enabled
        self.instances: dict     = {}   # key -> {'class_id': int, 'xs': [...], 'ys': [...]}
        self.key_remap: dict     = {}   # new_key -> canonical_key
        self.provisional_keys: set = set()
        # [bugfix] Counter negatif untuk key hasil seed_instance() — lihat
        # docstring seed_instance soal kenapa ini krusial untuk kebenaran.
        self._next_seed_id = -1

    def seed_instance(self, class_id: int, x: float, y: float, weight: int,
                       tracker_id: int = None):
        """
        [fix5] Reconstruct instance dari state lama (mis. semantic_map.yaml
        saat restart, atau snapshot sesi sebelumnya saat replay multi-sesi)
        — langsung CONFIRMED (bukan provisional), direplikasi `weight`
        synthetic point di posisi mean.

        [BUG KRITIS yang diperbaiki di sini] tracker_id TIDAK BOLEH memakai
        angka mentah dari sesi lama. SimpleIoUTracker (yolo_detector.py)
        me-reset next_id ke 1 setiap node restart — jadi objek PERTAMA yang
        terdeteksi di sesi baru HAMPIR PASTI dapat tracker_id=1 lagi. Kalau
        seed_instance memakai tracker_id mentah itu sebagai key, instance
        lama (mis. key=(class_id,1) dari sesi sebelumnya) akan COLLIDE
        persis dengan key deteksi baru — dan add_detection() akan diam-diam
        MENIMPA/MENGGABUNG keduanya lewat pencocokan key literal, BUKAN
        lewat find_reid_candidate (perbandingan jarak). Untuk kasus Fase K1
        (satu objek, restart berulang) ini nyaris SELALU terjadi karena
        objek pertama yang terdeteksi tiap sesi lazimnya dapat tracker_id=1
        — sehingga "keberhasilan" re-id yang teramati sebenarnya cuma
        tabrakan angka, BUKAN pencocokan posisi yang sedang diuji.

        Solusi: default tracker_id=None -> pakai counter NEGATIF internal,
        dijamin tidak akan pernah collide dengan tracker_id live asli
        (SimpleIoUTracker selalu di rentang 1..254). Setiap deteksi baru
        WAJIB melalui provisional -> promote_ready() -> find_reid_candidate,
        tidak ada jalan pintas lewat key literal.

        Returns key yang dipakai (untuk caller yang perlu tahu, mis. label YAML).
        """
        if tracker_id is None:
            tracker_id = self._next_seed_id
            self._next_seed_id -= 1
        key = (class_id, tracker_id)
        self.instances[key] = {
            'class_id': class_id,
            'xs': [x] * weight,
            'ys': [y] * weight,
        }
        return key

    def add_detection(self, class_id: int, tracker_id: int,
                       x: float, y: float, n_points: int = 1):
        """
        Tambah satu observasi deteksi. TIDAK memicu pengecekan re-id di sini
        — pengecekan dilakukan terpisah lewat promote_ready() setelah semua
        deteksi pada frame yang sama selesai ditambahkan (supaya urutan
        pemrosesan sesama deteksi baru di frame yang sama tidak memengaruhi
        satu sama lain, lihat find_reid_candidate).

        Returns key aktual (setelah remap) — dipakai caller untuk logging.
        """
        key = (class_id, tracker_id)
        key = self.key_remap.get(key, key)

        if key not in self.instances:
            self.instances[key] = {'class_id': class_id, 'xs': [], 'ys': []}
            self.provisional_keys.add(key)

        self.instances[key]['xs'].extend([x] * n_points)
        self.instances[key]['ys'].extend([y] * n_points)
        return key

    def promote_ready(self) -> dict:
        """
        Cek semua provisional key yang sudah cukup data (>= min_points),
        jalankan re-id, promosikan jadi confirmed (merged ATAU instance
        baru). Dipanggil SEKALI per frame setelah seluruh add_detection()
        frame itu selesai — bukan per add_detection().

        [D1] Iterasi provisional_keys via sorted(), BUKAN urutan asli set
        Python (tidak dijamin stabil). Ini penting: kalau >1 key crossing
        min_points di frame yang sama, urutan pemrosesan memengaruhi hasil
        (key yang diproses lebih dulu bisa jadi kandidat merge untuk key
        berikutnya). sorted() membuat urutan ini deterministik dan sama
        persis antara live run dan replay run.

        Returns {key: {'merged': bool, 'target': key_or_None, 'dist': float_or_None}}
        untuk key yang diproses pada panggilan ini (dipakai caller untuk
        logging/timing — len(return) == n_checked).
        """
        events = {}
        promoted = set()
        for key in sorted(self.provisional_keys):
            if key not in self.instances:
                # Sudah di-merge ke instance lain di iterasi sebelumnya
                promoted.add(key)
                continue
            if len(self.instances[key]['xs']) >= self.min_points:
                merged, target, dist = self._try_reidentify(key)
                events[key] = {'merged': merged, 'target': target, 'dist': dist}
                promoted.add(key)
        self.provisional_keys -= promoted
        return events

    def _try_reidentify(self, new_key: tuple):
        """
        Returns (merged: bool, target_key_or_None, dist_or_None).
        """
        cid, _iid = new_key
        entry = self.instances[new_key]
        new_x = float(np.mean(entry['xs']))
        new_y = float(np.mean(entry['ys']))

        best_key, best_dist = find_reid_candidate(
            new_x, new_y, cid, self.instances, self.provisional_keys,
            self.merge_dist, new_key, self.reid_enabled)

        if best_key is None:
            return False, None, None

        self.instances[best_key]['xs'].extend(entry['xs'])
        self.instances[best_key]['ys'].extend(entry['ys'])
        del self.instances[new_key]
        self.key_remap[new_key] = best_key
        return True, best_key, best_dist

    @property
    def n_confirmed(self) -> int:
        return len(self.instances) - len(self.provisional_keys)

    def snapshot(self, min_points: int = None) -> dict:
        """
        {key: {'class_id', 'x' (mean), 'y' (mean), 'cell_count'}} untuk
        seluruh instance yang tercatat. min_points (default: self.min_points)
        memfilter instance yang belum cukup data — sama seperti perilaku
        _export_yaml lama.
        """
        threshold = self.min_points if min_points is None else min_points
        out = {}
        for key, entry in self.instances.items():
            if len(entry['xs']) < threshold:
                continue
            out[key] = {
                'class_id':   entry['class_id'],
                'x':          float(np.mean(entry['xs'])),
                'y':          float(np.mean(entry['ys'])),
                'cell_count': len(entry['xs']),
            }
        return out
