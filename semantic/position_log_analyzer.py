#!/usr/bin/env python3
"""
position_log_analyzer.py — Skrip perhitungan metrik untuk Fase K1/K2
(instrumen penelitian, proposal subbab 3.4.4 & 3.6).

Tidak bergantung pada ROS — murni baca CSV hasil position logger
(semantic_map_exporter.py, param enable_position_logger:=true) dan
hitung statistik. Bisa dijalankan di laptop tanpa ROS ter-source.

Dua mode:
  1. Fase K1 — noise floor characterization
     noise-floor: baca position_log.csv, kelompokkan per session_id,
     hitung deviasi posisi objek statis lintas sesi restart/relokalisasi,
     lalu usulkan T_calibration = D_max + margin (guardbanding,
     JCGM 100:2008 / ILAC G8:09/2019 — lihat subbab 3.4.5 proposal).

  2. Fase K2 — evaluasi ambang batas re-identifikasi
     evaluate_reid(): nearest-neighbor gating (subbab 3.4.6) dijalankan
     terhadap data yang SUDAH dilabeli identitas fisik ground truth
     (subbab 3.4.7), menghasilkan precision/recall/false_merge_rate
     (Tabel 3.7 & 3.8).

  3. Biaya komputasi (Tujuan 6, Tabel 3.8)
     timing-report: baca reid_timing_log.csv (param
     enable_position_logger:=true di semantic_map_exporter.py), hitung
     mean/std/p95/max ms per frame — dibandingkan lintas sesi dengan
     reid_enabled/merge_dist berbeda (baseline naif vs heuristik vs
     kalibrasi) dengan menjalankan perintah ini terpisah per sesi.

Format position_log.csv: lihat POSITION_LOG_FIELDS di semantic_map_exporter.py
Format reid_timing_log.csv: lihat REID_TIMING_FIELDS di semantic_map_exporter.py
Format ground truth CSV (checklist G2): lihat GROUND_TRUTH_FIELDS di bawah —
  session_id, object_uid, class_name, gt_x, gt_y, gt_z, marker_id,
  measured_wall, instrument, note. object_uid: identitas objek FISIK
  permanen dari stiker penanda (checklist G1), BUKAN tracker_id sistem.
  Disiapkan manual oleh peneliti per sesi (lihat CLI subcommand 'record-gt').

[E2] Replay (replay_session/replay_multi_session) dan evaluate_reid() di
modul ini memanggil ReidAccumulator/find_reid_candidate dari reid_core.py
— MODUL YANG SAMA dipakai semantic_map_exporter.py saat live. Tidak ada
reimplementasi gating terpisah di sini. Ini prasyarat mutlak Uji Terima H2
(kesetiaan replay) — tanpa ini, kurva sweep T tidak sah.
"""

import argparse
import csv
import datetime
import os
import itertools
import math
import statistics
import subprocess
from collections import defaultdict

import numpy as np
from scipy.optimize import linear_sum_assignment

from reid_core import ReidAccumulator


# ─────────────────────────────────────────────────────────────────
# I/O
# ─────────────────────────────────────────────────────────────────
def load_position_log(csv_path: str) -> list:
    """Baca position_log.csv → list of dict, kolom numerik sudah di-cast."""
    rows = []
    with open(csv_path, newline='') as f:
        for row in csv.DictReader(f):
            for key in ('frame_seq', 'class_id', 'tracker_id', 'n_points'):
                if row.get(key):
                    row[key] = int(row[key])
            for key in ('x', 'y', 'z', 'timestamp_wall'):
                if row.get(key):
                    row[key] = float(row[key])
            rows.append(row)
    return rows


# [G2] Kolom ground truth persis checklist — object_uid adalah identitas
# FISIK permanen (stiker di objek, checklist G1), TIDAK PERNAH berubah
# sepanjang eksperimen, BUKAN tracker_id sistem. gt_z & marker_id opsional
# (kosong untuk objek yang diukur tanpa marker lantai — checklist G2b:
# L1/L2 dinding cukup diukur sekali, tidak butuh marker).
GROUND_TRUTH_FIELDS = [
    'session_id', 'object_uid', 'class_name',
    'gt_x', 'gt_y', 'gt_z',
    'marker_id', 'measured_wall', 'instrument', 'note',
]


def load_labeled_detections(csv_path: str) -> list:
    """Baca ground-truth CSV (checklist G2) untuk evaluate_reid()."""
    out = []
    with open(csv_path, newline='') as f:
        for row in csv.DictReader(f):
            out.append({
                'session_id':    row.get('session_id', ''),
                'object_uid':    row['object_uid'],
                'class_name':    row['class_name'],
                'x':             float(row['gt_x']),
                'y':             float(row['gt_y']),
                'z':             float(row['gt_z']) if row.get('gt_z') else None,
                'marker_id':     row.get('marker_id', ''),
                'measured_wall': row.get('measured_wall', ''),
                'instrument':    row.get('instrument', ''),
                'note':          row.get('note', ''),
            })
    return out


def append_ground_truth(csv_path: str, session_id: str, object_uid: str,
                         class_name: str, gt_x: float, gt_y: float,
                         gt_z: float = None, marker_id: str = '',
                         measured_wall: str = None, instrument: str = '',
                         note: str = '') -> None:
    """
    [G2/3.4.7] Tambah satu baris ground truth hasil pengukuran manual
    (meteran/laser distance meter) ke CSV — dipanggil dari CLI subcommand
    'record-gt' saat sesi lapangan K1/K2, supaya peneliti tidak perlu edit
    CSV tangan. object_uid: identitas FISIK permanen dari stiker penanda
    (checklist G1), BUKAN tracker_id/inst_id sistem apa pun.

    measured_wall default ke wall-clock SEKARANG (ISO-8601) kalau tidak
    diisi eksplisit — praktis untuk field, tapi override manual kalau
    pengukuran dicatat belakangan (bukan real-time saat diukur).
    """
    if measured_wall is None:
        measured_wall = datetime.datetime.now().isoformat()
    is_new = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0
    os.makedirs(os.path.dirname(os.path.abspath(csv_path)), exist_ok=True)
    with open(csv_path, 'a', newline='') as f:
        writer = csv.writer(f)
        if is_new:
            writer.writerow(GROUND_TRUTH_FIELDS)
        writer.writerow([session_id, object_uid, class_name, gt_x, gt_y,
                          gt_z if gt_z is not None else '',
                          marker_id, measured_wall, instrument, note])


def load_reid_timing_log(csv_path: str) -> list:
    """Baca reid_timing_log.csv → list of dict, kolom numerik sudah di-cast."""
    rows = []
    with open(csv_path, newline='') as f:
        for row in csv.DictReader(f):
            row['frame_seq']             = int(row['frame_seq'])
            row['reid_ms']               = float(row['reid_ms'])
            row['n_checked']             = int(row['n_checked'])
            row['n_confirmed_instances'] = int(row['n_confirmed_instances'])
            row['merge_dist']            = float(row['merge_dist'])
            rows.append(row)
    return rows


def timing_report(rows: list) -> dict:
    """
    [Tujuan 6 / Tabel 3.8] Statistik biaya komputasi mekanisme re-id.
    p95 dilaporkan selain mean/std karena real-time budget peduli pada
    ekor distribusi (worst-case per frame), bukan cuma rata-rata.
    """
    if not rows:
        raise ValueError('Tidak ada baris timing untuk dianalisis.')

    ms = [r['reid_ms'] for r in rows]
    ms_sorted = sorted(ms)
    p95_idx = min(len(ms_sorted) - 1, int(round(0.95 * (len(ms_sorted) - 1))))

    return {
        'n_frames':                  len(ms),
        'mean_ms':                   statistics.mean(ms),
        'std_ms':                    statistics.pstdev(ms) if len(ms) > 1 else 0.0,
        'max_ms':                    max(ms),
        'p95_ms':                    ms_sorted[p95_idx],
        'mean_n_checked':            statistics.mean(r['n_checked'] for r in rows),
        'mean_n_confirmed_instances': statistics.mean(r['n_confirmed_instances'] for r in rows),
    }


# ─────────────────────────────────────────────────────────────────
# Fase K1 — noise floor characterization
# ─────────────────────────────────────────────────────────────────
def session_centroids(rows: list, class_name: str) -> dict:
    """
    Kelompokkan baris position_log per session_id, hitung mean posisi.
    Asumsi Fase K1: SATU objek fisik dari class_name per sesi (setup
    terkontrol proposal subbab 3.4.5 — 1 unit cone, tidak ada objek lain
    sekelas di frame). Kalau ada >1 tracker_id per sesi untuk class_name
    yang sama, semuanya tetap digabung ke satu centroid sesi (representasi
    "apa yang dicatat sistem pada sesi ini"), bukan per-tracker_id.
    """
    by_session = defaultdict(lambda: {'xs': [], 'ys': [], 'zs': []})
    for r in rows:
        if r.get('class_name') != class_name:
            continue
        s = by_session[r['session_id']]
        s['xs'].append(r['x'])
        s['ys'].append(r['y'])
        s['zs'].append(r['z'])

    centroids = {}
    for session_id, s in by_session.items():
        n = len(s['xs'])
        if n == 0:
            continue
        centroids[session_id] = {
            'x': sum(s['xs']) / n,
            'y': sum(s['ys']) / n,
            'z': sum(s['zs']) / n,
            'n_rows': n,
        }
    return centroids


def noise_floor_report(centroids: dict) -> dict:
    """
    Hitung statistik deviasi posisi lintas sesi (subbab 3.4.5).

    D_max = deviasi maksimum satu sesi terhadap grand mean (posisi
    referensi terbaik yang tersedia dari data), sesuai definisi
    "jarak deviasi posisi maksimum yang teramati di antara seluruh
    siklus restart/relokalisasi". max_pairwise_dist dilaporkan juga
    sebagai statistik deskriptif pelengkap (subbab 3.4.5: "seluruh
    statistik deskriptif ... akan tetap dilaporkan secara lengkap").
    """
    if len(centroids) < 2:
        raise ValueError(
            'Butuh >= 2 sesi untuk karakterisasi noise floor '
            f'(dapat {len(centroids)}). Jalankan lebih banyak siklus restart/relokalisasi.')

    xs = [c['x'] for c in centroids.values()]
    ys = [c['y'] for c in centroids.values()]
    grand_x, grand_y = sum(xs) / len(xs), sum(ys) / len(ys)

    deviations = [math.hypot(x - grand_x, y - grand_y) for x, y in zip(xs, ys)]

    pairwise = [
        math.hypot(xs[i] - xs[j], ys[i] - ys[j])
        for i, j in itertools.combinations(range(len(xs)), 2)
    ]

    return {
        'n_sessions':        len(centroids),
        'grand_mean_x':      grand_x,
        'grand_mean_y':      grand_y,
        'mean_deviation':    statistics.mean(deviations),
        'std_deviation':     statistics.pstdev(deviations) if len(deviations) > 1 else 0.0,
        'd_max':             max(deviations),
        'max_pairwise_dist': max(pairwise),
        'per_session_deviation': dict(zip(centroids.keys(), deviations)),
    }


def guardband_threshold(d_max: float, margin: float) -> float:
    """T_calibration = D_max + M (JCGM 100:2008 guardbanding, subbab 3.4.5)."""
    return d_max + margin


# ─────────────────────────────────────────────────────────────────
# [E1/E2] Replay position_log.csv lewat ReidAccumulator — mekanisme
# SAMA PERSIS dengan live (semantic_map_exporter.py _cloud_cb).
# ─────────────────────────────────────────────────────────────────
def _batch_rows_by_frame(rows: list) -> list:
    """Kelompokkan rows per (session_id, frame_seq), urut kemunculan pertama.
    Replikasi batching live: satu pointcloud message = beberapa deteksi
    ditambahkan dulu, promote_ready() baru dipanggil sekali per message."""
    frames = defaultdict(list)
    order = []
    for r in rows:
        fkey = (r['session_id'], r['frame_seq'])
        if fkey not in frames:
            order.append(fkey)
        frames[fkey].append(r)
    return [frames[fkey] for fkey in order]


def _replay_into(acc: ReidAccumulator, rows: list) -> None:
    for frame_rows in _batch_rows_by_frame(rows):
        for r in frame_rows:
            acc.add_detection(r['class_id'], r['tracker_id'], r['x'], r['y'], r['n_points'])
        acc.promote_ready()


def replay_session(rows: list, merge_dist: float, min_points: int,
                    reid_enabled: bool = True) -> dict:
    """
    [E1] Replay SATU sesi position_log.csv (list of dict dari
    load_position_log(), difilter ke satu session_id oleh caller kalau
    file berisi banyak sesi) lewat ReidAccumulator.

    Returns {(class_id, tracker_id): {'class_id','x','y','cell_count'}}
    — daftar instance final, format sama dengan yang ditulis exporter
    ke semantic_map.yaml (subbab E1: "id, kelas, posisi rata-rata,
    jumlah deteksi penyusun").
    """
    acc = ReidAccumulator(merge_dist=merge_dist, min_points=min_points,
                           reid_enabled=reid_enabled)
    _replay_into(acc, rows)
    return acc.snapshot()


def replay_multi_session(session_rows_list: list, merge_dist: float,
                          min_points: int, reid_enabled: bool = True) -> dict:
    """
    [E1] Replay BANYAK sesi berurutan. Tiap sesi baru di-seed dari snapshot
    akhir sesi sebelumnya — meniru perilaku live sesungguhnya (restart node
    = load semantic_map.yaml = ReidAccumulator.seed_instance), BUKAN
    menggabungkan semua baris jadi satu sesi datar (itu akan menghilangkan
    fakta bahwa tracker_id di-reset tiap sesi, yang justru jadi inti
    persoalan cross-session re-identification yang diuji penelitian ini).

    session_rows_list: list of list-of-row-dict, satu list per sesi,
    urut kronologis sesuai urutan sesi sungguhan dijalankan.
    """
    acc = ReidAccumulator(merge_dist=merge_dist, min_points=min_points,
                           reid_enabled=reid_enabled)
    for i, rows in enumerate(session_rows_list):
        _replay_into(acc, rows)
        if i < len(session_rows_list) - 1:
            snap = acc.snapshot()
            acc = ReidAccumulator(merge_dist=merge_dist, min_points=min_points,
                                   reid_enabled=reid_enabled)
            for entry in snap.values():
                # [bugfix seed key] TIDAK memakai tracker_id sesi sebelumnya
                # sebagai key baru — lihat ReidAccumulator.seed_instance.
                acc.seed_instance(entry['class_id'], entry['x'], entry['y'],
                                   entry['cell_count'])
    return acc.snapshot()


# ─────────────────────────────────────────────────────────────────
# Fase K2 — evaluasi ambang batas re-identifikasi
# ─────────────────────────────────────────────────────────────────
def evaluate_reid(detections: list, threshold: float) -> dict:
    """
    [E2] Klasifikasi TP/FP/FN/TN (Tabel 3.7) per PASANGAN deteksi sekelas,
    berdasarkan kecocokan keputusan SISTEM (dijalankan lewat
    ReidAccumulator — reid_core.py, BUKAN reimplementasi terpisah) vs
    ground truth (object_uid). Precision/recall/false_merge_rate
    (subbab 3.6).

    detections: list of dict {'x','y','class_name','object_uid',...}
    (format load_labeled_detections() — checklist G2), tiap baris
    diperlakukan sebagai satu deteksi/observasi diskrit (min_points=1 —
    gating langsung, tidak menunggu akumulasi titik).
    """
    acc = ReidAccumulator(merge_dist=threshold, min_points=1, reid_enabled=True)
    class_id_of = {}
    assigned = []
    for i, det in enumerate(detections):
        cname = det['class_name']
        if cname not in class_id_of:
            class_id_of[cname] = len(class_id_of)
        cid = class_id_of[cname]
        orig_key = (cid, i)   # tracker_id unik per baris — tiap observasi diskrit
        acc.add_detection(cid, i, det['x'], det['y'], n_points=1)
        acc.promote_ready()
        assigned.append(acc.key_remap.get(orig_key, orig_key))

    tp = fp = fn = tn = 0
    for i, j in itertools.combinations(range(len(detections)), 2):
        if detections[i]['class_name'] != detections[j]['class_name']:
            continue   # gating hanya membandingkan objek sekelas
        same_gt  = detections[i]['object_uid'] == detections[j]['object_uid']
        same_sys = assigned[i] == assigned[j]
        if same_sys and same_gt:
            tp += 1
        elif same_sys and not same_gt:
            fp += 1
        elif not same_sys and same_gt:
            fn += 1
        else:
            tn += 1

    precision = tp / (tp + fp) if (tp + fp) else float('nan')
    recall = tp / (tp + fn) if (tp + fn) else float('nan')
    false_merge_rate = fp / (fp + tn) if (fp + tn) else float('nan')

    return {
        'threshold': threshold, 'tp': tp, 'fp': fp, 'fn': fn, 'tn': tn,
        'precision': precision, 'recall': recall,
        'false_merge_rate': false_merge_rate,
        'n_instances': len(set(assigned)),
    }


# ─────────────────────────────────────────────────────────────────
# [E3] Sweep T — subbab 3.4.5/3.6, Tabel 3.8
# ─────────────────────────────────────────────────────────────────
def sweep_threshold(detections: list, t_min: float = 0.0, t_max: float = 2.0,
                     t_step: float = 0.05) -> list:
    """
    [E3] Loop T dari t_min ke t_max step t_step (default 0,00–2,00 step
    0,05 → 41 nilai, persis seperti diminta checklist). evaluate_reid()
    dipanggil di setiap T (lewat ReidAccumulator yang sama dengan live —
    lihat E2), bukan diulang manual.

    detections: format load_labeled_detections() (checklist G2).
    Returns list of dict {T, precision, recall, false_merge_rate,
    n_instances, tp, fp, fn, tn} — urut T menaik.
    """
    n_steps = int(round((t_max - t_min) / t_step)) + 1
    results = []
    for i in range(n_steps):
        T = round(t_min + i * t_step, 10)   # round: hindari drift floating point (0.1+0.2 dst)
        r = evaluate_reid(detections, threshold=T)
        results.append({
            'T': T, 'precision': r['precision'], 'recall': r['recall'],
            'false_merge_rate': r['false_merge_rate'], 'n_instances': r['n_instances'],
            'tp': r['tp'], 'fp': r['fp'], 'fn': r['fn'], 'tn': r['tn'],
        })
    return results


def write_sweep_csv(results: list, output_path: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['T', 'precision', 'recall', 'false_merge_rate',
                          'n_instances', 'tp', 'fp', 'fn', 'tn'])
        for r in results:
            writer.writerow([r['T'], r['precision'], r['recall'],
                              r['false_merge_rate'], r['n_instances'],
                              r['tp'], r['fp'], r['fn'], r['tn']])


# ─────────────────────────────────────────────────────────────────
# [E4] Pencocokan instance hasil replay ke ground truth (Hungarian)
# ─────────────────────────────────────────────────────────────────
def match_instances_to_ground_truth(instances: dict, class_names: dict,
                                     ground_truth: list, max_dist: float = None) -> dict:
    """
    [E4] Cocokkan instance hasil replay (mis. dari replay_session()) ke
    object_uid ground truth berdasarkan jarak terdekat, PER KELAS, pakai
    Hungarian assignment (scipy.optimize.linear_sum_assignment) — bukan
    greedy nearest-neighbor yang hasilnya bisa tidak deterministik kalau
    ada kasus ambigu (>1 instance sama-sama dekat >1 GT). Hungarian
    meminimalkan total jarak SECARA GLOBAL per kelas, hasilnya unik untuk
    kombinasi jarak yang sama (tie-break oleh scipy secara deterministik).

    instances: {key: {'class_id','x','y','cell_count'}} — snapshot()
    ReidAccumulator (replay ATAU live).
    class_names: {class_id: class_name} — buat menerjemahkan instances
    (class_id integer) ke class_name (string, sama dengan ground truth).
    ground_truth: list of dict {'object_uid','class_name','x','y',...}
    (load_labeled_detections()). Kalau >1 baris per object_uid (diukur
    ulang tiap sesi), baris TERAKHIR dalam list yang dipakai.
    max_dist: pasangan dengan jarak > max_dist dibuang dari 'matched',
    masuk unmatched — None berarti semua pasangan valid.

    Returns {'matched': [(instance_key, object_uid, dist), ...],
             'unmatched_instances': [instance_key, ...],
             'unmatched_gt': [object_uid, ...]}
    """
    gt_by_uid = {}
    for row in ground_truth:
        gt_by_uid[row['object_uid']] = row   # baris belakangan menang

    matched = []
    unmatched_instances = set(instances.keys())
    unmatched_gt = set(gt_by_uid.keys())

    classes = ({class_names.get(v['class_id'], f"class{v['class_id']}") for v in instances.values()}
               | {v['class_name'] for v in gt_by_uid.values()})

    for cls in classes:
        inst_keys = [k for k, v in instances.items()
                     if class_names.get(v['class_id'], f"class{v['class_id']}") == cls]
        gt_uids   = [uid for uid, v in gt_by_uid.items() if v['class_name'] == cls]
        if not inst_keys or not gt_uids:
            continue

        cost = np.zeros((len(inst_keys), len(gt_uids)))
        for i, ik in enumerate(inst_keys):
            for j, uid in enumerate(gt_uids):
                cost[i, j] = math.hypot(instances[ik]['x'] - gt_by_uid[uid]['x'],
                                         instances[ik]['y'] - gt_by_uid[uid]['y'])

        row_idx, col_idx = linear_sum_assignment(cost)
        for r, c in zip(row_idx, col_idx):
            dist = float(cost[r, c])
            if max_dist is not None and dist > max_dist:
                continue
            ik, uid = inst_keys[r], gt_uids[c]
            matched.append((ik, uid, dist))
            unmatched_instances.discard(ik)
            unmatched_gt.discard(uid)

    return {
        'matched': matched,
        'unmatched_instances': sorted(unmatched_instances),
        'unmatched_gt': sorted(unmatched_gt),
    }


# ─────────────────────────────────────────────────────────────────
# [E5] Reproduktibilitas — versi skrip di keluaran
# ─────────────────────────────────────────────────────────────────
def script_git_commit() -> str:
    """[E5] Hash commit git skrip ini saat dijalankan — dicetak di CLI
    supaya keluaran bisa dilacak balik ke versi kode yang menghasilkannya."""
    try:
        result = subprocess.run(
            ['git', '-C', os.path.dirname(os.path.abspath(__file__)), 'rev-parse', 'HEAD'],
            capture_output=True, text=True, timeout=3.0)
        out = result.stdout.strip()
        return out if out else 'unknown'
    except Exception:
        return 'unknown'


# ─────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────
def _cmd_noise_floor(args):
    rows = load_position_log(args.log)
    centroids = session_centroids(rows, args.class_name)
    report = noise_floor_report(centroids)

    print(f"Fase K1 — noise floor '{args.class_name}' ({report['n_sessions']} sesi)")
    print(f"  grand mean       : ({report['grand_mean_x']:.4f}, {report['grand_mean_y']:.4f})")
    print(f"  mean deviation   : {report['mean_deviation']:.4f} m")
    print(f"  std deviation    : {report['std_deviation']:.4f} m")
    print(f"  D_max            : {report['d_max']:.4f} m")
    print(f"  max pairwise dist: {report['max_pairwise_dist']:.4f} m")
    for sid, dev in report['per_session_deviation'].items():
        print(f"    session {sid}: deviasi {dev:.4f} m")

    if args.margin is not None:
        t = guardband_threshold(report['d_max'], args.margin)
        print(f"\n  T_calibration = D_max + M = {report['d_max']:.4f} + {args.margin} = {t:.4f} m")


def _cmd_evaluate(args):
    detections = load_labeled_detections(args.labeled_csv)
    result = evaluate_reid(detections, args.threshold)
    print(f"Fase K2 — evaluasi T={args.threshold} m")
    for k, v in result.items():
        print(f"  {k}: {v}")


def _cmd_timing_report(args):
    rows = load_reid_timing_log(args.log)
    report = timing_report(rows)
    print(f"Biaya komputasi mekanisme re-id ({report['n_frames']} frame)")
    print(f"  mean_ms                    : {report['mean_ms']:.4f}")
    print(f"  std_ms                     : {report['std_ms']:.4f}")
    print(f"  p95_ms                     : {report['p95_ms']:.4f}")
    print(f"  max_ms                     : {report['max_ms']:.4f}")
    print(f"  mean_n_checked             : {report['mean_n_checked']:.2f}")
    print(f"  mean_n_confirmed_instances : {report['mean_n_confirmed_instances']:.2f}")
    if rows:
        print(f"  (enable_reid={rows[0]['enable_reid']}, merge_dist={rows[0]['merge_dist']} "
              f"— cek konsisten sepanjang file kalau digabung dari >1 sesi)")


def _cmd_record_gt(args):
    append_ground_truth(
        args.csv, args.session_id, args.object_uid, args.class_name,
        args.gt_x, args.gt_y, gt_z=args.gt_z, marker_id=args.marker_id,
        measured_wall=args.measured_wall, instrument=args.instrument, note=args.note)
    print(f"Ditambahkan ke {args.csv}: "
          f"session={args.session_id}, object_uid={args.object_uid}, "
          f"class={args.class_name}, gt=({args.gt_x},{args.gt_y},{args.gt_z})")


def _class_names_from_rows(rows: list) -> dict:
    """class_id -> class_name, dari kolom position_log.csv itu sendiri."""
    out = {}
    for r in rows:
        if r.get('class_id') is not None and r.get('class_name'):
            out[r['class_id']] = r['class_name']
    return out


def _cmd_replay(args):
    """[E1] Replay satu atau banyak sesi position_log.csv, cetak daftar
    instance final (id, kelas, posisi rata-rata, jumlah deteksi penyusun)."""
    session_rows_list = []
    class_names = {}
    for path in args.log:
        rows = load_position_log(path)
        session_rows_list.append(rows)
        class_names.update(_class_names_from_rows(rows))

    if len(session_rows_list) == 1:
        snap = replay_session(session_rows_list[0], merge_dist=args.threshold,
                               min_points=args.min_points, reid_enabled=not args.baseline_naif)
    else:
        snap = replay_multi_session(session_rows_list, merge_dist=args.threshold,
                                     min_points=args.min_points, reid_enabled=not args.baseline_naif)

    print(f"Replay {len(session_rows_list)} sesi, T={args.threshold}m, "
          f"min_points={args.min_points}, baseline_naif={args.baseline_naif}")
    print(f"{len(snap)} instance final:")
    for (cid, tid), entry in sorted(snap.items()):
        name = class_names.get(cid, f'class{cid}')
        print(f"  ({cid},{tid}) {name}: x={entry['x']:.4f} y={entry['y']:.4f} "
              f"cell_count={entry['cell_count']}")

    if args.out:
        rows_out = [{'class_id': cid, 'tracker_id': tid, 'class_name': class_names.get(cid, f'class{cid}'),
                     'x': e['x'], 'y': e['y'], 'cell_count': e['cell_count']}
                    for (cid, tid), e in snap.items()]
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['class_id', 'tracker_id', 'class_name', 'x', 'y', 'cell_count'])
            w.writeheader()
            w.writerows(rows_out)
        print(f"Ditulis ke {args.out}")


def _cmd_sweep_t(args):
    detections = load_labeled_detections(args.labeled_csv)
    results = sweep_threshold(detections, t_min=args.t_min, t_max=args.t_max, t_step=args.t_step)
    write_sweep_csv(results, args.out)
    print(f"Sweep T: {args.t_min} s.d. {args.t_max} step {args.t_step} "
          f"({len(results)} nilai) → {args.out}")


def _cmd_match_gt(args):
    session_rows_list = []
    class_names = {}
    for path in args.log:
        rows = load_position_log(path)
        session_rows_list.append(rows)
        class_names.update(_class_names_from_rows(rows))
    snap = (replay_session(session_rows_list[0], merge_dist=args.threshold, min_points=args.min_points)
            if len(session_rows_list) == 1 else
            replay_multi_session(session_rows_list, merge_dist=args.threshold, min_points=args.min_points))

    ground_truth = load_labeled_detections(args.labeled_csv)
    result = match_instances_to_ground_truth(snap, class_names, ground_truth, max_dist=args.max_dist)

    print(f"[E4] Pencocokan instance replay ↔ ground truth (T={args.threshold}m):")
    for ik, uid, dist in result['matched']:
        print(f"  {ik} ↔ {uid}  (dist={dist:.4f}m)")
    print(f"  unmatched_instances: {result['unmatched_instances']}")
    print(f"  unmatched_gt: {result['unmatched_gt']}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)

    p1 = sub.add_parser('noise-floor', help='Fase K1 — karakterisasi noise floor')
    p1.add_argument('--log', required=True, help='path ke position_log.csv')
    p1.add_argument('--class-name', required=True, dest='class_name')
    p1.add_argument('--margin', type=float, default=None,
                     help='guardband margin M — kalau diisi, langsung hitung T_calibration')
    p1.set_defaults(func=_cmd_noise_floor)

    p2 = sub.add_parser('evaluate', help='Fase K2 — precision/recall/false merge rate')
    p2.add_argument('--labeled-csv', required=True, dest='labeled_csv',
                     help='CSV ground truth (checklist G2 — lihat GROUND_TRUTH_FIELDS)')
    p2.add_argument('--threshold', type=float, required=True)
    p2.set_defaults(func=_cmd_evaluate)

    p3 = sub.add_parser('timing-report', help='Tujuan 6 — biaya komputasi ms/frame')
    p3.add_argument('--log', required=True, help='path ke reid_timing_log.csv')
    p3.set_defaults(func=_cmd_timing_report)

    p4 = sub.add_parser('record-gt', help='checklist G — tambah satu baris ground truth hasil ukur manual')
    p4.add_argument('--csv', required=True, help='path ground truth CSV (dibuat kalau belum ada)')
    p4.add_argument('--session-id', required=True, dest='session_id')
    p4.add_argument('--object-uid', required=True, dest='object_uid',
                     help='identitas objek FISIK permanen dari stiker penanda, mis. "CONE_A" (checklist G1)')
    p4.add_argument('--class-name', required=True, dest='class_name')
    p4.add_argument('--gt-x', type=float, required=True, dest='gt_x')
    p4.add_argument('--gt-y', type=float, required=True, dest='gt_y')
    p4.add_argument('--gt-z', type=float, default=None, dest='gt_z')
    p4.add_argument('--marker-id', default='', dest='marker_id',
                     help='penanda lantai (checklist G2b — kosongkan untuk objek dinding tetap L1/L2)')
    p4.add_argument('--measured-wall', default=None, dest='measured_wall',
                     help='ISO-8601 — default: waktu sekarang')
    p4.add_argument('--instrument', default='', help='alat ukur, mis. "laser distance meter"')
    p4.add_argument('--note', default='', help='catatan bebas, mis. "setelah perpindahan 0.3m"')
    p4.set_defaults(func=_cmd_record_gt)

    p5 = sub.add_parser('replay', help='[E1] Replay 1/banyak sesi -> daftar instance final')
    p5.add_argument('--log', required=True, nargs='+',
                     help='satu atau lebih path position_log.csv, urut kronologis kalau >1 (lintas sesi)')
    p5.add_argument('--threshold', type=float, required=True, dest='threshold', help='merge_dist (T)')
    p5.add_argument('--min-points', type=int, default=20, dest='min_points')
    p5.add_argument('--baseline-naif', action='store_true', dest='baseline_naif',
                     help='matikan re-id (reid_enabled=false) — setiap deteksi jadi instance baru')
    p5.add_argument('--out', default=None, help='opsional: tulis hasil ke CSV')
    p5.set_defaults(func=_cmd_replay)

    p6 = sub.add_parser('sweep-t', help='[E3] Loop T, hitung precision/recall/false_merge_rate per T')
    p6.add_argument('--labeled-csv', required=True, dest='labeled_csv')
    p6.add_argument('--t-min', type=float, default=0.0, dest='t_min')
    p6.add_argument('--t-max', type=float, default=2.0, dest='t_max')
    p6.add_argument('--t-step', type=float, default=0.05, dest='t_step')
    p6.add_argument('--out', required=True, help='path output CSV')
    p6.set_defaults(func=_cmd_sweep_t)

    p7 = sub.add_parser('match-gt', help='[E4] Cocokkan instance replay ke object_uid ground truth (Hungarian)')
    p7.add_argument('--log', required=True, nargs='+', help='satu/banyak path position_log.csv')
    p7.add_argument('--labeled-csv', required=True, dest='labeled_csv')
    p7.add_argument('--threshold', type=float, required=True)
    p7.add_argument('--min-points', type=int, default=20, dest='min_points')
    p7.add_argument('--max-dist', type=float, default=None, dest='max_dist',
                     help='pasangan lebih jauh dari ini dianggap tidak match')
    p7.set_defaults(func=_cmd_match_gt)

    args = p.parse_args()
    # [E5] Reproduktibilitas — versi skrip dicatat di setiap keluaran CLI.
    print(f"[position_log_analyzer.py git commit: {script_git_commit()}]")
    args.func(args)


if __name__ == '__main__':
    main()
