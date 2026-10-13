#!/usr/bin/env python3
"""
room_type_classifier.py — Klasifikasi jenis ruangan dari fitur ter-agregasi.

Ada 2 jalur, dipilih sesuai data yang tersedia:

1. FUSI RULE-BASED (`fuse_room_tag`/`fuse_all_rooms`) — TIDAK butuh label
   ground-truth maupun training sama sekali. Cuma butuh --features. Ambang
   batasnya mirror classify_scene() (segformer_node.py) yg sudah established
   di project ini, tapi diterapkan ke RASIO TERAGREGASI SELURUH TRAJECTORY
   per ruangan (dari room_feature_extractor.py) — bukan ke 1 frame noisy.
   Realistisnya cuma ~2-3 instance ruangan fisik yg ada di gedung ini
   (corridor/workshop/staircase) — jauh terlalu sedikit utk melatih model
   yg bisa generalisasi (lihat memory sesi ini), jadi fusi rule-based inilah
   cara SegFormer beneran berkontribusi ke scene-zone TANPA butuh N sampel
   besar.

2. ML/LOOCV (opsional, kalau --labels diberi & cukup sampel) — leave-one-out
   cross-validation, 3 konfigurasi ablation (segformer-only/yolo-only/
   combined) + baseline classify_scene()-majority. Berguna KALAU suatu saat
   ada banyak instance ruangan (mis. gabung data dari beberapa gedung/lab) —
   utk sekarang N terlalu kecil, bagian ini akan skip otomatis kalau sampel
   tidak cukup (lihat run_loocv).

Import sklearn DITUNDA ke dalam run_loocv()/main() — supaya jalur (1) yang
jadi andalan sekarang bisa dipakai TANPA install scikit-learn sama sekali.
"""

import argparse
import os

import numpy as np
import yaml

SEGFORMER_PREFIX = 'ratio_'
YOLO_PREFIX      = 'has_'

# ── Fusi rule-based (mirror ambang classify_scene() di segformer_node.py,
#    tapi diterapkan ke rasio teragregasi per-ruangan) ──────────────────────
LAB_OBJECT_NAMES        = ('shelf', 'table', 'cabinet', 'laptop', 'tv')
STAIR_RATIO_THRESHOLD   = 0.05
DOOR_RATIO_THRESHOLD    = 0.05
CORRIDOR_FLOOR_THRESHOLD = 0.30
CORRIDOR_WALL_THRESHOLD  = 0.20


def fuse_room_tag(record: dict) -> str:
    """1 ruangan (record dari room_features.yaml) -> label, murni rule-based,
    TIDAK butuh training/ground-truth. Prioritas (mirror classify_scene()):
    1. objek YOLO stairs/staircase ATAU ratio_stairs teragregasi tinggi -> staircase
    2. ada objek LAB_OBJECTS (shelf/table/cabinet/laptop/tv)            -> workshop
    3. ratio_door teragregasi tinggi                                    -> doorway
    4. ratio_floor tinggi DAN ratio_wall tinggi                         -> corridor
    5. selain itu                                                      -> unknown
    """
    has_stairs_object = bool(record.get('has_staircase') or record.get('has_stairs'))
    if has_stairs_object or record.get('ratio_stairs', 0.0) > STAIR_RATIO_THRESHOLD:
        return 'staircase'
    if any(record.get(f'has_{name}') for name in LAB_OBJECT_NAMES):
        return 'workshop'
    if record.get('ratio_door', 0.0) > DOOR_RATIO_THRESHOLD:
        return 'doorway'
    if (record.get('ratio_floor', 0.0) > CORRIDOR_FLOOR_THRESHOLD
            and record.get('ratio_wall', 0.0) > CORRIDOR_WALL_THRESHOLD):
        return 'corridor'
    return 'unknown'


def fuse_all_rooms(features: dict) -> dict:
    """-> {room_id: label} utk semua ruangan di room_features.yaml."""
    return {room_id: fuse_room_tag(record) for room_id, record in features.items()}


def load_room_features(path: str) -> dict:
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return {int(k): v for k, v in data.items()}


def load_room_labels(path: str) -> dict:
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return {int(k): str(v) for k, v in data.items()}


def _feature_columns(features: dict, prefix: str) -> list:
    cols = set()
    for rec in features.values():
        cols.update(k for k in rec if k.startswith(prefix))
    return sorted(cols)


def build_feature_matrix(features: dict, labels: dict, config: str):
    """config: 'segformer' | 'yolo' | 'combined'. -> (X, y, room_ids, columns)."""
    room_ids = sorted(rid for rid in labels if rid in features)
    if not room_ids:
        raise ValueError('Tidak ada room_id yang overlap antara features dan labels.')

    seg_cols  = _feature_columns(features, SEGFORMER_PREFIX)
    yolo_cols = _feature_columns(features, YOLO_PREFIX)
    if config == 'segformer':
        cols = seg_cols
    elif config == 'yolo':
        cols = yolo_cols
    elif config == 'combined':
        cols = seg_cols + yolo_cols
    else:
        raise ValueError(f"config tidak dikenal: {config!r} (pilih segformer/yolo/combined)")

    X = np.array([[float(features[rid].get(c, 0.0)) for c in cols] for rid in room_ids])
    y = np.array([labels[rid] for rid in room_ids])
    return X, y, room_ids, cols


def run_loocv(X: np.ndarray, y: np.ndarray, model):
    """-> (accuracy, confusion_matrix, label_order, y_pred). Butuh scikit-learn
    (import ditunda di sini, bukan di top-level file — lihat docstring modul)."""
    from sklearn.model_selection import LeaveOneOut, cross_val_predict
    from sklearn.metrics import accuracy_score, confusion_matrix

    n_classes = len(set(y))
    if len(y) <= n_classes:
        raise ValueError(
            f'N sampel ({len(y)}) tidak lebih besar dari jumlah kelas ({n_classes}) '
            '— LOOCV tidak bermakna, label lebih banyak ruangan dulu.')
    y_pred = cross_val_predict(model, X, y, cv=LeaveOneOut())
    acc = accuracy_score(y, y_pred)
    label_order = sorted(set(y) | set(y_pred))
    cm = confusion_matrix(y, y_pred, labels=label_order)
    return acc, cm, label_order, y_pred


def evaluate_baseline(features: dict, labels: dict, baseline_map: dict = None):
    """Baseline naif: classify_scene() per-frame-majority vs ground truth.
    baseline_map: opsional {label_classify_scene: label_vocab_anda}, default identity.
    Akurasi cuma exact-match rate — gak butuh sklearn."""
    baseline_map = baseline_map or {}
    room_ids = sorted(rid for rid in labels if rid in features)

    y_true, y_pred, n_missing = [], [], 0
    for rid in room_ids:
        raw = features[rid].get('classify_scene_majority')
        if raw is None:
            n_missing += 1
            continue
        y_true.append(labels[rid])
        y_pred.append(baseline_map.get(raw, raw))

    if n_missing:
        print(f'[Baseline] {n_missing} room tanpa classify_scene_majority (dilewati)')
    if not y_true:
        return None
    return sum(a == b for a, b in zip(y_true, y_pred)) / len(y_true)


def main(args=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--features', required=True,
                     help='room_features.yaml dari room_feature_extractor.py')
    ap.add_argument('--labels', default=None,
                     help='opsional: room_labels_<versi>.yaml hand-authored '
                          '{room_id: "label"}. Kalau tidak diberi, cuma jalur '
                          'fusi rule-based yang dicetak (tidak butuh ground-truth).')
    ap.add_argument('--baseline-map', default=None,
                     help='YAML opsional {label_classify_scene: label_vocab_anda}, '
                          'default identity (dipakai kalau vocab label anda beda dari '
                          'staircase/lab_workshop/doorway/corridor/unknown)')
    cli_args = ap.parse_args(args=args)

    features = load_room_features(cli_args.features)

    print('=== Fusi rule-based (SegFormer ratio teragregasi + YOLO presence, '
          'no training) ===')
    fused = fuse_all_rooms(features)
    for room_id in sorted(fused):
        print(f'  room {room_id}: {fused[room_id]}')

    if not cli_args.labels:
        print('\n[Info] --labels tidak diberi — skip ablation ML & baseline '
              '(butuh ground-truth utk dibandingkan). Fusi rule-based di atas '
              'sudah bisa dipakai langsung tanpa itu.')
        return

    labels = load_room_labels(cli_args.labels)
    n_overlap = len(set(features) & set(labels))
    print(f'\n[Data] {len(features)} room di features, {len(labels)} room dilabel manual, '
          f'{n_overlap} overlap dipakai training/eval.')

    fused_acc = sum(fused.get(rid) == labels[rid] for rid in labels if rid in fused)
    n_fused_compared = sum(1 for rid in labels if rid in fused)
    if n_fused_compared:
        print(f'[Fusi rule-based] accuracy vs ground-truth = '
              f'{fused_acc / n_fused_compared:.2%} ({n_fused_compared} room)')

    baseline_map = {}
    if cli_args.baseline_map:
        with open(cli_args.baseline_map) as f:
            baseline_map = yaml.safe_load(f) or {}

    print('\n=== ML/LOOCV opsional: Ablation SegFormer-only / YOLO-only / '
          'Combined ===')
    try:
        from sklearn.linear_model import LogisticRegression
        from sklearn.tree import DecisionTreeClassifier
    except ImportError:
        print('  scikit-learn tidak terinstall — skip bagian ML (fusi rule-based '
              'di atas tidak terpengaruh, tidak butuh sklearn).')
        return

    models = {
        'LogisticRegression':   LogisticRegression(max_iter=1000),
        'DecisionTree(depth=3)': DecisionTreeClassifier(max_depth=3, random_state=0),
    }

    for config in ('segformer', 'yolo', 'combined'):
        try:
            X, y, room_ids, cols = build_feature_matrix(features, labels, config)
        except ValueError as e:
            print(f'\n[{config}] SKIP: {e}')
            continue
        print(f'\n[{config}] {len(cols)} fitur x {len(room_ids)} ruangan: {cols}')
        for model_name, model in models.items():
            try:
                acc, cm, label_order, _ = run_loocv(X, y, model)
                print(f'  {model_name:24s} LOOCV accuracy = {acc:.2%}')
                print(f'    labels: {label_order}')
                print(f'    confusion matrix:\n{cm}')
            except ValueError as e:
                print(f'  {model_name:24s} SKIP: {e}')

    print('\n=== Baseline naif: classify_scene() per-frame-majority ===')
    baseline_acc = evaluate_baseline(features, labels, baseline_map)
    if baseline_acc is None:
        print('  Tidak ada classify_scene_majority di features — skip.')
    else:
        print(f'  Accuracy = {baseline_acc:.2%}')


if __name__ == '__main__':
    main()
