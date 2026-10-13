#!/usr/bin/env python3
"""
world_tracker.py — Tracking berbasis jarak dunia-nyata (frame map), reusable
lintas node (person_detector.py, lab_robot_detector.py, dst).

Dipisah dari SimpleIoUTracker (yolo_detector.py) krn itu berbasis IoU mask —
model deteksi-only (bounding box, gak ada mask) butuh cara beda: matching ke
posisi 3D terdekat antar frame, bukan overlap piksel. Cocok utk objek yang
jarang banyak sekaligus di 1 frame (orang, robot lab) dan gak "teleport"
(posisi antar frame gak akan jauh beda).
"""

import numpy as np


def quat_to_rotation_matrix(x, y, z, w):
    """Sama persis dgn semantic_map_exporter.py — no scipy dependency."""
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


class WorldTracker:
    def __init__(self, max_match_dist=1.0, max_age_sec=1.5,
                 speed_history_sec=1.0, moving_threshold_mps=0.15):
        self.max_match_dist = max_match_dist
        self.max_age_sec = max_age_sec
        self.speed_history_sec = speed_history_sec
        self.moving_threshold = moving_threshold_mps
        self.next_id = 1
        self.tracks = {}  # tid -> {'history': [(t, x, y), ...], 'last_seen': t, 'label': str}

    def update(self, detections, now):
        """
        detections: list of (x, y, label) posisi map-frame + label kelas.
        Matching HANYA antar detection & track dgn label SAMA (dell-e gak
        boleh "jadi" i-seater kalau kebetulan posisinya deket).
        Return list of (tid, speed, moving) sejajar urutan input detections.
        """
        for tid in list(self.tracks.keys()):
            if now - self.tracks[tid]['last_seen'] > self.max_age_sec:
                del self.tracks[tid]

        results = []
        used_tracks = set()
        for (x, y, label) in detections:
            best_tid, best_dist = None, self.max_match_dist
            for tid, track in self.tracks.items():
                if tid in used_tracks or track['label'] != label:
                    continue
                _, lx, ly = track['history'][-1]
                d = ((x - lx) ** 2 + (y - ly) ** 2) ** 0.5
                if d < best_dist:
                    best_dist, best_tid = d, tid

            if best_tid is None:
                best_tid = self.next_id
                self.next_id += 1
                self.tracks[best_tid] = {'history': [], 'last_seen': now, 'label': label}

            used_tracks.add(best_tid)
            track = self.tracks[best_tid]
            track['history'].append((now, x, y))
            track['history'] = [
                h for h in track['history'] if now - h[0] <= self.speed_history_sec
            ]
            track['last_seen'] = now

            speed = self._estimate_speed(track['history'])
            results.append((best_tid, speed, speed >= self.moving_threshold))
        return results

    @staticmethod
    def _estimate_speed(history):
        if len(history) < 2:
            return 0.0
        t0, x0, y0 = history[0]
        t1, x1, y1 = history[-1]
        dt = t1 - t0
        if dt <= 0.05:
            return 0.0
        return ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5 / dt
