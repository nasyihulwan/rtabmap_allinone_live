#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  Semantic Map Capture — 4 Versi Output                          ║
║                                                                  ║
║  Output (4 file dengan timestamp sama):                         ║
║    semantic_map_YYYYMMDD_HHMMSS_a_class.png    — warna/class   ║
║    semantic_map_YYYYMMDD_HHMMSS_b_outline.png  — kontur/inst   ║
║    semantic_map_YYYYMMDD_HHMMSS_c_centroid.png — kotak tengah  ║
║    semantic_map_YYYYMMDD_HHMMSS_d_dense.png    — fill/instance ║
║                                                                  ║
║  Cara pakai:                                                     ║
║    Terminal 1: ./run.sh                                         ║
║    Terminal 2: ./run_semantic.sh                                ║
║    Terminal 3: python3 capture_semantic_map.py                  ║
║    → Ctrl+C kapanpun untuk capture dan simpan 4 PNG            ║
╚══════════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                        QoSHistoryPolicy, QoSDurabilityPolicy)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2

import numpy as np
import struct
import cv2
from datetime import datetime
import sys
import os


# ── Palette warna per instance (RGB) ─────────────────────────────
INSTANCE_COLORS_RGB = [
    (255,56,56),(56,56,255),(56,255,56),(255,178,29),(207,10,249),
    (72,249,10),(146,23,204),(219,61,134),(147,26,52),(0,212,187),
    (168,153,44),(0,194,255),(52,147,69),(100,0,255),(255,24,0),
    (132,255,56),(82,133,0),(203,56,255),(255,55,149),(55,199,255),
    (200,100,50),(50,200,100),(100,50,200),(255,200,0),(0,255,200),
    (200,0,255),(255,100,200),(100,255,0),(0,100,255),(255,0,100),
    (150,75,0),(0,150,75),(75,0,150),(255,150,75),(75,255,150),
    (150,255,75),(75,150,255),(255,75,150),(150,0,75),(0,75,150),
]

# ── Palette warna per class (RGB) — warna berbeda per jenis objek ─
CLASS_COLORS_RGB = [
    (255,0,0),(0,0,255),(0,255,0),(255,165,0),(128,0,128),
    (0,255,255),(255,20,147),(139,69,19),(0,128,128),(255,215,0),
    (70,130,180),(34,139,34),(220,20,60),(148,0,211),(64,224,208),
    (255,140,0),(0,191,255),(50,205,50),(255,105,180),(188,143,143),
]

COCO_NAMES = [
    'person','bicycle','car','motorcycle','airplane','bus','train','truck',
    'boat','traffic light','fire hydrant','stop sign','parking meter','bench',
    'bird','cat','dog','horse','sheep','cow','elephant','bear','zebra','giraffe',
    'backpack','umbrella','handbag','tie','suitcase','frisbee','skis','snowboard',
    'sports ball','kite','baseball bat','baseball glove','skateboard','surfboard',
    'tennis racket','bottle','wine glass','cup','fork','knife','spoon','bowl',
    'banana','apple','sandwich','orange','broccoli','carrot','hot dog','pizza',
    'donut','cake','chair','couch','potted plant','bed','dining table','toilet',
    'tv','laptop','mouse','remote','keyboard','cell phone','microwave','oven',
    'toaster','sink','refrigerator','book','clock','vase','scissors','teddy bear',
    'hair drier','toothbrush'
]


class SemanticMapCapture(Node):

    def __init__(self, output_dir):
        super().__init__('semantic_map_capture')

        self.output_dir     = output_dir
        self.base_map       = None
        self.sem_cloud      = None
        self.inst_class_map = {}  # {instance_id → class_id}

        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_map = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(OccupancyGrid, '/map',
                                 self._base_cb, qos_map)
        self.create_subscription(PointCloud2, '/semantic/grid_colored',
                                 self._sem_cb, qos_be)
        self.create_subscription(PointCloud2, '/semantic/pointcloud',
                                 self._rawcloud_cb, qos_be)

        self.create_timer(5.0, self._print_status)

        self.get_logger().info(
            'SemanticMapCapture ready — Tekan Ctrl+C untuk capture 4 PNG'
        )

    def _base_cb(self, msg):
        self.base_map = msg

    def _sem_cb(self, msg):
        self.sem_cloud = msg

    def _rawcloud_cb(self, msg):
        if msg.width == 0:
            return
        try:
            raw     = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.width, 4)
            rgb_raw = raw[:, 3].view(np.uint32)
            cls_ids  = ((rgb_raw >> 16) & 0xFF).astype(np.int32)
            inst_ids = ((rgb_raw >> 8)  & 0xFF).astype(np.int32)
            for cls_id, inst_id in zip(cls_ids, inst_ids):
                if inst_id > 0 and inst_id not in self.inst_class_map:
                    self.inst_class_map[int(inst_id)] = int(cls_id)
        except Exception:
            pass

    def _print_status(self):
        base  = '✓' if self.base_map  else '✗'
        sem   = '✓' if self.sem_cloud else '✗'
        pts   = self.sem_cloud.width if self.sem_cloud else 0
        known = [f'{COCO_NAMES[c] if 0<=c<len(COCO_NAMES) else "?"}_{i}'
                 for i, c in sorted(self.inst_class_map.items())]
        self.get_logger().info(
            f'Base: {base} | Semantic: {sem} ({pts} pts) | '
            f'Instances: {len(known)} [{", ".join(known[:10])}{"..." if len(known)>10 else ""}]\n'
            f'  → Ctrl+C untuk capture'
        )

    # ─────────────────────────────────────────────────────────────────
    # MAIN CAPTURE — generate 4 file
    # ─────────────────────────────────────────────────────────────────
    def capture(self):
        if self.base_map is None:
            print('[ERROR] Base map belum ada.')
            return
        if self.sem_cloud is None:
            print('[WARN] Semantic cloud belum ada, hanya base map.')

        ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
        base_img = self._render_base_map(self.base_map)

        # Parse semantic cloud satu kali, pakai untuk semua versi
        parsed = self._parse_cloud(self.sem_cloud, self.base_map)

        versions = {
            'a_class':    self._render_a_class,
            'b_outline':  self._render_b_outline,
            'c_centroid': self._render_c_centroid,
            'd_dense':    self._render_d_dense,
        }

        print(f'\n[INFO] Generating 4 versi map...')
        for suffix, render_fn in versions.items():
            overlay, legend = render_fn(parsed, self.base_map)
            combined        = self._combine(base_img, overlay)
            legend_panel    = self._render_legend(legend, combined.shape[0])
            final           = self._hstack(combined, legend_panel)
            final           = self._add_title(final, suffix, ts)

            fname = os.path.join(self.output_dir, f'semantic_map_{ts}_{suffix}.png')
            cv2.imwrite(fname, cv2.cvtColor(final, cv2.COLOR_RGB2BGR))
            print(f'  ✓ {fname}')

        print(f'\n[INFO] 4 file tersimpan di: {self.output_dir}')


    # ─────────────────────────────────────────────────────────────────
    # PARSE CLOUD — ambil data mentah sekali, pakai berulang
    # ─────────────────────────────────────────────────────────────────
    def _parse_cloud(self, cloud, grid):
        """Return list of {px, py, inst_id, cls_id, color_inst, color_cls}"""
        if cloud is None or cloud.width == 0:
            return []

        w_map = grid.info.width
        h_map = grid.info.height
        res   = grid.info.resolution
        ox    = grid.info.origin.position.x
        oy    = grid.info.origin.position.y

        raw     = np.frombuffer(bytes(cloud.data), dtype=np.float32).reshape(cloud.width, 4)
        xy      = raw[:, :2]
        rgb_raw = raw[:, 3].view(np.uint32)
        r_ch    = ((rgb_raw >> 16) & 0xFF).astype(np.uint8)
        g_ch    = ((rgb_raw >> 8)  & 0xFF).astype(np.uint8)
        b_ch    = ( rgb_raw        & 0xFF).astype(np.uint8)

        points = []
        for i in range(cloud.width):
            px = int((xy[i, 0] - ox) / res)
            py = int((xy[i, 1] - oy) / res)
            py = h_map - 1 - py

            if not (0 <= px < w_map and 0 <= py < h_map):
                continue

            color       = (int(r_ch[i]), int(g_ch[i]), int(b_ch[i]))
            inst_id     = self._color_to_instance(color)
            cls_id      = self.inst_class_map.get(inst_id, -1)
            color_inst  = INSTANCE_COLORS_RGB[inst_id % len(INSTANCE_COLORS_RGB)]
            color_cls   = CLASS_COLORS_RGB[cls_id % len(CLASS_COLORS_RGB)] if cls_id >= 0 else (128, 128, 128)

            points.append({
                'px': px, 'py': py,
                'inst_id': inst_id, 'cls_id': cls_id,
                'color_inst': color_inst,
                'color_cls':  color_cls,
            })
        return points

    def _color_to_instance(self, color):
        min_d, best = float('inf'), 0
        for i, (r, g, b) in enumerate(INSTANCE_COLORS_RGB):
            d = (color[0]-r)**2 + (color[1]-g)**2 + (color[2]-b)**2
            if d < min_d:
                min_d, best = d, i
        return best + 1

    # ─────────────────────────────────────────────────────────────────
    # VERSI A — Warna per class (semua person merah, semua chair biru)
    # ─────────────────────────────────────────────────────────────────
    def _render_a_class(self, points, grid):
        w_map = grid.info.width
        h_map = grid.info.height
        overlay = np.zeros((h_map, w_map, 3), dtype=np.uint8)

        for p in points:
            overlay[p['py'], p['px']] = p['color_cls']

        overlay = self._dilate(overlay)

        # Legend per class (bukan per instance)
        seen_classes = {}
        for p in points:
            cid = p['cls_id']
            if cid >= 0 and cid not in seen_classes:
                name = COCO_NAMES[cid] if cid < len(COCO_NAMES) else '?'
                seen_classes[name] = p['color_cls']

        return overlay, seen_classes

    # ─────────────────────────────────────────────────────────────────
    # VERSI B — Outline/kontur per instance
    # ─────────────────────────────────────────────────────────────────
    def _render_b_outline(self, points, grid):
        w_map = grid.info.width
        h_map = grid.info.height

        # Kelompokkan pixel per instance
        inst_pixels = {}
        for p in points:
            iid = p['inst_id']
            if iid not in inst_pixels:
                inst_pixels[iid] = {'pixels': [], 'color': p['color_inst'], 'cls_id': p['cls_id']}
            inst_pixels[iid]['pixels'].append((p['px'], p['py']))

        overlay = np.zeros((h_map, w_map, 3), dtype=np.uint8)
        legend  = {}

        for iid, data in inst_pixels.items():
            color  = data['color']
            cls_id = data['cls_id']
            name   = COCO_NAMES[cls_id] if 0 <= cls_id < len(COCO_NAMES) else 'object'
            label  = f'{name}_{iid}'

            # Buat mask per instance
            mask = np.zeros((h_map, w_map), dtype=np.uint8)
            for (px, py) in data['pixels']:
                mask[py, px] = 255

            # Dilate mask dulu
            kernel = np.ones((7, 7), np.uint8)
            mask   = cv2.dilate(mask, kernel)

            # Ambil kontur
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(overlay, contours, -1, color, 2)

            if label not in legend:
                legend[label] = color

        return overlay, legend

    # ─────────────────────────────────────────────────────────────────
    # VERSI C — Centroid marker (kotak kecil di tengah tiap instance)
    # ─────────────────────────────────────────────────────────────────
    def _render_c_centroid(self, points, grid):
        w_map = grid.info.width
        h_map = grid.info.height

        # Centroid per instance
        inst_data = {}
        for p in points:
            iid = p['inst_id']
            if iid not in inst_data:
                inst_data[iid] = {'xs': [], 'ys': [],
                                   'color': p['color_inst'],
                                   'cls_id': p['cls_id']}
            inst_data[iid]['xs'].append(p['px'])
            inst_data[iid]['ys'].append(p['py'])

        overlay = np.zeros((h_map, w_map, 3), dtype=np.uint8)
        legend  = {}

        for iid, data in inst_data.items():
            color  = data['color']
            cls_id = data['cls_id']
            name   = COCO_NAMES[cls_id] if 0 <= cls_id < len(COCO_NAMES) else 'object'
            label  = f'{name}_{iid}'

            cx = int(np.mean(data['xs']))
            cy = int(np.mean(data['ys']))

            # Gambar kotak di centroid
            box_size = 8
            x1 = max(0, cx - box_size)
            y1 = max(0, cy - box_size)
            x2 = min(w_map - 1, cx + box_size)
            y2 = min(h_map - 1, cy + box_size)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (255, 255, 255), 1)

            if label not in legend:
                legend[label] = color

        return overlay, legend

    # ─────────────────────────────────────────────────────────────────
    # VERSI D — Dense fill per instance (output asli kita)
    # ─────────────────────────────────────────────────────────────────
    def _render_d_dense(self, points, grid):
        w_map = grid.info.width
        h_map = grid.info.height
        overlay = np.zeros((h_map, w_map, 3), dtype=np.uint8)
        legend  = {}

        for p in points:
            overlay[p['py'], p['px']] = p['color_inst']
            iid    = p['inst_id']
            cls_id = p['cls_id']
            name   = COCO_NAMES[cls_id] if 0 <= cls_id < len(COCO_NAMES) else 'object'
            label  = f'{name}_{iid}'
            if label not in legend:
                legend[label] = p['color_inst']

        overlay = self._dilate(overlay)
        return overlay, legend

    # ─────────────────────────────────────────────────────────────────
    # HELPERS
    # ─────────────────────────────────────────────────────────────────
    def _render_base_map(self, grid):
        w    = grid.info.width
        h    = grid.info.height
        data = np.array(grid.data, dtype=np.int8).reshape(h, w)
        img  = np.full((h, w, 3), 128, dtype=np.uint8)
        img[data == 0]   = [220, 220, 220]
        img[data == 100] = [30,  30,  30]
        return np.flipud(img)

    def _dilate(self, overlay):
        kernel = np.ones((5, 5), np.uint8)
        for c in range(3):
            overlay[:, :, c] = cv2.dilate(overlay[:, :, c], kernel)
        return overlay

    def _combine(self, base, overlay):
        if overlay is None or not overlay.any():
            return base.copy()
        return cv2.addWeighted(base, 1.0, overlay, 0.7, 0)

    def _hstack(self, img_a, img_b):
        h_a, h_b = img_a.shape[0], img_b.shape[0]
        if h_a > h_b:
            pad = np.full((h_a - h_b, img_b.shape[1], 3), 45, dtype=np.uint8)
            img_b = np.vstack([img_b, pad])
        elif h_b > h_a:
            pad = np.full((h_b - h_a, img_a.shape[1], 3), 45, dtype=np.uint8)
            img_a = np.vstack([img_a, pad])
        return np.hstack([img_a, img_b])

    def _add_title(self, img, suffix, ts):
        label_map = {
            'a_class':    '(A) Warna per Class',
            'b_outline':  '(B) Outline per Instance',
            'c_centroid': '(C) Centroid per Instance',
            'd_dense':    '(D) Dense Fill per Instance',
        }
        title_bar = np.full((40, img.shape[1], 3), 30, dtype=np.uint8)
        text = f'Semantic Map  {ts}  —  {label_map.get(suffix, suffix)}'
        cv2.putText(title_bar, text, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        return np.vstack([title_bar, img])

    def _render_legend(self, legend_dict, height):
        legend_w = 230
        row_h    = 30
        pad      = 10
        box_sz   = 18
        n        = max(len(legend_dict), 1)
        legend_h = max(height, n * row_h + pad * 2 + 40)
        panel    = np.full((legend_h, legend_w, 3), 45, dtype=np.uint8)

        cv2.putText(panel, 'Instances/Classes', (pad, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.line(panel, (pad, 32), (legend_w - pad, 32), (100, 100, 100), 1)

        if not legend_dict:
            cv2.putText(panel, 'Tidak ada data', (pad, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
            return panel

        for i, (label, color) in enumerate(sorted(legend_dict.items())):
            y = pad + 40 + i * row_h
            if y + row_h > legend_h:
                break
            cv2.rectangle(panel, (pad, y), (pad + box_sz, y + box_sz), color, -1)
            cv2.rectangle(panel, (pad, y), (pad + box_sz, y + box_sz), (200,200,200), 1)
            cv2.putText(panel, label, (pad + box_sz + 6, y + 13),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (230, 230, 230), 1)
        return panel


def main():
    rclpy.init()

    script_dir  = os.path.dirname(os.path.abspath(__file__))
    default_dir = os.path.join(script_dir, 'semantic_results')
    output_dir  = sys.argv[1] if len(sys.argv) > 1 else default_dir
    os.makedirs(output_dir, exist_ok=True)

    node = SemanticMapCapture(output_dir)

    print('\n  Status update setiap 5 detik.')
    print('  Tekan Ctrl+C kapanpun untuk capture 4 PNG.\n')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.capture()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
