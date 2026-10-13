#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  YOLO Detector Node — Segmentation + Instance Tracking          ║
║                                                                  ║
║  Subscribe:                                                      ║
║    /camera/camera/color/image_raw  (sensor_msgs/Image)          ║
║                                                                  ║
║  Publish:                                                        ║
║    /semantic/image_annotated   (rgb8)  — mask + label overlay   ║
║    /semantic/label_image       (mono8) — class_id+1 per pixel   ║
║    /semantic/instance_image    (mono8) — instance_id per pixel  ║
║    /semantic/confidence_image  (mono8) — round(conf*255)/pixel  ║
║    /yolo/detection_classes     (String) — JSON class name list  ║
║    /yolo/detections            (String) — JSON: stamp + list    ║
║       {class_id,class_name,confidence,tracker_id,bbox_xyxy}     ║
║       per deteksi (checklist J2 — data mentah utk rekonstruksi  ║
║       bbox dari rosbag tanpa jalankan YOLO ulang)               ║
╚══════════════════════════════════════════════════════════════════╝
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from ultralytics import YOLO

INSTANCE_COLORS_BGR = [
    (255,56,56),(56,56,255),(56,255,56),(255,178,29),(207,10,249),
    (72,249,10),(146,23,204),(219,61,134),(147,26,52),(0,212,187),
    (168,153,44),(0,194,255),(52,147,69),(100,0,255),(255,24,0),
    (132,255,56),(82,133,0),(203,56,255),(255,55,149),(55,199,255),
    (200,100,50),(50,200,100),(100,50,200),(255,200,0),(0,255,200),
    (200,0,255),(255,100,200),(100,255,0),(0,100,255),(255,0,100),
    (150,75,0),(0,150,75),(75,0,150),(255,150,75),(75,255,150),
    (150,255,75),(75,150,255),(255,75,150),(150,0,75),(0,75,150),
]


# ─────────────────────────────────────────────────────────────────
# [J2] Pure helper — bentuk payload deteksi mentah (bbox+class+
# confidence+tracker_id) untuk /yolo/detections. Dipisah dari callback()
# supaya bisa diuji tanpa mock Node/YOLO Results.
# ─────────────────────────────────────────────────────────────────
def _build_detections_payload(cls_ids: list, inst_ids: list, confs: list,
                               bboxes: list, class_names: dict) -> list:
    """
    cls_ids/inst_ids/confs/bboxes: list sejajar panjang N (satu per deteksi).
    bboxes: list of [x1,y1,x2,y2] piksel, resolusi citra asli.
    class_names: {class_id: name} — dari model.names.

    Returns list of dict: {class_id, class_name, confidence, tracker_id, bbox_xyxy}.
    """
    out = []
    for cls_id, iid, cf, bbox in zip(cls_ids, inst_ids, confs, bboxes):
        out.append({
            'class_id':   cls_id,
            'class_name': class_names.get(cls_id, str(cls_id)),
            'confidence': round(cf, 4),
            'tracker_id': iid,
            'bbox_xyxy':  [round(v, 2) for v in bbox],
        })
    return out


class SimpleIoUTracker:
    def __init__(self, iou_threshold=0.3, max_age=10):
        self.iou_threshold = iou_threshold
        self.max_age = max_age
        self.next_id = 1
        self.tracks = {}

    def update(self, masks, class_ids):
        for tid in list(self.tracks.keys()):
            self.tracks[tid]['age'] += 1
            if self.tracks[tid]['age'] > self.max_age:
                del self.tracks[tid]

        # Snapshot track dari frame-frame sebelumnya — semua matching di
        # bawah ini hanya boleh merujuk ke sini, bukan ke self.tracks yang
        # sedang berubah, supaya track baru yang dibuat oleh satu deteksi
        # tidak "kelihatan" oleh deteksi lain di frame yang sama.
        existing_tracks = dict(self.tracks)

        # Cari kandidat match untuk tiap deteksi dulu, tanpa eksekusi apa pun.
        best_matches = []
        for mask, cls_id in zip(masks, class_ids):
            best_iou, best_tid = 0.0, None
            for tid, track in existing_tracks.items():
                if track['class_id'] != cls_id:
                    continue
                iou = self._iou(mask, track['mask'])
                if iou > best_iou:
                    best_iou, best_tid = iou, tid
            best_matches.append(best_tid if best_iou >= self.iou_threshold else None)

        # Baru eksekusi: update track lama yang cocok...
        instance_ids = [None] * len(best_matches)
        for i, tid in enumerate(best_matches):
            if tid is not None:
                self.tracks[tid].update({'mask': masks[i], 'age': 0})
                instance_ids[i] = tid

        # ...lalu buat track baru untuk sisanya, ditambahkan bareng di akhir.
        for i, tid in enumerate(best_matches):
            if tid is None:
                nid = self.next_id
                self.next_id = (self.next_id % 254) + 1
                self.tracks[nid] = {'class_id': class_ids[i], 'mask': masks[i], 'age': 0}
                instance_ids[i] = nid

        return instance_ids

    @staticmethod
    def _iou(m1, m2):
        if m1.shape != m2.shape:
            return 0.0
        i = np.logical_and(m1, m2).sum()
        u = np.logical_or(m1, m2).sum()
        return float(i) / float(u) if u > 0 else 0.0


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('model_path',
            '/root/ws/project/rtabmap_allinone/models/best_brin_yolo11m_v2_chaircone.pt')
        self.declare_parameter('confidence',      0.5)
        self.declare_parameter('device',          'cpu')
        self.declare_parameter('process_every_n', 3)
        self.declare_parameter('iou_threshold',   0.3)
        self.declare_parameter('max_track_age',   10)
        # [D3] Sebelumnya TIDAK di-set eksplisit — Ultralytics diam-diam
        # pakai default (iou=0.7, imgsz=640) yang tidak pernah tercatat di
        # mana pun. Checklist D3: "Nilainya tidak berubah antar sesi" tidak
        # bisa dijamin kalau nilainya bahkan tidak pernah eksplisit/direkam.
        self.declare_parameter('iou_nms',         0.7)
        self.declare_parameter('imgsz',           640)

        model_path   = self.get_parameter('model_path').value
        self.conf    = self.get_parameter('confidence').value
        self.dev     = self.get_parameter('device').value
        self.every_n = self.get_parameter('process_every_n').value
        iou_thr      = self.get_parameter('iou_threshold').value
        max_age      = self.get_parameter('max_track_age').value
        self.iou_nms = self.get_parameter('iou_nms').value
        self.imgsz   = self.get_parameter('imgsz').value

        self.get_logger().info(f'Loading: {model_path}')
        self.model = YOLO(model_path, task='segment')
        # Tidak pakai .to() karena TRT engine tidak support method ini
        # device dikirim langsung ke predict() saat inference
        self.tracker = SimpleIoUTracker(iou_thr, max_age)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)

        self.sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.callback, qos)
        self.pub_ann   = self.create_publisher(Image,  '/semantic/image_annotated',    10)
        self.pub_lbl   = self.create_publisher(Image,  '/semantic/label_image',         10)
        self.pub_inst  = self.create_publisher(Image,  '/semantic/instance_image',      10)
        # [confidence] mono8, nilai = round(confidence*255) per piksel mask —
        # dipacking projector ke channel B pointcloud (sebelumnya selalu 0).
        self.pub_conf  = self.create_publisher(Image,  '/semantic/confidence_image',    10)
        self.pub_cls   = self.create_publisher(String, '/yolo/detection_classes',       10)
        # [J2] Data deteksi mentah terstruktur (bbox+class+confidence+
        # tracker_id) — SEBELUMNYA tidak ada topik untuk ini sama sekali,
        # cuma citra ter-encode per-piksel (label/instance/confidence_image)
        # atau citra yang sudah tergambar (image_annotated). Topik ini
        # dipakai supaya bounding box bisa digambar ulang dari hasil rekam
        # rosbag tanpa perlu jalankan YOLO lagi. JSON via String — pola
        # sama dengan /yolo/detection_classes, bukan vision_msgs (paket itu
        # tidak dipakai di mana pun di project ini, tidak mau nambah
        # dependency baru yang tidak bisa saya build-test di sini).
        self.pub_det   = self.create_publisher(String, '/yolo/detections',              10)

        self._fc, self._td = 0, 0
        self.get_logger().info(
            f'Ready — {len(self.model.names)} classes | '
            f'device: {self.dev} | every {self.every_n} frames | '
            f'tracker IoU: {iou_thr} | max_age: {max_age} | '
            f'conf: {self.conf} | NMS IoU: {self.iou_nms} | imgsz: {self.imgsz}')

    def callback(self, msg):
        self._fc += 1
        if self._fc % self.every_n != 0:
            return

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3)).copy()
        h, w = img.shape[:2]

        results = self.model(img, conf=self.conf, iou=self.iou_nms, imgsz=self.imgsz,
                              device=self.dev, verbose=False)
        result  = results[0]

        lbl  = np.zeros((h, w), dtype=np.uint8)
        inst = np.zeros((h, w), dtype=np.uint8)
        conf_img = np.zeros((h, w), dtype=np.uint8)
        ann  = img.copy()

        detections_payload = []

        if result.masks is not None and result.boxes is not None:
            raw_masks = result.masks.data.cpu().numpy()
            cls_ids   = [int(result.boxes.cls[i]) for i in range(len(raw_masks))]
            confs     = [float(result.boxes.conf[i]) for i in range(len(raw_masks))]
            # [J2] xyxy piksel, di resolusi citra ASLI (msg.width x msg.height)
            # — sama dengan raw_masks, TIDAK perlu diskalakan ulang karena
            # Ultralytics sudah mengembalikan xyxy pada resolusi input asli.
            bboxes    = result.boxes.xyxy.cpu().numpy().tolist()
            bin_masks = [
                cv2.resize(raw_masks[i], (w, h),
                           interpolation=cv2.INTER_NEAREST) > 0.5
                for i in range(len(raw_masks))
            ]
            inst_ids = self.tracker.update(bin_masks, cls_ids)

            detections_payload = _build_detections_payload(
                cls_ids, inst_ids, confs, bboxes, self.model.names)

            for mask, cls_id, iid, cf in zip(bin_masks, cls_ids, inst_ids, confs):
                lbl[mask]  = np.uint8(cls_id + 1)
                inst[mask] = np.uint8(iid)
                # round supaya 1.0 -> 255 persis, bukan 254 akibat truncation
                conf_img[mask] = np.uint8(round(min(max(cf, 0.0), 1.0) * 255))

                color   = INSTANCE_COLORS_BGR[iid % len(INSTANCE_COLORS_BGR)]
                colored = np.zeros_like(ann)
                colored[mask] = color
                ann = cv2.addWeighted(ann, 1.0, colored, 0.5, 0)

                ys, xs = np.where(mask)
                if len(xs) > 0:
                    cx, cy = int(xs.mean()), int(ys.mean())
                    name = self.model.names.get(cls_id, str(cls_id))
                    cv2.putText(ann, f'{name}_{iid}', (cx-20, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

            self._td += len(raw_masks)

        self.pub_lbl.publish(self._mono8(lbl, msg.header))
        self.pub_inst.publish(self._mono8(inst, msg.header))
        self.pub_conf.publish(self._mono8(conf_img, msg.header))
        self.pub_ann.publish(self._rgb8(ann, msg.header))

        # [J2] std_msgs/String tidak punya field header -> stamp dari
        # msg.header (frame RGB yang diproses) disertakan EKSPLISIT di
        # payload, supaya post-processing bisa sinkronkan ke frame RGB yang
        # tepat meski String sendiri tidak ter-timestamp oleh ROS.
        det_msg = String()
        det_msg.data = json.dumps({
            'stamp_sec':     msg.header.stamp.sec,
            'stamp_nanosec': msg.header.stamp.nanosec,
            'frame_id':      msg.header.frame_id,
            'detections':    detections_payload,
        })
        self.pub_det.publish(det_msg)

        # Publish class names for classify_scene() in segformer_node
        detected_names = [
            self.model.names.get(int(result.boxes.cls[i]), str(int(result.boxes.cls[i])))
            for i in range(len(result.boxes))
        ] if result.boxes is not None else []
        cls_msg      = String()
        cls_msg.data = json.dumps(detected_names)
        self.pub_cls.publish(cls_msg)

        if (self._fc // self.every_n) % 30 == 1:
            self.get_logger().info(
                f'[Frame {self._fc}] Det: {len(result.boxes) if result.boxes else 0} | '
                f'Tracks: {len(self.tracker.tracks)} | Total: {self._td}')

    def _mono8(self, img, header):
        out = Image()
        out.header = header
        out.height, out.width = img.shape
        out.encoding = 'mono8'
        out.step = out.width
        out.data = bytes(img)
        return out

    def _rgb8(self, img, header):
        out = Image()
        out.header = header
        out.height, out.width = img.shape[:2]
        out.encoding = 'rgb8'
        out.step = out.width * 3
        out.data = bytes(img)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
