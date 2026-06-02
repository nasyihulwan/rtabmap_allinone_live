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
║    /yolo/detection_classes     (String) — JSON class name list  ║
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

        instance_ids = []
        for mask, cls_id in zip(masks, class_ids):
            best_iou, best_tid = 0.0, None
            for tid, track in self.tracks.items():
                if track['class_id'] != cls_id:
                    continue
                iou = self._iou(mask, track['mask'])
                if iou > best_iou:
                    best_iou, best_tid = iou, tid

            if best_iou >= self.iou_threshold and best_tid is not None:
                self.tracks[best_tid].update({'mask': mask, 'age': 0})
                instance_ids.append(best_tid)
            else:
                nid = self.next_id
                self.next_id = (self.next_id % 254) + 1
                self.tracks[nid] = {'class_id': cls_id, 'mask': mask, 'age': 0}
                instance_ids.append(nid)
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
            '/root/ws/project/rtabmap_allinone/models/best_brin_yolo11m_v1_20260517_0724.pt')
        self.declare_parameter('confidence',      0.5)
        self.declare_parameter('device',          'cpu')
        self.declare_parameter('process_every_n', 3)
        self.declare_parameter('iou_threshold',   0.3)
        self.declare_parameter('max_track_age',   10)

        model_path   = self.get_parameter('model_path').value
        self.conf    = self.get_parameter('confidence').value
        self.dev     = self.get_parameter('device').value
        self.every_n = self.get_parameter('process_every_n').value
        iou_thr      = self.get_parameter('iou_threshold').value
        max_age      = self.get_parameter('max_track_age').value

        self.get_logger().info(f'Loading: {model_path}')
        self.model = YOLO(model_path)
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
        self.pub_cls   = self.create_publisher(String, '/yolo/detection_classes',       10)

        self._fc, self._td = 0, 0
        self.get_logger().info(
            f'Ready — {len(self.model.names)} classes | '
            f'device: {self.dev} | every {self.every_n} frames | '
            f'IoU: {iou_thr} | max_age: {max_age}')

    def callback(self, msg):
        self._fc += 1
        if self._fc % self.every_n != 0:
            return

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3)).copy()
        h, w = img.shape[:2]

        results = self.model(img, conf=self.conf, device=self.dev, verbose=False)
        result  = results[0]

        lbl  = np.zeros((h, w), dtype=np.uint8)
        inst = np.zeros((h, w), dtype=np.uint8)
        ann  = img.copy()

        if result.masks is not None and result.boxes is not None:
            raw_masks = result.masks.data.cpu().numpy()
            cls_ids   = [int(result.boxes.cls[i]) for i in range(len(raw_masks))]
            bin_masks = [
                cv2.resize(raw_masks[i], (w, h),
                           interpolation=cv2.INTER_NEAREST) > 0.5
                for i in range(len(raw_masks))
            ]
            inst_ids = self.tracker.update(bin_masks, cls_ids)

            for mask, cls_id, iid in zip(bin_masks, cls_ids, inst_ids):
                lbl[mask]  = np.uint8(cls_id + 1)
                inst[mask] = np.uint8(iid)

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
        self.pub_ann.publish(self._rgb8(ann, msg.header))

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
