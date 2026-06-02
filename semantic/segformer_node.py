#!/usr/bin/env python3
"""
segformer_node.py — SegFormer-B0 semantic segmentation node for ROS2.

Subscribes:
  /camera/camera/color/image_raw      (sensor_msgs/Image)
  /yolo/detection_classes             (std_msgs/String, JSON list of class names)

Publishes:
  /semantic/segformer/mask_filtered   (sensor_msgs/Image, mono8)
  /semantic/segformer/mask_full       (sensor_msgs/Image, mono8)
  /semantic/segformer/scene_label     (std_msgs/String)
  /semantic/segformer/image_colored   (sensor_msgs/Image, rgb8)

Model: nvidia/segformer-b0-finetuned-ade-512-512
transformers==4.44.2  |  PyTorch 2.3.0  |  Orin 50W ~16.5fps parallel YOLO
Benchmark config: input_size=512, process_every_n=1, device=cuda
"""

import json
import numpy as np
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import torch
import torch.nn.functional as F
from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation


# ── Navigation classes published to mask_filtered (Nav2 costmap) ───────────
KEEP_CLASSES = {0, 3, 5, 6, 29, 52, 53, 55}
# wall(0), floor(3), ceiling(5), corridor(6),
# stairs(29), door(52), window(53), railing(55)

# ── classify_scene() object sets ───────────────────────────────────────────
LAB_OBJECTS   = {'shelf', 'work-desk', 'metal-cabinet', 'robotic-arm', 'dell-e'}
STAIR_OBJECTS = {'staircase', 'stairs'}


def ade20k_colormap() -> np.ndarray:
    """Official ADE20K colormap (150 classes). class-0 = wall, must have colour."""
    cmap = np.zeros((150, 3), dtype=np.uint8)
    for i in range(150):
        r, g, b = 0, 0, 0
        c = i + 1
        for j in range(8):
            r |= ((c >> 0) & 1) << (7 - j)
            g |= ((c >> 1) & 1) << (7 - j)
            b |= ((c >> 2) & 1) << (7 - j)
            c >>= 3
        cmap[i] = [r, g, b]
    return cmap


COLORMAP = ade20k_colormap()


def classify_scene(yolo_detections: list, seg_mask: np.ndarray) -> str:
    """
    Rule-based scene classifier — CPU only, <1ms.

    Args:
        yolo_detections: list of class-name strings from current YOLO frame.
        seg_mask:        full H×W uint8 array of ADE20K class indices (mask_full).

    Returns:
        One of: "staircase" | "lab_workshop" | "doorway" | "corridor" | "unknown"
    """
    seg_counts   = Counter(seg_mask.flatten().tolist())
    total_pixels = seg_mask.size

    floor_ratio = seg_counts.get(3,  0) / total_pixels
    wall_ratio  = seg_counts.get(0,  0) / total_pixels
    stair_ratio = seg_counts.get(29, 0) / total_pixels
    door_ratio  = seg_counts.get(52, 0) / total_pixels

    yolo_set = set(yolo_detections)

    if yolo_set & STAIR_OBJECTS or stair_ratio > 0.05:
        return "staircase"
    elif len(yolo_set & LAB_OBJECTS) >= 2:
        return "lab_workshop"
    elif 'dell-e' in yolo_set:
        return "lab_workshop"
    elif door_ratio > 0.05:
        return "doorway"
    elif floor_ratio > 0.3 and wall_ratio > 0.2 and not (yolo_set & LAB_OBJECTS):
        return "corridor"
    else:
        return "unknown"


class SegformerNode(Node):

    def __init__(self):
        super().__init__('segformer_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('model_name',      'nvidia/segformer-b0-finetuned-ade-512-512')
        self.declare_parameter('device',          'cuda')
        self.declare_parameter('input_size',      512)
        self.declare_parameter('process_every_n', 1)

        model_name      = self.get_parameter('model_name').value
        device_str      = self.get_parameter('device').value
        self.input_size = self.get_parameter('input_size').value
        self.every_n    = self.get_parameter('process_every_n').value

        # ── Device setup ──────────────────────────────────────────────────
        self.device = torch.device(
            device_str if torch.cuda.is_available() else 'cpu'
        )
        self.get_logger().info(f'Using device: {self.device}')

        # ── Load model ────────────────────────────────────────────────────
        self.get_logger().info(f'Loading {model_name} …')
        self.processor = SegformerImageProcessor.from_pretrained(model_name)
        self.model = SegformerForSemanticSegmentation.from_pretrained(model_name)
        self.model.to(self.device).eval()
        self.get_logger().info('SegFormer-B0 loaded ✓')

        # ── State ─────────────────────────────────────────────────────────
        self.bridge          = CvBridge()
        self._frame_count    = 0
        self._yolo_detections: list = []   # updated via /yolo/detection_classes

        # ── QoS ───────────────────────────────────────────────────────────
        qos_cam = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)

        # ── Subscribers ───────────────────────────────────────────────────
        self.sub_image = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self._image_callback,
            qos_cam,           # BEST_EFFORT — match camera publisher
        )
        # Receives JSON list of YOLO class names, e.g. '["shelf", "work-desk"]'
        # Published by yolo_detector.py on every detection frame
        self.sub_yolo = self.create_subscription(
            String,
            '/yolo/detection_classes',
            self._yolo_callback,
            qos_rel,
        )

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_filtered = self.create_publisher(Image,  '/semantic/segformer/mask_filtered', qos_rel)
        self.pub_full     = self.create_publisher(Image,  '/semantic/segformer/mask_full',      qos_rel)
        self.pub_label    = self.create_publisher(String, '/semantic/segformer/scene_label',    qos_rel)
        self.pub_colored  = self.create_publisher(Image,  '/semantic/segformer/image_colored',  qos_rel)

        self.get_logger().info('segformer_node ready — waiting for images…')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _yolo_callback(self, msg: String):
        """Receive latest YOLO detection class names for classify_scene()."""
        try:
            self._yolo_detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON from /yolo/detection_classes')
            self._yolo_detections = []

    def _image_callback(self, msg: Image):
        self._frame_count += 1
        if self._frame_count % self.every_n != 0:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        # ── Inference ─────────────────────────────────────────────────────
        mask_full = self._run_inference(bgr)

        # ── Filtered mask → Nav2 semantic costmap ─────────────────────────
        mask_filtered = self._build_filtered_mask(mask_full)

        # ── Scene classification ──────────────────────────────────────────
        scene = classify_scene(self._yolo_detections, mask_full)

        # ── Colourised visualisation ──────────────────────────────────────
        colored = self._colorize(mask_full)

        # ── Publish ───────────────────────────────────────────────────────
        header = self._make_header(msg.header.stamp, msg.header.frame_id)

        img_full = self.bridge.cv2_to_imgmsg(mask_full,     encoding='mono8')
        img_full.header = header
        self.pub_full.publish(img_full)

        img_filt = self.bridge.cv2_to_imgmsg(mask_filtered, encoding='mono8')
        img_filt.header = header
        self.pub_filtered.publish(img_filt)

        img_col = self.bridge.cv2_to_imgmsg(colored,        encoding='rgb8')
        img_col.header = header
        self.pub_colored.publish(img_col)

        label_msg      = String()
        label_msg.data = scene
        self.pub_label.publish(label_msg)

    # ── Helpers ───────────────────────────────────────────────────────────

    def _run_inference(self, bgr_image: np.ndarray) -> np.ndarray:
        """Return H×W uint8 mask of ADE20K class indices (original resolution)."""
        orig_h, orig_w = bgr_image.shape[:2]
        rgb = bgr_image[:, :, ::-1]  # BGR → RGB

        inputs = self.processor(
            images=rgb,
            return_tensors='pt',
            size={'height': self.input_size, 'width': self.input_size},
        )
        pixel_values = inputs['pixel_values'].to(self.device)

        with torch.no_grad():
            logits = self.model(pixel_values=pixel_values).logits  # (1, C, H/4, W/4)

        logits_up = F.interpolate(
            logits,
            size=(orig_h, orig_w),
            mode='bilinear',
            align_corners=False,
        )
        return logits_up.argmax(dim=1).squeeze(0).cpu().numpy().astype(np.uint8)

    def _build_filtered_mask(self, full_mask: np.ndarray) -> np.ndarray:
        """Zero out classes not in KEEP_CLASSES."""
        return np.where(
            np.isin(full_mask, list(KEEP_CLASSES)), full_mask, 0
        ).astype(np.uint8)

    def _colorize(self, mask: np.ndarray) -> np.ndarray:
        """Map class indices → RGB using ADE20K colormap."""
        colored       = np.zeros((*mask.shape, 3), dtype=np.uint8)
        valid         = mask < 150
        colored[valid] = COLORMAP[mask[valid]]
        return colored

    @staticmethod
    def _make_header(stamp, frame_id: str) -> Header:
        h            = Header()
        h.stamp      = stamp
        h.frame_id   = frame_id
        return h


def main(args=None):
    rclpy.init(args=args)
    node = SegformerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
