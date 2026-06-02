#!/usr/bin/env python3
"""
decompress_depth_fix.py — Decompress compressedDepth topic ke raw 16UC1

Kenapa file ini ada:
  ros2 run image_transport republish compressedDepth → tidak support di
  ROS2 Humble karena SubscriberPlugin interface berubah. Script ini
  pakai cv_bridge + numpy langsung sebagai workaround.

Usage (standalone):
  python3 decompress_depth_fix.py

Usage (dari launch file dengan argumen):
  python3 decompress_depth_fix.py \
    --input_topic  /camera/camera/aligned_depth_to_color/image_raw/compressedDepth \
    --output_topic /camera/depth/image_rect_raw

Usage (via ros-args jika dipanggil dari launch):
  python3 decompress_depth_fix.py --ros-args \
    -p input_topic:=/camera/camera/aligned_depth_to_color/image_raw/compressedDepth \
    -p output_topic:=/camera/depth/image_rect_raw
"""

import sys
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo


def parse_args():
    """Parse argumen standar (bukan --ros-args). Toleran terhadap keduanya."""
    parser = argparse.ArgumentParser(
        description='Decompress compressedDepth → raw 16UC1',
        add_help=False  # jangan crash kalau ada --ros-args
    )
    parser.add_argument('--input_topic',
        default='/camera/camera/aligned_depth_to_color/image_raw/compressedDepth',
        help='Topic CompressedImage (compressedDepth format)')
    parser.add_argument('--output_topic',
        default='/camera/depth/image_rect_raw',
        help='Topic Image output (16UC1 raw)')

    # Filter --ros-args dan seterusnya sebelum parse
    filtered = []
    skip_next = False
    for arg in sys.argv[1:]:
        if skip_next:
            skip_next = False
            continue
        if arg == '--ros-args':
            break   # semua setelah --ros-args adalah ROS args, abaikan
        if arg in ('-p', '--remap', '-r', '--enclave', '-e'):
            skip_next = True
            continue
        if arg.startswith('__'):
            continue
        filtered.append(arg)

    args, _ = parser.parse_known_args(filtered)
    return args


class DepthDecompressor(Node):

    def __init__(self, input_topic: str, output_topic: str):
        super().__init__('decompress_depth_fix')

        # Coba baca dari ROS parameter dulu (kalau dipanggil via --ros-args)
        self.declare_parameter('input_topic', input_topic)
        self.declare_parameter('output_topic', output_topic)

        in_topic  = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.get_logger().info(f'Input  : {in_topic}')
        self.get_logger().info(f'Output : {out_topic}')

        self._pub = self.create_publisher(Image, out_topic, 10)
        self._sub = self.create_subscription(
            CompressedImage, in_topic, self._callback, 10)

        self._count = 0

    def _callback(self, msg: CompressedImage):
        try:
            img_msg = self._decompress(msg)
            self._pub.publish(img_msg)
            self._count += 1
            if self._count % 100 == 1:
                self.get_logger().info(
                    f'Decompressed {self._count} depth frames — '
                    f'format: {msg.format}'
                )
        except Exception as e:
            self.get_logger().warn(f'Decompress error: {e}')

    def _decompress(self, msg: CompressedImage) -> Image:
        """
        Decompress CompressedImage (compressedDepth) → Image 16UC1.

        Format yang dihandle:
          - '16UC1; compressedDepth png'   → PNG berisi uint16
          - '16UC1; compressedDepth'        → raw prefix + zlib atau PNG
          - format lain → fallback via cv2 imdecode
        """
        data = bytes(msg.data)

        img = Image()
        img.header    = msg.header
        img.encoding  = '16UC1'
        img.is_bigendian = 0

        fmt = msg.format.lower()

        # ── Kasus 1: PNG yang berisi depth uint16 ─────────────────────────
        if 'png' in fmt:
            import cv2
            arr = np.frombuffer(data, dtype=np.uint8)
            depth_img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
            if depth_img is None:
                raise ValueError('cv2.imdecode gagal (PNG)')
            if depth_img.dtype != np.uint16:
                depth_img = depth_img.astype(np.uint16)
            img.height = depth_img.shape[0]
            img.width  = depth_img.shape[1]
            img.step   = img.width * 2
            img.data   = depth_img.tobytes()

        # ── Kasus 2: compressedDepth dengan header 12-byte ────────────────
        # Format: [4 byte depth_quantization_factor float][4 byte max_val float]
        #         [4 byte depth_range float] + zlib compressed data
        elif 'compresseddepth' in fmt and 'png' not in fmt:
            # Skip 12-byte header (3x float32)
            try:
                import zlib
                raw = zlib.decompress(data[12:])
                arr = np.frombuffer(raw, dtype=np.uint16)

                # Estimasi dimensi dari ukuran data
                # Coba aspect ratio umum D455 (640x480, 848x480, 1280x720)
                candidates = [(640, 480), (848, 480), (1280, 720), (424, 240)]
                h, w = None, None
                for cw, ch in candidates:
                    if arr.size == cw * ch:
                        w, h = cw, ch
                        break
                if h is None:
                    # Fallback: ambil dari cache camera_info kalau ada
                    raise ValueError(
                        f'Tidak bisa determine dimensi dari {arr.size} pixels. '
                        f'Pastikan camera_info tersedia.'
                    )

                depth_img = arr.reshape(h, w)
                img.height = h
                img.width  = w
                img.step   = w * 2
                img.data   = depth_img.tobytes()

            except Exception:
                # Fallback ke cv2
                import cv2
                arr = np.frombuffer(data, dtype=np.uint8)
                depth_img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
                if depth_img is None:
                    raise
                if depth_img.dtype != np.uint16:
                    depth_img = depth_img.astype(np.uint16)
                img.height = depth_img.shape[0]
                img.width  = depth_img.shape[1]
                img.step   = img.width * 2
                img.data   = depth_img.tobytes()

        # ── Kasus 3: Fallback universal via cv2 ───────────────────────────
        else:
            import cv2
            arr = np.frombuffer(data, dtype=np.uint8)
            depth_img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
            if depth_img is None:
                raise ValueError(f'cv2.imdecode gagal untuk format: {msg.format}')
            if depth_img.dtype != np.uint16:
                depth_img = depth_img.astype(np.uint16)
            img.height = depth_img.shape[0]
            img.width  = depth_img.shape[1]
            img.step   = img.width * 2
            img.data   = depth_img.tobytes()

        return img


def main():
    args = parse_args()

    rclpy.init()
    node = DepthDecompressor(
        input_topic=args.input_topic,
        output_topic=args.output_topic,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
