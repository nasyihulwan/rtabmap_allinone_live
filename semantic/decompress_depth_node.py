#!/usr/bin/env python3
"""
decompress_depth_node.py — [H2/J1] Decompress compressedDepth → raw 16UC1

Diporting dari ws/project/rtabmap_allinone/decompress_depth_node.py (repo
lama, read-only, cuma referensi — TIDAK diedit). Alasan file ini ada:
`ros2 run image_transport republish compressedDepth` tidak bekerja di
ROS2 Humble (SubscriberPlugin::subscribeImpl error, bug dikonfirmasi di
repo lama) — jadi dipakai node kustom cv2+numpy sebagai workaround.

Topik disesuaikan dengan konvensi rtabmap_allinone_live SEKARANG (BEDA
dari repo lama yang publish ke /camera/depth/image_rect_raw):
  IN : /camera/camera/aligned_depth_to_color/image_raw/compressedDepth
  OUT: /camera/camera/aligned_depth_to_color/image_raw   (persis nama yang
       di-subscribe semantic_projector.py — lihat sub_dep di sana)

Tidak relay camera_info — semantic_projector.py hanya butuh
/camera/camera/color/camera_info (intrinsics warna, dipakai untuk depth
yang sudah di-align), dan topik itu SUDAH raw CameraInfo di bag (tidak
terkompresi), tidak perlu didekompresi/direlay.

Usage (standalone):
    python3 decompress_depth_node.py

Usage (via ExecuteProcess):
    ExecuteProcess(cmd=['python3', 'decompress_depth_node.py'])
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
import cv2

INPUT_TOPIC  = '/camera/camera/aligned_depth_to_color/image_raw/compressedDepth'
OUTPUT_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'


class DecompressDepthNode(Node):

    def __init__(self):
        super().__init__('decompress_depth')

        self.sub_compressed = self.create_subscription(
            CompressedImage, INPUT_TOPIC, self.callback_depth, 10)
        self.pub_depth = self.create_publisher(Image, OUTPUT_TOPIC, 10)

        self._count = 0
        self.get_logger().info(
            f'decompress_depth_node ready\n  IN : {INPUT_TOPIC}\n  OUT: {OUTPUT_TOPIC}')

    def callback_depth(self, msg: CompressedImage):
        """
        Decode compressedDepth → 16UC1 raw Image.

        Format compressedDepth: 12-byte header opsional (width, height,
        depth_quantization, compression_method) diikuti data PNG/RVL.
        Beberapa varian tidak punya header — dicoba beberapa cara decode
        berurutan, sama seperti versi asli di repo lama.
        """
        try:
            data = bytes(msg.data)

            if len(data) > 16 and data[12:16] == b'\x89PNG':
                png_data = np.frombuffer(data[12:], dtype=np.uint8)
            elif data[:8] == b'\x89PNG\r\n\x1a\n':
                png_data = np.frombuffer(data, dtype=np.uint8)
            else:
                png_data = np.frombuffer(data[12:], dtype=np.uint8)

            depth_cv = cv2.imdecode(png_data, cv2.IMREAD_UNCHANGED)

            if depth_cv is None:
                raw = np.frombuffer(data, dtype=np.uint8)
                depth_cv = cv2.imdecode(raw, cv2.IMREAD_UNCHANGED)

            if depth_cv is None:
                self.get_logger().warn('Gagal decode depth frame', throttle_duration_sec=5.0)
                return

            if depth_cv.dtype != np.uint16:
                depth_cv = depth_cv.astype(np.uint16)

            out = Image()
            out.header       = msg.header
            out.height       = depth_cv.shape[0]
            out.width        = depth_cv.shape[1]
            out.encoding     = '16UC1'
            out.is_bigendian = False
            out.step         = depth_cv.shape[1] * 2
            out.data         = depth_cv.tobytes()

            self.pub_depth.publish(out)
            self._count += 1
            if self._count % 100 == 1:
                self.get_logger().info(f'Depth: {self._count} frame ter-decompress')

        except Exception as e:
            self.get_logger().error(f'Error decompress depth: {e}', throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = DecompressDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
