"""
test_cleanup.py
Verifikasi bahwa cleanup live-only berhasil:
  - File rosbag sudah tidak ada
  - File live mode masih ada dan run.sh executable
"""

import sys
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PASS = "\033[0;32m✓ PASS\033[0m"
FAIL = "\033[0;31m✗ FAIL\033[0m"

results = []


def check_absent(label, relpath):
    full = os.path.join(ROOT, relpath)
    absent = not os.path.exists(full)
    status = PASS if absent else FAIL
    print(f"  {status}  HAPUS: {relpath}" + ("" if absent else " (masih ada!)"))
    results.append(absent)


def check_present(label, relpath, executable=False):
    full = os.path.join(ROOT, relpath)
    exists = os.path.exists(full)
    if exists and executable:
        exists = os.access(full, os.X_OK)
        label = label + " (executable)"
    status = PASS if exists else FAIL
    print(f"  {status}  ADA  : {relpath}" + ("" if exists else " (tidak ditemukan!)"))
    results.append(exists)


print("test_cleanup.py — live-only cleanup verifikasi\n")

print("[ Cleanup — harus sudah dihapus ]\n")
check_absent("run_rosbag.sh",       "run_rosbag.sh")
check_absent("run_rosbag_step1.sh", "run_rosbag_step1.sh")
check_absent("run_rosbag_step2.sh", "run_rosbag_step2.sh")
check_absent("run_rosbag_step3.sh", "run_rosbag_step3.sh")
check_absent("run_live.sh",         "run_live.sh")
check_absent("run_semantic.sh",     "run_semantic.sh")
check_absent("run_step3_live.sh",   "run_step3_live.sh")
check_absent("launch/rosbag/",      "launch/rosbag")
check_absent("config/laptop/",                        "config/laptop")
check_absent("rtabmap_allinone/",                     "rtabmap_allinone")
check_absent("sanity_check.py",                       "sanity_check.py")
check_absent("decompress_depth_node.py",              "decompress_depth_node.py")
check_absent("launch/step1_yolo_detection.launch.py", "launch/step1_yolo_detection.launch.py")
check_absent("launch/step2_semantic_projection.launch.py", "launch/step2_semantic_projection.launch.py")

print("\n[ Live mode — harus masih ada ]\n")
check_present("run.sh",                              "run.sh",                              executable=True)
check_present("launch/live/static_tf_live.launch.py","launch/live/static_tf_live.launch.py")
check_present("launch/live/rtabmap_live.launch.py",  "launch/live/rtabmap_live.launch.py")
check_present("launch/semantic_only.launch.py",      "launch/semantic_only.launch.py")
check_present("semantic/yolo_detector.py",           "semantic/yolo_detector.py")
check_present("semantic/semantic_projector.py",      "semantic/semantic_projector.py")
check_present("semantic/semantic_grid_node.py",      "semantic/semantic_grid_node.py")
check_present("decompress_depth_fix.py",             "decompress_depth_fix.py")
check_present("capture_semantic_map.py",             "capture_semantic_map.py")
check_present("tests/test_tf.py",                    "tests/test_tf.py")
check_present("tests/test_topics.py",                "tests/test_topics.py")
check_present("tests/test_models.py",                "tests/test_models.py")
check_present("tests/test_semantic_nodes.py",        "tests/test_semantic_nodes.py")
check_present("semantic_results/ folder",            "semantic_results")
check_present("semantic/object_goal_nav.py",         "semantic/object_goal_nav.py")
check_present("semantic/semantic_map_viz.py",        "semantic/semantic_map_viz.py")
check_present("semantic/semantic_map_exporter.py",   "semantic/semantic_map_exporter.py")
check_present("config/nav2_params.yaml",             "config/nav2_params.yaml")
check_present("tests/test_nav2_params.py",           "tests/test_nav2_params.py")
check_present("tests/test_semantic_exporter.py",     "tests/test_semantic_exporter.py")
check_present("tests/test_object_goal_nav.py",       "tests/test_object_goal_nav.py")
check_present("tests/test_deploy.py",                "tests/test_deploy.py")

print()
passed = results.count(True)
failed = results.count(False)
if failed == 0:
    print(f"Semua {passed} check PASS — cleanup berhasil.")
    sys.exit(0)
else:
    print(f"{failed} check FAIL, {passed} PASS.")
    sys.exit(1)
