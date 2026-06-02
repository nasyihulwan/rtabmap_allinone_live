"""
test_semantic_nodes.py
Verifikasi bahwa semua semantic node bisa diimport tanpa error.
"""

import sys
import os
import importlib.util

SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

TARGETS = [
    ("yolo_detector",       os.path.join(SCRIPT_DIR, "semantic", "yolo_detector.py")),
    ("semantic_projector",  os.path.join(SCRIPT_DIR, "semantic", "semantic_projector.py")),
    ("semantic_grid_node",  os.path.join(SCRIPT_DIR, "semantic", "semantic_grid_node.py")),
    ("decompress_depth_fix",os.path.join(SCRIPT_DIR, "decompress_depth_fix.py")),
]

PASS = "\033[0;32m✓ PASS\033[0m"
FAIL = "\033[0;31m✗ FAIL\033[0m"


def test_importable(name, path):
    if not os.path.isfile(path):
        print(f"  {FAIL}  {name}: file tidak ditemukan ({path})")
        return False
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        print(f"  {PASS}  {name}")
        return True
    except Exception as e:
        print(f"  {FAIL}  {name}: {e}")
        return False


if __name__ == "__main__":
    print("test_semantic_nodes.py — import check\n")
    results = [test_importable(n, p) for n, p in TARGETS]
    print()
    if all(results):
        print("Semua import OK.")
        sys.exit(0)
    else:
        print(f"{results.count(False)} file gagal diimport.")
        sys.exit(1)
