"""
test_tf.py
Verifikasi TF tree: transform kritis ada dan nilai sesuai konfigurasi.
Requires: ROS2 running, static TF sudah dipublish.
"""

import sys
import subprocess
import json

PASS = "\033[0;32m✓ PASS\033[0m"
FAIL = "\033[0;31m✗ FAIL\033[0m"
WARN = "\033[1;33m⚠ WARN\033[0m"

TOLERANCE = 0.01  # meter / radian

# (source_frame, target_frame, expected_translation_xyz or None)
TF_CHECKS = [
    ("a200_1060/base_link", "velodyne",
     (0.1, 0.0, 0.5)),
    ("a200_1060/base_link", "camera_link",
     (0.2, 0.0, 0.25)),
    ("base_link", "a200_1060/base_link",
     (0.0, 0.0, 0.0)),
]

results = []


def lookup_tf(source, target, timeout=5):
    """Returns (x, y, z) translation or None on failure."""
    import time
    try:
        cmd = (
            "source /opt/ros/humble/setup.bash 2>/dev/null; "
            f"ros2 run tf2_ros tf2_echo '{source}' '{target}'"
        )
        proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True
        )
        end = time.time() + timeout
        while time.time() < end:
            line = proc.stdout.readline()
            if not line:
                time.sleep(0.05)
                continue
            if "- Translation:" in line:
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except Exception:
                    proc.kill()
                # Format: "- Translation: [0.100, 0.000, 0.500]"
                inner = line.split("[")[-1].replace("]", "").strip()
                parts = inner.split(",")
                return tuple(float(p.strip()) for p in parts)
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except Exception:
            proc.kill()
    except Exception:
        pass
    return None


def near(a, b, tol=TOLERANCE):
    return abs(a - b) <= tol


print("test_tf.py — TF tree check\n")

for src, tgt, expected_xyz in TF_CHECKS:
    tf = lookup_tf(src, tgt)
    label = f"{src} → {tgt}"
    if tf is None:
        print(f"  {FAIL}  {label}: transform tidak ditemukan")
        results.append(False)
        continue

    if expected_xyz is None:
        print(f"  {PASS}  {label}: ada")
        results.append(True)
        continue

    x_ok = near(tf[0], expected_xyz[0])
    y_ok = near(tf[1], expected_xyz[1])
    z_ok = near(tf[2], expected_xyz[2])
    all_ok = x_ok and y_ok and z_ok
    status = PASS if all_ok else FAIL
    print(f"  {status}  {label}")
    if not all_ok:
        print(f"           ditemukan:  x={tf[0]:.3f} y={tf[1]:.3f} z={tf[2]:.3f}")
        print(f"           diharapkan: x={expected_xyz[0]:.3f} y={expected_xyz[1]:.3f} z={expected_xyz[2]:.3f}")
    results.append(all_ok)

print()
if all(results):
    print("Semua TF OK.")
    sys.exit(0)
else:
    print(f"{results.count(False)} TF bermasalah.")
    sys.exit(1)
