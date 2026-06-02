"""
test_topics.py
Verifikasi sensor topics ada dan memenuhi target Hz.
Requires: ROS2 running, sensor nodes aktif.
"""

import sys
import subprocess
import threading
import time

PASS = "\033[0;32m✓ PASS\033[0m"
FAIL = "\033[0;31m✗ FAIL\033[0m"
WARN = "\033[1;33m⚠ WARN\033[0m"

SAMPLE_SECS = 3
TOPICS = [
    ("/velodyne_points",                       8.0),
    ("/camera/camera/color/image_raw",         15.0),
    ("/camera/camera/depth/image_rect_raw",    15.0),
    ("/a200_1060/sensors/imu_0/data",           0.0),  # hanya cek ada
]
ODOM_TOPICS = [
    "/a200_1060/platform/odom/filtered",
    "/odometry/filtered",
    "/odom",
]
ODOM_MIN_HZ = 45.0

results = []


def get_topic_list():
    try:
        out = subprocess.check_output(
            ["ros2", "topic", "list"], timeout=5, stderr=subprocess.DEVNULL
        ).decode()
        return set(out.strip().splitlines())
    except Exception:
        return set()


def measure_hz(topic, duration=SAMPLE_SECS):
    """Returns average Hz over `duration` seconds, or None if topic not found."""
    lines = []
    proc = subprocess.Popen(
        ["ros2", "topic", "hz", topic, "--window", "20"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        text=True
    )
    try:
        end = time.time() + duration
        while time.time() < end:
            line = proc.stdout.readline()
            if line:
                lines.append(line)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()

    for line in reversed(lines):
        if "average rate:" in line:
            try:
                return float(line.split("average rate:")[-1].strip().split()[0])
            except ValueError:
                pass
    return None


def check_topic(topic, min_hz):
    topic_list = get_topic_list()
    if topic not in topic_list:
        print(f"  {WARN}  {topic}: tidak muncul (pastikan sensor aktif)")
        results.append(False)
        return

    if min_hz <= 0:
        print(f"  {PASS}  {topic}: ada")
        results.append(True)
        return

    hz = measure_hz(topic)
    if hz is None:
        print(f"  {WARN}  {topic}: ada tapi Hz tidak terukur")
        results.append(False)
        return

    ok = hz >= min_hz
    status = PASS if ok else FAIL
    print(f"  {status}  {topic}: {hz:.1f} Hz (target ≥ {min_hz})")
    results.append(ok)


print("test_topics.py — sensor topic check\n")
print(f"  Sampling {SAMPLE_SECS}s per topic...\n")

for topic, min_hz in TOPICS:
    check_topic(topic, min_hz)

# Odom — terima salah satu
topic_list = get_topic_list()
odom_found = next((t for t in ODOM_TOPICS if t in topic_list), None)
if odom_found:
    check_topic(odom_found, ODOM_MIN_HZ)
else:
    print(f"  {WARN}  odom: tidak ada ({', '.join(ODOM_TOPICS)})")
    results.append(False)

print()
if all(results):
    print("Semua topic OK.")
    sys.exit(0)
else:
    print(f"{results.count(False)} topic bermasalah.")
    sys.exit(1)
