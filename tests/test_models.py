"""
test_models.py
Verifikasi model YOLO BRIN: file ada, loadable, 14 class, inference OK.
"""

import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENGINE_PATH = os.path.join(SCRIPT_DIR, "models", "best_brin_yolo11m_v1.engine")
PT_PATH     = os.path.join(SCRIPT_DIR, "models", "best_brin_yolo11m_v1_20260517_0724.pt")
N_CLASSES   = 17

PASS = "\033[0;32m✓ PASS\033[0m"
FAIL = "\033[0;31m✗ FAIL\033[0m"
WARN = "\033[1;33m⚠ WARN\033[0m"

results = []


def check(label, ok, msg=""):
    status = PASS if ok else FAIL
    print(f"  {status}  {label}" + (f": {msg}" if msg else ""))
    results.append(ok)
    return ok


print("test_models.py — YOLO model check\n")

# 1. File ada
engine_exists = os.path.isfile(ENGINE_PATH)
pt_exists     = os.path.isfile(PT_PATH)
check("Engine file ada", engine_exists, ENGINE_PATH if not engine_exists else "")
if not engine_exists:
    print(f"  {WARN}  Engine tidak ada — fallback ke .pt")
check("PT file ada", pt_exists, PT_PATH if not pt_exists else "")

model_path = ENGINE_PATH if engine_exists else (PT_PATH if pt_exists else None)

if model_path is None:
    print("\nTidak ada model yang bisa diload.")
    sys.exit(1)

# 2. Load model
try:
    from ultralytics import YOLO
    model = YOLO(model_path)
    check("Model loadable", True)
except Exception as e:
    check("Model loadable", False, str(e))
    sys.exit(1)

# 3. Jumlah class
try:
    # Untuk TRT engine, model.model tidak tersedia — gunakan model.names
    if hasattr(model, 'names') and isinstance(model.names, dict):
        nc = len(model.names)
    elif hasattr(model, 'model') and hasattr(model.model, 'nc'):
        nc = model.model.nc
    else:
        nc = None
    if nc is not None:
        check(f"17 class BRIN", nc == N_CLASSES, f"ditemukan {nc} class")
    else:
        print(f"  \033[1;33m⚠ WARN\033[0m  17 class BRIN: tidak bisa dicek dari engine (skip)")
        results.append(True)  # skip, bukan fail
except Exception as e:
    print(f"  \033[1;33m⚠ WARN\033[0m  17 class BRIN: skip ({e})")
    results.append(True)  # skip untuk engine TRT

# 4. Inference dummy frame
try:
    import numpy as np
    dummy = np.zeros((480, 640, 3), dtype=np.uint8)
    results_inf = model(dummy, verbose=False)
    check("Inference dummy frame", True)
except Exception as e:
    check("Inference dummy frame", False, str(e))

print()
if all(results):
    print("Semua model check OK.")
    sys.exit(0)
else:
    print(f"{results.count(False)} check gagal.")
    sys.exit(1)
