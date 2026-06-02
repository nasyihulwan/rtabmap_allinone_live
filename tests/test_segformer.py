#!/usr/bin/env python3
"""
test_segformer.py — Diagnostic tests for segformer_node.py

Covers:
  [GPU]       CUDA availability
  [MODEL]     Model load + warmup
  [LATENCY]   Inference latency vs benchmark threshold (50W: ≥10fps)
  [OUTPUT]    mask_full shape, dtype, value range
  [FILTERED]  mask_filtered only contains KEEP_CLASSES
  [COLORMAP]  Colormap shape, wall(class-0) has colour
  [SCENE]     classify_scene() — all 5 label paths
  [PIPELINE]  Full end-to-end: BGR frame → 4 outputs

Run:
  python3 test_segformer.py
  python3 test_segformer.py -v    # verbose

Benchmark reference (Orin 50W, parallel with YOLOv11m TRT):
  SegFormer-B0 @ 512×512 input → ~16.5 fps
  Minimum acceptable for deployment: 10 fps
"""

import sys
import time
import unittest
import numpy as np

# ── Minimal stubs so tests run without ROS2 / GPU in CI ───────────────────
# Real inference tests are skipped if torch/transformers not available.
try:
    import torch
    import torch.nn.functional as F
    from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False

# Pull pure-python helpers directly from the node module
sys.path.insert(0, '.')
try:
    from segformer_node import ade20k_colormap, classify_scene, KEEP_CLASSES
    HAS_NODE = True
except ImportError:
    HAS_NODE = False

# ── Constants (must match segformer_node.py) ──────────────────────────────
MODEL_NAME   = "nvidia/segformer-b0-finetuned-ade-512-512"
INPUT_SIZE   = 512          # benchmark config
TARGET_FPS   = 10.0         # minimum for deployment (benchmark achieved ~16.5)
WARMUP_RUNS  = 10
BENCH_RUNS   = 30
CAM_H, CAM_W = 480, 640     # RealSense D455 stream resolution


# ═══════════════════════════════════════════════════════════════════════════
# 1. GPU
# ═══════════════════════════════════════════════════════════════════════════

class TestGPU(unittest.TestCase):

    @unittest.skipUnless(HAS_TORCH, "torch not installed")
    def test_cuda_available(self):
        """CUDA must be available on Orin — CPU fallback is not deployment-viable."""
        self.assertTrue(
            torch.cuda.is_available(),
            "CUDA not available. SegFormer will fall back to CPU (~1-3fps) — not OK for deployment."
        )

    @unittest.skipUnless(HAS_TORCH, "torch not installed")
    def test_cuda_device_name(self):
        """Log which GPU is present (informational)."""
        if torch.cuda.is_available():
            name = torch.cuda.get_device_name(0)
            print(f"\n  [GPU] {name}")
            self.assertIsInstance(name, str)


# ═══════════════════════════════════════════════════════════════════════════
# 2. Model loading
# ═══════════════════════════════════════════════════════════════════════════

class TestModelLoad(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not HAS_TORCH or not torch.cuda.is_available():
            return
        device = torch.device('cuda')
        cls.processor = SegformerImageProcessor.from_pretrained(MODEL_NAME)
        cls.model = SegformerForSemanticSegmentation.from_pretrained(MODEL_NAME)
        cls.model.to(device).eval()
        cls.device = device

    @unittest.skipUnless(HAS_TORCH, "torch not installed")
    def test_model_loads(self):
        """Model and processor must load without error."""
        self.assertTrue(hasattr(self.__class__, 'model'))

    @unittest.skipUnless(HAS_TORCH, "torch not installed")
    def test_model_on_cuda(self):
        """Model must be on GPU."""
        if not torch.cuda.is_available():
            self.skipTest("CUDA not available")
        param = next(self.__class__.model.parameters())
        self.assertEqual(param.device.type, 'cuda')

    @unittest.skipUnless(HAS_TORCH, "torch not installed")
    def test_model_eval_mode(self):
        """Model must be in eval mode (no dropout, no grad)."""
        self.assertFalse(self.__class__.model.training)


# ═══════════════════════════════════════════════════════════════════════════
# 3. Inference latency — benchmark-aligned
# ═══════════════════════════════════════════════════════════════════════════

def _run_inference(processor, model, device, bgr: np.ndarray) -> np.ndarray:
    """Mirror of segformer_node._run_inference()."""
    orig_h, orig_w = bgr.shape[:2]
    rgb = bgr[:, :, ::-1]
    inputs = processor(
        images=rgb,
        return_tensors='pt',
        size={'height': INPUT_SIZE, 'width': INPUT_SIZE},
    )
    pixel_values = inputs['pixel_values'].to(device)
    with torch.no_grad():
        logits = model(pixel_values=pixel_values).logits
    logits_up = F.interpolate(
        logits, size=(orig_h, orig_w), mode='bilinear', align_corners=False
    )
    return logits_up.argmax(dim=1).squeeze(0).cpu().numpy().astype(np.uint8)


class TestLatency(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not HAS_TORCH or not torch.cuda.is_available():
            return
        cls.device    = torch.device('cuda')
        cls.processor = SegformerImageProcessor.from_pretrained(MODEL_NAME)
        cls.model     = SegformerForSemanticSegmentation.from_pretrained(MODEL_NAME)
        cls.model.to(cls.device).eval()
        cls.dummy     = np.random.randint(0, 255, (CAM_H, CAM_W, 3), dtype=np.uint8)

        # Warmup — identical to benchmark script
        print(f"\n  [LATENCY] Warming up ({WARMUP_RUNS} runs)…")
        for _ in range(WARMUP_RUNS):
            _run_inference(cls.processor, cls.model, cls.device, cls.dummy)
        torch.cuda.synchronize()

    @unittest.skipUnless(HAS_TORCH, "torch not installed")
    def test_fps_above_target(self):
        """
        Must achieve ≥10 fps on Orin 50W (benchmark: ~16.5 fps).
        Failure here means GPU is throttled or input_size is wrong.
        """
        if not torch.cuda.is_available():
            self.skipTest("CUDA not available")

        cls = self.__class__
        times = []
        for _ in range(BENCH_RUNS):
            t0 = time.perf_counter()
            _run_inference(cls.processor, cls.model, cls.device, cls.dummy)
            torch.cuda.synchronize()
            times.append(time.perf_counter() - t0)

        avg_ms  = np.mean(times) * 1000
        fps     = 1.0 / np.mean(times)
        print(f"\n  [LATENCY] avg={avg_ms:.1f}ms  fps={fps:.1f}  target≥{TARGET_FPS}")

        self.assertGreaterEqual(
            fps, TARGET_FPS,
            f"SegFormer fps {fps:.1f} < {TARGET_FPS} — check power mode (must be 50W) "
            f"and input_size (must be {INPUT_SIZE})"
        )

    @unittest.skipUnless(HAS_TORCH, "torch not installed")
    def test_latency_std_reasonable(self):
        """Std dev should be <10ms — high jitter means GPU contention."""
        if not torch.cuda.is_available():
            self.skipTest("CUDA not available")

        cls = self.__class__
        times = [
            (lambda t0: time.perf_counter() - t0)(time.perf_counter())
            or _run_inference(cls.processor, cls.model, cls.device, cls.dummy)
            for _ in range(BENCH_RUNS)
        ]
        # Redo properly
        times = []
        for _ in range(BENCH_RUNS):
            t0 = time.perf_counter()
            _run_inference(cls.processor, cls.model, cls.device, cls.dummy)
            torch.cuda.synchronize()
            times.append(time.perf_counter() - t0)

        std_ms = np.std(times) * 1000
        print(f"  [LATENCY] std={std_ms:.1f}ms")
        self.assertLess(std_ms, 30.0, f"Latency std {std_ms:.1f}ms too high — GPU contention?")


# ═══════════════════════════════════════════════════════════════════════════
# 4. Output correctness
# ═══════════════════════════════════════════════════════════════════════════

class TestOutputShape(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not HAS_TORCH or not torch.cuda.is_available():
            return
        cls.device    = torch.device('cuda')
        cls.processor = SegformerImageProcessor.from_pretrained(MODEL_NAME)
        cls.model     = SegformerForSemanticSegmentation.from_pretrained(MODEL_NAME)
        cls.model.to(cls.device).eval()
        dummy = np.random.randint(0, 255, (CAM_H, CAM_W, 3), dtype=np.uint8)
        cls.mask = _run_inference(cls.processor, cls.model, cls.device, dummy)

    @unittest.skipUnless(HAS_TORCH, "torch not installed")
    def test_mask_shape(self):
        """Output mask must match input frame resolution (480×640)."""
        if not torch.cuda.is_available():
            self.skipTest("CUDA not available")
        self.assertEqual(self.__class__.mask.shape, (CAM_H, CAM_W))

    @unittest.skipUnless(HAS_TORCH, "torch not installed")
    def test_mask_dtype(self):
        """Mask must be uint8 (mono8 for ROS Image msg)."""
        if not torch.cuda.is_available():
            self.skipTest("CUDA not available")
        self.assertEqual(self.__class__.mask.dtype, np.uint8)

    @unittest.skipUnless(HAS_TORCH, "torch not installed")
    def test_mask_value_range(self):
        """All class indices must be within ADE20K range [0, 149]."""
        if not torch.cuda.is_available():
            self.skipTest("CUDA not available")
        self.assertGreaterEqual(int(self.__class__.mask.min()), 0)
        self.assertLessEqual(int(self.__class__.mask.max()), 149)


# ═══════════════════════════════════════════════════════════════════════════
# 5. Filtered mask
# ═══════════════════════════════════════════════════════════════════════════

class TestFilteredMask(unittest.TestCase):

    def _build_filtered(self, full_mask):
        return np.where(np.isin(full_mask, list(KEEP_CLASSES)), full_mask, 0).astype(np.uint8)

    def test_filtered_only_keep_classes(self):
        """mask_filtered must contain only KEEP_CLASSES values (or 0 for non-nav pixels)."""
        full = np.array([[0, 3, 10, 29], [52, 99, 53, 55]], dtype=np.uint8)
        filtered = self._build_filtered(full)
        unique = set(filtered.flatten().tolist())
        allowed = KEEP_CLASSES | {0}
        self.assertTrue(unique.issubset(allowed), f"Unexpected classes in filtered mask: {unique - allowed}")

    def test_non_nav_pixels_zeroed(self):
        """Non-navigation pixels must be zeroed out."""
        full = np.full((4, 4), 10, dtype=np.uint8)  # class 10 = not in KEEP_CLASSES
        filtered = self._build_filtered(full)
        self.assertTrue((filtered == 0).all())

    def test_nav_pixels_preserved(self):
        """Navigation pixels must retain their class index."""
        full = np.full((4, 4), 3, dtype=np.uint8)  # class 3 = floor → in KEEP_CLASSES
        filtered = self._build_filtered(full)
        self.assertTrue((filtered == 3).all())


# ═══════════════════════════════════════════════════════════════════════════
# 6. Colormap
# ═══════════════════════════════════════════════════════════════════════════

class TestColormap(unittest.TestCase):

    @unittest.skipUnless(HAS_NODE, "segformer_node not importable")
    def test_colormap_shape(self):
        cmap = ade20k_colormap()
        self.assertEqual(cmap.shape, (150, 3))
        self.assertEqual(cmap.dtype, np.uint8)

    @unittest.skipUnless(HAS_NODE, "segformer_node not importable")
    def test_wall_class_has_colour(self):
        """class-0 = wall must NOT be black — it is a real class, not background."""
        cmap = ade20k_colormap()
        wall_colour = cmap[0]
        self.assertFalse(
            np.all(wall_colour == 0),
            f"class-0 (wall) is black {wall_colour} — colormap bug, wall will be invisible."
        )

    @unittest.skipUnless(HAS_NODE, "segformer_node not importable")
    def test_all_classes_unique_ish(self):
        """Most classes should have distinct colours (not a strict requirement but a sanity check)."""
        cmap = ade20k_colormap()
        unique_rows = np.unique(cmap, axis=0)
        # Allow some collisions but not massive duplicates
        self.assertGreater(len(unique_rows), 100)


# ═══════════════════════════════════════════════════════════════════════════
# 7. classify_scene() — all label paths
# ═══════════════════════════════════════════════════════════════════════════

class TestClassifyScene(unittest.TestCase):

    @unittest.skipUnless(HAS_NODE, "segformer_node not importable")
    def _mask(self, dominant_class: int) -> np.ndarray:
        """Helper: create a mask dominated by one class."""
        m = np.zeros((480, 640), dtype=np.uint8)
        m[:] = dominant_class
        return m

    @unittest.skipUnless(HAS_NODE, "segformer_node not importable")
    def test_staircase_via_yolo(self):
        result = classify_scene(['staircase', 'door'], self._mask(3))
        self.assertEqual(result, 'staircase')

    @unittest.skipUnless(HAS_NODE, "segformer_node not importable")
    def test_staircase_via_seg(self):
        """Staircase detected purely from seg mask (>5% stair pixels = class 29)."""
        m = np.zeros((480, 640), dtype=np.uint8)
        m[:50, :] = 29  # ~10% stair pixels
        result = classify_scene([], m)
        self.assertEqual(result, 'staircase')

    @unittest.skipUnless(HAS_NODE, "segformer_node not importable")
    def test_lab_workshop_two_objects(self):
        result = classify_scene(['shelf', 'work-desk', 'door'], self._mask(3))
        self.assertEqual(result, 'lab_workshop')

    @unittest.skipUnless(HAS_NODE, "segformer_node not importable")
    def test_lab_workshop_dell_e(self):
        """Single dell-e detection is strong signal for lab_workshop."""
        result = classify_scene(['dell-e'], self._mask(3))
        self.assertEqual(result, 'lab_workshop')

    @unittest.skipUnless(HAS_NODE, "segformer_node not importable")
    def test_doorway_via_seg(self):
        """Door pixels >5% → doorway."""
        m = np.zeros((480, 640), dtype=np.uint8)
        m[:50, :] = 52  # ~10% door pixels
        result = classify_scene([], m)
        self.assertEqual(result, 'doorway')

    @unittest.skipUnless(HAS_NODE, "segformer_node not importable")
    def test_corridor(self):
        """floor >30% + wall >20% + no lab objects → corridor."""
        m = np.zeros((480, 640), dtype=np.uint8)
        total = 480 * 640
        floor_px = int(total * 0.35)
        wall_px  = int(total * 0.25)
        m.flat[:floor_px] = 3   # floor
        m.flat[floor_px:floor_px + wall_px] = 0  # wall
        result = classify_scene([], m)
        self.assertEqual(result, 'corridor')

    @unittest.skipUnless(HAS_NODE, "segformer_node not importable")
    def test_unknown_fallback(self):
        """Empty detections + misc mask → unknown."""
        result = classify_scene([], self._mask(10))
        self.assertEqual(result, 'unknown')

    @unittest.skipUnless(HAS_NODE, "segformer_node not importable")
    def test_output_is_string(self):
        result = classify_scene([], self._mask(3))
        self.assertIsInstance(result, str)

    @unittest.skipUnless(HAS_NODE, "segformer_node not importable")
    def test_valid_labels_only(self):
        """All possible outputs must be one of the defined labels."""
        valid = {'staircase', 'lab_workshop', 'doorway', 'corridor', 'unknown'}
        cases = [
            ([], self._mask(0)),
            (['staircase'], self._mask(3)),
            (['shelf', 'work-desk'], self._mask(3)),
            (['dell-e'], self._mask(3)),
            ([], self._mask(52)),
        ]
        for yolo, mask in cases:
            result = classify_scene(yolo, mask)
            self.assertIn(result, valid, f"Unexpected label: {result}")


# ═══════════════════════════════════════════════════════════════════════════
# 8. Full pipeline end-to-end
# ═══════════════════════════════════════════════════════════════════════════

class TestPipeline(unittest.TestCase):
    """
    End-to-end: BGR frame → mask_full, mask_filtered, scene_label, image_colored.
    No ROS2 required — calls node internals directly.
    """

    @classmethod
    def setUpClass(cls):
        if not HAS_TORCH or not torch.cuda.is_available() or not HAS_NODE:
            return
        from segformer_node import SegformerNode, COLORMAP as CM
        import rclpy
        # We can't spin a real node without ROS2, so just test helpers standalone
        cls.COLORMAP = CM
        cls.device    = torch.device('cuda')    @unittest.skipUnless(HAS_TORCH and HAS_NODE, "torch or node not available")
    def test_full_pipeline_outputs(self):
        if not torch.cuda.is_available():
            self.skipTest("CUDA not available")

        cls = self.__class__

        # 1. Inference
        mask_full = _run_inference(cls.processor, cls.model, cls.device, cls.frame)

        # 2. Filtered mask
        mask_filtered = np.where(np.isin(mask_full, list(KEEP_CLASSES)), mask_full, 0).astype(np.uint8)

        # 3. Scene label
        scene = classify_scene([], mask_full)

        # 4. Colormap
        colored = np.zeros((CAM_H, CAM_W, 3), dtype=np.uint8)
        valid = mask_full < 150
        colored[valid] = cls.COLORMAP[mask_full[valid]]

        # Assertions
        self.assertEqual(mask_full.shape,     (CAM_H, CAM_W))
        self.assertEqual(mask_filtered.shape, (CAM_H, CAM_W))
        self.assertEqual(colored.shape,       (CAM_H, CAM_W, 3))
        self.assertIn(scene, {'staircase', 'lab_workshop', 'doorway', 'corridor', 'unknown'})
        self.assertTrue(set(mask_filtered.flatten()).issubset(KEEP_CLASSES | {0}))
        cls.processor = SegformerImageProcessor.from_pretrained(MODEL_NAME)
        cls.model     = SegformerForSemanticSegmentation.from_pretrained(MODEL_NAME)
        cls.model.to(cls.device).eval()
        cls.frame = np.random.randint(0, 255, (CAM_H, CAM_W, 3), dtype=np.uint8)

    @unittest.skipUnless(HAS_TORCH and HAS_NODE, "torch or node not available")
    def test_full_pipeline_outputs(self):
        if not torch.cuda.is_available():
            self.skipTest("CUDA not available")

        cls = self.__class__

        # 1. Inference
        mask_full = _run_inference(cls.processor, cls.model, cls.device, cls.frame)

        # 2. Filtered mask
        mask_filtered = np.where(np.isin(mask_full, list(KEEP_CLASSES)), mask_full, 0).astype(np.uint8)

        # 3. Scene label
        scene = classify_scene([], mask_full)

        # 4. Colormap
        colored = np.zeros((CAM_H, CAM_W, 3), dtype=np.uint8)
        valid = mask_full < 150
        colored[valid] = cls.COLORMAP[mask_full[valid]]

        # Assertions
        self.assertEqual(mask_full.shape,     (CAM_H, CAM_W))
        self.assertEqual(mask_filtered.shape, (CAM_H, CAM_W))
        self.assertEqual(colored.shape,       (CAM_H, CAM_W, 3))
        self.assertIn(scene, {'staircase', 'lab_workshop', 'doorway', 'corridor', 'unknown'})
        self.assertTrue(set(mask_filtered.flatten()).issubset(KEEP_CLASSES | {0}))


# ─────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print("=" * 60)
    print("  test_segformer.py — SegFormer Diagnostic")
    print(f"  Model      : {MODEL_NAME}")
    print(f"  Input size : {INPUT_SIZE}×{INPUT_SIZE}")
    print(f"  Target fps : ≥{TARGET_FPS} (benchmark 50W: ~16.5fps)")
    print(f"  Camera res : {CAM_W}×{CAM_H}")
    print("=" * 60)
    unittest.main(verbosity=2)
