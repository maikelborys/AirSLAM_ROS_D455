#!/usr/bin/env python
"""Numerical diff for XFeat between onnxruntime (CPU reference) and the
TensorRT 10 engine. Run after any change to xfeat.cpp, the ONNX export, or
the trtexec flags to confirm we have not regressed.

The XFeat ONNX has three outputs (feats / keypts / rel). We report:
- shape match (catches a broken export immediately)
- per-output cosine similarity vs the CPU reference
- per-output max abs diff and mean abs diff

Pass gate (FP32 + --noTF32 + optLvl=5):
- cosine ≥ 0.999 per output
- max abs diff ≤ 1e-3
TF32 or FP16 will not pass this gate — that is intentional.

Usage:
    source ~/.airslam_venv/bin/activate
    python scripts/numerical_diff_xfeat.py \\
        --image /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/<ts>.png \\
        --onnx output/xfeat_trt10.onnx \\
        --engine output/xfeat.engine
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile

import cv2
import numpy as np
import onnxruntime as ort


# Input dims used for the numerical-diff run. Matches EuRoC (480x752) so we
# also exercise the dynamic-shape path in the engine.
H, W = 480, 752


def load_image(path: str) -> np.ndarray:
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"cannot read {path}")
    print(f"original size: {img.shape}")
    # XFeat's FCN downsamples by 8, so H,W must be multiples of 8.
    img_resized = cv2.resize(img, (W, H))
    arr = img_resized.astype(np.float32) / 255.0
    return arr.reshape(1, 1, H, W)


def run_onnxruntime(onnx_path: str, arr: np.ndarray):
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    feats, keypts, rel = sess.run(["feats", "keypts", "rel"], {"image": arr})
    return {"feats": feats, "keypts": keypts, "rel": rel}


def run_trtexec(engine_path: str, arr: np.ndarray):
    with tempfile.TemporaryDirectory() as td:
        inp_bin = os.path.join(td, "image.bin")
        arr.astype(np.float32).tofile(inp_bin)
        out_json = os.path.join(td, "output.json")
        cmd = [
            "trtexec",
            f"--loadEngine={engine_path}",
            f"--shapes=image:1x1x{H}x{W}",
            f"--loadInputs=image:{inp_bin}",
            f"--exportOutput={out_json}",
            "--iterations=1",
            "--warmUp=0",
            "--avgRuns=1",
        ]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout[-2000:])
            print(r.stderr[-2000:])
            sys.exit("trtexec failed")
        with open(out_json) as f:
            data = json.load(f)
    out = {}
    for entry in data:
        name = entry["name"]
        vals = np.array(entry["values"], dtype=np.float32)
        dim_str = entry["dimensions"].strip("()")
        sep = "x" if "x" in dim_str else ","
        dims = tuple(int(d.strip()) for d in dim_str.split(sep) if d.strip())
        out[name] = vals.reshape(dims)
    return out


def report_diff(name: str, ref: np.ndarray, trt: np.ndarray) -> bool:
    ok = True
    if ref.shape != trt.shape:
        print(f"  {name}: SHAPE MISMATCH ref={ref.shape} trt={trt.shape}")
        return False
    diff = ref - trt
    max_abs = float(np.max(np.abs(diff)))
    mean_abs = float(np.mean(np.abs(diff)))
    cos = float(
        (ref.flatten() @ trt.flatten())
        / (np.linalg.norm(ref) * np.linalg.norm(trt) + 1e-12)
    )
    cos_ok = cos >= 0.999
    max_ok = max_abs <= 1e-3
    flag = "ok" if (cos_ok and max_ok) else "FAIL"
    print(f"  {name:7s} cos={cos:.6f}  max|d|={max_abs:.6e}  mean|d|={mean_abs:.6e}  [{flag}]")
    return cos_ok and max_ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--engine", required=True)
    args = ap.parse_args()

    arr = load_image(args.image)

    print("\n=== onnxruntime (CPU reference) ===")
    ref = run_onnxruntime(args.onnx, arr)
    for n, v in ref.items():
        print(f"  {n:7s} shape={v.shape}  range=[{v.min():.4f}, {v.max():.4f}]  "
              f"norm={np.linalg.norm(v):.4f}")

    print("\n=== TensorRT 10 engine ===")
    trt = run_trtexec(args.engine, arr)
    for n in ("feats", "keypts", "rel"):
        v = trt.get(n)
        if v is None:
            sys.exit(f"engine did not emit output named '{n}'")
        print(f"  {n:7s} shape={v.shape}  range=[{v.min():.4f}, {v.max():.4f}]  "
              f"norm={np.linalg.norm(v):.4f}")

    print("\n=== diff (gate: cos ≥ 0.999, max|d| ≤ 1e-3) ===")
    all_ok = True
    for n in ("feats", "keypts", "rel"):
        all_ok &= report_diff(n, ref[n], trt[n])

    print()
    if all_ok:
        print("PASS — XFeat engine matches CPU reference within tolerance.")
        return 0
    print("FAIL — engine output drifted. Investigate trtexec flags or ONNX patching.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
