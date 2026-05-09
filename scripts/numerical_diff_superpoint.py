#!/usr/bin/env python
"""Numerical diff for SuperPoint between onnxruntime (PyTorch-equivalent) and
the TensorRT 10 engine. Helps localise where precision regresses.

Reads one EuRoC image, resizes to 512x512 like AirSLAM does, runs both
backends, and reports:
- keypoint count delta
- top-K keypoint position L2 distance
- descriptor matrix L2 norm difference
- score histogram divergence

Usage:
    source ~/.airslam_venv/bin/activate
    pip install opencv-python numpy   # already in the venv
    python scripts/numerical_diff_superpoint.py \
        --image /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/<ts>.png \
        --onnx output/superpoint_v1_sim_int32_trt10.onnx \
        --engine output/superpoint_v1_sim_int32.engine
"""
import argparse
import os
import sys

import cv2
import numpy as np
import onnxruntime as ort


def load_image_512(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"cannot read {path}")
    print(f"original size: {img.shape}")
    img512 = cv2.resize(img, (512, 512))
    arr = img512.astype(np.float32) / 255.0
    arr = arr.reshape(1, 1, 512, 512)
    return arr, img512


def run_onnxruntime(onnx_path, arr):
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    outs = sess.run(None, {"input": arr})
    # Output order: scores (1xHxW), descriptors (1x256x H/8 x W/8)
    scores = outs[0]
    desc = outs[1]
    return scores, desc


def detect_keypoints(scores, threshold=0.004, border=4, top_k=400):
    """Replicates the C++ detect_point logic so we can compare keypoint sets."""
    # scores: (1, H, W) or (H, W)
    s = scores.squeeze()
    H, W = s.shape
    mask = (s >= threshold).astype(np.uint8)
    mask[:border, :] = 0
    mask[H - border:, :] = 0
    mask[:, :border] = 0
    mask[:, W - border:] = 0
    ys, xs = np.where(mask > 0)
    sc = s[ys, xs]
    if len(sc) > top_k:
        idx = np.argsort(-sc)[:top_k]
        ys, xs, sc = ys[idx], xs[idx], sc[idx]
    return np.stack([sc, xs.astype(np.float32), ys.astype(np.float32)], axis=1)


def trt_run(engine_path, arr):
    """Run TRT 10 engine on a single 512x512 image, return (scores, desc)."""
    import tempfile
    import subprocess
    # Use trtexec --loadEngine --shapes to run inference and dump output.
    # We pipe a fixed input via numpy save + --loadInputs.
    with tempfile.TemporaryDirectory() as td:
        inp_bin = os.path.join(td, "input.bin")
        arr.astype(np.float32).tofile(inp_bin)
        out_dir = td
        cmd = [
            "trtexec",
            f"--loadEngine={engine_path}",
            f"--shapes=input:1x1x512x512",
            f"--loadInputs=input:{inp_bin}",
            f"--exportOutput={out_dir}/output.json",
            "--iterations=1",
            "--warmUp=0",
            "--avgRuns=1",
        ]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout[-2000:])
            print(r.stderr[-2000:])
            sys.exit("trtexec failed")
        # Parse JSON
        import json
        with open(f"{out_dir}/output.json") as f:
            data = json.load(f)
        scores = None
        desc = None
        for entry in data:
            name = entry["name"]
            vals = np.array(entry["values"], dtype=np.float32)
            dim_str = entry["dimensions"].strip("()")
            # trtexec writes "1x256x64x64", not "(1,256,64,64)"
            sep = "x" if "x" in dim_str else ","
            dims = tuple(int(d.strip()) for d in dim_str.split(sep) if d.strip())
            vals = vals.reshape(dims)
            if name == "scores":
                scores = vals
            elif name == "descriptors":
                desc = vals
        return scores, desc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--engine", required=True)
    args = ap.parse_args()

    arr, img512 = load_image_512(args.image)

    print("\n=== onnxruntime (PyTorch-equivalent reference) ===")
    ort_scores, ort_desc = run_onnxruntime(args.onnx, arr)
    print(f"scores shape: {ort_scores.shape}, desc shape: {ort_desc.shape}")
    ort_kpts = detect_keypoints(ort_scores)
    print(f"detected keypoints: {len(ort_kpts)}")
    print(f"score range: {ort_scores.min():.6f} .. {ort_scores.max():.6f}")
    print(f"descriptor norm: {np.linalg.norm(ort_desc):.4f}")

    print("\n=== TensorRT 10 engine ===")
    trt_scores, trt_desc = trt_run(args.engine, arr)
    if trt_scores is None or trt_desc is None:
        sys.exit("trt run did not return both outputs")
    print(f"scores shape: {trt_scores.shape}, desc shape: {trt_desc.shape}")
    trt_kpts = detect_keypoints(trt_scores)
    print(f"detected keypoints: {len(trt_kpts)}")
    print(f"score range: {trt_scores.min():.6f} .. {trt_scores.max():.6f}")
    print(f"descriptor norm: {np.linalg.norm(trt_desc):.4f}")

    print("\n=== diff ===")
    print(f"keypoint count: ort={len(ort_kpts)}, trt={len(trt_kpts)}, "
          f"delta={len(trt_kpts) - len(ort_kpts)}")
    score_l2 = np.linalg.norm(ort_scores - trt_scores)
    print(f"score map L2: {score_l2:.4f}")
    desc_l2 = np.linalg.norm(ort_desc - trt_desc)
    print(f"descriptor L2: {desc_l2:.4f}")
    desc_cos = (ort_desc.flatten() @ trt_desc.flatten()) / (
        np.linalg.norm(ort_desc) * np.linalg.norm(trt_desc) + 1e-12)
    print(f"descriptor cosine sim: {desc_cos:.6f}")

    # Compare top-50 keypoints
    K = min(50, len(ort_kpts), len(trt_kpts))
    if K > 0:
        ort_xy = ort_kpts[:K, 1:3]
        trt_xy = trt_kpts[:K, 1:3]
        # Greedy nearest-neighbor matching by score-rank
        d = np.linalg.norm(ort_xy - trt_xy, axis=1).mean()
        print(f"top-{K} same-rank pixel L2 dist (mean): {d:.3f}")


if __name__ == "__main__":
    sys.exit(main() or 0)
