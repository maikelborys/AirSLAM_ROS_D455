#!/usr/bin/env python
"""Numerical diff for SuperPoint+LightGlue between onnxruntime and TRT 10.

Generates synthetic but structured keypoint inputs (uniform grid + noise) and
runs the same pair through both backends. Compares match scores and indices.
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

import numpy as np
import onnxruntime as ort


def make_inputs(N=400, seed=42):
    rng = np.random.default_rng(seed)
    # Two slightly-different keypoint sets, simulating left/right stereo with
    # a small horizontal disparity.
    xs = rng.uniform(0.05, 0.95, N)
    ys = rng.uniform(0.05, 0.95, N)
    kp0 = np.stack([xs, ys], axis=1).astype(np.float32)
    kp1 = kp0.copy()
    kp1[:, 0] -= rng.uniform(0.005, 0.025, N)  # disparity
    # Random unit-norm 256-d descriptors, slightly correlated between sides.
    base = rng.standard_normal((N, 256)).astype(np.float32)
    base /= np.linalg.norm(base, axis=1, keepdims=True)
    de0 = base
    de1 = base + 0.05 * rng.standard_normal((N, 256)).astype(np.float32)
    de1 /= np.linalg.norm(de1, axis=1, keepdims=True)
    return (kp0[None, :, :].astype(np.float32),
            kp1[None, :, :].astype(np.float32),
            de0[None, :, :].astype(np.float32),
            de1[None, :, :].astype(np.float32))


def run_ort(onnx_path, kp0, kp1, de0, de1):
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    feeds = {
        "keypoints_0": kp0, "keypoints_1": kp1,
        "descriptors_0": de0, "descriptors_1": de1,
    }
    return sess.run(None, feeds)


def run_trt(engine_path, kp0, kp1, de0, de1):
    with tempfile.TemporaryDirectory() as td:
        for name, arr in (("keypoints_0", kp0), ("keypoints_1", kp1),
                          ("descriptors_0", de0), ("descriptors_1", de1)):
            arr.astype(np.float32).tofile(f"{td}/{name}.bin")
        N = kp0.shape[1]
        cmd = [
            "trtexec",
            f"--loadEngine={engine_path}",
            f"--shapes=keypoints_0:1x{N}x2,keypoints_1:1x{N}x2,"
            f"descriptors_0:1x{N}x256,descriptors_1:1x{N}x256",
            f"--loadInputs=keypoints_0:{td}/keypoints_0.bin,"
            f"keypoints_1:{td}/keypoints_1.bin,"
            f"descriptors_0:{td}/descriptors_0.bin,"
            f"descriptors_1:{td}/descriptors_1.bin",
            f"--exportOutput={td}/output.json",
            "--iterations=1", "--warmUp=0", "--avgRuns=1",
        ]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout[-2000:]); print(r.stderr[-2000:]); sys.exit("trtexec failed")
        with open(f"{td}/output.json") as f:
            data = json.load(f)
        out = {}
        for entry in data:
            vals = np.array(entry["values"], dtype=np.float32)
            ds = entry["dimensions"].strip("()")
            sep = "x" if "x" in ds else ","
            dims = tuple(int(d.strip()) for d in ds.split(sep) if d.strip())
            out[entry["name"]] = vals.reshape(dims)
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--engine", required=True)
    ap.add_argument("-N", type=int, default=400)
    args = ap.parse_args()

    kp0, kp1, de0, de1 = make_inputs(args.N)
    print(f"\n=== onnxruntime ({args.onnx}) ===")
    ort_out = run_ort(args.onnx, kp0, kp1, de0, de1)
    print("output names: -")
    for i, o in enumerate(ort_out):
        print(f"  out[{i}]: shape={o.shape}, range=[{o.min():.4f}, {o.max():.4f}], norm={np.linalg.norm(o):.4f}")

    print(f"\n=== TensorRT 10 ({args.engine}) ===")
    trt_out = run_trt(args.engine, kp0, kp1, de0, de1)
    for name, o in trt_out.items():
        print(f"  {name}: shape={o.shape}, range=[{o.min():.4f}, {o.max():.4f}], norm={np.linalg.norm(o):.4f}")

    # Take "scores" output for diff.
    if len(ort_out) == 1 and "scores" in trt_out:
        ort_s = ort_out[0]
        trt_s = trt_out["scores"]
        if ort_s.shape == trt_s.shape:
            d = ort_s - trt_s
            print(f"\nscore L2 diff: {np.linalg.norm(d):.6f}")
            print(f"score max abs diff: {np.abs(d).max():.6f}")
            print(f"cosine: {(ort_s.flatten() @ trt_s.flatten()) / (np.linalg.norm(ort_s) * np.linalg.norm(trt_s) + 1e-12):.6f}")


if __name__ == "__main__":
    sys.exit(main() or 0)
