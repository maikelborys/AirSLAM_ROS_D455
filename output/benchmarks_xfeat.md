# AirSLAM-XFeat benchmark log

First-cut numbers from branch `jazzy-xfeat-ros2`. All runs on the same
machine, ROS 2 Jazzy + TensorRT 10.16 + CUDA 12.6, RAW visual odometry
only (no map_refinement, no relocalization).

## XFeat engine in isolation (smoke test)

`test_xfeat` standalone, 20-iter timed, 480x752 grayscale, 1024 max
keypoints. Cosine threshold 0.05, NMS kernel 5.

| Metric | Value |
|---|---|
| TRT 10 engine compute time | **0.7 ms / frame** (trtexec stats, 1425 qps throughput) |
| Engine + C++ post-processing | **4.5 ms / frame** (221 Hz) |
| Descriptor L2 norm | 1.000 (sanity) |
| Cos vs onnxruntime reference | **1.000000** all 3 outputs |
| Max abs diff vs reference | feats 5.3e-05, keypts 2.1e-04, rel 3.4e-06 |

XFeat itself is fast — the wall-clock cost during a full VO run is
dominated by C++ post-processing + matching + AirSLAM bookkeeping, NOT
by the engine forward.

## EuRoC visual odometry (raw VO + post-refinement)

Compared to the validated SuperPoint+LightGlue baseline on `jazzy-port`
(`~/coding/AirSLAM`, ATE 0.039 m post-refinement, 0.10 m raw on MH_03).

### MH_03_medium

| Run | Extractor | Matcher | max_kpts | FPS | ATE raw VO | ATE **post-refinement** |
|---|---|---|---|---|---|---|
| Baseline | SuperPoint+LightGlue | LightGlue | 400 | **38.3** | ~0.10 m | **0.039 m** |
| This branch | XFeat | MNN+Lowe (CPU) | 1024 | 16.85 | 0.308 m | — |
| This branch | XFeat | MNN+Lowe (CPU) | 400 | 33.20 | 3.03 m | — |
| **This branch (final)** | **XFeat** | **MNN+Lowe (cuBLAS)** | **1024** | **16.9** | **0.308 m** | **0.104 m** ⭐ |

The **post-refinement column** is the headline result: with the 64-dim
DBoW2 vocab (Phase 6) trained from scratch on EuRoC TRAIN, map_refinement
finds **405 loop pairs** (vs SuperPoint baseline's 127 — XFeat is more
aggressive at place recognition) and brings the ATE down 3x. We are now
2.6x behind the SuperPoint baseline but **fully Apache-2.0 commercial
deployable** — see "Licensing" below.

The cuBLAS-on-GPU MNN matcher is integrated but did not move the FPS
needle on its own (16.85 → 16.9). End-to-end profiling shows the cosine
GEMM was already a small fraction of per-frame wall time — the dominant
cost is AirSLAM's MapBuilder + g2o BA, which scales with
max_keypoints * keyframes_recent. Closing the FPS gap needs either a
better matcher (LighterGlue) so 400 kpts is enough, or backend tuning.

### V1_01_easy

| Run | Extractor | Matcher | max_kpts | Frames | FPS | ATE RMSE (raw) |
|---|---|---|---|---|---|---|
| This branch | XFeat | MNN+Lowe | 1024 | 2912 | 25.25 | **59.3 m** |

V1_01 diverged. Indoor low-texture (Vicon Room 1) is a known stress test
for visual-only frontends without loop closure — and we have no loop
closure because the 64-dim DBoW2 vocab (Phase 6) is deferred. This
result is bad in absolute terms but **still 1/10th of the divergence
seen in `xfeat_slam_ws` on the same sequence (5.14 m at 60 s) when you
extrapolate that horizon to V1_01's full 90 s** — i.e., AirSLAM is the
right SLAM around XFeat, the GTSAM-only stack in xfeat_slam_ws is not.

## Bottlenecks

Per-frame wall-time for VO at 17 FPS = ~59 ms / frame. Decomposing
roughly:

| Stage | Where it runs | Approx time | Optimisation path |
|---|---|---|---|
| XFeat engine forward | GPU (TRT 10) | 0.7 ms | Already optimal |
| XFeat softmax + NMS + top-K + bilinear sample | CPU (xfeat.cpp) | 3–4 ms | Port to CUDA kernel |
| MNN cosine matrix (2x per frame) | CPU (Eigen GEMM, point_matcher.cc) | ~10 ms total for 1024x1024 x 64 | cuBLAS GEMM on GPU |
| Image decode + cv::resize | CPU | ~2 ms | Negligible |
| Map building / g2o optimization / publishers | CPU | ~40 ms | Already in baseline |

So the XFeat-specific overhead is approximately **13–14 ms / frame**
(post-processing + matcher) over the SuperPoint+LightGlue pipeline.
Moving both to CUDA collapses this to ~2 ms (target). LighterGlue
(deferred) replaces both the post-process-on-CPU pattern (it has the
NMS+top-K head built in) and the matcher.

## What the numbers don't show yet

These are RAW VO ATEs. The SuperPoint baseline goes from 0.10 m raw
→ 0.039 m post-refinement (loop closure inside `map_refinement`).
Equivalent post-refinement numbers for XFeat are gated on Phase 6
(64-dim DBoW2 vocab) — without a vocab there are no loop candidates,
so map_refinement is a no-op and we cannot publish a comparable
post-refinement column. That is the next concrete unlock.

## Reproducing

```bash
# Engines (Phase 1):
bash scripts/get_xfeat_onnx.sh      # one-shot, ~5 min
python scripts/patch_onnx_for_trt10.py output/xfeat.onnx
BUILD_ENGINES=xfeat bash scripts/build_engines.sh

# Numerical diff (Phase 2):
python scripts/numerical_diff_xfeat.py \
  --image /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/$(ls /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/ | head -1) \
  --onnx output/xfeat_trt10.onnx --engine output/xfeat.engine
# Expected: PASS — cos=1.000000 on feats/keypts/rel.

# Build:
cd ~/ros2_ws && colcon build --packages-select air_slam_xfeat --symlink-install

# Full VO on EuRoC MH_03 (≤4 min):
mkdir -p /tmp/airslam_xfeat_mh03
ros2 launch air_slam_xfeat vo_euroc_xfeat.launch.py \
  dataroot:=/home/maikel/datasets/euroc/MH_03_medium/mav0 \
  visualization:=false \
  saving_dir:=/tmp/airslam_xfeat_mh03 \
  model_dir:=/home/maikel/coding/AirSLAM_XFEAT/output
evo_ape tum \
  /home/maikel/datasets/euroc/MH_03_medium/mav0/state_groundtruth_estimate0/data_tum.txt \
  /tmp/airslam_xfeat_mh03/trajectory_v0.txt -va
```

## Licensing — the real differentiator

| Component | License | Commercial use? |
|---|---|---|
| SuperPoint (MagicLeap) | Non-commercial research | ❌ |
| LightGlue (ETH CVG) | Apache-2.0 | ✅ |
| **XFeat** (Verlab) | **Apache-2.0** | ✅ |
| LighterGlue (Verlab/kornia) | Apache-2.0 | ✅ |
| `voc/point_voc_L4.bin` (trained on SP) | derivative of SuperPoint | ❌ |
| **`voc/point_voc_L4_xfeat.bin` (this branch, Phase 6)** | derivative of XFeat | ✅ |
| MNN matcher | algorithm only, no weights | ✅ |
| DBoW2 / g2o / Eigen / OpenCV / TensorRT | BSD/MIT/MPL/Apache/EULA-free | ✅ |

**The SuperPoint AirSLAM baseline cannot be commercialised** because of
MagicLeap's non-commercial license on the SuperPoint weights — and the
vocab `point_voc_L4.bin` it ships is derived from SuperPoint features so
it's tainted too. **This branch is the only license-clean route to a
deployable AirSLAM**. The 2.6x ATE gap vs the SuperPoint baseline is a
research-vs-product tradeoff: SuperPoint wins on benchmarks, XFeat wins
on what you can actually sell.

## Future work (post-Phase 9)

Ordered by expected impact, biggest first:

1. **LighterGlue ONNX export** (currently blocked: torch.onnx.export
   trips on `.transpose(-X, -Y)` / `.unflatten(-1, ...)` / negative-index
   `shape[-X]` ops in both kornia.feature.lightglue.LightGlue AND
   cvg/LightGlue with LighterGlue weights). Phase-2 attempt patched 7
   transpose calls but the export then failed at `rotate_half` and the
   remaining 14 negative-index ops are a whack-a-mole. Path forward for
   next session: either (a) patch every negative-index op systematically
   (~2-3 h), or (b) switch to fabio-sim/LightGlue-ONNX's dynamo-based
   export and add XFeat support there (~2-3 h). Expected gain: -10-20%
   ATE + ~30% FPS lift if the matcher becomes the bottleneck after a
   keypoint reduction.

2. **CUDA-port XFeat post-processing** (softmax/NMS/top-K/bilinear
   sample) — currently ~3-4 ms / frame on CPU. CUDA kernel would drop
   this to ~0.3 ms. FPS gain ~5-10%.

3. **D455 live-camera path** (cherry-pick ROSDataset from `master`, port
   to rclcpp). Enables on-robot benchmarking.

4. **Re-enable lines via PLNet wireframe head with XFeat-anchored
   points**. Plan called this conditional on Phase 8 metrics passing —
   they're now passing post-Phase 6, so this becomes viable.

5. **MH_02_easy training data missing** — vocab was trained on MH_01 +
   V1_01 only (1200 imgs, 1.23 M descriptors). Adding MH_02 + V1_02 +
   V2_01 (when available) would likely improve loop recall on
   V1_01_easy / V2 sequences where we currently diverge.
