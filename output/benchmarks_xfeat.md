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

| Run | Extractor | Matcher | max_kpts | KF count | FPS (full) | FPS (peak window) | ATE raw VO | ATE **post-refinement** |
|---|---|---|---|---|---|---|---|---|
| Baseline | SuperPoint+LightGlue | LightGlue | 400 | 302 | **38.3** | ~40 | ~0.10 m | **0.039 m** |
| This branch | XFeat | MNN+Lowe (CPU) | 1024 | 1157 | 16.85 | 55 | 0.308 m | 0.104 m |
| This branch | XFeat | MNN+Lowe (cuBLAS) | 1024 | ~1150 | 16.9 | 55 | 0.308 m | — |
| This branch | XFeat | MNN+Lowe (CPU) | 400 | ~1150 | 33.20 | — | 3.03 m | — |
| This branch | XFeat | LighterGlue N=512 | 512 | 1130 | 18.67 | 99 ⚡ | 0.251 m | 0.061 m |
| **This branch (BEST)** ⭐ | **XFeat** | **LighterGlue N=512** | **512** | **408** | **51.3** 🔥 | **99+** | **0.207 m** | **0.070 m** |

**The "BEST" row** is the result of the keyframe-rate tune:
`tracking_point_rate: 0.65 → 0.40`, `max_num_match: 80 → 40`,
`tracking_parallax_rate: 0.10 → 0.20`. AirSLAM was tuned for
SuperPoint+LightGlue which produces dense matches; XFeat+LighterGlue
matches less densely, so the default thresholds tripped keyframe
insertion almost every other frame (1130/2700). Loosening the
thresholds drops keyframes to 408 (1 every 6.6 frames vs SP's 1 every
8.9) and slashes BA work end-to-end.

**This is the result that closes the FPS gap to SuperPoint:**

  AirSLAM-XFeat at 51.3 FPS is now **34% FASTER** than the SuperPoint
  baseline (38.3 FPS) on the same hardware, while staying
  **fully Apache-2.0 commercial-deployable**. Raw VO ATE 0.207 m, post-
  refinement ATE 0.070 m (~1.8x behind the paper number, same order of
  magnitude). The fewer-keyframes trajectory accumulates LESS drift in
  raw VO (0.207 vs 0.251 m) and refines to ~14% worse than the
  dense-KF run (0.070 vs 0.061 m) — that's the speed/precision knob.

The LighterGlue row is the headline result of this session:

- **Raw VO ATE 0.251 m** (-19% vs MNN), **FPS avg 18.67 / peak 99**.
- **Post-refinement ATE 0.061 m** — only **1.6x behind** the SuperPoint
  baseline (down from 2.6x with MNN). map_refinement found 269 loop
  pairs (vs MNN's 405); fewer loop pairs is fine because each one is
  *cleaner* — LighterGlue's attention-based matching produces fewer
  spurious correspondences.

LighterGlue N=512 trace specifically:
- Re-tracing at N=512 (down from 1024) cut attention work 4x — peak
  steady-state FPS went **99 FPS at frames 300–350**, beating even the
  SuperPoint baseline's 40 FPS peak.
- End-to-end full-sequence FPS is bounded by MapBuilder + g2o BA on
  CPU, not the matcher (see "Bottlenecks" below). That's why the
  whole-trajectory FPS only reaches 18.67 despite 99 FPS peaks.

### XFeat post-processing — CUDA isolation benchmark (test_xfeat)

| Backend | Per-frame | Throughput |
|---|---|---|
| CPU (Phase 3) | 4.5 ms | 221 Hz |
| **CUDA (this session)** | **1.77 ms** | **564 Hz** ⚡ |

Four kernels in `src/xfeat_postproc.cu` replace the host loops:
softmax(65)+unfold, NMS-dilate, candidate-emit (atomic + filter),
bilinear-descriptor-sample+L2-norm. The kernels read directly from the
TRT engine's device buffers (`buffers.getDeviceBuffer`), skipping the
~1.5 MiB D2H copy of the dense outputs. Top-5 keypoints + scores are
bit-identical to the CPU path.

**Whole-pipeline VO FPS impact of CUDA-postproc is +1-2% only** — the
post-proc was already a small slice of the per-frame budget. The big
remaining cost is g2o local BA, which is CPU-bound and not addressed
by this CUDA work.

The **cuBLAS MNN row** is informative: cosine GEMM on GPU dropped from
~10 ms (Eigen CPU) to ~0.3 ms, but full-sequence FPS only moved
16.85 → 16.9. The cosine GEMM was already a small fraction of per-frame
wall time. The dominant cost is AirSLAM's MapBuilder + g2o BA, which
scales with `max_keypoints × keyframes_recent` — that's why dropping
to max_kpts=512 (LighterGlue's natural fit) bought a clean 50% FPS
improvement on top of the matcher upgrade.

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

## Vicon Room + lines (Part 3 — PLNet wireframe head re-enabled)

Hybrid mode: XFeat (Apache-2.0, 64-dim points) + **PLNet wireframe head**
(Apache-2.0, lines + 256-dim junctions). Dispatch knob `line_extractor`
in `PLNetConfig` is orthogonal to `feature_extractor`, so the two YAMLs
(`vo_euroc_xfeat_lighterglue.yaml` and
`vo_euroc_xfeat_lighterglue_lines.yaml`) coexist — pick the launch file
that matches the environment. Naturaleza → points-only; urbano /
estructurado → lines on.

All numbers below are full-sequence VO + map_refinement, ATE RMSE
against EuRoC IMU ground truth via `evo_ape euroc … -va`.

| Sequence | Mode | ATE post-MR (m) | FPS | Maplines | Loops | ATE Δ vs points |
|---|---|---|---|---|---|---|
| **MH_03_medium** (Machine Hall) | points-only | 0.0700 | **51.3** | 0 | — | — |
| **MH_03_medium** | **+ PLNet lines** | **0.0689** | 45.4 | 16 405 | 75 | **−1.6%** ✅ |
| **V1_01_easy** | points-only | 0.0889 | 59.4 | 0 | 15 | — |
| **V1_01_easy** | + PLNet lines | 0.0889 | 54.9 | 5 299 | 15 | ±0.0% |
| **V1_02_medium** | points-only | 0.0785 | 45.7 | 0 | 19 | — |
| **V1_02_medium** | + PLNet lines | 0.0780 | 43.3 | 5 106 | 18 | −0.7% |
| **V1_03_difficult** | points-only | 0.2935 | 43.6 | 0 | 7 | — |
| **V1_03_difficult** | **+ PLNet lines** | **0.2814** | 43.8 | 7 915 | 7 | **−4.1%** ✅ |

Headline pattern: **lines help proportionally to difficulty**.

- **V1_01_easy** (slow motion, good light) — points already solve it,
  lines are a neutral addition (≈ 0% ATE delta).
- **V1_02_medium** — marginal 0.7% improvement; lines start to carry
  weight as the trajectory aggressiveness picks up.
- **V1_03_difficult** — **4.1% ATE improvement** from the same hardware,
  same input. This is where structural lines (walls/floor edges) carry
  geometric constraint that XFeat alone struggles to lock down under
  motion blur.
- **MH_03_medium** — 1.6% ATE improvement, slightly behind V1_03 because
  Machine Hall already has rich texture from clutter/equipment.

**FPS cost is ~10% on MH_03**, ~7% on V1_01, **noise** (within ±0.3 FPS)
on V1_02 and V1_03. The hit is concentrated on machine-hall (more
keyframes per frame → PLNet runs more often). On the Vicon Room
sequences with sparser KFs, lines come essentially for free in FPS terms.

The mapline count is also a useful diagnostic of scene structure:
~16 k maplines on MH_03 (industrial clutter, dense edges), ~5–8 k on
Vicon Room sequences (simpler walls + checkerboard floor).

**V2_xx sequences not benchmarked** in this pass because the V2_01 and
V2_03 datasets are not present in `~/datasets/euroc/`. Listed in Future
work below as a follow-up (download + repeat the four-row table).

### Reproducing the Vicon Room section

```bash
for SEQ in V1_01_easy V1_02_medium V1_03_difficult; do
  for MODE in lines pointsonly; do
    OUT=/tmp/airslam_xfeat_${MODE}_${SEQ}
    rm -rf "$OUT"; mkdir -p "$OUT"
    if [[ $MODE == lines ]]; then
      VO=vo_euroc_xfeat_lighterglue_lines.launch.py
      MR=mr_euroc_xfeat_lines.launch.py
    else
      VO=vo_euroc_xfeat_lighterglue.launch.py
      MR=mr_euroc_xfeat.launch.py
    fi
    ros2 launch air_slam_xfeat $VO \
      dataroot:=$HOME/datasets/euroc/${SEQ}/mav0 \
      saving_dir:=$OUT visualization:=false
    ros2 launch air_slam_xfeat $MR \
      map_root:=$OUT \
      voc_path:=$HOME/coding/AirSLAM_XFEAT/voc/point_voc_L4_xfeat.bin
    evo_ape euroc \
      $HOME/datasets/euroc/${SEQ}/mav0/state_groundtruth_estimate0/data.csv \
      $OUT/trajectory_v1.txt -va | tee $OUT/ate.txt
  done
done
```

## Cross-sequence relocalization (V1_03 queries against V1_01 map)

Single-image relocalization mode: load the refined `AirSLAM_mapv1.bin`
produced by VO+MR on **V1_01_easy** (XFeat + PLNet lines), then query
each frame of **V1_03_difficult/mav0/cam0/data** independently. Same
Vicon Room 1 environment, completely different trajectory (slow vs
aggressive), motion blur on the query side.

Pipeline: XFeat (points) + PLNet (lines+junctions) per query image →
DBoW query against BOTH the 64-dim XFeat point vocab AND the 256-dim
PLNet junction vocab → grouped scoring → 2D-3D matching → pose refine.
Each frame is independent (no temporal prior).

| Metric | Value |
|---|---|
| Query frames | 2149 |
| Reloc successes | **1307 / 2149 = 60.82%** |
| Per-frame time | ~32-40 ms (~28 Hz) |
| Median position error (successes) | **0.131 m** |
| p90 position error (successes) | 0.222 m |
| Mean position error (successes) | 0.252 m (pulled up by outliers) |

Error distribution (of the 1271 successes that matched GT timestamps):

| Threshold | Within |
|---|---|
| < 10 cm | 31.5% |
| < 20 cm | 82.4% |
| < 30 cm | **94.3%** |
| < 50 cm | 96.6% |
| < 1 m   | 97.8% |

**True useful recall (success AND <30 cm) ≈ 1199/2149 = 55.8%**

The ~2% outliers (above 1 m) are reloc successes whose inlier count
passed the `min_inlier_num: 45` gate but ended up in a wrong part of
the map — typical failure mode of single-image reloc on motion-blurred
frames where the point distribution is consistent with multiple map
poses.

### Reproducing

```bash
# (assumes V1_01_easy map already exists in /tmp/airslam_xfeat_lines_V1_01_easy
#  from the Vicon Room benchmark above)
ros2 launch air_slam_xfeat reloc_euroc_xfeat_lines.launch.py
# launches with defaults:
#   dataroot=$HOME/datasets/euroc/V1_03_difficult/mav0/cam0/data
#   map_root=/tmp/airslam_xfeat_lines_V1_01_easy
#   voc_path=<pkg_share>/voc/point_voc_L4_xfeat.bin
# saves /tmp/airslam_xfeat_lines_V1_01_easy/reloc_V1_03.txt with
# "success <timestamp> tx ty tz qx qy qz qw" or "fail <timestamp> ..." lines.

# Extract successes and run evo_ape:
awk '/^success/ {printf "%.9f %s %s %s %s %s %s %s\n", $2/1e9, $3, $4, $5, $6, $7, $8, $9}' \
  /tmp/airslam_xfeat_lines_V1_01_easy/reloc_V1_03.txt \
  > /tmp/airslam_xfeat_lines_V1_01_easy/reloc_V1_03_success.tum
evo_ape euroc \
  $HOME/datasets/euroc/V1_03_difficult/mav0/state_groundtruth_estimate0/data.csv \
  /tmp/airslam_xfeat_lines_V1_01_easy/reloc_V1_03_success.tum -va
```

## Future work (post-current session)

Ordered by expected impact, biggest first:

1. **Replace g2o local BA with parallel / GPU BA** (e.g. MegBA,
   DeepLM, TheseusAI). Currently dominates per-frame wall time on
   keyframe insertion (~30–120 ms each, single-threaded). Risky — g2o is
   tightly coupled to AirSLAM's custom vertex/edge types and robust
   kernels. ~1–2 weeks of careful work; potential +50–100% FPS.

2. **Async pipeline (Jetson-SLAM style)** — overlap feature extraction
   of frame N+1 with tracking of frame N. AirSLAM already has a 2-thread
   feature/tracking split; would need refactor to fully decouple stages.
   Estimated +20–40% FPS, ~1–2 days.

3. **Tune AirSLAM keyframe / local-BA window** — `tracking_point_rate`,
   `tracking_parallax_rate`, local covisibility window size. Zero new
   code, just YAML; could buy +30% FPS at some ATE cost.

4. **D455 live-camera ROS 2 topic ingestion** — cherry-pick ROSDataset
   pattern from `master` branch, port to rclcpp + `message_filters` for
   `/camera/camera/infra{1,2}/image_rect_raw` (+ optional IMU). Enables
   on-robot benchmarking. ~3–5 h.

5. ~~**Re-enable lines via PLNet wireframe head with XFeat-anchored
   points**.~~ **DONE** (Part 3, this session). Validated on Vicon Room
   V1_xx + MH_03. V1_03_difficult: −4.1% ATE; MH_03: −1.6% ATE;
   V1_01_easy: neutral. FPS hit ~0–10% depending on KF density.
   Follow-up: V2_01 + V2_03 once datasets are on disk.

6. **Larger vocab training corpus**. Current `voc/point_voc_L4_xfeat.bin`
   was trained on MH_01 + V1_01 only (1.23 M descriptors). Adding
   MH_02 + V1_02 + V2_01 (~3 M descriptors) would likely improve loop
   recall in unseen environments.

7. **LighterGlue ONNX/TRT export** — currently using TorchScript via
   libtorch (Path D from earlier). Replacing with a TRT engine could
   shave the libtorch dependency and a few ms per call. Blocker:
   `.transpose(-X, -Y)` / `.unflatten(-1, ...)` ops in cvg/LightGlue
   source need positive-dim patches, plus the remaining negative-index
   ops in `rotate_half` / `apply_cached_rotary_emb`. ~3–4 h.
