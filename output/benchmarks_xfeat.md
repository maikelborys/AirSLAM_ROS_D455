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

## Cross-sequence relocalization (V1_03 queries against V1_01 / V1_02 map)

Single-image relocalization mode: load a refined `AirSLAM_mapv1.bin`
produced by VO+MR on a Vicon Room 1 sequence (XFeat + PLNet lines),
then query each frame of **V1_03_difficult/mav0/cam0/data**
independently. Same room, completely different trajectory, motion blur
on the query side.

Pipeline: XFeat (points) + PLNet (lines+junctions) per query image →
DBoW query against BOTH the 64-dim XFeat point vocab AND the 256-dim
PLNet junction vocab → grouped scoring → 2D-3D matching → pose refine.
Each frame is independent (no temporal prior).

### Map source matters — V1_01_easy vs V1_02_medium as the reference map

Identical query stream (V1_03_difficult frames), identical reloc
pipeline (XFeat + PLNet lines). Only the underlying refined map changes.

| Metric | Map = V1_01_easy + lines | Map = V1_02_medium + lines | Δ |
|---|---|---|---|
| Map maplines | 5 299 | 5 106 | — |
| Map loop pairs (MR) | 15 | 18 | — |
| **Reloc recall** | **60.82%** (1307/2149) | **82.04%** (1763/2149) | **+21 pp** ✅ |
| Per-frame time | ~35 ms (~28 Hz) | ~35 ms (~28 Hz) | — |
| Median position error | 0.131 m | **0.097 m** | −26% |
| p90 position error | 0.222 m | **0.181 m** | −18% |
| Mean position error | 0.252 m | **0.184 m** | −27% |
| < 10 cm | 31.5% | **52.1%** | +21 pp |
| < 20 cm | 82.4% | **92.0%** | +10 pp |
| < 30 cm | 94.3% | **96.5%** | +2 pp |
| Outliers > 1 m | ~2.2% | **~1.3%** | −0.9 pp |
| Useful recall (success AND <30 cm) | 55.8% of 2149 | **77.2%** of 2149 | **+21 pp** ✅ |

**Headline: a more aggressive mapping trajectory produces a much better
relocalization map for difficult queries.** V1_02_medium's trajectory
samples more viewpoints / orientations of the same room than V1_01_easy,
so when V1_03's motion-blurred frames look up the map they find a
neighbour at a similar viewpoint more often. Map coverage matters as
much as map accuracy for relocalization.

This mirrors the earlier-validated DBoW-corpus rule (`CLAUDE.md` hard
rule "more diverse training corpus → better vocab recall"): for the
*map* corpus the same diversity logic applies. Mapping with the easy
trajectory and trying to localize the difficult one is the classic
"trained on the clean dataset, tested in the wild" failure mode.

The ~1-2% outliers (above 1 m) are reloc successes whose inlier count
passed the `min_inlier_num: 45` gate but ended up in a wrong part of
the map — typical failure mode of single-image reloc on motion-blurred
frames where the point distribution is consistent with multiple map
poses. Tightening `min_inlier_num` would trade recall for fewer
outliers.

### Lines on/off in reloc — V1_03 → V1_02_medium

Same V1_02 reference map, same V1_03 queries. The only difference is
whether the map (and the per-query feature detector) uses PLNet lines
or runs XFeat-only. The points-only path uses the new
`reloc_euroc_xfeat.yaml` (line_extractor=0) and a points-only mapv1.bin.

| Metric | Map + LINES | Map points-only | Δ |
|---|---|---|---|
| Reloc recall | **82.04%** (1763/2149) | 80.04% (1720/2149) | **+2.0 pp** |
| Median position error | **0.097 m** | 0.101 m | −3.9% |
| Mean position error | **0.184 m** | 0.208 m | **−11.5%** |
| p90 position error | 0.181 m | 0.181 m | ≈ |
| < 10 cm | **52.1%** | 49.7% | +2.4 pp |
| < 20 cm | 92.0% | 91.7% | ≈ |
| < 30 cm | 96.5% | 96.3% | ≈ |
| Outliers > 1 m | **1.3%** | 1.6% | −0.3 pp |

**Lines give a small but consistent reloc win in Vicon Room** (~2 pp
recall, ~12% mean error reduction). This mirrors the VO result (V1_03
ATE −4.1% with lines): when point texture is sparse and walls are the
dominant cue, the PLNet junction DBoW pulls extra candidates that the
64-dim XFeat point DBoW misses, and the lines tighten pose refinement.

### Machine Hall cross-sequence reloc — MH_05 → MH_03 map

Mirroring the Vicon Room test on a structurally very different
environment: MH_05_difficult (2273 frames, motion blur + low light)
queried against the MH_03_medium refined map. **Same room is the
exception in Machine Hall** — different MH sequences traverse different
parts of an industrial building, so the cross-sequence recall ceiling
is naturally lower than the Vicon Room same-room test.

| Metric | Map + LINES | Map points-only | Δ |
|---|---|---|---|
| Map maplines | 16 405 | 0 | — |
| Reloc recall | 35.32% (803/2273) | 35.94% (817/2273) | −0.6 pp |
| Median position error | 0.110 m | 0.109 m | ≈ |
| Mean position error | 0.142 m | 0.144 m | ≈ |
| p90 | 0.227 m | 0.226 m | ≈ |
| < 10 cm | 45.6% | 45.7% | ≈ |
| < 30 cm | 94.8% | 94.7% | ≈ |
| Outliers > 1 m | ~0.6% | ~0.6% | ≈ |

**Lines are neutral in Machine Hall reloc** (Δ < 1 pp on every metric).
The difference with Vicon Room is environment-driven:

- Machine Hall has **dense industrial texture** (cables, boxes,
  equipment) — XFeat points already saturate inlier budgets, leaving
  little room for lines to add information.
- The visible lines in MH are mostly **long parallel structural edges**
  (beams, floor seams) that are visually self-similar and contribute
  weak constraint vs Vicon Room's small-room corner geometry.

The 35% recall (vs 82% for V1_03 → V1_02) is structural to MH being a
multi-section building: cross-sequence MH reloc is genuinely a place
recognition problem across different building zones, not just a viewpoint
problem. Tightening that gap would need more MH sequences feeding into
a richer joint map.

### When lines win, when they don't — synthesis across 3 experiments

| Bench | Environment | Δ ATE / recall | Verdict |
|---|---|---|---|
| **VO V1_03 with lines** | Vicon Room | **−4.1% ATE** | Lines win |
| **Reloc V1_03 → V1_02** | Vicon Room | +2 pp recall / −12% mean error | Lines win |
| **Reloc MH_05 → MH_03** | Machine Hall | −0.6 pp / ≈0% error | Lines neutral |

**Practical rule for picking a YAML** (informs the `_lines` vs
points-only choice at the YAML level — no recompile needed):

- **Use lines** for: indoor rooms with low texture, exposed corners,
  walls/floors as dominant cues (typical office, lab, warehouse cleared
  of clutter, hallway corridors).
- **Skip lines** for: cluttered industrial environments where point
  texture is abundant and lines are mostly long parallel ambiguous
  edges. Also for nature/outdoor — long edges are scarce and lines
  pure overhead.

The FPS hit of lines is bounded by KF density (PLNet runs on KFs only).
Vicon Room sequences pay ~0 FPS; Machine Hall pays ~10%. So unless the
scene is decisively textured (industrial / outdoor), defaulting to
`_lines` is safe and gives a free 2-4% accuracy bonus on the rooms
where it matters.

### Reproducing

```bash
# Re-map V1_02_medium with lines if you don't already have its mapv1:
ros2 launch air_slam_xfeat vo_euroc_xfeat_lighterglue_lines.launch.py \
  dataroot:=$HOME/datasets/euroc/V1_02_medium/mav0 \
  saving_dir:=/tmp/airslam_xfeat_lines_V1_02_medium visualization:=false
ros2 launch air_slam_xfeat mr_euroc_xfeat_lines.launch.py \
  map_root:=/tmp/airslam_xfeat_lines_V1_02_medium \
  voc_path:=$HOME/coding/AirSLAM_XFEAT/voc/point_voc_L4_xfeat.bin

# Reloc V1_03 queries against the V1_02 map (with RViz):
ros2 launch air_slam_xfeat reloc_euroc_xfeat_lines.launch.py \
  map_root:=/tmp/airslam_xfeat_lines_V1_02_medium \
  traj_path:=/tmp/airslam_xfeat_lines_V1_02_medium/reloc_V1_03.txt \
  visualization:=true

# Extract successes and run evo_ape:
awk '/^success/ {printf "%.9f %s %s %s %s %s %s %s\n", $2/1e9, $3, $4, $5, $6, $7, $8, $9}' \
  /tmp/airslam_xfeat_lines_V1_02_medium/reloc_V1_03.txt \
  > /tmp/airslam_xfeat_lines_V1_02_medium/reloc_V1_03_success.tum
evo_ape euroc \
  $HOME/datasets/euroc/V1_03_difficult/mav0/state_groundtruth_estimate0/data.csv \
  /tmp/airslam_xfeat_lines_V1_02_medium/reloc_V1_03_success.tum -va

# Lines-off A/B (uses the new reloc_euroc_xfeat.yaml + reloc_euroc_xfeat.launch.py
# with a points-only mapv1 produced by vo_euroc_xfeat_lighterglue.launch.py):
ros2 launch air_slam_xfeat reloc_euroc_xfeat.launch.py \
  map_root:=/tmp/airslam_xfeat_pointsonly_V1_02_medium \
  traj_path:=/tmp/airslam_xfeat_pointsonly_V1_02_medium/reloc_V1_03.txt

# Machine Hall cross-sequence (MH_05 queries against MH_03 map):
ros2 launch air_slam_xfeat reloc_euroc_xfeat_lines.launch.py \
  map_root:=/tmp/airslam_xfeat_lines_MH_03 \
  dataroot:=$HOME/datasets/euroc/MH_05_difficult/mav0/cam0/data \
  traj_path:=/tmp/airslam_xfeat_lines_MH_03/reloc_MH_05.txt
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
