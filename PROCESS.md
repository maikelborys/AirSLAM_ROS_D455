# PROCESS — AirSLAM-XFeat journey

How we got from "replace SuperPoint with XFeat" to **51.3 FPS, ATE
0.070 m post-refinement on EuRoC MH_03, Apache-2.0 commercial-deployable**.
Includes the failed paths — those matter as much as the successes.

For the current state see `STATUS.md`. For architecture see
`ARCHITECTURE.md`. For tuning hard rules see `CLAUDE.md`. For raw
numbers see `output/benchmarks_xfeat.md`.

## Mission framing

Starting point: `jazzy-port` of `sair-lab/AirSLAM` validated at
**0.039 m ATE post-refinement, 38.3 FPS on MH_03** — but the SuperPoint
weights are MagicLeap non-commercial, and the shipped DBoW2 vocab is
derived from SuperPoint features, so the whole stack was tainted.

Goal: keep the AirSLAM SLAM backend (g2o BA, IMU, keyframe management,
DBoW2 loop closure) and swap the perception layer for an
**Apache-2.0** stack. XFeat (Verlab) + LighterGlue (kornia/Verlab) are
the natural choices.

## Phase-by-phase chronicle

### Phase 0 — Repo bring-up (commit `33ef0f3`)

Cloned `maikelborys/AirSLAM_ROS_D455@jazzy-port`, branched off as
`jazzy-xfeat-ros2`, renamed package `air_slam` → `air_slam_xfeat` so
both stacks coexist under `~/ros2_ws/src/`. Symlink into the workspace.

### Phase 1 — XFeat ONNX export + TRT 10 engine (commits `8c4fa43`, `fc181bb`)

**First gotcha**: `torch.onnx.export` with `dynamic_axes={H, W}` failed:

```
torch.onnx.errors.SymbolicValueError: Unsupported: ONNX export of
operator Unfold, input size not accessible.
```

`modules/model.py:135` in upstream XFeat traces through an
InstanceNorm + Unfold combo that needs static shape at trace time.

**Fix**: drop `dynamic_axes`, default to static (480x752) export. Live
with cv::resize-to-fixed-size at runtime. This is enshrined as
**CLAUDE.md rule X1**.

Engine built at FP32 + `--noTF32` + `--builderOptimizationLevel=5`.
**0.7 ms/frame on GPU**, 1425 qps trtexec throughput.

### Phase 2 — Numerical-diff regression guard (commit `fc181bb`)

`scripts/numerical_diff_xfeat.py` compares onnxruntime CPU reference
to trtexec engine output. Pass gate: **cos ≥ 0.999** per output (feats,
keypts, rel) and max abs diff ≤ 1e-3.

First-run validation on MH_03_medium:
```
feats   cos=1.000000  max|d|=5.32e-05
keypts  cos=1.000000  max|d|=2.09e-04
rel     cos=1.000000  max|d|=3.44e-06
```

Bit-equivalent to onnxruntime CPU reference. From here on, this script
is the gate before any change to the engine, ONNX patching, or
trtexec flags.

### Phase 3 — XFeat C++ class (commit `c7af229`)

`include/xfeat.h` + `src/xfeat.cpp` mirror SuperPoint's 5-step TRT 10
`infer()`. Output is a 67-row Eigen matrix (3 + 64 dims). Post-process
on CPU at this point: softmax(65) + drop dustbin + 8x8 unfold + cv::
dilate NMS + top-K + bilinear sample + L2-norm.

Standalone smoke test (`demo/test_xfeat.cpp`): **4.5 ms/frame, 221 Hz**,
1024 keypoints, descriptor norms = 1.000 (sanity).

### Phase 4 — Descriptor-dim plumbing (commit `bad4df4`)

Decision point: how to thread 64-dim XFeat descriptors through
AirSLAM's 256-dim-aware code? Three options considered:

  (a) Refactor everything to template on `kDescriptorDim`. **Huge diff
      across the whole repo.**
  (b) Replace `Database` / `FSuperpoint` / matchers with 64-dim
      parallels. **Medium diff, lots of duplication.**
  (c) **Zero-pad XFeat's 64-dim descriptor to 256** in
      `feature_detector.cc::DetectXFeat`. Padded rows contribute 0 to
      L2 distance, so the existing 256-dim vocab + descriptor blocks
      keep working. **Minimal diff.**

**Picked (c).** The XFeat-mode feature matrix is 259-row: rows 0..2
xy+score, rows 3..66 XFeat 64-dim, rows 67..258 zero. CLAUDE.md X2
documents this contract. Consumers in XFeat mode must read only rows
3..66 — anything reading 256-dim sees garbage in the pad.

### Phase 5 — Matcher: MNN (commit `a969bb2`)

LighterGlue ONNX export looked risky upfront (correctly — see Phase 5b
below), so first matcher was a pure-Eigen MNN + Lowe ratio cosine
matrix. `cv::findFundamentalMat` RANSAC for outlier rejection (same
as the LightGlue / SuperGlue paths).

Defaults from `xfeat_slam_ws/.../stereo_matcher.hpp`:
- `min_cosine_similarity: 0.70` (drop below)
- `lowe_ratio: 0.95` (keep if best * 0.95 > second_best — XFeat is less
  peaky than SIFT so 0.80 over-filters)
- `require_mutual_nn: 1`

### Phase 6 — DBoW2 vocab + map_refinement (commit `0f70628`)

Same zero-pad trick used to dodge a Database refactor: train the vocab
as `SuperpointVocabulary` (L=256) over XFeat descriptors padded to
256-dim. Zeros contribute 0 to L2 → the vocab is effectively 64-dim in
disguise, but every existing call site keeps working.

`demo/train_voc_xfeat.cpp` reads EuRoC TRAIN sequences (MH_01 + V1_01;
MH_02 not in the local dataset), extracts ~1.23 M XFeat descriptors,
trains DBoW2 k=10 L=4 TF_IDF L1_NORM. Output:
`voc/point_voc_L4_xfeat.bin` (11 MiB, **Apache-2.0** — first
license-clean DBoW2 vocab for this stack).

Map refinement on the XFeat-MNN map of MH_03 found **405 loop pairs**
(vs SuperPoint baseline's 127 — XFeat is more aggressive at place
recognition). Pose graph + mappoint merge + global BA converged.

**First headline result**: raw VO ATE 0.308 m → **0.104 m post-
refinement, 3x improvement**. Now within an order of magnitude of the
SP baseline.

### Phase 7+8 — Configs + benchmarks (commit `0c47088`)

First end-to-end VO run with XFeat+MNN on MH_03:
- 16.85 FPS (vs 38.3 SP baseline) — **2.3x slower**
- 0.308 m raw VO ATE (vs ~0.10 m SP) — **3x worse**

Documented in `output/benchmarks_xfeat.md`. The user pushed back on
the speed: **"why so slow?"**

### Threshold sweep (failed, commits not landed)

Tried tuning `min_cosine_similarity` to 0.82 + `lowe_ratio` to 0.85
(stricter MNN). Result: SLAM failed to initialise (matches starved).
At 0.75 / 0.90: trajectory diverged to 70 m ATE.

**Conclusion**: cos=0.70 + lowe=0.95 was already near-optimal for the
matcher. Threshold tuning hit a ceiling for MNN. To break through we
needed either a better matcher (LighterGlue) or a different bottleneck
to attack.

### cuBLAS MNN matcher (commit `d256826`) — failed as FPS lever

Hypothesis: MNN's CPU Eigen GEMM was ~10 ms/call x 2 calls/frame =
20 ms/frame of CPU. Moving to cuBLAS SGEMM on GPU should drop this to
sub-ms.

Implementation: `PointMatcher` caches a `cublasHandle_t` + device
buffers; the N0xN1 cosine matrix is computed by a single
`cublasSgemm(op_a=T, op_b=N)`. To avoid a CPU transpose pass, we
compute `D1^T @ D0` so the col-major C output has the same byte layout
as row-major S — no transpose needed on host.

Result: GEMM dropped to ~0.3 ms (~30x speedup in isolation). Full
end-to-end VO FPS moved 16.85 → **16.9 (~no change)**.

**Lesson**: the cosine GEMM was never the bottleneck. The dominant
per-frame cost is g2o local BA on CPU during keyframe insertion, which
scales with `max_keypoints × recent_keyframes`. The matcher cost was a
small slice already. Kept the cuBLAS path because the architecture is
correct — if a future change makes the matcher the hot spot, the
ceiling is now higher.

This was the key insight that informed every later decision: **stop
optimising the matcher, look at the actual cost distribution.**

### Phase 5b — LighterGlue ONNX attempts (failed, commits `5d1f9c4`, later cleanup)

Switched to LighterGlue for descriptor-aware attention matching.
Plan: export from upstream PyTorch → ONNX → patch for TRT 10 → engine.

**Attempt 1 (kornia LightGlue)**:
```
torch.onnx.errors.SymbolicValueError: IndexError in symbolic_opset9.transpose
  axes[dim0], axes[dim1] = axes[dim1], axes[dim0]
```

`.transpose(-1, -2)` / `.transpose(-2, -3)` calls in kornia's
`feature/lightglue.py` confuse the symbolic exporter when the tensor
rank can't be inferred statically.

**Attempt 2 (cvg/LightGlue source, the upstream LightGlue repo)**:
Same error. The two implementations share the same negative-index
patterns.

**Attempt 3 (sed-patched cvg/LightGlue)**: Manually replaced 7
transposes with explicit positive dims. The export then failed at
`rotate_half` which uses `.unflatten(-1, (-1, 2))` + `.unbind(dim=-1)`.

After ~14 negative-index ops surfaced in `rotate_half` /
`apply_cached_rotary_emb`, abandoned the ONNX path. Time-boxed
~45 min, hit a wall.

**Attempt 4 (`torch.onnx.dynamo_export`)**: newer FX-based exporter,
handles more ops. Failed with a generic OnnxExporterError.

Committed `scripts/export_lighterglue_onnx.py` as a reference for the
next session.

### Phase 5b — LighterGlue via TorchScript (commit `31ed03b`) ✅

**Pivot**: `torch.jit.trace` records aten ops directly, doesn't go
through the ONNX symbolic at all. The same kornia + cvg LightGlue code
that broke ONNX traced cleanly:

```
torch.jit.trace(wrapper, (kp0, d0, kp1, d1), strict=False)
torchscript saved 4.5 MiB
reload roundtrip: matches mismatch=0, scores max|d|=0.0
```

Bit-identical to PyTorch forward. Loaded in C++ via libtorch's
`torch::jit::load`. CLAUDE.md X4 documents this as the chosen path.

**Caveat**: tracing on CPU left the `image_size` buffer's device as
CPU, so loading on CUDA crashed with "Expected all tensors to be on
the same device". Fix: trace on CUDA from the start (install
`torch==2.4.0+cu121` wheel in the export venv).

### LighterGlue static-N re-trace (commit `bc3ae53`) — quick double FPS

First trace was at N=1024 — too aggressive. XFeat detects ~400-500
real keypoints per frame on EuRoC, so 1024 was mostly padding wasted
on the O(N²) attention pass.

Re-traced at N=512: **FPS jumped 18 → 27 (on 200-frame window)**.
The N=1024 attention was 4x more expensive per call. CLAUDE.md X4
documents the trace-N / config max_keypoints coupling.

### LighterGlue full benchmark (commit `44266c7`)

Full MH_03 with the N=512 trace:
- **18.67 FPS** (vs 16.85 MNN — small win)
- **Raw VO ATE 0.251 m** (vs 0.308 MNN — **-19%**)
- **Post-refinement ATE 0.061 m** (vs 0.104 MNN — **-42%**)

Loop pairs found by refinement: 269 (vs MNN's 405). Fewer loop pairs
but cleaner — attention-based matching produces fewer spurious
correspondences. The post-refinement ATE was suddenly only **1.6x
behind** the SP baseline.

### CUDA-port XFeat post-processing (commit `2c3cd43`)

Tier 1 CUDA work. `src/xfeat_postproc.cu` has 4 kernels:
1. softmax(65) over each (Hp, Wp) cell + drop dustbin + 8x8 unfold →
   full-res heatmap. One thread per cell.
2. NMS dilate (k=5 default): one thread per pixel, brute kxk max.
3. Emit candidates: atomic-counter writes (x, y, h * bilinear(rel))
   tuples for survivors.
4. Sample descriptors at top-K (host partial_sort) positions; bilinear
   sample 64 channels of `feats` per keypoint, L2-norm via shared-
   memory reduction.

Reads engine outputs directly from device buffers via
`BufferManager::getDeviceBuffer` — skips the H2D copy of the
130-channel dense outputs (~1.5 MiB).

Isolation benchmark (`test_xfeat`):
- Before: 4.5 ms/frame, 221 Hz
- **After: 1.77 ms/frame, 564 Hz** (2.5x speedup)
- Top-5 keypoints + scores **bit-identical** to CPU reference

Full-VO impact: +1-2% FPS only. Confirmed — post-proc was already a
small slice. Wins are isolated but cumulative if many CUDA wins land.

### The keyframe-rate revelation (commit `cd0fa51`) ⭐

After CUDA postproc + LighterGlue + everything else, full-sequence FPS
sat at 18.7. Investigation: AirSLAM was inserting **1130 keyframes**
in 2700 frames (1 per 2.4 frames). SuperPoint baseline inserts 302
(1 per 9 frames). **3.7x more keyframes = 3.7x more BA work**.

Read `MapBuilder::AddKeyframeCheck` source carefully and discovered
the threshold semantics were the **opposite** of intuition:

| YAML knob | Higher = | What I thought first | Reality |
|---|---|---|---|
| `tracking_point_rate` | MORE KFs | "stricter tolerance" → fewer | Higher threshold triggers KF SOONER as tracking degrades |
| `max_num_match` | MORE KFs | "raise the ceiling" | Triggers when matches DROP below this — so high = always trigger |
| `tracking_parallax_rate` | FEWER KFs | (✓ I had this right) | Requires more parallax for parallax-driven KF |

Applied **inverted** tuning:
```yaml
tracking_point_rate: 0.65 → 0.40
max_num_match:       80   → 40
tracking_parallax_rate: 0.10 → 0.20
```

Result on MH_03 with same XFeat+LighterGlue+CUDA pipeline:

| Metric | Before tune | After tune |
|---|---|---|
| Keyframes | 1130 | **408** |
| FPS (full) | 18.67 | **51.3** 🔥 (+175%) |
| Raw VO ATE | 0.251 m | **0.207 m** (-18%) |
| Post-refinement ATE | 0.061 m | 0.070 m (+14%) |

**51.3 FPS is faster than the SuperPoint baseline (38.3 FPS).** Raw
VO ATE actually improved because each KF insertion is itself a small
drift source. Post-refinement slightly worse because fewer KFs means
fewer loop-closure candidates — that's the speed/precision dial.

### V1_01_easy validation (in commit `ad95697`'s message)

Vicon Room 1 (indoor low-texture) — historically a stress test for
visual-only frontends.

Before KF tune: **diverged to 59.3 m ATE**.
After KF tune: **0.083 m raw VO at 63.5 FPS**.

The KF tune wasn't just a speed knob — it **rescued V1_01 from
outright failure**. Excessive keyframe insertion was generating noisy
short baselines and corrupting tracking on low-texture scenes.

### Phase 9 — Docs + closeout (commits `eef43f2`, `ad95697`, `PROCESS.md`)

Final docs structure:
- `CLAUDE.md` (orientation + hard rules X1–X8)
- `STATUS.md` (measured numbers + future work)
- `ARCHITECTURE.md` (pipeline + file map)
- `README_JAZZY_XFEAT.md` (delta vs jazzy-port)
- `output/benchmarks_xfeat.md` (benchmark log)
- `PROCESS.md` (this file — the journey)

Branch pushed to `origin/jazzy-xfeat-ros2` on
`maikelborys/AirSLAM_ROS_D455`.

## Lessons learned

1. **Profile before optimising.** The cuBLAS MNN port was technically
   correct but a wasted FPS lever — the matcher was never the
   bottleneck. The keyframe rate was hiding in plain sight in the
   YAML.

2. **Read the source for threshold semantics.** AirSLAM's keyframe
   thresholds did the *opposite* of what their names suggested.
   3 hours of `MapBuilder::AddKeyframeCheck` reading would have saved
   the dead-end strict-threshold sweep.

3. **`torch.jit.trace` is the ONNX escape hatch.** When
   `torch.onnx.export` chokes on negative-index ops, scripting or
   tracing usually doesn't. libtorch's runtime is heavy (~1.5 GiB)
   but ships with the OKVIS-X stack anyway, so the cost was zero.

4. **The zero-pad trick is gold.** Padding the 64-dim XFeat
   descriptors to 256 lets us reuse `Database` + `FSuperpoint` +
   `Vocabulary` + `block(3, ..., 256, ...)` consumer code unchanged.
   The 192 zero dims cost a few µs per cosine computation — invisible
   in practice. The "do the right thing" alternative (template
   `Database` on FeatureClass) would have been a 2-day refactor.

5. **Engines are fast; post-processing was the silent CPU cost.** The
   XFeat TRT engine ran at 0.7 ms but the surrounding softmax+NMS+
   sample loop ate 3-4 ms more. CUDA-porting it isolated a 2.5x
   speedup but only +1-2% in VO. The full picture only emerges with
   end-to-end timing.

6. **Architectural decisions taken under uncertainty paid off.** Even
   though cuBLAS MNN didn't move VO FPS, the architecture is now ready
   for when the matcher COULD become the hot spot (e.g. after a GPU BA
   replacement). Don't undo work just because the FPS didn't move
   today.

7. **Commercial license is a hard constraint that drives technical
   choices.** SuperPoint was off-limits; everything we did was framed
   by the Apache-2.0 license requirement. That ruled out some matcher
   candidates (e.g. SuperGlue weights from MagicLeap) before we even
   ran a benchmark.

## What didn't ship in this branch

- LighterGlue as a TRT engine (only TorchScript via libtorch). Future
  work; would let us drop the libtorch dependency.
- D455 live-camera ROS 2 topic ingestion. The wiring exists in
  `master` (ROS 1) but wasn't cherry-picked. Future work.
- Line features (PLNet wireframe head). Plan called it conditional on
  Phase 6+ metrics passing — they're passing, so re-enabling lines is
  the next unlock for V2 sequences specifically.
- Replacing g2o local BA with a GPU BA library (MegBA / DeepLM /
  Theseus). The biggest remaining FPS lever; ~1-2 weeks of risky work.

## Final numbers (snapshot at branch push)

| Stage | Outcome |
|---|---|
| **EuRoC MH_03_medium** | FPS 51.3 (>SuperPoint 38.3) · Raw 0.207 m · Post-ref 0.070 m |
| **EuRoC V1_01_easy** | FPS 63.5 · Raw 0.083 m |
| **XFeat isolation** | 564 Hz (1.77 ms/frame, CUDA postproc) |
| **License** | Apache-2.0 (XFeat + LighterGlue + new vocab) |
| **Realtime margin** | ~2x camera frame rate (30 Hz target) |
| **Branch** | `jazzy-xfeat-ros2` @ `maikelborys/AirSLAM_ROS_D455` |
