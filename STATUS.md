# STATUS — AirSLAM-XFeat (branch jazzy-xfeat-ros2)

Snapshot of the current measured state. For pipeline diagrams see
`ARCHITECTURE.md`. For the orientation map + hard rules see `CLAUDE.md`.
For the full benchmark log see `output/benchmarks_xfeat.md`.

## TL;DR

**AirSLAM-XFeat at 51.3 FPS is now faster than the SuperPoint baseline
(38.3 FPS) on EuRoC MH_03**, with post-refinement ATE 0.070 m (vs the
paper's 0.039 m, ~1.8x behind) and **fully Apache-2.0 license-clean for
commercial deployment**. V1_01_easy (Vicon Room indoor, low texture)
runs at **63.5 FPS with raw-VO ATE 0.083 m** — a 700x improvement over
the same branch before keyframe-rate tuning.

## Measured performance

### EuRoC MH_03_medium (2700 frames, stereo + IMU)

| Metric | Current value | vs SuperPoint paper |
|---|---|---|
| **FPS** (full sequence) | **51.3** | 1.34x faster |
| FPS (peak steady-state window) | 99+ | 2.5x faster |
| XFeat engine forward | 0.7 ms (GPU, TRT 10) | comparable to SP's ~5 ms |
| XFeat + post-proc isolated | 1.77 ms (564 Hz) | n/a |
| LighterGlue per call | ~5 ms (libtorch GPU) | comparable to LightGlue TRT |
| Keyframes | 408 (1 per 6.6 frames) | SP: 302 (1 per 8.9) |
| Raw VO ATE | 0.207 m | SP: ~0.10 m, 2.0x behind |
| **Post-refinement ATE** | **0.070 m** | SP: 0.039 m, **1.8x behind** |
| Loop pairs found | 269 (clean) | SP: 127 |

### EuRoC V1_01_easy (2912 frames, Vicon Room 1, indoor low-texture)

| Metric | Current value (KF-tuned) | Before KF tune |
|---|---|---|
| **FPS** (full sequence) | **63.5** | 25.3 |
| Keyframes | 219 | (similar) |
| **Raw VO ATE** | **0.083 m** | **59.3 m** (diverged) |
| trajectory.size | 219 poses | 1391 poses (drifted) |

The V1_01 win is the most striking single result of the recent work:
the same code with the WRONG keyframe thresholds was hopelessly broken,
and the KF tune brought it to sub-10 cm without any algorithmic change.

## Is this real-time?

**Yes.** 51-63 FPS on 480x752 stereo + IMU input, 2-3x camera frame
rate (D455 IR streams come in at 20-30 Hz). The per-frame budget at
30 Hz is 33 ms; we're consistently under 20 ms in steady state. The
worst-case spike during a keyframe insertion + local BA is around
60-80 ms — still under 30 Hz IF the next frame can be processed in
parallel (today it's serial — see "Future work" #2).

## Pipeline placement on GPU vs CPU

```
Component                    Device    Time / frame    Notes
────────────────────────────  ────────  ──────────────  ──────────────────────
1. cv::resize + normalize     CPU       ~0.5 ms         negligible
2. XFeat engine forward       GPU/TRT   0.7 ms          static 480x752 ONNX
3. XFeat post-proc (4 kerns)  GPU/CUDA  ~0.5 ms         src/xfeat_postproc.cu
4. LighterGlue (per call,
   typically 2-3 calls/frame
   for stereo + temporal)    GPU/libtorch  ~5 ms each   .pt loaded once
5. Frame creation, mappoint
   association, ego pose      CPU       ~5 ms          shared_ptr-heavy
6. cv::findFundamentalMat
   RANSAC inlier filter       CPU       ~2 ms each     OpenCV CPU
7. g2o local BA on keyframe   CPU       30-120 ms      single-thread g2o
   insertion (not every frame!)
8. RosPublisher topic emit    CPU       ~0.5 ms        async
────────────────────────────  ────────  ──────────────  ──────────────────────
Steady-state (no KF)                    ~12-15 ms      ≈ 70-80 FPS
Keyframe insertion                      ~60-100 ms     ≈ 10-15 FPS spike
Full-sequence average on MH_03          ~19.5 ms       ≈ 51 FPS
```

The remaining FPS gap to a Jetson-SLAM-class number (60+ FPS sustained)
is entirely **g2o single-threaded BA**, which is shared with the
SuperPoint baseline.

## What works

- ✅ **XFeat extractor on TRT 10**, validated cos = 1.000 vs onnxruntime
  CPU reference.
- ✅ **CUDA post-proc** (softmax + NMS-dilate + emit + descriptor sample
  + L2-norm) — 4 kernels, bit-identical to the CPU reference path,
  564 Hz in isolation.
- ✅ **MNN matcher** on cuBLAS (descriptor matrix on GPU; the cosine
  matrix isn't the FPS bottleneck so this is mostly a cleanliness win).
- ✅ **LighterGlue via TorchScript** (Path D — `torch.jit.trace`
  handles negative-index ops that `torch.onnx.export` rejected).
- ✅ **64-dim DBoW2 vocab** via the zero-pad trick — no Database refactor
  needed. `voc/point_voc_L4_xfeat.bin` is Apache-2.0 (trained from
  scratch on EuRoC TRAIN sequences, no SuperPoint taint).
- ✅ **Map refinement** with the new vocab on the XFeat map — 269 loop
  pairs found on MH_03, global BA converges, post-refinement ATE 0.070 m.
- ✅ **Keyframe rate tuning** for XFeat-density matches — 1130 → 408
  keyframes, 175% FPS gain with negligible ATE cost.
- ✅ **ROS 2 topics** all unchanged from jazzy-port: `/AirSLAM/feature`,
  `/AirSLAM/frame_pose`, `/AirSLAM/odometry` (path), `/AirSLAM/map`
  (mappoint markers), `/AirSLAM/keyframe`. RViz config at
  `rviz/vo_jazzy.rviz` shows path + landmarks live.
- ✅ **EuRoC MH_03 medium and V1_01_easy** end-to-end validated.

## What's known to be limited

- ⚠️ **MNN matcher with kpts=400** breaks SLAM (3.03 m ATE on MH_03).
  The default config uses kpts=1024 for MNN, kpts=512 for LighterGlue.
  Sticking with these is documented in CLAUDE.md rule X5.
- ⚠️ **g2o BA spikes on keyframe insertion** (~60-100 ms) cap the
  sustained FPS. Not fixable without replacing the optimizer.
- ⚠️ **Vocab trained on MH_01 + V1_01 only** (1.23 M descriptors). MH_02
  isn't in the local EuRoC dataset. Adding it would broaden loop recall
  on unseen scenes.
- ⚠️ **Lines disabled** (XFeat = points-only this branch). PLNet
  wireframe head is conditional on Phase 8 metrics passing — they
  passed; re-enabling lines is on the Future-work list.
- ⚠️ **D455 live-camera path** is on the original `master` branch
  (ROS 1) and not yet cherry-picked into jazzy-xfeat-ros2. Listed in
  Future-work.

## Future work (ordered by expected impact)

1. **Replace g2o local BA with GPU BA** (MegBA / DeepLM / TheseusAI).
   Expected +50-100% FPS. **HIGH risk** — g2o is tightly coupled to
   AirSLAM's custom vertex/edge types and robust kernels. ~1-2 weeks.

2. **Async feature pipeline** (Jetson-SLAM style). Overlap XFeat
   forward of frame N+1 with tracking + small g2o of frame N. AirSLAM
   already has a 2-thread feature/tracking split, but they're not fully
   decoupled. Expected +20-40% FPS. ~1-2 days refactor.

3. **D455 live-camera ROS 2 path**. Cherry-pick the ROSDataset wiring
   pattern from `master` branch, port to rclcpp +
   `message_filters::ApproximateTimeSynchronizer` for
   `/camera/camera/infra{1,2}/image_rect_raw` + optional IMU. Enables
   on-robot benchmarking outside EuRoC. ~3-5 h.

4. **Re-enable lines via PLNet wireframe head with XFeat-anchored
   points**. Phase 6+ metrics passed the promotion gate from the
   original plan. ~4 h. Should help V2 sequences specifically.

5. **Bigger DBoW2 vocab corpus**. Adding MH_02 + V1_02 + V2_01 to the
   training set (~3 M descriptors total instead of 1.23 M) would
   improve loop recall in unseen environments.

6. **LighterGlue ONNX/TRT export** to drop the libtorch dependency.
   Currently blocked by `.transpose(-X, -Y)` / `.unflatten(-1, ...)`
   ops in cvg/LightGlue source. Requires systematic positive-dim
   patching of the forward. ~3-4 h.

7. **Multi-threaded g2o** (OpenMP build flag). Some +30-50% on the
   keyframe BA spike. Risk: numerical drift if reduction order shifts.

## License posture

| Component | License | Commercial use |
|---|---|---|
| SuperPoint (MagicLeap) | Non-commercial research | ❌ |
| `voc/point_voc_L4.bin` (SP-derived) | Tainted | ❌ |
| **XFeat (Verlab)** | **Apache-2.0** | ✅ |
| **LighterGlue (Verlab/kornia)** | **Apache-2.0** | ✅ |
| **MNN matcher** | algorithm only, no weights | ✅ |
| **`voc/point_voc_L4_xfeat.bin`** (this branch) | Apache-2.0 (XFeat-derived) | ✅ |
| AirSLAM code | Apache-2.0 | ✅ |
| DBoW2 / g2o / Eigen / OpenCV / TensorRT | BSD/MIT/MPL/Apache/EULA | ✅ |
| libtorch | BSD-3 | ✅ |

The SuperPoint baseline (`jazzy-port` of `maikelborys/AirSLAM_ROS_D455`)
**cannot be deployed commercially**. This branch can.

## Commit log (jazzy-port → jazzy-xfeat-ros2)

```
cd0fa51 KF tuning win: 51.3 FPS (>SuperPoint baseline) + 0.070 m post-refinement
44266c7 benchmarks: LighterGlue post-refinement ATE 0.061 m (1.6x behind SP) + CUDA postproc smoke
2c3cd43 CUDA-port XFeat post-processing (564 Hz isolated, 4 kernels)
bc3ae53 LighterGlue: re-trace at N=512 (4x less attention work, FPS 18 → 27)
31ed03b Phase 5b: LighterGlue via TorchScript + libtorch (Path D worked!)
53443ab visual_odometry: --max-frames + --skip_save_map for fast iteration
d8e9af2 benchmarks: add post-refinement ATE column + licensing section
d256826 CUDA-port MNN matcher via cuBLAS SGEMM (no FPS impact — wrong bottleneck)
0f70628 Phase 6: 64-dim DBoW2 vocab + map_refinement — 0.308 -> 0.104m ATE (3x)
5d1f9c4 scripts/export_lighterglue_onnx.py — first attempt (BLOCKED)
eef43f2 Phase 9: docs + closeout — XFeat-mode rules, README delta, benchmarks
0c47088 Phase 7+8: vo_euroc_xfeat config/launch + first XFeat ATE on MH_03
a969bb2 Phase 5: MNN+Lowe matcher (XFeat 64-dim, pure Eigen, no engine)
bad4df4 Phase 4: FeatureDetector dispatch on feature_extractor (0/1/2)
c7af229 Phase 3: XFeat C++ class + smoke test (4.5ms/frame, 1024 kpts)
fc181bb Phase 1+2: XFeat ONNX exported, TRT 10 engine validated
8c4fa43 Phase 1: XFeat ONNX export scripts + engine build wiring
33ef0f3 Phase 0: rename package air_slam -> air_slam_xfeat
```
