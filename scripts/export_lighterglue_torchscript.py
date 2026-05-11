"""Export LighterGlue (kornia LightGlue arch + XFeat weights) -> TorchScript .pt.

This is Path D from the AirSLAM-XFeat plan: instead of fighting torch.onnx
to emit LightGlue ops (it can't — negative-index transposes and unflatten
ops in the kornia/cvg LightGlue forward trip symbolic_opset9), trace the
model with torch.jit.trace and save a .pt. The C++ side (src/lighter_glue.cc)
loads it via libtorch's torch::jit::load and runs forward on GPU.

Inputs (static batch=1, static N — bake the trace at --num-kpts):
  keypoints_0   (1, N, 2)   float32  pixel coords image-0
  descriptors_0 (1, N, 64)  float32  L2-normalised XFeat descriptors
  keypoints_1   (1, N, 2)
  descriptors_1 (1, N, 64)

Outputs (TorchScript tuple):
  matches0   (1, N)  int64    index into image-1 keypoints (or -1)
  mscores0   (1, N)  float32  match confidence in [0, 1]

The trace uses the cvg/LightGlue source — we vendor it via the same temp
venv as scripts/get_xfeat_onnx.sh, no upstream patching needed because
torch.jit.trace handles negative-index ops correctly (unlike torch.onnx).

Usage:
  source ~/.cache/airslam_xfeat/export_venv/bin/activate
  python scripts/export_lighterglue_torchscript.py \\
    --xfeat-repo /home/maikel/.cache/airslam_xfeat/accelerated_features \\
    --cvg-lightglue /tmp/cvg_lightglue \\
    --output output/lighterglue.pt \\
    [--num-kpts 1024] [--img-w 752] [--img-h 480]
"""
from __future__ import annotations

import argparse
import hashlib
import os
import sys
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--xfeat-repo", required=True, type=Path,
                   help="Clone of verlab/accelerated_features (any commit at "
                        "or after 8728d2f for xfeat-lighterglue.pt).")
    p.add_argument("--cvg-lightglue", required=True, type=Path,
                   help="Clone of github.com/cvg/LightGlue (any recent commit).")
    p.add_argument("--weights", type=Path, default=None,
                   help="Path to xfeat-lighterglue.pt; defaults to "
                        "<xfeat-repo>/weights/xfeat-lighterglue.pt")
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--num-kpts", type=int, default=1024)
    p.add_argument("--img-w", type=int, default=752)
    p.add_argument("--img-h", type=int, default=480)
    args = p.parse_args()

    if not (args.xfeat_repo / "weights" / "xfeat-lighterglue.pt").exists() and not args.weights:
        sys.exit(f"weights not found under {args.xfeat_repo}/weights — pass --weights")
    if not (args.cvg_lightglue / "lightglue" / "lightglue.py").exists():
        sys.exit(f"cvg/LightGlue not at {args.cvg_lightglue}/lightglue/lightglue.py")

    sys.path.insert(0, str(args.cvg_lightglue))
    sys.path.insert(0, str(args.xfeat_repo))
    import torch
    from lightglue.lightglue import LightGlue

    # LighterGlue is cvg/LightGlue architecture with the XFeat-specific config
    # below. Critical flags for ONNX/TorchScript export:
    #   flash=False              FlashAttention has no aten-only path.
    #   depth_confidence=-1      no early-exit branches (would break trace).
    #   width_confidence=-1      no point pruning (dynamic gather).
    cfg = dict(
        name="xfeat", input_dim=64, descriptor_dim=96, add_scale_ori=False,
        n_layers=6, num_heads=1, flash=False, mp=False,
        depth_confidence=-1, width_confidence=-1, filter_threshold=0.1,
        weights=None,
    )
    net = LightGlue(features=None, **cfg).eval()

    weights_path = args.weights or (args.xfeat_repo / "weights" / "xfeat-lighterglue.pt")
    state = torch.load(str(weights_path), map_location="cpu", weights_only=False)
    # Verlab's xfeat-lighterglue.pt nests the matcher under "matcher." and
    # uses ".self_attn.{i}." / ".cross_attn.{i}." — rename to the cvg layout
    # which is ".transformers.{i}.{self,cross}_attn." with no "matcher." prefix.
    for i in range(cfg["n_layers"]):
        state = {k.replace(f"self_attn.{i}", f"transformers.{i}.self_attn"): v
                  for k, v in state.items()}
        state = {k.replace(f"cross_attn.{i}", f"transformers.{i}.cross_attn"): v
                  for k, v in state.items()}
    state = {k.replace("matcher.", ""): v for k, v in state.items()}
    missing, unexpected = net.load_state_dict(state, strict=False)
    if missing:
        print(f"[WARN] {len(missing)} missing keys: {list(missing)[:3]}")
    # The 122 unexpected keys are the XFeat extractor weights bundled in the
    # checkpoint — they're not part of the matcher graph, drop them silently.

    class ExportWrapper(torch.nn.Module):
        def __init__(self, inner, img_w, img_h):
            super().__init__()
            self.inner = inner
            # Stash as Python floats — trace will literalise them and the
            # device-bearing tensor is created from kpts0.new_tensor inside
            # forward, so it inherits whatever device kpts0 is on at run time.
            # This dodges the cpu/cuda buffer mismatch you get from any
            # register_buffer / register_parameter approach.
            self._w = float(img_w)
            self._h = float(img_h)

        def forward(self, kpts0, desc0, kpts1, desc1):
            sz = kpts0.new_tensor([[self._w, self._h]])
            data = {
                "image0": {"keypoints": kpts0, "descriptors": desc0,
                            "image_size": sz},
                "image1": {"keypoints": kpts1, "descriptors": desc1,
                            "image_size": sz},
            }
            out = self.inner(data)
            return out["matches0"], out["matching_scores0"]

    # Trace on CUDA when available so the captured constants (and
    # `kpts0.new_tensor`-style helpers) inherit the device. Tracing on CPU
    # bakes CPU constants into the graph and breaks at load time on a CUDA
    # device with "Expected all tensors to be on the same device, but found
    # at least two devices, cuda:0 and cpu!"
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Tracing on {device}")
    wrapper = ExportWrapper(net, args.img_w, args.img_h).eval().to(device)

    N = args.num_kpts
    kp0 = (torch.rand(1, N, 2) * torch.tensor([[[args.img_w, args.img_h]]])).to(device)
    kp1 = (torch.rand(1, N, 2) * torch.tensor([[[args.img_w, args.img_h]]])).to(device)
    d0  = torch.randn(1, N, 64, device=device)
    d1  = torch.randn(1, N, 64, device=device)
    d0 = d0 / d0.norm(dim=-1, keepdim=True)
    d1 = d1 / d1.norm(dim=-1, keepdim=True)

    with torch.no_grad():
        out_ref = wrapper(kp0, d0, kp1, d1)
        traced = torch.jit.trace(wrapper, (kp0, d0, kp1, d1), strict=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    traced.save(str(args.output))

    # Reload + compare to catch silent drift in the trace.
    reloaded = torch.jit.load(str(args.output))
    out_ts = reloaded(kp0, d0, kp1, d1)
    m_mismatch = (out_ref[0] != out_ts[0]).sum().item()
    s_maxdiff = (out_ref[1] - out_ts[1]).abs().max().item()
    sha = hashlib.sha256(args.output.read_bytes()).hexdigest()
    print(f"Wrote {args.output} ({args.output.stat().st_size} bytes)")
    print(f"SHA256: {sha}")
    print(f"Trace regression: matches mismatch={m_mismatch}, "
          f"scores max|d|={s_maxdiff}")
    print(f"Static trace shape: N={N}, image_size=({args.img_w}, {args.img_h})")
    return 0 if (m_mismatch == 0 and s_maxdiff < 1e-5) else 1


if __name__ == "__main__":
    sys.exit(main())
