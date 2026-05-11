"""Export LighterGlue (kornia LightGlue + XFeat weights) -> ONNX.

Upstream verlab/accelerated_features ships LighterGlue as a thin wrapper
around kornia.feature.lightglue.LightGlue with the XFeat-specific weights
file `xfeat-lighterglue.pt`. There is NO upstream ONNX export script — we
write one here and bake in the static-shape compromises that the kornia
graph requires:

  - flash=False         : FlashAttention is not ONNX-exportable.
  - depth_confidence=-1 : early-exit branches give dynamic outputs.
  - width_confidence=-1 : point pruning gives dynamic outputs.
  - mp=False            : autocast mixed precision off (FP32 graph).

Inputs (B=1, N fixed at --num-kpts):
  keypoints_0   (1, N, 2)  float32  pixel coords in image-0 frame
  keypoints_1   (1, N, 2)  float32  pixel coords in image-1 frame
  descriptors_0 (1, N, 64) float32  L2-normalised XFeat descriptors
  descriptors_1 (1, N, 64) float32  L2-normalised XFeat descriptors

Output:
  log_assignment (1, N+1, N+1) float32  the matching matrix (rows/cols
                                        N+1 carry the dustbin).

We deliberately skip the matches0 / matches1 / matching_scores* outputs
because they are produced by an argmax + topk post-process inside kornia
that doesn't ONNX-export cleanly. We do the argmax + dustbin filter on
the C++ side after the engine returns log_assignment.

Note: image_size is hardcoded inside the wrapper as a 1x2 buffer
matching the XFeat input (480 wide, 752 tall = (W, H) order per kornia).
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
                   help="Clone of verlab/accelerated_features (commit post 8728d2f).")
    p.add_argument("--weights", type=Path, default=None,
                   help="Path to xfeat-lighterglue.pt; defaults to "
                        "<repo>/weights/xfeat-lighterglue.pt")
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--num-kpts", type=int, default=1024,
                   help="Static keypoint count baked into the engine.")
    p.add_argument("--img-w", type=int, default=752)
    p.add_argument("--img-h", type=int, default=480)
    p.add_argument("--opset", type=int, default=17)
    args = p.parse_args()

    if not (args.xfeat_repo / "modules" / "lighterglue.py").exists():
        sys.exit(f"LighterGlue not at {args.xfeat_repo}/modules/lighterglue.py "
                 "(make sure you've checked out a commit at or after 8728d2f)")
    sys.path.insert(0, str(args.xfeat_repo))

    import torch
    from kornia.feature.lightglue import LightGlue

    # Build LighterGlue with the XFeat-specific config but with the ONNX-hostile
    # features explicitly disabled. We do NOT import modules.lighterglue.LighterGlue
    # directly because it pins flash=True at class creation; we re-create the
    # kornia LightGlue with our preferred config and then load the same weights.
    cfg = {
        "name": "xfeat",
        "input_dim": 64,
        "descriptor_dim": 96,
        "add_scale_ori": False,
        "add_laf": False,
        "scale_coef": 1.0,
        "n_layers": 6,
        "num_heads": 1,
        "flash": False,             # CRITICAL: ONNX-incompatible otherwise
        "mp": False,
        "depth_confidence": -1,     # CRITICAL: early-exit branches break ONNX
        "width_confidence": -1,     # CRITICAL: point pruning breaks ONNX
        "filter_threshold": 0.1,
        "weights": None,
    }
    LightGlue.default_conf = cfg
    net = LightGlue(None).eval()

    weights_path = args.weights or (args.xfeat_repo / "weights" / "xfeat-lighterglue.pt")
    if not weights_path.exists():
        sys.exit(f"Weights not found: {weights_path}")
    state = torch.load(weights_path, map_location="cpu", weights_only=False)
    # Rename keys per upstream lighterglue.py.
    for i in range(cfg["n_layers"]):
        pat = (f"self_attn.{i}", f"transformers.{i}.self_attn")
        state = {k.replace(*pat): v for k, v in state.items()}
        pat = (f"cross_attn.{i}", f"transformers.{i}.cross_attn")
        state = {k.replace(*pat): v for k, v in state.items()}
    state = {k.replace("matcher.", ""): v for k, v in state.items()}
    missing, unexpected = net.load_state_dict(state, strict=False)
    if unexpected:
        print(f"[WARN] {len(unexpected)} unexpected keys: {list(unexpected)[:3]}…")

    class ExportWrapper(torch.nn.Module):
        def __init__(self, inner, img_w, img_h):
            super().__init__()
            self.inner = inner
            self.register_buffer(
                "image_size",
                torch.tensor([[img_w, img_h]], dtype=torch.float32),
                persistent=False,
            )

        def forward(self, kpts0, desc0, kpts1, desc1):
            data = {
                "image0": {"keypoints": kpts0, "descriptors": desc0,
                            "image_size": self.image_size},
                "image1": {"keypoints": kpts1, "descriptors": desc1,
                            "image_size": self.image_size},
            }
            out = self.inner(data)
            # Return only the dense assignment matrix; the rest is decoded
            # on the C++ side (argmax + dustbin filter).
            return out["log_assignment"]

    wrapper = ExportWrapper(net, args.img_w, args.img_h).eval()

    N = args.num_kpts
    dummy_kpts0 = torch.zeros(1, N, 2, dtype=torch.float32)
    dummy_kpts1 = torch.zeros(1, N, 2, dtype=torch.float32)
    dummy_d0 = torch.randn(1, N, 64, dtype=torch.float32)
    dummy_d1 = torch.randn(1, N, 64, dtype=torch.float32)
    # L2-normalise the dummy descriptors so the trace sees realistic values.
    dummy_d0 = dummy_d0 / (dummy_d0.norm(dim=-1, keepdim=True) + 1e-12)
    dummy_d1 = dummy_d1 / (dummy_d1.norm(dim=-1, keepdim=True) + 1e-12)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        wrapper,
        (dummy_kpts0, dummy_d0, dummy_kpts1, dummy_d1),
        args.output.as_posix(),
        input_names=["keypoints_0", "descriptors_0",
                     "keypoints_1", "descriptors_1"],
        output_names=["log_assignment"],
        opset_version=args.opset,
        # Dynamic axes: N may be smaller than the engine bound at runtime,
        # but kornia's attention has lots of `.shape[-1]` operations that
        # collapse when N is dynamic. Static N is the safe bet for now.
        do_constant_folding=True,
    )
    sha = hashlib.sha256(args.output.read_bytes()).hexdigest()
    print(f"Wrote {args.output} ({args.output.stat().st_size} bytes)")
    print(f"SHA256: {sha}")
    print(f"Static shapes: N={N}, image_size=({args.img_w}, {args.img_h})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
