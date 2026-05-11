"""Export XFeat (PyTorch) -> ONNX for the AirSLAM-XFeat pipeline.

Upstream `verlab/accelerated_features` does NOT ship an ONNX export script,
so we wrap the FCN forward (heatmap + descriptor map + reliability map) in
a tiny `nn.Module` and export that. NMS, top-K, and bilinear descriptor
sampling stay on the C++ side — see `src/xfeat.cpp` (Phase 3).

Tensor shapes (matches AirSLAM's TRT 10 5-step infer pattern; dynamic H, W):

    input  "image"   (1, 1, H, W)   float32 in [0, 1]  (grayscale)
    output "feats"   (1, 64, H/8, W/8)   float32 dense descriptor map
    output "keypts"  (1, 65, H/8, W/8)   float32 raw logits (softmax over 65)
    output "rel"     (1,  1, H/8, W/8)   float32 reliability heatmap

Run via `scripts/get_xfeat_onnx.sh` (sets up the temp venv + pinned repo).
Direct invocation:

    python scripts/export_xfeat_onnx.py \\
        --xfeat-repo /tmp/accelerated_features \\
        --output output/xfeat.onnx
"""

from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path


def _add_xfeat_to_path(repo_path: Path) -> None:
    if not (repo_path / "modules" / "xfeat.py").exists():
        sys.exit(f"XFeat repo missing modules/xfeat.py: {repo_path}")
    sys.path.insert(0, str(repo_path))


def _build_export_module(xfeat_repo: Path):
    """Wrap the XFeat torch model so torch.onnx.export sees a clean forward."""
    _add_xfeat_to_path(xfeat_repo)
    import torch
    from modules.xfeat import XFeat as _XFeatTop

    class XFeatNetExport(torch.nn.Module):
        def __init__(self, xfeat_top):
            super().__init__()
            # Underlying FCN with .forward returning (feats, keypoints, heatmap).
            self.net = xfeat_top.net
            # XFeat trained on 3-channel input; EuRoC / D455 IR are grayscale.
            # Repeat single channel x3 inside the model so the consumer (AirSLAM
            # SuperPoint feeder) stays 1-channel and we do not need to touch
            # upstream weights.
            self.register_buffer(
                "rgb_repeat", torch.ones(1, 3, 1, 1), persistent=False)

        def forward(self, gray_img):  # gray_img: (1, 1, H, W) in [0, 1]
            x = gray_img * self.rgb_repeat   # (1, 3, H, W)
            feats, keypts, rel = self.net(x)
            return feats, keypts, rel

    inner = _XFeatTop(weights=None)  # weights loaded explicitly below
    return inner, XFeatNetExport


def main() -> int:
    p = argparse.ArgumentParser(description="Export XFeat FCN to ONNX.")
    p.add_argument("--xfeat-repo", required=True, type=Path,
                   help="Path to a clone of verlab/accelerated_features.")
    p.add_argument("--weights", type=Path, default=None,
                   help="Path to xfeat.pt; defaults to <repo>/weights/xfeat.pt")
    p.add_argument("--output", required=True, type=Path,
                   help="Where to write the .onnx file.")
    p.add_argument("--height", type=int, default=480,
                   help="Dummy-input height for tracing. AirSLAM uses dynamic H.")
    p.add_argument("--width", type=int, default=752,
                   help="Dummy-input width for tracing. AirSLAM uses dynamic W.")
    p.add_argument("--opset", type=int, default=17)
    p.add_argument("--dynamic-shapes", action="store_true",
                   help="Mark H, W as dynamic axes. XFeat's InstanceNorm + Unfold "
                        "path is NOT compatible with dynamic-axes tracing under "
                        "torch.onnx.export (it raises `Unsupported: ONNX export of "
                        "operator Unfold, input size not accessible`). Off by "
                        "default: the engine is fixed at H,W and AirSLAM resizes "
                        "the image to that size before calling infer().")
    args = p.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)

    import torch  # imported here so --help works without torch installed

    inner_top, ExportWrapper = _build_export_module(args.xfeat_repo)
    weights_path = args.weights or (args.xfeat_repo / "weights" / "xfeat.pt")
    if not weights_path.exists():
        sys.exit(f"Weights not found: {weights_path}")
    state = torch.load(weights_path, map_location="cpu", weights_only=True)
    inner_top.net.load_state_dict(state)
    wrapper = ExportWrapper(inner_top).eval()

    dummy = torch.zeros(1, 1, args.height, args.width, dtype=torch.float32)

    dynamic_axes = None
    if args.dynamic_shapes:
        # Opt-in; will fail unless upstream replaces InstanceNorm with a path
        # that does not trace through Unfold under dynamic shapes.
        dynamic_axes = {
            "image":  {2: "H",  3: "W"},
            "feats":  {2: "H8", 3: "W8"},
            "keypts": {2: "H8", 3: "W8"},
            "rel":    {2: "H8", 3: "W8"},
        }

    torch.onnx.export(
        wrapper,
        dummy,
        args.output.as_posix(),
        input_names=["image"],
        output_names=["feats", "keypts", "rel"],
        opset_version=args.opset,
        dynamic_axes=dynamic_axes,
        do_constant_folding=True,
    )

    sha = hashlib.sha256(args.output.read_bytes()).hexdigest()
    print(f"Wrote {args.output} ({args.output.stat().st_size} bytes)")
    print(f"SHA256: {sha}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
