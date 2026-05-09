#!/usr/bin/env python
"""Patch AirSLAM ONNX models for TensorRT 10 compatibility.

TRT 10's ONNX importer is stricter about integer types: Concat / Gather / Where
nodes that mix Int64 and Int32 inputs no longer work. This script casts every
Int32 input feeding a Concat (and any other type-mismatched ops) to Int64,
producing a `_trt10.onnx` file beside each source ONNX. Also runs
`onnx-graphsurgeon` cleanup + shape inference to keep the graph minimal.

Usage:
    python scripts/patch_onnx_for_trt10.py output/superpoint_v1_sim_int32.onnx \
        output/plnet_s0.onnx output/plnet_s1.onnx ...

The script edits in-place when given `--in-place`, otherwise writes a sibling
named `<stem>_trt10.onnx`.
"""
import argparse
import sys
from pathlib import Path

import onnx
import onnx_graphsurgeon as gs
from onnx import TensorProto


INT64 = TensorProto.INT64


def cast_to_int64(graph: gs.Graph, tensor: gs.Tensor, dst_name: str) -> gs.Tensor:
    """Insert a Cast node converting `tensor` -> int64. Returns the new tensor."""
    cast_out = gs.Variable(name=dst_name, dtype=onnx.helper.tensor_dtype_to_np_dtype(INT64))
    cast_node = gs.Node(
        op="Cast",
        name=f"{dst_name}_cast",
        attrs={"to": INT64},
        inputs=[tensor],
        outputs=[cast_out],
    )
    graph.nodes.append(cast_node)
    return cast_out


# Ops that TRT 10 will reject if their inputs mix Int32 + Int64.
DTYPE_MIX_OPS = {
    "Concat",
    "Add", "Sub", "Mul", "Div", "Mod", "Pow",
    "And", "Or", "Xor",
    "Equal", "Greater", "Less", "GreaterOrEqual", "LessOrEqual",
    "Where",
    "Min", "Max",
}


def patch_dtype_mix(graph: gs.Graph) -> int:
    """Find ops that mix int32 + int64 inputs and cast int32 -> int64.
    Returns the number of nodes patched."""
    patched = 0
    for node in list(graph.nodes):
        if node.op not in DTYPE_MIX_OPS:
            continue

        # Collect dtypes of inputs that have a known dtype.
        dtypes = []
        for inp in node.inputs:
            dt = getattr(inp, "dtype", None)
            if dt is not None:
                dtypes.append(str(dt).lower())

        unique = set(dtypes)
        has_i32 = any("int32" in dt for dt in unique)
        has_i64 = any("int64" in dt for dt in unique)
        if not (has_i32 and has_i64):
            continue

        new_inputs = []
        for i, inp in enumerate(node.inputs):
            dt = str(getattr(inp, "dtype", "")).lower()
            if "int32" in dt:
                base = node.name if node.name else node.op.lower()
                cast_out = cast_to_int64(graph, inp, f"{base}_in{i}_to_i64")
                new_inputs.append(cast_out)
            else:
                new_inputs.append(inp)
        node.inputs = new_inputs
        patched += 1
    return patched


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("models", nargs="+", help="Paths to ONNX models")
    ap.add_argument("--in-place", action="store_true",
                    help="Overwrite the input file instead of writing _trt10.onnx")
    args = ap.parse_args()

    for path_str in args.models:
        src = Path(path_str)
        if not src.exists():
            print(f"[SKIP] {src} does not exist")
            continue

        print(f"\n[LOAD] {src}")
        model = onnx.load(str(src))

        # Run shape inference so dtype hints are populated where the original
        # exporter did not stamp them.
        try:
            model = onnx.shape_inference.infer_shapes(model)
        except Exception as e:  # noqa: BLE001
            print(f"  shape inference failed: {e}; continuing without it")

        graph = gs.import_onnx(model)
        n_patched = patch_dtype_mix(graph)
        print(f"  patched {n_patched} ops (Int32 -> Int64 casts inserted)")

        graph.cleanup().toposort()
        out_model = gs.export_onnx(graph)

        # Validate the produced model.
        try:
            onnx.checker.check_model(out_model)
        except Exception as e:  # noqa: BLE001
            print(f"  WARN: onnx.checker complained: {e}")

        dst = src if args.in_place else src.with_name(src.stem + "_trt10" + src.suffix)
        onnx.save(out_model, str(dst))
        print(f"[SAVE] {dst}")


if __name__ == "__main__":
    sys.exit(main() or 0)
