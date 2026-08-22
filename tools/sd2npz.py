#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
#
# sd2npz — PyTorch state-dict checkpoint -> state-dict .npz, WITHOUT torch.
#
# Authors: Simon Davidson & Claude
# Created: 2026-08-22
# Last modified: 2026-08-22
#
# Why this exists
# ---------------
# Clement's nblocks=20/40 Monarch checkpoints ship only config.json + g_best (a
# torch.save archive); there is no ONNX export, which is what
# nsnet2_monarch_model.dims_from_onnx / precision.load_monarch_tensors read.
# Those modules already fall back to a sibling "g_best.npz" holding the same
# block factors under the ONNX names, and nsnet2_deploy_monarch.py's comment
# points at "tools/sd2npz.py" for the conversion — but the tool was never
# committed, so the nblocks=20/40 deploy was not reproducible from a clean tree.
# This is that tool.
#
# There is no torch on the build machines, so the archive is read directly: a
# torch.save file is a zip holding a pickle (data.pkl) plus one raw buffer per
# storage under data/<key>.  Unpickling with a custom find_class /
# persistent_load resolves the tensors into numpy arrays with no torch import.
# Only the plain dense-storage rebuild path is supported (_rebuild_tensor_v2),
# which is all these checkpoints use.
#
# Naming: the state dict uses the ONNX names minus the "base." prefix
# (fc_in.weight vs base.fc_in.weight), so the prefix is re-added on the way out.
#
# Usage:
#   python3 sd2npz.py <checkpoint> [out.npz]
#   python3 sd2npz.py ~/Downloads/.../cp_monarch_40/g_best
#     -> writes ~/Downloads/.../cp_monarch_40/g_best.npz
import io
import os
import pickle
import sys
import zipfile

import numpy as np

# torch storage class -> numpy dtype.  BFloat16 is carried as raw uint16 and
# widened below; every other type maps straight through.
_DTYPE = {
    "FloatStorage": np.float32,   "DoubleStorage": np.float64,
    "HalfStorage":  np.float16,   "BFloat16Storage": np.uint16,
    "LongStorage":  np.int64,     "IntStorage":   np.int32,
    "ShortStorage": np.int16,     "CharStorage":  np.int8,
    "ByteStorage":  np.uint8,     "BoolStorage":  np.bool_,
}

# State-dict key -> ONNX initialiser name expected downstream.
_PREFIX = "base."


def load_state_dict(path):
    """Read a torch.save archive into {name: np.ndarray} without importing torch."""
    if not zipfile.is_zipfile(path):
        raise SystemExit(f"{path}: not a torch zip archive "
                         "(legacy pickle checkpoints are not supported)")
    z = zipfile.ZipFile(path)
    root = z.namelist()[0].split("/")[0]

    class _Storage(object):
        __slots__ = ("key", "dtype")

        def __init__(self, key, dtype):
            self.key, self.dtype = key, dtype

    def _rebuild_tensor_v2(storage, offset, size, stride, *_rest):
        raw = z.read(f"{root}/data/{storage.key}")
        flat = np.frombuffer(raw, dtype=storage.dtype)
        if not size:                                    # 0-d tensor
            return flat[offset:offset + 1].reshape(())
        view = np.lib.stride_tricks.as_strided(
            flat[offset:], shape=tuple(size),
            strides=tuple(s * flat.dtype.itemsize for s in stride))
        return view.copy()                              # detach from the buffer

    class _Unpickler(pickle.Unpickler):
        def find_class(self, module, name):
            if name == "_rebuild_tensor_v2":
                return _rebuild_tensor_v2
            if name in _DTYPE:
                return type(name, (), {"dtype": _DTYPE[name]})
            if module == "collections" and name == "OrderedDict":
                import collections
                return collections.OrderedDict
            return super().find_class(module, name)

        def persistent_load(self, pid):
            _tag, storage_cls, key, _location, _numel = pid
            return _Storage(key, getattr(storage_cls, "dtype", np.float32))

    obj = _Unpickler(io.BytesIO(z.read(f"{root}/data.pkl"))).load()

    # Unwrap the training harness's outer dict ({'generator': state_dict}, or a
    # 'state_dict' key) until the tensors themselves are in hand.
    while isinstance(obj, dict) and not any(
            isinstance(k, str) and k.endswith(".weight") for k in obj):
        inner = [k for k in obj if k in ("generator", "state_dict", "model", "g")]
        if not inner:
            raise SystemExit(f"{path}: cannot find the tensors; top keys "
                             f"{sorted(str(k) for k in obj)}")
        obj = obj[inner[0]]
    return obj


def main():
    if len(sys.argv) < 2:
        print(__doc__ or "usage: sd2npz.py <checkpoint> [out.npz]")
        return 2
    src = os.path.expanduser(sys.argv[1])
    dst = (os.path.expanduser(sys.argv[2]) if len(sys.argv) > 2
           else os.path.join(os.path.dirname(src) or ".",
                             os.path.basename(src) + ".npz"))

    sd = load_state_dict(src)
    out = {}
    for k, v in sd.items():
        if not hasattr(v, "shape"):
            continue
        a = np.asarray(v)
        if a.dtype == np.uint16:                        # bfloat16 -> float32
            a = (a.astype(np.uint32) << 16).view(np.float32)
        out[k if k.startswith(_PREFIX) else _PREFIX + k] = a

    if not out:
        raise SystemExit(f"{src}: no tensors found")
    np.savez(dst, **out)
    print(f"{src} -> {dst}")
    for k in sorted(out):
        print(f"    {k:36s} {out[k].shape}  {out[k].dtype}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
