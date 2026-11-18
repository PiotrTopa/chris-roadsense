"""Restricted reader for openpilot ONNX metadata (downloaded files are untrusted).

`output_slices` in openpilot ONNX files is a base64 pickle of {name: slice}. We only allow the
builtins needed to rebuild that (dict/list/tuple/slice/str/int); anything else raises.
"""
import base64
import io
import pickle

_ALLOWED = {("builtins", n) for n in ("dict", "list", "slice", "str", "int", "tuple")}


class _Restricted(pickle.Unpickler):
    def find_class(self, module, name):
        if (module, name) in _ALLOWED:
            import builtins
            return getattr(builtins, name)
        raise pickle.UnpicklingError(f"blocked global {module}.{name}")


def safe_loads(b: bytes):
    return _Restricted(io.BytesIO(b)).load()


def onnx_meta(model):
    """Return metadata_props as dict; decode output_slices with the restricted unpickler."""
    md = {p.key: p.value for p in model.metadata_props}
    out = dict(md)
    if "output_slices" in md:
        out["output_slices"] = safe_loads(base64.b64decode(md["output_slices"]))
    return out
