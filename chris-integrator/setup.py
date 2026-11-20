"""Build script: chris-integrator is pure Python (pyproject.toml has the metadata); this file only adds the
optional native core, `chris_integrator.native._chris_native` (one C file, the CPython limited API).

    pip install .                           builds the native core when a C compiler and the Python headers are
                                            there; without them the build of the extension fails with a warning
                                            and the package installs pure Python (it is complete without it)
    CHRIS_NATIVE_BUILD=0 pip install .      pure Python on purpose

`python -m chris_integrator.native` says which implementation an installation runs. docs/NATIVE.md."""
import os

from setuptools import Extension, setup

ext = Extension(
    "chris_integrator.native._chris_native",
    sources=["chris_integrator/native/src/chris_native.c"],
    py_limited_api=True,                    # one binary for every CPython >= 3.11 (abi3)
    extra_compile_args=["-std=c99", "-O3", "-ffp-contract=off", "-fno-fast-math", "-fvisibility=hidden"],
    libraries=["m"],
    optional=True,                          # no compiler, no headers, Python 3.10: a warning, not an error
)
build = os.environ.get("CHRIS_NATIVE_BUILD", "1") not in ("0", "no", "off", "false")
setup(ext_modules=[ext] if build else [],
      options={"bdist_wheel": {"py_limited_api": "cp311"}} if build else {})
