"""Call the C++ lap integrator (build/liblapsim.so, built by `make build`) through ctypes."""
import ctypes
from dataclasses import astuple

import numpy as np

from model import DS, ROOT, Car, Track

LIB_PATH = ROOT / "build" / "liblapsim.so"


class CarParams(ctypes.Structure):
    # Must match struct CarParams in cpp/lapsim.hpp, field for field.
    _fields_ = [(name, ctypes.c_double) for name in ("mass", "cda", "cla", "mu", "crr",
                                                     "p_ice", "p_k_max", "p_harvest_max")]
    _fields_ += [("taper", ctypes.c_int)]
    _fields_ += [(name, ctypes.c_double) for name in ("e_window", "harvest_per_lap", "eta")]


_lib = None


def _load():
    global _lib
    if _lib is None:
        if not LIB_PATH.exists():
            raise FileNotFoundError(f"{LIB_PATH} missing, run `make build` first")
        _lib = ctypes.CDLL(str(LIB_PATH))
        dbl = np.ctypeslib.ndpointer(np.float64, flags="C_CONTIGUOUS")
        _lib.simulate_lap.restype = ctypes.c_double
        _lib.simulate_lap.argtypes = [ctypes.POINTER(CarParams), ctypes.c_int, ctypes.c_double, dbl, dbl,
                                      ctypes.c_double, ctypes.c_double, dbl, dbl, dbl, dbl,
                                      np.ctypeslib.ndpointer(np.uint8, flags="C_CONTIGUOUS")]
    return _lib


def simulate_cpp(car: Car, track: Track, cmd, e0, v0=None):
    """Same contract as model.simulate (traces + lap time), computed in C++."""
    lib = _load()
    n = len(track.s)
    params = CarParams(*astuple(car))  # dataclass field order == struct field order
    v, e, p_k, dt = (np.empty(n) for _ in range(4))
    clipped = np.empty(n, np.uint8)
    v0 = track.v_real[0] if v0 is None else v0
    lap_time = lib.simulate_lap(ctypes.byref(params), n, DS, np.ascontiguousarray(track.v_corner, float),
                                np.ascontiguousarray(cmd, float), e0, v0, v, e, p_k, dt, clipped)
    return dict(v=v, e=e, p_k=p_k, dt=dt, clipped=clipped.astype(bool), lap_time=lap_time)
