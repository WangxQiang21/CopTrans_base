"""Reuse existing C++ local A connectors, without running the old frontend."""
import ctypes
import hashlib
import math
import os
from pathlib import Path
import subprocess

import numpy as np

from .kernel_adapter import PROJECT_ROOT, Action, replay, wrap


class LocalConnections:
    schema = "existing_cpp_local_csc_rs_v1"

    def __init__(self):
        source = PROJECT_ROOT/"rl/native/local_connections.cpp"
        include = PROJECT_ROOT/"catkin_ws/src/fwis_common/include"
        header = include/"fwis_common/reeds_shepp.hpp"
        identity = hashlib.sha256(source.read_bytes()+header.read_bytes()).hexdigest()
        directory = Path("/tmp/coptrans-rl-native")/identity
        directory.mkdir(parents=True, exist_ok=True)
        library = directory/"connections.so"
        if not library.exists():
            temporary = directory/("connections-%d.so" % os.getpid())
            subprocess.run(["g++", "-std=c++17", "-O3", "-shared", "-fPIC",
                            "-I", str(include), str(source), "-o", str(temporary)],
                           check=True, capture_output=True, text=True)
            temporary.replace(library)
        self.identity = identity
        self._library = ctypes.CDLL(str(library))
        self._call = self._library.coptrans_connections
        pointer = ctypes.POINTER(ctypes.c_double)
        self._call.argtypes = [pointer, pointer, ctypes.c_int]
        self._call.restype = ctypes.c_int

    def ackermann(self, start, target, radius):
        values = np.asarray([start.x, start.y, start.th, target.x, target.y,
                             target.th, radius], dtype=np.float64)
        if not np.all(np.isfinite(values)) or radius <= 0:
            raise ValueError("invalid_local_connector_input")
        output = np.zeros((9, 16), dtype=np.float64)
        pointer = ctypes.POINTER(ctypes.c_double)
        count = self._call(values.ctypes.data_as(pointer),
                           output.ctypes.data_as(pointer), len(output))
        seen = set()
        for row in output[:count]:
            actions = tuple(Action(0, float(row[1+2*i]), float(row[2+2*i]))
                            for i in range(int(row[0])))
            end = replay(start, actions)[-1]
            # Validate the actual geometry returned by the native formulas.
            if (math.hypot(end.x-target.x, end.y-target.y) > 1e-6 or
                    abs(wrap(end.th-target.th)) > 1e-6):
                continue
            key = tuple((round(a.amount, 8), round(a.control, 8)) for a in actions)
            if key not in seen:
                seen.add(key)
                yield actions
