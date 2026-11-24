"""Durable report helpers for optimizer runs."""
import json
import os
import tempfile


def atomic_write_json(path, payload):
    """Write one complete JSON report without leaving partial final files."""
    target = os.path.abspath(os.fspath(path))
    directory = os.path.dirname(target)
    os.makedirs(directory, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=directory,
                prefix="." + os.path.basename(target) + ".",
                suffix=".tmp", delete=False) as stream:
            temporary = stream.name
            json.dump(payload, stream, allow_nan=False,
                      separators=(",", ":"), sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
        temporary = None
        return target
    finally:
        if temporary is not None:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass
