"""Give the simulator and each native tracker separate physical cores when available."""
import os
from pathlib import Path


def partition(cores, vehicles):
    """Partition whole physical cores, keeping at least two available for the simulation."""
    groups = [tuple(sorted(cpu_ids)) for _, cpu_ids in sorted(cores.items())]
    if len(groups) < len(vehicles) + 2:
        return {}, ()
    first = (len(groups) - len(vehicles)) // 2
    chosen = groups[first:first + len(vehicles)]
    remaining = tuple(cpu for group in groups[:first] + groups[first + len(vehicles):] for cpu in group)
    return dict(zip(vehicles, chosen, strict=True)), remaining


def allocate(vehicles):
    if os.environ.get("COPTRANS_CONTROL_AFFINITY", "auto") == "off":
        return {}
    cores = {}
    try:
        for cpu in sorted(os.sched_getaffinity(0)):
            topology = Path(f"/sys/devices/system/cpu/cpu{cpu}/topology")
            key = tuple(int((topology / name).read_text()) for name in ("physical_package_id", "core_id"))
            cores.setdefault(key, []).append(cpu)
        lanes, background = partition(cores, vehicles)
        if background:
            # Tracker processes receive their original allowed CPUs via taskset.
            # All other children created by this launch inherit the remaining set.
            os.sched_setaffinity(0, background)
    except (AttributeError, OSError, ValueError):
        return {}
    return lanes
