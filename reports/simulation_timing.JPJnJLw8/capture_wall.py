"""Collect monotonic launch events and sampled per-process CPU times."""

import json
from pathlib import Path
import re
import subprocess
from threading import Event, Thread
from time import perf_counter

import psutil


def main():
    directory = Path(__file__).resolve().parent
    root = directory.parents[1]
    command = [str(root / "scripts/test.sh"), "--task", "multi", "--scenario",
               "factory_indoor_01", "--experiment", "short_turn", "--no-build"]
    started = perf_counter()
    proc = subprocess.Popen(command, cwd=root, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, bufsize=1)
    ps_root = psutil.Process(proc.pid)
    stop = Event()
    labels = {}
    maxima = {}
    native_loaded = set()

    def capture_cpu():
        while not stop.wait(.20):
            try:
                processes = ps_root.children(recursive=True)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
            for child in processes:
                try:
                    stat = child.cpu_times()
                    label = labels.get(child.pid)
                    if label is None:
                        label = next((labels[parent.pid] for parent in child.parents()
                                      if parent.pid in labels), child.name())
                    maxima[child.pid] = (label, stat.user + stat.system)
                    if "formation_uniform_planner" in label and child.pid not in native_loaded:
                        maps = Path(f"/proc/{child.pid}/maps").read_text()
                        if "_formation_collision_native" in maps:
                            native_loaded.add(child.pid)
                except (psutil.Error, OSError):
                    continue

    sampler = Thread(target=capture_cpu)
    sampler.start()
    events = []
    try:
        with (directory / "launch_lines.jsonl").open("w") as lines:
            for line in proc.stdout:
                elapsed = perf_counter() - started
                text = line.rstrip()
                events.append((elapsed, text))
                lines.write(json.dumps({"wall_s": elapsed, "text": text}, ensure_ascii=False) + "\n")
                match = re.search(r"\[([^\]]+)\]: process started with pid \[(\d+)\]", text)
                if match:
                    labels[int(match[2])] = match[1]
                if "Run directory" in text or "Experiment passed" in text or text.startswith("Result "):
                    print(f"{elapsed:.3f}s  {text}", flush=True)
        code = proc.wait()
    finally:
        elapsed = perf_counter() - started
        stop.set()
        sampler.join()
    cpu = {}
    for label, seconds in maxima.values():
        cpu[label] = cpu.get(label, 0.) + seconds
    run = next(text.removeprefix("Run directory ") for _, text in events if text.startswith("Run directory "))
    report = {
        "command": command, "wall_s": elapsed, "exit_code": code, "run": run,
        "planner_loaded_native_extension": bool(native_loaded),
        "cpu_s_by_process_group_sampled_at_0_2s": dict(sorted(cpu.items())),
    }
    (directory / "capture.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)
    raise SystemExit(code)


if __name__ == "__main__":
    main()
