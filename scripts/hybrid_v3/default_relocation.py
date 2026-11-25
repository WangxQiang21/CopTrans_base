"""Reproduce public and frozen CMake installs using the same compiled objects."""

from hashlib import sha256
from pathlib import Path
import re
import shlex
import subprocess
from tempfile import TemporaryDirectory


TARGETS = {
    "fwis_control/lib/fwis_control/ranger_tracking_controller_cpp": "ranger_tracking_controller_cpp",
    "fwis_hybrid_joint_ros/lib/fwis_hybrid_joint_ros/hybrid_joint_planner": "hybrid_joint_planner",
    "fwis_hybrid_joint_ros/lib/fwis_hybrid_joint_ros/hybrid_joint_executor": "hybrid_joint_executor",
    "fwis_hybrid_joint_ros/lib/libhybrid_codec.so": "hybrid_codec",
    "fwis_hybrid_joint/lib/libfwis_hybrid_joint.so": "fwis_hybrid_joint",
}
TRACKER_UNIT = "ros2_ws/src/fwis_control/src/tracking_controller_node.cpp"
SCHEMA = "coptrans.v3.default_cmake_relink.v1"


def file_hash(path):
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def configure_source_location(clone, snapshot):
    """Reproduce the tracker log's __FILE__ without changing the codec's build identity."""
    clone, snapshot = Path(clone).resolve(), Path(snapshot).resolve()
    launcher = clone / "ros2_ws/build/.coptrans-e7-compiler"
    launcher.parent.mkdir(exist_ok=True, parents=True)
    source = shlex.quote(str(clone / TRACKER_UNIT))
    flag = shlex.quote(f"-fmacro-prefix-map={clone}={snapshot}")
    launcher.write_text(
        "#!/bin/sh\n"
        "for arg do\n"
        f'  if [ "$arg" = {source} ]; then\n'
        f'    exec "$@" {flag}\n'
        "  fi\n"
        "done\n"
        'exec "$@"\n'
    )
    launcher.chmod(0o755)
    return launcher


def runpath(path):
    dump = subprocess.check_output(["readelf", "-d", str(path)], text=True)
    values = re.findall(r"\((?:RUNPATH|RPATH)\).*?\[(.*?)\]", dump)
    if len(values) > 1:
        raise ValueError(f"Multiple ELF runtime paths at {path}")
    return values[0] if values else ""


def bracket(value):
    # CMake bracket quoting preserves the literal bytes of build paths.
    delimiter = "="
    while "]" + delimiter + "]" in value:
        delimiter += "="
    return f"[{delimiter}[{value}]{delimiter}]"


def relocate_search_path(paths, old_root, new_root):
    mapped = []
    for entry in paths.split(":"):
        if entry and Path(entry).is_relative_to(old_root):
            entry = str(new_root / Path(entry).relative_to(old_root))
        mapped.append(entry)
    return ":".join(mapped)


def replay(link, work, dest, new_root, old_root, new_rpath):
    args = list(link)
    if Path(args[0]).resolve() != Path("/usr/bin/c++").resolve() or args.count("-o") != 1:
        raise ValueError("Expected the CMake C++ linker invocation")
    prefix = "-Wl,-rpath,"
    rpaths = [i for i, arg in enumerate(args) if arg.startswith(prefix)]
    if len(rpaths) != 1:
        raise ValueError(f"Expected exactly one CMake build RUNPATH at {work}")
    index = rpaths[0]
    desired = relocate_search_path(args[index].removeprefix(prefix), old_root, new_root)
    args[index] = prefix + desired
    args[args.index("-o") + 1] = str(dest)
    subprocess.run(args, cwd=work, check=True)
    if runpath(dest) != desired:
        raise ValueError("Relinked runtime path differs from the requested CMake layout")

    script = dest.with_name(dest.name + ".cmake")
    script.write_text(
        f"file(RPATH_CHANGE FILE {bracket(str(dest))} OLD_RPATH {bracket(desired)} "
        f"NEW_RPATH {bracket(new_rpath)})\n"
    )
    subprocess.run(["cmake", "-P", script], check=True, stdout=subprocess.DEVNULL)
    if runpath(dest) != new_rpath:
        raise ValueError("Relinked install path changed")
    return file_hash(dest)


def collect(clone, snapshot):
    clone, snapshot = Path(clone).resolve(), Path(snapshot).resolve()
    targets = {}
    with TemporaryDirectory(prefix="coptrans-e7-relink-") as temp:
        for relative, target in TARGETS.items():
            package = relative.split("/", 1)[0]
            build = clone / "ros2_ws/build" / package
            command = build / "CMakeFiles" / (target + ".dir") / "link.txt"
            args = shlex.split(command.read_text())
            objects = {arg: file_hash(build / arg) for arg in args if arg.endswith((".o", ".a"))}
            if not objects:
                raise ValueError(f"Empty native link input set for {relative}")
            actual_install = runpath(clone / "ros2_ws/install" / relative)
            qualified_install = runpath(snapshot / "ros2_ws/install" / relative)
            if relocate_search_path(actual_install, clone, snapshot) != qualified_install:
                raise ValueError(f"Install RUNPATH differs beyond the two workspaces: {relative}")
            evidence = {
                "link_command_sha256": file_hash(command),
                "object_inputs": objects,
                "actual_install_runpath": actual_install,
            }
            for variant, root in (("actual", clone), ("qualified", snapshot)):
                installed = root / "ros2_ws/install" / relative
                installed_hash = file_hash(installed)
                output = Path(temp) / f"{variant}-{target}"
                reproduced = replay(args, build, output, root, clone, runpath(installed))
                if reproduced != installed_hash:
                    raise ValueError(f"{target} {variant} whole-file mismatch {reproduced} != {installed_hash}")
                evidence[variant + "_sha256"] = installed_hash
                evidence[variant + "_relinked_sha256"] = reproduced
            targets[relative] = evidence

    return {
        "schema": SCHEMA,
        "public_source_root": str(clone),
        "qualified_source_root": str(snapshot),
        "macro_prefix_map": f"{clone}={snapshot}",
        "source_location_unit": TRACKER_UNIT,
        "verified_targets": targets,
    }


def covers(proof, actual, expected):
    targets = proof.get("verified_targets", {})
    if proof.get("schema") != SCHEMA or set(targets) != set(TARGETS):
        return False
    for key, row in targets.items():
        if not row.get("link_command_sha256") or not row.get("object_inputs"):
            return False
        for scope, installed in (("actual", actual), ("qualified", expected)):
            digest = installed.get(key)
            if not digest or digest != row.get(scope + "_sha256") or digest != row.get(scope + "_relinked_sha256"):
                return False
    return True
