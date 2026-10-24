#!/usr/bin/env bash
set -euo pipefail

PACKAGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(cd -- "$PACKAGE_DIR/../../.." && pwd)"
EXPECTED_CASADI_REVISION="0e672301aa9046162d311f69421f6375805f4fca"
BUILD_ROOT="${FWIS_BONMINSEED_BUILD_DIR:-$WORKSPACE_ROOT/build/bonminseed/$EXPECTED_CASADI_REVISION}"
SOURCE_DIR="$BUILD_ROOT/casadi-src"
PATCH_FILE="$PACKAGE_DIR/patches/casadi-0e672301-bonminseed.patch"
OUTPUT_DIR="${FWIS_BONMINSEED_OUT_DIR:-$WORKSPACE_ROOT/catkin_ws/devel/lib/fwis_v3_optimizer/casadi_plugins}"
PYTHON="${PYTHON:-python3}"

CASADI_ROOT="$("$PYTHON" -c 'import pathlib, casadi; print(pathlib.Path(casadi.__file__).resolve().parent)')"
CASADI_VERSION="$("$PYTHON" -c 'import casadi; print(casadi.__version__)')"
CASADI_REVISION="$("$PYTHON" - "$CASADI_ROOT/include/casadi/config.h" <<'PY'
import re
import sys
from pathlib import Path
text = Path(sys.argv[1]).read_text(encoding="utf-8")
match = re.search(r'^#define CASADI_GIT_REVISION "([^"]+)"', text, re.M)
if not match:
    raise SystemExit("CasADi config.h has no CASADI_GIT_REVISION")
print(match.group(1))
PY
)"

if [[ "$CASADI_VERSION" != "3.7.2" || "$CASADI_REVISION" != "$EXPECTED_CASADI_REVISION" ]]; then
  echo "Expected CasADi 3.7.2 ($EXPECTED_CASADI_REVISION), found $CASADI_VERSION ($CASADI_REVISION)." >&2
  exit 2
fi
if [[ ! -f "$PATCH_FILE" ]]; then
  echo "Missing CasADi plugin patch: $PATCH_FILE" >&2
  exit 2
fi

mkdir -p "$BUILD_ROOT"
if [[ ! -d "$SOURCE_DIR/.git" ]]; then
  git clone --no-checkout --filter=blob:none https://github.com/casadi/casadi.git "$SOURCE_DIR"
fi
git -C "$SOURCE_DIR" fetch --depth=1 origin "$EXPECTED_CASADI_REVISION"
git -C "$SOURCE_DIR" checkout --force --detach "$EXPECTED_CASADI_REVISION"
git -C "$SOURCE_DIR" clean -fdx
patch -d "$SOURCE_DIR" -p1 < "$PATCH_FILE"

EXPORT_HEADER="$SOURCE_DIR/casadi/interfaces/bonmin/casadi_nlpsol_bonmin_export.h"
cat > "$EXPORT_HEADER" <<'HEADER'
#ifndef CASADI_NLPSOL_BONMIN_EXPORT_H
#define CASADI_NLPSOL_BONMIN_EXPORT_H
#if defined(_WIN32)
#define CASADI_NLPSOL_BONMIN_EXPORT __declspec(dllexport)
#else
#define CASADI_NLPSOL_BONMIN_EXPORT __attribute__((visibility("default")))
#endif
#endif
HEADER

mapfile -t CASADI_FLAGS < <("$PYTHON" - "$CASADI_ROOT/include/casadi/config.h" <<'PY'
import re
import shlex
import sys
from pathlib import Path
text = Path(sys.argv[1]).read_text(encoding="utf-8")
match = re.search(r'^#define CASADI_COMPILER_FLAGS "([^"]*)"', text, re.M)
if not match:
    raise SystemExit("CasADi config.h has no CASADI_COMPILER_FLAGS")
for flag in shlex.split(match.group(1)):
    print(flag)
PY
)

mkdir -p "$OUTPUT_DIR"
OUTPUT="$OUTPUT_DIR/libcasadi_nlpsol_bonminseed.so"
"${CXX:-c++}" -shared -std=c++17 -O2 -fPIC -fvisibility=hidden \
  "${CASADI_FLAGS[@]}" -D_GLIBCXX_USE_CXX11_ABI=0 \
  -I"$SOURCE_DIR" \
  -I"$SOURCE_DIR/casadi/interfaces/bonmin" \
  -I"$CASADI_ROOT/include" \
  -I"$CASADI_ROOT/include/coin" \
  -I"$CASADI_ROOT/include/coin-or" \
  "$SOURCE_DIR/casadi/interfaces/bonmin/bonmin_interface.cpp" \
  "$SOURCE_DIR/casadi/interfaces/bonmin/bonmin_nlp.cpp" \
  "$SOURCE_DIR/casadi/interfaces/bonmin/bonmin_interface_meta.cpp" \
  -L"$CASADI_ROOT" -Wl,-rpath,"$CASADI_ROOT" -Wl,--no-as-needed \
  -lcasadi -lbonmin -lCbcSolver -lCbc -lCgl -lOsiClp -lClpSolver -lClp \
  -lcoinmumps -lOsi -lCoinUtils -lz -lipopt \
  -o "$OUTPUT"

echo "Built $OUTPUT"
echo "For an opt-in run, set CASADIPATH=$OUTPUT_DIR before starting ROS."
