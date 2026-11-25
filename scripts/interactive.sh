#!/usr/bin/env bash
set -e

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec "$root/scripts/test.sh" --task multi --interactive --architecture-profile optimized \
    --multi-native-frontier --multi-convex-corridor --multi-reachable-clock \
    --tracker-implementation cpp --experiment full_route "$@"
