#!/usr/bin/env bash
set -e

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec "$root/scripts/test.sh" --task multi --interactive --experiment full_route "$@"
