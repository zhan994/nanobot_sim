#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

export WORLD_FILE="$PACKAGE_DIR/worlds/uav_complex_120m.world"
exec "$SCRIPT_DIR/rspx4.sh" "$@"
