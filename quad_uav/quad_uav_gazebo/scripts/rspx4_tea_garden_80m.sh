#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

export WORLD_NAME="${WORLD_NAME:-mountain_tea_garden_80m.world}"
export SPAWN_X="${SPAWN_X:--34.0}"
export SPAWN_Y="${SPAWN_Y:--33.0}"
export SPAWN_Z="${SPAWN_Z:-4.8}"

exec "$SCRIPT_DIR/rspx4.sh" "$@"
