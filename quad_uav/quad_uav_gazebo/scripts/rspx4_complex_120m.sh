#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

export NANOBOT_WORLD="uav_complex_120m"
exec "$SCRIPT_DIR/rspx4.sh" "$@"
