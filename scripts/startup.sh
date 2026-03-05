#!/bin/bash
# Launch full bringup (no pre-flight verification). For verified launch, use start_mission.sh.
# Usage: ./scripts/startup.sh [launch args...]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/start_mission.sh" --no-verify "$@"
