#!/bin/bash
# Mission launch with pre-flight verification. Wrapper for start_mission.sh.
# Usage: ./scripts/mission_launch.sh [launch args...]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/start_mission.sh" "$@"
