#!/bin/bash
# Create a tarball backup of the workspace, excluding build artifacts.
# Usage: bash scripts/backup_workspace.sh [output_dir]
# Output: ugv_ws_backup_YYYYMMDD_HHMMSS.tar.gz

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_WS="${UGV_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
OUT_DIR="${1:-$HOME}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
ARCHIVE="$OUT_DIR/ugv_ws_backup_${TIMESTAMP}.tar.gz"

echo "=== Workspace Backup ==="
echo "Source: $UGV_WS"
echo "Output: $ARCHIVE"
echo ""

# Exclude build artifacts, logs, and large generated files
cd "$(dirname "$UGV_WS")"
tar czf "$ARCHIVE" \
  --exclude='build' \
  --exclude='install' \
  --exclude='log' \
  --exclude='*.pyc' \
  --exclude='__pycache__' \
  --exclude='.ros' \
  --exclude='*.egg-info' \
  "$(basename "$UGV_WS")"

SIZE=$(du -h "$ARCHIVE" | cut -f1)
echo ""
echo "Done. Archive: $ARCHIVE ($SIZE)"
echo "To restore: tar xzf $ARCHIVE -C /destination/parent/"
