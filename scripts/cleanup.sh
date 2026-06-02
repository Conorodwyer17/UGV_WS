#!/bin/bash
# Clean build artifacts and optional temporary files to free disk space.
# Usage: ./scripts/cleanup.sh [--execute] [--force]
#   Default: dry run (show what would be removed).
#   --execute: actually remove.
#   --force: skip disk space check when executing.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_WS="${UGV_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
EXECUTE=false
FORCE=false

for arg in "$@"; do
  case "$arg" in
    --execute) EXECUTE=true ;;
    --force)   FORCE=true ;;
    -h|--help)
      echo "Usage: $0 [--execute] [--force]"
      echo "  Dry run: show what would be removed."
      echo "  --execute: actually remove build/, install/, log/."
      echo "  --force: skip free-space check."
      exit 0
      ;;
  esac
done

echo "=== Workspace cleanup ($UGV_WS) ==="

if $EXECUTE && ! $FORCE; then
  FREE_KB=$(df -k "$UGV_WS" 2>/dev/null | awk 'NR==2 {print $4}')
  FREE_GB=$((FREE_KB / 1024 / 1024))
  if [[ -n "$FREE_GB" ]] && [[ "$FREE_GB" -lt 5 ]]; then
    echo "WARNING: Less than 5 GB free. Use --force to override."
    exit 1
  fi
fi

for d in build install log; do
  if [[ -d "$UGV_WS/$d" ]]; then
    SIZE=$(du -sh "$UGV_WS/$d" 2>/dev/null | cut -f1)
    if $EXECUTE; then
      echo "Removing $UGV_WS/$d ($SIZE)..."
      rm -rf "$UGV_WS/$d"
    else
      echo "Would remove: $UGV_WS/$d ($SIZE)"
    fi
  fi
done

if $EXECUTE; then
  echo "Cleanup complete."
else
  echo "Dry run. Use --execute to remove."
fi
