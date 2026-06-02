#!/bin/bash
# Install ugv_mission systemd service for auto-start on boot.
# Usage: ./scripts/install_service.sh [username] [workspace_path]
#   username: system user to run as (default: $USER)
#   workspace_path: path to ugv_ws (default: ~/ugv_ws)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_WS="${2:-$(cd "$SCRIPT_DIR/.." && pwd)}"
SERVICE_USER="${1:-$USER}"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Run with sudo: sudo $0 $SERVICE_USER $UGV_WS"
  exit 1
fi

SERVICE_FILE="/etc/systemd/system/ugv_mission.service"
TEMPLATE="$SCRIPT_DIR/ugv_mission.service.example"

if [[ ! -f "$TEMPLATE" ]]; then
  echo "Template not found: $TEMPLATE"
  exit 1
fi

sed -e "s|USER|$SERVICE_USER|g" -e "s|/home/USER|/home/$SERVICE_USER|g" "$TEMPLATE" > "$SERVICE_FILE"
echo "Installed $SERVICE_FILE (User=$SERVICE_USER, Workspace=$UGV_WS)"
systemctl daemon-reload
echo "To enable on boot: sudo systemctl enable ugv_mission"
echo "To start now:     sudo systemctl start ugv_mission"
