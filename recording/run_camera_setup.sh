#!/usr/bin/env bash
# Camera setup window: live picture of every connected camera + a name drop-down; Save overwrites
# config/real_cameras.json (old file -> real_cameras.json.bak). See recording/camera_setup_gui.py.
#   run_camera_setup.sh [--config PATH] [--all-interfaces]
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -z "${LEROBOT_PY:-}" ]; then
  LEROBOT_PY="$HERE/../.venv_record/bin/python"
  [ -x "$LEROBOT_PY" ] || LEROBOT_PY="$HOME/Robotic_infra/lerobot/.venv/bin/python"
fi
exec "$LEROBOT_PY" "$HERE/camera_setup_gui.py" "$@"
