#!/usr/bin/bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1
export ATHENA_HOST='wss://athena.konik.ai'
export API_HOST='https://api.konik.ai'
export MAPBOX_TOKEN="${MAPBOX_TOKEN:-}"


if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="10.1"
fi

export STAGING_ROOT="/data/safe_staging"

# Keep the loopback window API separate from the driving manager so a parked
# diagnostic handover can stop/restore that manager without killing the API.
# Runtime registration works with AGNOS's read-only /etc; this saved launcher
# registers it again after a device reboot. Failure must not block driving boot.
TOAD_WINDOW_API_UNIT="/data/openpilot/selfdrive/frogpilot/window_control/toad-window-api.service"
if [ -f /TICI ] && [ -f "$TOAD_WINDOW_API_UNIT" ]; then
  sudo -n systemctl --no-block enable --runtime --now "$TOAD_WINDOW_API_UNIT" >/dev/null 2>&1 || true
fi
unset TOAD_WINDOW_API_UNIT
