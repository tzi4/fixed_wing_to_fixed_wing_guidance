#!/usr/bin/env bash
# =====================================================================
# start_simulation.sh - MAIN LAUNCHER (Erenimbus + video guidance)
# =====================================================================
# This is the ONLY entry point to the package. Fighter aircraft models/emir_aircraft_temp
# ("Erenimbus" - 1.5 kg, gazebo-plane derivative of the Emir known to work);
# Bumblebee's actual hardware spec has been ported to the camera
# (1920x1080, hfov 0.42 rad, near 0.1, 30 Hz, /webcam/image_raw).
#
# HISTORY: this file as control environment for bumblebee model A/B
# was born. Bumblebee model.sdf due to unresolved flight behavior
# The old Bumblebee model was abandoned; this Erenimbus environment became the main environment.
#
# USE:
#   ./start_simulation.sh              # GUI (gzclient + QGC + bbox window)
#   ./start_simulation.sh --headless #no-GUI,bbox --no-display
#   ./start_simulation.sh --stop # closes everything started by this package
#   BUMBLEBEE_VIDEO=1 ./start_simulation.sh # save camera video with bbox
#   BUMBLEBEE_VIDEO=1 BUMBLEBEE_VIDEO_PATH=videos/2026-08-11 ./start_simulation.sh
#
# Note: kernel initiator (bumblebee_guidance.sh) and BUMBLEBEE_* env names
# keeps their old names - on purpose rather than changing them one by one in code
# was left; Functionally, they install the Erenimbus environment.
# =====================================================================
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Explicitly select the world, parameters, and hunter model for this environment.
export BUMBLEBEE_WORLD="$SCRIPT_DIR/worlds/temp_multi_uav.world"
export BUMBLEBEE_PARAM="$SCRIPT_DIR/params/temp_emir.parm"
export BUMBLEBEE_HUNTER_MODEL="$SCRIPT_DIR/models/emir_aircraft_temp/model.sdf"
# The vehicle name to be written to the right of the "Code: ..." line in the video overlay.
export BUMBLEBEE_AIRCRAFT="${BUMBLEBEE_AIRCRAFT:-Erenimbus}"

exec "$SCRIPT_DIR/bumblebee_guidance.sh" "$@"
