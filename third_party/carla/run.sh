#!/usr/bin/env bash
# Run CARLA simulator.
#
# Knobs, all environment variables so the systemd unit, a shell and the benchmark
# harness launch the same way:
#
#   CARLA_DIR      CARLA installation      (default ~/Downloads/CARLA_0.9.16)
#   CARLA_PORT     RPC port                (default 2000)
#   CARLA_MAP      town to boot into       (default Town01; empty leaves the install's own)
#   CARLA_QUALITY  -quality-level          (default Low)
#   CARLA_RENDER   offscreen | window      (default offscreen)
#
# Why CARLA_MAP matters: the package boots Town10HD_Opt, the heaviest map it ships,
# and the bridge then loads the scenario's town on top. Booting on the town the run
# actually uses is worth ~4 GB of peak RSS and ~3.9 GB of VRAM (measured with
# scripts/carla_bench.py, 0.9.16 on an RTX 5090):
#
#   default (Town10HD_Opt -> Town01)   peak RSS 7368 MB   VRAM 5591 MB   tick 286 Hz
#   Town01 from boot                   peak RSS 3292 MB   VRAM 1740 MB   tick 318 Hz
#
# The startup map cannot be set on the command line in 0.9.16 -- neither
# `CarlaUE4.sh Town01` nor `CarlaUE4.sh /Game/Carla/Maps/Town01` has any effect, the
# packaged build always loads GameDefaultMap. So the map is applied by patching
# CarlaUE4/Config/DefaultEngine.ini, idempotently, right here. The original file is
# kept beside it as DefaultEngine.ini.orig.
#
# Not available: -nullrhi. It would drop the rendering backend entirely, but CARLA
# 0.9.16 segfaults on startup with it (SIGSEGV before the RPC server comes up).
#
# Requires a display (DISPLAY must be set before running).
set -e

CARLA_DIR="${CARLA_DIR:-$HOME/Downloads/CARLA_0.9.16}"
PORT="${CARLA_PORT:-2000}"
MAP="${CARLA_MAP-Town01}"
QUALITY="${CARLA_QUALITY:-Low}"
RENDER="${CARLA_RENDER:-offscreen}"

export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

cd "$CARLA_DIR"

if [ -n "$MAP" ]; then
    ini="CarlaUE4/Config/DefaultEngine.ini"
    want="GameDefaultMap=/Game/Carla/Maps/$MAP.$MAP"
    if ! grep -qxF "$want" "$ini"; then
        [ -f "$ini.orig" ] || cp "$ini" "$ini.orig"
        sed -i "s|^GameDefaultMap=.*|$want|" "$ini"
        echo "Startup map set to $MAP in $ini"
    fi
fi

args=("-quality-level=$QUALITY" "-carla-rpc-port=$PORT" -nosound)
case "$RENDER" in
    offscreen) args+=(-RenderOffScreen) ;;
    window)    ;;
    *) echo "Unknown CARLA_RENDER='$RENDER' (want offscreen or window)" >&2; exit 1 ;;
esac

exec ./CarlaUE4.sh "${args[@]}"
