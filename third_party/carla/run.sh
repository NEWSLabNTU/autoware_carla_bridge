#!/usr/bin/env bash
# Run CARLA simulator.
#
#   run.sh [map:=Town01] [quality:=epic|low] [render:=offscreen|window] [extra_args:="..."]
#          [port:=2000]
#
# What CARLA runs is arguments (scripts/carla_args.bash says what each one does); where it
# runs is the environment:
#
#   CARLA_DIR      CARLA installation      (default ~/Downloads/CARLA_0.9.16)
#   CARLA_PORT     RPC port, when port:= is not given (default 2000)
#
# The old CARLA_MAP / CARLA_QUALITY / CARLA_RENDER / CARLA_EXTRA_ARGS are refused.
#
# Why map:= matters: the package boots Town10HD_Opt, the heaviest map it ships,
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

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../scripts" && pwd)/carla_args.bash"
carla_reject_env
carla_parse_args "$@"

CARLA_DIR="${CARLA_DIR:-$HOME/Downloads/CARLA_0.9.16}"
PORT="$CARLA_ARG_PORT"
MAP="$CARLA_ARG_MAP"
# Epic by default, Low only when asked. With -quality-level=Low, CARLA 0.9.16 segfaults on
# load_world to another town (and reload_world hangs): UE 4.26's
# FLandscapeRenderSystem::FGetSectionLODBiasesTask reads UTexture2D::GetNumResidentMips on a
# texture freed by the level swap (carla-simulator/carla#4940); reload_world on the same town
# crashes the same way. A run that stays on the town CARLA booted on is unaffected; Low saves
# ~275 MiB of VRAM there (carla-scenario-bridge docs/design/carla-server-tuning.md, 2026-10-19).
QUALITY="$CARLA_ARG_QUALITY"
RENDER="$CARLA_ARG_RENDER"

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
esac

# Anything else to hand the engine, space separated. Deliberately unquoted so it splits.
#
# The reason this exists: Unreal lowers its own core limit at startup and logs "Disabling core
# dumps.", so a segfault leaves nothing to read even when the unit's soft limit is infinity --
# which is why docs/issues/017 has three crashes and no core. The engine takes switches for
# that, and this is where to pass one without editing the launcher.
if [ -n "$CARLA_ARG_EXTRA" ]; then
    # shellcheck disable=SC2206
    args+=($CARLA_ARG_EXTRA)
fi

# Pin Vulkan to the NVIDIA driver on hosts that also have an iGPU or a software renderer:
# left to choose, a restarted CARLA came up on the AMD iGPU / lavapipe (18 MiB on the NVIDIA
# card), took 4-8 min to serve RPC, and froze in apply_settings(synchronous)
# (carla-scenario-bridge roadmap 016, gap 13). Set VK_ICD_FILENAMES yourself to override.
nvidia_icd=/usr/share/vulkan/icd.d/nvidia_icd.json
if [ -z "${VK_ICD_FILENAMES:-}" ] && [ -f "$nvidia_icd" ]; then
    export VK_ICD_FILENAMES="$nvidia_icd"
fi

exec ./CarlaUE4.sh "${args[@]}"
