# CARLA launch arguments, shared by third_party/carla/run.sh and scripts/carla_start.sh.
# Bash only; sourcing defines functions and nothing else.
#
# What CARLA runs (town, quality, render mode, extra engine switches) is a project setting:
# name:=value arguments, as for a launch file. Where it runs (CARLA_DIR, the port, DISPLAY,
# VK_ICD_FILENAMES) is the host's, from the environment.
#
#   map:=Town01         town to boot into; map:= (empty) leaves the installation's own
#   quality:=epic       epic | low (-quality-level). Low saves memory but cannot change
#                       towns: load_world to another town and reload_world segfault
#                       (carla-simulator/carla#4940). Use it only for single-town runs.
#   render:=offscreen   offscreen | window
#   extra_args:="..."   more engine switches, space separated
#   port:=2000          RPC port (default $CARLA_PORT, then 2000)

# Environment variables that used to choose these, each with its replacement. Refused while
# set (even empty), so an old shell or .env cannot change a run silently.
CARLA_RETIRED_ENV=(
    "CARLA_MAP|map:=<town>"
    "CARLA_QUALITY|quality:=epic|low"
    "CARLA_RENDER|render:=offscreen|window"
    "CARLA_EXTRA_ARGS|extra_args:=\"<switches>\""
)

carla_reject_env() {
    local e var bad=0
    for e in "${CARLA_RETIRED_ENV[@]}"; do
        var="${e%%|*}"
        if [ -n "${!var+x}" ]; then
            echo "[carla] $var is no longer read; pass ${e#*|} instead" >&2
            bad=1
        fi
    done
    if [ "$bad" = 1 ]; then
        echo "[carla] CARLA's launch settings are arguments now (name:=value); unset the" \
             "variable(s) above, in this shell and in .env/.envrc, and pass the argument." >&2
        exit 2
    fi
}

# carla_parse_args ARG...: sets CARLA_ARG_MAP, CARLA_ARG_QUALITY (Low|Epic, as CARLA spells
# it), CARLA_ARG_RENDER, CARLA_ARG_EXTRA, CARLA_ARG_PORT. Exits 2 on anything else.
carla_parse_args() {
    CARLA_ARG_MAP=Town01
    CARLA_ARG_QUALITY=epic
    CARLA_ARG_RENDER=offscreen
    CARLA_ARG_EXTRA=""
    CARLA_ARG_PORT="${CARLA_PORT:-2000}"
    local a
    for a in "$@"; do
        case "$a" in
            map:=*)        CARLA_ARG_MAP="${a#*:=}" ;;
            quality:=*)    CARLA_ARG_QUALITY="${a#*:=}" ;;
            render:=*)     CARLA_ARG_RENDER="${a#*:=}" ;;
            extra_args:=*) CARLA_ARG_EXTRA="${a#*:=}" ;;
            port:=*)       CARLA_ARG_PORT="${a#*:=}" ;;
            *)
                echo "[carla] unknown argument '$a' (map:=, quality:=epic|low," \
                     "render:=offscreen|window, extra_args:=, port:=)" >&2
                exit 2 ;;
        esac
    done
    case "${CARLA_ARG_QUALITY,,}" in
        epic) CARLA_ARG_QUALITY=Epic ;;
        low)  CARLA_ARG_QUALITY=Low ;;
        *) echo "[carla] quality:=$CARLA_ARG_QUALITY: expected epic or low" >&2; exit 2 ;;
    esac
    case "$CARLA_ARG_RENDER" in
        offscreen|window) ;;
        *) echo "[carla] render:=$CARLA_ARG_RENDER: expected offscreen or window" >&2; exit 2 ;;
    esac
    case "$CARLA_ARG_PORT" in
        ''|*[!0-9]*) echo "[carla] port:=$CARLA_ARG_PORT: expected a number" >&2; exit 2 ;;
    esac
}
