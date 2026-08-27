#!/usr/bin/env bash
# Run a batch of scenarios on one ego stack and score each run.
#
# Repeated scenario runs are how this project finds out whether a change helped, and doing
# it by hand is how it got several answers wrong. This encodes what those mistakes taught:
#
#   * Score lateral tracking, not just pass/fail. A change that regressed tracking five-fold
#     still passed its scenario; pass/fail called it a success (acb docs/issues/016).
#   * Separate "no data" from "failed". A run whose junit or trace never landed says nothing
#     about the change, and counting it as a failure invents an effect.
#   * Check CARLA is alive before each stack and stop if it is not. The server segfaults
#     under long uptime (issue 017); a crash that halts a batch is recoverable, one that
#     silently removes data from one arm of a comparison is not.
#   * Never `pkill -f` a pattern that your own command line contains -- it kills the shell
#     doing the killing. Use scripts/safe_kill.sh.
#   * Run 1 on a fresh stack is not comparable to later runs. Report them apart.
#
# Usage:
#   run_scenarios.sh [-n RUNS] [-s STACKS] [-o OUT.tsv] [SCENARIO]
#
# Environment passed through to `just ego-av`, e.g. REPORT_MEASURED_STEERING,
# STEERING_MULTIPLIER, EGO_ROS_DOMAIN_ID, is inherited -- which is how the arms of an A/B
# are set. Alternate arms ACROSS stacks in one sitting rather than comparing against numbers
# from a previous session; interleaving is what turned a misleading result into a real one.
set -u

RUNS=5
STACKS=1
OUT="scenario_results.tsv"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${ACB_CSB_REPO:-$HOME/repos/carla-scenario-bridge}"
SCENARIO="$REPO/scenarios/town01_ego_drive.xosc"

while getopts "n:s:o:h" opt; do
    case "$opt" in
        n) RUNS="$OPTARG" ;;
        s) STACKS="$OPTARG" ;;
        o) OUT="$OPTARG" ;;
        h) sed -n '2,26p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) exit 2 ;;
    esac
done
shift $((OPTIND - 1))
[ $# -ge 1 ] && SCENARIO="$1"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
printf "stack\trun\tverdict\tn\tlat_sd\tlat_p95\tlat_max\txt_sd\txt_max\tdiag\tnote\n" > "$OUT"

carla_alive() { pgrep -f CarlaUE4-Linux >/dev/null 2>&1; }

for stack in $(seq 1 "$STACKS"); do
    if ! carla_alive; then
        echo "[run] CARLA is not running; stopping rather than leaving this batch short"
        echo "[run] of data in a way that reads as a result. See docs/issues/017."
        exit 1
    fi
    echo "[run] stack $stack/$STACKS: restarting the ego stack"
    bash "$HERE/safe_kill.sh" openscenario_interpreter >/dev/null 2>&1
    bash "$HERE/safe_kill.sh" play_launch >/dev/null 2>&1
    bash "$HERE/safe_kill.sh" "/opt/autoware" >/dev/null 2>&1
    sleep 8
    python3 - <<'PY'
import carla
c = carla.Client("localhost", 2000); c.set_timeout(20.0)
w = c.get_world()
s = w.get_settings(); s.synchronous_mode = False; s.fixed_delta_seconds = None
w.apply_settings(s)
for a in list(w.get_actors().filter("vehicle.*")) + list(w.get_actors().filter("sensor.*")):
    try: a.destroy()
    except Exception: pass
print("[run] world reset")
PY
    ( cd "$REPO" && setsid just ego-av > "$TMP/ego.log" 2>&1 & )
    up=0
    for _ in $(seq 1 100); do
        grep -q "Startup complete" "$TMP/ego.log" 2>/dev/null && { up=1; break; }
        sleep 10
    done
    [ "$up" = 1 ] || { echo "[run] stack $stack never came up; skipping"; continue; }
    sleep 55

    for run in $(seq 1 "$RUNS"); do
        bash "$HERE/safe_kill.sh" openscenario_interpreter >/dev/null 2>&1
        sleep 6
        rm -rf /tmp/scenario_test_runner
        trace="$TMP/trace_${stack}_${run}.txt"
        ( cd "$REPO" && timeout 420 bash -c "
            source /opt/autoware/1.5.0/setup.bash >/dev/null 2>&1
            source '$REPO/install/setup.bash'
            export CYCLONEDDS_URI='file://$REPO/config/cyclonedds-localhost.xml'
            export ROS_DOMAIN_ID='${EGO_ROS_DOMAIN_ID:-0}'
            python3 -u '$HERE/trace_run.py' 400" > "$trace" 2>&1 & )
        sleep 4
        ( cd "$REPO" && setsid just scenario "$SCENARIO" > "$TMP/scen.log" 2>&1 & )

        verdict=NO_RESULT
        for _ in $(seq 1 75); do
            if [ -f /tmp/scenario_test_runner/result.junit.xml ]; then
                sleep 2
                grep -q 'failures="0" errors="0"' /tmp/scenario_test_runner/result.junit.xml \
                    && verdict=PASS || verdict=FAIL
                break
            fi
            sleep 5
        done
        for _ in $(seq 1 60); do pgrep -f trace_run.py >/dev/null || break; sleep 5; done

        score=$(python3 "$HERE/score_tracking.py" "$trace" 2>/dev/null \
                || printf "0\tnan\tnan\tnan\tnan\tnan\t-")
        # A run with no trace data cannot speak to the change either way; mark it so nobody
        # counts it as a failure later.
        note=""
        [ "$(printf '%s' "$score" | cut -f1)" = "0" ] && note="no-data"
        [ "$verdict" = NO_RESULT ] && note="${note:+$note,}no-junit"
        printf "%s\t%s\t%s\t%s\t%s\n" "$stack" "$run" "$verdict" "$score" "$note" >> "$OUT"
        echo "[run] stack $stack run $run: $verdict ${note:+($note)} $score"
        sleep 10
    done
done

echo
echo "[run] wrote $OUT"
python3 "$HERE/summarise_runs.py" "$OUT" 2>/dev/null || true
