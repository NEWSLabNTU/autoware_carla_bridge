#!/usr/bin/env bash
# Bring the ego/scenario stacks down and WAIT until they are actually gone.
#
# Two traps this exists to avoid, both of which bit repeatedly:
#
# 1. play_launch respawns its nodes. Killing the nodes first just makes it restart them, and
#    the count goes up rather than down. Kill the launchers, wait for them to be gone, and
#    only then reap whatever they left behind.
# 2. Do not filter on the command line. `pgrep -x play_launch` already matches the executable
#    and cannot match the shell doing the matching, whereas a `*claude*` guard skips the very
#    processes to be killed here -- the launcher's own log path lives under a directory with
#    that name, so every launcher was silently spared.
set -u

deadline=$(( $(date +%s) + 120 ))
while :; do
    for p in $(pgrep -x play_launch); do kill -9 "$p" 2>/dev/null; done
    sleep 2
    [ "$(pgrep -cx play_launch)" -eq 0 ] && break
    [ "$(date +%s)" -gt "$deadline" ] && { echo "launchers will not die" >&2; exit 1; }
done

# Nothing is left to respawn them now.
for _ in $(seq 1 20); do
    pkill -9 -x component_node 2>/dev/null
    pkill -9 -x component_container_mt 2>/dev/null
    pkill -9 -x acb_bridge 2>/dev/null
    pkill -9 -x openscenario_interpreter_node 2>/dev/null
    sleep 2
    n=$(pgrep -c component_node); a=$(pgrep -c acb_bridge)
    if [ "$n" -eq 0 ] && [ "$a" -eq 0 ]; then
        echo "stacks down (launchers 0, nodes 0, bridges 0)"; exit 0
    fi
done
echo "leftovers: nodes=$(pgrep -c component_node) bridges=$(pgrep -c acb_bridge)" >&2; exit 1
