#!/usr/bin/env bash
# Fail unless CARLA is actually serving RPC on $CARLA_PORT within the timeout.
#
# Run as the unit's ExecStartPost, so "the service is up" means "a client can talk to it"
# rather than "the process exists". A CARLA that is alive but not serving is worse than one
# that is down: anything checking the process or the port passes, and only a real RPC probe
# notices. That state has been observed after an automatic restart -- the process ran at 121%
# CPU with 158 threads for 26 minutes, its port listening with 36 connections queued and never
# accepted, while both bridges timed out against it. See docs/issues/017.
#
# Failing here makes systemd stop the unit, so the honest state is "down", which the health
# check and `just scenario` already refuse to run against.
set -u
PORT="${CARLA_PORT:-2000}"
TIMEOUT="${CARLA_READY_TIMEOUT:-180}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

deadline=$(( $(date +%s) + TIMEOUT ))
while [ "$(date +%s)" -lt "$deadline" ]; do
    if python3 "$HERE/carla_health.py" --port "$PORT" --timeout 10 >/dev/null 2>&1; then
        echo "CARLA on port $PORT is serving RPC"
        exit 0
    fi
    sleep 5
done

echo "CARLA on port $PORT did not serve RPC within ${TIMEOUT}s; failing the unit so the" >&2
echo "state is 'down' rather than 'up but unusable'. See docs/issues/017." >&2
exit 1
