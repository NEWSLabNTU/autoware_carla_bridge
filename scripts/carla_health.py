#!/usr/bin/env python3
"""Decide whether CARLA is fit to run a scenario against, and say why if not.

Exit 0 if healthy, 1 if not. Prints one line of reasoning either way, because a check that
only sets an exit code teaches nobody anything the next time it fails.

The subtlety this exists for: **a fresh client cannot count actors on a synchronous server.**
The actor list is derived from the episode snapshot, and a client that has never observed a
tick has no snapshot, so it reports zero actors and frame zero on a perfectly healthy server:

    sync, no ticker:
       old client (has seen ticks): 174 actors, snapshot frame 4345539
       new client (never ticked)  : 0 actors, snapshot frame 0

A census that ignores this reads "empty world" and looks exactly like a wedged server. That
mistake was made, written up as a new failure mode, and cost two unnecessary restarts --
see docs/issues/017.

So the mode is read first, and the census is only trusted where it means something. Ticking
to force a snapshot is never an option here: the ticker belongs to whichever process put the
server in synchronous mode, and a second one corrupts its run.
"""

import argparse
import sys

# A loaded map carries static props, traffic lights and the spectator the server creates
# itself, so an asynchronous server reporting nothing at all is broken rather than idle.
MIN_ACTORS_WHEN_ASYNC = 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="seconds to allow each RPC; the direct ones answer in milliseconds when healthy",
    )
    args = ap.parse_args()

    try:
        import carla
    except ImportError as e:
        print(f"[carla-health] cannot import the carla module: {e}")
        return 1

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(args.timeout)
        version = client.get_server_version()
    except Exception as e:
        print(
            f"[carla-health] no answer from {args.host}:{args.port} "
            f"({type(e).__name__}). Start it with `just carla-start`."
        )
        return 1

    try:
        world = client.get_world()
        settings = world.get_settings()
        town = world.get_map().name
    except Exception as e:
        print(
            f"[carla-health] server {version} answered its version but not its world "
            f"({type(e).__name__}); it is wedged. Stop it (SIGKILL if SIGTERM will not "
            f"take) and `just carla-start`."
        )
        return 1

    if settings.synchronous_mode:
        # Somebody owns the tick. That is normal mid-run, and normal between runs too:
        # the bridge leaves synchronous mode set until its next Initialize.
        print(
            f"[carla-health] ok: server {version} on {town}, synchronous "
            f"(dt={settings.fixed_delta_seconds}). Another process owns the tick, so the "
            f"actor count is not checked -- a client that has seen no tick would read zero."
        )
        return 0

    try:
        actors = len(world.get_actors())
    except Exception as e:
        print(f"[carla-health] asynchronous server would not list its actors ({type(e).__name__})")
        return 1

    if actors < MIN_ACTORS_WHEN_ASYNC:
        print(
            f"[carla-health] server {version} on {town} is asynchronous and reports "
            f"{actors} actors. A loaded map has static props and a spectator, so this "
            f"world is broken. Restart it: `just carla-stop && just carla-start`."
        )
        return 1

    print(
        f"[carla-health] ok: server {version} on {town}, asynchronous, {actors} actors"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
