#!/usr/bin/env python3
"""Load a town into CARLA unless it is already loaded, before anything attaches to it.

A load starts a new CARLA episode, and acb_bridge ends its stack on an episode change
(scenario-world-bridge roadmap 051): time starts again, and Autoware cannot follow it back.
So a launch that starts acb_bridge and also loads a town (the demo scenario does when its
map differs) must load first. `just sim` runs this before its launch; the demo scenario then
finds the town loaded and keeps the episode.

    scripts/carla_load_town.py --port 2000 --town Town01

The load keeps the world's settings (reset_settings=False): a synchronous server stays
synchronous. Exit 0 when the town is loaded, 1 when CARLA does not answer or the load fails.
"""

import argparse
import sys


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--town", required=True)
    ap.add_argument("--timeout", type=float, default=120.0,
                    help="seconds to allow the load (a town loads in 10-30 s)")
    args = ap.parse_args()
    try:
        import carla

        client = carla.Client(args.host, args.port)
        client.set_timeout(args.timeout)
        world = client.get_world()
        current = world.get_map().name
        if args.town in current:
            print(f"{args.town} is loaded (episode {world.id})")
            return 0
        print(f"loading {args.town} (was {current.split('/')[-1]})", flush=True)
        world = client.load_world(args.town, reset_settings=False)
        print(f"{args.town} loaded (episode {world.id})")
        return 0
    except Exception as e:  # noqa: BLE001 -- one line, then the exit code
        print(f"cannot load {args.town} on {args.host}:{args.port}: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
