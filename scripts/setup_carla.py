#!/usr/bin/env python3
"""Connect to CARLA and configure map and synchronous mode."""

import argparse
import sys
import time


def main():
    parser = argparse.ArgumentParser(description="Configure CARLA map and synchronous mode")
    parser.add_argument(
        "--port", "-p", type=int, default=2000, help="CARLA server port (default: 2000)"
    )
    parser.add_argument(
        "--host", type=str, default="localhost", help="CARLA server host (default: localhost)"
    )
    parser.add_argument(
        "--map", "-m", type=str, default=None, help="Map to load (e.g., Town01, Town10HD)"
    )
    parser.add_argument(
        "--timeout", "-t", type=float, default=10.0, help="Connection timeout in seconds (default: 10.0)"
    )

    sync_group = parser.add_mutually_exclusive_group()
    sync_group.add_argument(
        "--sync", action="store_true", help="Enable synchronous mode"
    )
    sync_group.add_argument(
        "--async", dest="async_mode", action="store_true", help="Enable asynchronous mode"
    )

    parser.add_argument(
        "--fixed-delta", type=float, default=0.05,
        help="Fixed delta seconds for synchronous mode (default: 0.05)"
    )

    args = parser.parse_args()

    # Import CARLA after parsing args for faster --help
    try:
        import carla
    except ImportError:
        print("Error: CARLA Python API not found", file=sys.stderr)
        print("Install with: pip install carla", file=sys.stderr)
        sys.exit(1)

    # Determine sync mode
    if args.sync:
        sync_mode = True
    elif args.async_mode:
        sync_mode = False
    else:
        sync_mode = None  # Don't change

    # Connect to CARLA
    print(f"Connecting to CARLA at {args.host}:{args.port}...")
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(args.timeout)

        # Test connection
        version = client.get_server_version()
        if isinstance(version, str):
            print(f"Connected to CARLA {version}")
        else:
            print(f"Connected to CARLA {version.major}.{version.minor}.{version.patch}")

    except RuntimeError as e:
        print(f"Error: Failed to connect to CARLA: {e}", file=sys.stderr)
        sys.exit(1)

    # Load map if specified
    if args.map:
        current_world = client.get_world()
        current_map = current_world.get_map().name.split('/')[-1]

        if current_map == args.map:
            print(f"Map '{args.map}' is already loaded")
        else:
            print(f"Loading map '{args.map}'...")
            try:
                client.load_world(args.map)
                print(f"Map '{args.map}' loaded successfully")
                time.sleep(1)  # Wait for world to stabilize
            except RuntimeError as e:
                print(f"Error: Failed to load map: {e}", file=sys.stderr)
                sys.exit(1)

    # Configure synchronous mode
    if sync_mode is not None:
        world = client.get_world()
        settings = world.get_settings()

        if sync_mode:
            print(f"Enabling synchronous mode (fixed_delta_seconds={args.fixed_delta})...")
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = args.fixed_delta
        else:
            print("Enabling asynchronous mode...")
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = 0.0

        world.apply_settings(settings)

        # Verify settings
        new_settings = world.get_settings()
        mode_str = "synchronous" if new_settings.synchronous_mode else "asynchronous"
        print(f"Mode set to {mode_str}")
        if new_settings.synchronous_mode:
            print(f"Fixed delta: {new_settings.fixed_delta_seconds}s")

    print("Configuration complete")


if __name__ == "__main__":
    main()
