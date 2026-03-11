#!/usr/bin/env python3
"""
CARLA Demo Scenario

This script manages the CARLA simulation for the Autoware-CARLA bridge demo.
It connects to CARLA, loads the Town01 map, and configures asynchronous mode
for maximum simulation performance.

In asynchronous mode, CARLA runs as fast as possible without waiting for
client ticks, which provides higher frame rates for sensor data.
"""

import argparse
import sys
import time
from typing import Optional

try:
    import carla
except ImportError:
    print("Error: CARLA Python API not found")
    print("Make sure CARLA is installed and PYTHONPATH includes the CARLA egg file")
    sys.exit(1)


class DemoScenario:
    """Manages CARLA simulation for the Autoware-CARLA bridge demo"""

    def __init__(self, host: str = "127.0.0.1", port: int = 2000, map_name: str = "Town01"):
        """
        Initialize the demo scenario

        Args:
            host: CARLA server host
            port: CARLA server port
            map_name: Name of the CARLA map to load
        """
        self.host = host
        self.port = port
        self.map_name = map_name
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None

    def connect(self) -> bool:
        """
        Connect to CARLA server

        Returns:
            True if connection successful, False otherwise
        """
        try:
            print(f"Connecting to CARLA at {self.host}:{self.port}...")
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(10.0)

            # Test connection
            version = self.client.get_server_version()
            print(f"Connected to CARLA {version}")
            return True

        except RuntimeError as e:
            print(f"Failed to connect to CARLA: {e}")
            return False

    def setup(self) -> bool:
        """
        Set up the simulation environment

        Returns:
            True if setup successful, False otherwise
        """
        try:
            # Check current map
            current_world = self.client.get_world()
            current_map = current_world.get_map().name

            # Load the requested map if different
            if self.map_name not in current_map:
                print(f"Loading map: {self.map_name}")
                print("  (This may take 20-30 seconds...)")

                # Increase timeout for map loading (can take 20-30 seconds)
                self.client.set_timeout(60.0)
                self.world = self.client.load_world(self.map_name)

                # Restore normal timeout
                self.client.set_timeout(10.0)
                print(f"✓ Map loaded: {self.map_name}")
            else:
                print(f"Map {self.map_name} already loaded")
                self.world = current_world

            # Set to asynchronous mode for maximum performance
            # CARLA runs as fast as possible, bridge reads data as available
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None  # Variable time step
            self.world.apply_settings(settings)

            print("✓ Configured CARLA in asynchronous mode")
            print("  CARLA will run at maximum speed")

            return True

        except RuntimeError as e:
            print(f"Failed to setup simulation: {e}")
            return False

    def run(self) -> None:
        """Run the scenario monitoring loop"""
        print("\n=== Demo Scenario Running (Async Mode) ===")
        print("Press Ctrl+C to stop\n")

        try:
            while True:
                # In async mode, just monitor the world state
                actors = self.world.get_actors()
                vehicles = actors.filter('vehicle.*')
                sensors = actors.filter('sensor.*')
                walkers = actors.filter('walker.*')

                # Get vehicle position if available
                vehicle_pos_str = ""
                if len(vehicles) > 0:
                    vehicle = vehicles[0]
                    transform = vehicle.get_transform()
                    loc = transform.location
                    vehicle_pos_str = f" | Pos: ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})"

                print(f"[Monitor] Actors: {len(actors):3d} | "
                      f"Vehicles: {len(vehicles):2d} | "
                      f"Sensors: {len(sensors):2d} | "
                      f"Walkers: {len(walkers):2d}{vehicle_pos_str}")

                # Status update every second
                time.sleep(1.0)

        except KeyboardInterrupt:
            print("\n\n=== Stopping Demo Scenario ===")


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="CARLA Demo Scenario for Autoware Bridge")
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="CARLA server host (default: 127.0.0.1)"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=2000,
        help="CARLA server port (default: 2000)"
    )
    parser.add_argument(
        "--map",
        default="Town01",
        help="CARLA map to load (default: Town01)"
    )

    args = parser.parse_args()

    # Create and run scenario
    scenario = DemoScenario(host=args.host, port=args.port, map_name=args.map)

    # Retry connecting to CARLA until it's ready
    while True:
        if scenario.connect():
            break
        print("Retrying in 5 seconds...")
        try:
            time.sleep(5)
        except KeyboardInterrupt:
            print("\nAborted.")
            sys.exit(0)

    if not scenario.setup():
        sys.exit(1)

    scenario.run()


if __name__ == "__main__":
    main()
