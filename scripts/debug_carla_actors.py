#!/usr/bin/env python3
"""
Debug script to check all actors in CARLA and their positions
"""

import sys
try:
    import carla
except ImportError:
    print("Error: CARLA Python API not found")
    sys.exit(1)

def main():
    # Connect to CARLA
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)

    world = client.get_world()

    # Get all actors
    actors = world.get_actors()
    print(f"\n=== Total Actors: {len(actors)} ===\n")

    # Group by type
    vehicles = actors.filter('vehicle.*')
    sensors = actors.filter('sensor.*')
    walkers = actors.filter('walker.*')

    print(f"Vehicles: {len(vehicles)}")
    for vehicle in vehicles:
        transform = vehicle.get_transform()
        loc = transform.location
        print(f"  ID {vehicle.id}: {vehicle.type_id} at ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")

    print(f"\nSensors: {len(sensors)}")
    for sensor in sensors:
        transform = sensor.get_transform()
        loc = transform.location
        print(f"  ID {sensor.id}: {sensor.type_id} at ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")

    print(f"\nWalkers: {len(walkers)}")

    # Check for actors that might be vehicles but not matching filter
    print(f"\n=== All Actors by Type ===")
    actor_types = {}
    for actor in actors:
        type_id = actor.type_id
        if type_id not in actor_types:
            actor_types[type_id] = []
        actor_types[type_id].append(actor.id)

    for type_id, ids in sorted(actor_types.items()):
        if len(ids) <= 3:
            print(f"{type_id}: {ids}")
        else:
            print(f"{type_id}: {len(ids)} actors (IDs {ids[0]}-{ids[-1]})")

if __name__ == "__main__":
    main()
