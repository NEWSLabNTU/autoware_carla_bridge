#!/usr/bin/env python3
"""
Check valid spawn points in CARLA Town01 and find ones near the Autoware position
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
    client.set_timeout(30.0)

    # Load Town01
    print("Loading Town01...")
    world = client.load_world("Town01")
    print("Town01 loaded")

    # Get spawn points
    spawn_points = world.get_map().get_spawn_points()

    print(f"\n=== Total Spawn Points: {len(spawn_points)} ===\n")

    # Target position from Autoware (in CARLA coords: 19077, 13010, 0)
    target_x, target_y = 19077.0, 13010.0

    # Find closest spawn points
    closest_points = []
    for i, spawn in enumerate(spawn_points):
        loc = spawn.location
        distance = ((loc.x - target_x)**2 + (loc.y - target_y)**2)**0.5
        closest_points.append((distance, i, spawn))

    closest_points.sort()

    print(f"Target position (from Autoware): ({target_x:.1f}, {target_y:.1f})\n")
    print("10 closest spawn points:")
    for dist, idx, spawn in closest_points[:10]:
        loc = spawn.location
        rot = spawn.rotation
        print(f"  [{idx:3d}] ({loc.x:7.1f}, {loc.y:7.1f}, {loc.z:5.1f}) "
              f"yaw={rot.yaw:6.1f}° - distance: {dist:8.1f} cm ({dist/100:.1f}m)")

    print(f"\nFirst spawn point (for reference):")
    loc = spawn_points[0].location
    rot = spawn_points[0].rotation
    print(f"  ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f}) yaw={rot.yaw:.1f}°")

    # Print map bounds
    print(f"\n=== Map Bounds ===")
    all_x = [sp.location.x for sp in spawn_points]
    all_y = [sp.location.y for sp in spawn_points]
    print(f"X range: {min(all_x):.1f} to {max(all_x):.1f}")
    print(f"Y range: {min(all_y):.1f} to {max(all_y):.1f}")

if __name__ == "__main__":
    main()
