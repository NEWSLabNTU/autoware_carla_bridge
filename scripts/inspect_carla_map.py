#!/usr/bin/env python3
"""
Comprehensive CARLA map inspection script
Shows map bounds, spawn points, and waypoint sampling
"""

import sys
try:
    import carla
except ImportError:
    print("Error: CARLA Python API not found")
    sys.exit(1)

def main():
    # Connect to CARLA
    print("Connecting to CARLA at 127.0.0.1:2000...")
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(30.0)

    world = client.get_world()
    carla_map = world.get_map()

    print(f"\n=== CARLA Map: {carla_map.name} ===\n")

    # Get spawn points
    spawn_points = carla_map.get_spawn_points()
    print(f"Total spawn points: {len(spawn_points)}\n")

    if not spawn_points:
        print("No spawn points found!")
        return

    # Calculate map bounds from spawn points
    all_x = [sp.location.x for sp in spawn_points]
    all_y = [sp.location.y for sp in spawn_points]
    all_z = [sp.location.z for sp in spawn_points]

    print("=== Map Bounds (from spawn points) ===")
    print(f"X range: {min(all_x):.2f} to {max(all_x):.2f} cm ({min(all_x)/100:.2f} to {max(all_x)/100:.2f} m)")
    print(f"Y range: {min(all_y):.2f} to {max(all_y):.2f} cm ({min(all_y)/100:.2f} to {max(all_y)/100:.2f} m)")
    print(f"Z range: {min(all_z):.2f} to {max(all_z):.2f} cm ({min(all_z)/100:.2f} to {max(all_z)/100:.2f} m)")
    print(f"Map size: {(max(all_x)-min(all_x))/100:.2f}m × {(max(all_y)-min(all_y))/100:.2f}m\n")

    # Get map topology (road network)
    print("=== Map Topology ===")
    topology = carla_map.get_topology()
    print(f"Total road segments: {len(topology)}\n")

    # Sample waypoints to get better map bounds
    print("=== Sampling Waypoints ===")
    waypoints = carla_map.generate_waypoints(2.0)  # Every 2 meters
    print(f"Total waypoints sampled: {len(waypoints)}")

    if waypoints:
        wp_x = [wp.transform.location.x for wp in waypoints]
        wp_y = [wp.transform.location.y for wp in waypoints]
        wp_z = [wp.transform.location.z for wp in waypoints]

        print(f"\nActual Map Bounds (from waypoints):")
        print(f"X range: {min(wp_x):.2f} to {max(wp_x):.2f} cm ({min(wp_x)/100:.2f} to {max(wp_x)/100:.2f} m)")
        print(f"Y range: {min(wp_y):.2f} to {max(wp_y):.2f} cm ({min(wp_y)/100:.2f} to {max(wp_y)/100:.2f} m)")
        print(f"Z range: {min(wp_z):.2f} to {max(wp_z):.2f} cm ({min(wp_z)/100:.2f} to {max(wp_z)/100:.2f} m)")
        print(f"Map size: {(max(wp_x)-min(wp_x))/100:.2f}m × {(max(wp_y)-min(wp_y))/100:.2f}m\n")

    # Show first 10 spawn points
    print("=== First 10 Spawn Points ===")
    for i, sp in enumerate(spawn_points[:10]):
        loc = sp.location
        rot = sp.rotation
        print(f"[{i:3d}] x={loc.x:8.2f} cm ({loc.x/100:7.2f}m), "
              f"y={loc.y:8.2f} cm ({loc.y/100:7.2f}m), "
              f"z={loc.z:6.2f} cm, yaw={rot.yaw:6.1f}°")

    # Check if Autoware position is within map bounds
    print("\n=== Autoware Position Check ===")
    autoware_x_cm = 19077.0  # From bridge logs
    autoware_y_cm = 13010.0

    print(f"Autoware requests: ({autoware_x_cm:.1f}, {autoware_y_cm:.1f}) cm")
    print(f"                  ({autoware_x_cm/100:.2f}, {autoware_y_cm/100:.2f}) m")

    if waypoints:
        in_x = min(wp_x) <= autoware_x_cm <= max(wp_x)
        in_y = min(wp_y) <= autoware_y_cm <= max(wp_y)
        print(f"\nWithin X bounds: {in_x}")
        print(f"Within Y bounds: {in_y}")
        print(f"Within map: {in_x and in_y}")

        if not (in_x and in_y):
            print(f"\n⚠ Position is OUTSIDE map bounds!")
            print(f"  Distance from X bounds: {min(abs(autoware_x_cm - min(wp_x)), abs(autoware_x_cm - max(wp_x)))/100:.2f}m")
            print(f"  Distance from Y bounds: {min(abs(autoware_y_cm - min(wp_y)), abs(autoware_y_cm - max(wp_y)))/100:.2f}m")

    # Find nearest spawn point to Autoware position
    print("\n=== Nearest Spawn Point to Autoware Position ===")
    min_dist = float('inf')
    closest_sp_idx = None

    for i, sp in enumerate(spawn_points):
        dist = ((sp.location.x - autoware_x_cm)**2 +
                (sp.location.y - autoware_y_cm)**2)**0.5
        if dist < min_dist:
            min_dist = dist
            closest_sp_idx = i

    if closest_sp_idx is not None:
        sp = spawn_points[closest_sp_idx]
        print(f"Closest spawn point: [{closest_sp_idx}]")
        print(f"  Position: ({sp.location.x:.2f}, {sp.location.y:.2f}) cm")
        print(f"           ({sp.location.x/100:.2f}, {sp.location.y/100:.2f}) m")
        print(f"  Distance: {min_dist:.2f} cm ({min_dist/100:.2f} m)")
        print(f"  Yaw: {sp.rotation.yaw:.1f}°")

if __name__ == "__main__":
    main()
