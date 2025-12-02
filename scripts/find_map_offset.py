#!/usr/bin/env python3
"""
Find the offset between CARLA coordinates and Autoware map coordinates
by analyzing the lanelet2 map and comparing with CARLA spawn points
"""

import xml.etree.ElementTree as ET
import sys
try:
    import carla
except ImportError:
    print("Error: CARLA Python API not found")
    sys.exit(1)

def convert_latlon_to_local(lat, lon, mgrs_grid="31NAA"):
    """
    Convert lat/lon to local coordinates using MGRS projection
    MGRS 31NAA is approximately at lat=0, lon=0
    For local projection, we use simple conversions
    """
    # For MGRS local projection around origin (0, 0):
    # Rough approximation: 1 degree lon ≈ 111 km at equator
    # 1 degree lat ≈ 111 km

    # Convert to meters
    x = lon * 111320.0  # meters per degree longitude at equator
    y = lat * 110540.0  # meters per degree latitude

    return x, y

def main():
    # Parse lanelet2 map
    print("Parsing lanelet2 map...")
    tree = ET.parse('data/carla-autoware-bridge/Town01/lanelet2_map.osm')
    root = tree.getroot()

    # Get all node coordinates
    nodes = []
    for node in root.findall('node'):
        lat = float(node.get('lat'))
        lon = float(node.get('lon'))
        x, y = convert_latlon_to_local(lat, lon)
        nodes.append((x, y))

    if not nodes:
        print("No nodes found in map!")
        return

    # Calculate Autoware map bounds
    auto_x = [n[0] for n in nodes]
    auto_y = [n[1] for n in nodes]

    print(f"\n=== Autoware Map Bounds (from lanelet2) ===")
    print(f"X range: {min(auto_x):.2f} to {max(auto_x):.2f} meters")
    print(f"Y range: {min(auto_y):.2f} to {max(auto_y):.2f} meters")
    print(f"Center: ({(min(auto_x)+max(auto_x))/2:.2f}, {(min(auto_y)+max(auto_y))/2:.2f})")

    # Get CARLA spawn points
    print("\nConnecting to CARLA...")
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    spawn_points = world.get_map().get_spawn_points()

    # CARLA bounds (in centimeters, convert to meters)
    carla_x = [sp.location.x / 100.0 for sp in spawn_points]
    carla_y = [sp.location.y / 100.0 for sp in spawn_points]

    print(f"\n=== CARLA Map Bounds ===")
    print(f"X range: {min(carla_x):.2f} to {max(carla_x):.2f} meters")
    print(f"Y range: {min(carla_y):.2f} to {max(carla_y):.2f} meters")
    print(f"Center: ({(min(carla_x)+max(carla_x))/2:.2f}, {(min(carla_y)+max(carla_y))/2:.2f})")

    # Calculate offset
    # Offset = Autoware_center - CARLA_center
    offset_x = (min(auto_x) + max(auto_x))/2 - (min(carla_x) + max(carla_x))/2
    offset_y = (min(auto_y) + max(auto_y))/2 - (min(carla_y) + max(carla_y))/2

    print(f"\n=== Calculated Offset ===")
    print(f"Offset: ({offset_x:.2f}, {offset_y:.2f}) meters")
    print(f"Offset: ({offset_x*100:.1f}, {offset_y*100:.1f}) centimeters (for CARLA)")

    # Test with Autoware initial pose
    autoware_x, autoware_y = 190.77, -130.10
    carla_x_corrected = (autoware_x - offset_x) * 100  # meters to cm
    carla_y_corrected = -(autoware_y - offset_y) * 100  # meters to cm, Y-flip

    print(f"\n=== Corrected Coordinates ===")
    print(f"Autoware pose: ({autoware_x:.2f}, {autoware_y:.2f}) meters")
    print(f"  - Subtract offset: ({autoware_x - offset_x:.2f}, {autoware_y - offset_y:.2f})")
    print(f"  - Convert to CARLA: ({carla_x_corrected:.1f}, {carla_y_corrected:.1f}) cm")

    # Find closest spawn point
    min_dist = float('inf')
    closest_sp = None
    for sp in spawn_points:
        dist = ((sp.location.x - carla_x_corrected)**2 + (sp.location.y - carla_y_corrected)**2)**0.5
        if dist < min_dist:
            min_dist = dist
            closest_sp = sp

    if closest_sp:
        print(f"\nClosest spawn point: ({closest_sp.location.x:.1f}, {closest_sp.location.y:.1f}) cm")
        print(f"Distance: {min_dist:.1f} cm ({min_dist/100:.1f} m)")

if __name__ == "__main__":
    main()
