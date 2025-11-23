#!/usr/bin/env python3
"""
Debug script to find and dump vehicle state in CARLA.
Connects to CARLA and displays all spawned vehicles with their current state.
"""

import sys
import argparse

try:
    import carla
except ImportError:
    print("Error: CARLA Python API not found", file=sys.stderr)
    print("Please install CARLA Python package or add to PYTHONPATH", file=sys.stderr)
    sys.exit(1)


def dump_vehicle_state(vehicle):
    """Dump detailed state of a vehicle."""
    print(f"\n{'='*60}")
    print(f"Vehicle ID: {vehicle.id}")
    print(f"Type: {vehicle.type_id}")
    print(f"{'='*60}")

    # Transform
    transform = vehicle.get_transform()
    location = transform.location
    rotation = transform.rotation

    print(f"\nTransform:")
    print(f"  Location: x={location.x:.2f}, y={location.y:.2f}, z={location.z:.2f}")
    print(f"  Rotation: pitch={rotation.pitch:.2f}, yaw={rotation.yaw:.2f}, roll={rotation.roll:.2f}")

    # Velocity
    velocity = vehicle.get_velocity()
    speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
    print(f"\nVelocity:")
    print(f"  Vector: x={velocity.x:.2f}, y={velocity.y:.2f}, z={velocity.z:.2f}")
    print(f"  Speed: {speed:.2f} m/s ({speed * 3.6:.2f} km/h)")

    # Angular velocity
    angular_vel = vehicle.get_angular_velocity()
    print(f"\nAngular Velocity:")
    print(f"  x={angular_vel.x:.2f}, y={angular_vel.y:.2f}, z={angular_vel.z:.2f}")

    # Acceleration
    accel = vehicle.get_acceleration()
    print(f"\nAcceleration:")
    print(f"  x={accel.x:.2f}, y={accel.y:.2f}, z={accel.z:.2f}")

    # Control
    control = vehicle.get_control()
    print(f"\nControl:")
    print(f"  Throttle: {control.throttle:.3f}")
    print(f"  Steer: {control.steer:.3f}")
    print(f"  Brake: {control.brake:.3f}")
    print(f"  Hand Brake: {control.hand_brake}")
    print(f"  Reverse: {control.reverse}")
    print(f"  Manual Gear Shift: {control.manual_gear_shift}")
    print(f"  Gear: {control.gear}")

    # Physics control
    physics = vehicle.get_physics_control()
    print(f"\nPhysics:")
    print(f"  Mass: {physics.mass:.2f} kg")
    print(f"  Max RPM: {physics.max_rpm:.0f}")
    print(f"  Drag Coefficient: {physics.drag_coefficient:.3f}")

    # Traffic light state
    traffic_light_state = vehicle.get_traffic_light_state()
    print(f"\nTraffic Light State: {traffic_light_state}")

    # Attributes
    print(f"\nAttributes:")
    for attr in vehicle.attributes:
        print(f"  {attr}: {vehicle.attributes[attr]}")


def main():
    parser = argparse.ArgumentParser(description="Debug CARLA vehicle state")
    parser.add_argument('--host', default='127.0.0.1', help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--filter', default='vehicle.*', help='Actor filter pattern')
    parser.add_argument('--detailed', action='store_true', help='Show detailed state for each vehicle')
    args = parser.parse_args()

    try:
        # Connect to CARLA
        print(f"Connecting to CARLA at {args.host}:{args.port}...")
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)

        world = client.get_world()
        print(f"Connected to CARLA world: {world.get_map().name}")
        print(f"")

        # Get all actors
        all_actors = world.get_actors()
        print(f"Total actors in world: {len(all_actors)}")

        # Filter vehicles
        vehicles = world.get_actors().filter(args.filter)

        if len(vehicles) == 0:
            print(f"\n⚠ No vehicles found matching filter '{args.filter}'")
            print("\nAll actors in world:")
            for actor in all_actors:
                print(f"  - {actor.type_id} (ID: {actor.id})")
            return

        print(f"\nFound {len(vehicles)} vehicle(s) matching filter '{args.filter}':")
        print("")

        for i, vehicle in enumerate(vehicles):
            if args.detailed:
                dump_vehicle_state(vehicle)
            else:
                # Summary view
                transform = vehicle.get_transform()
                velocity = vehicle.get_velocity()
                speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5

                print(f"{i+1}. {vehicle.type_id} (ID: {vehicle.id})")
                print(f"   Position: ({transform.location.x:.1f}, {transform.location.y:.1f}, {transform.location.z:.1f})")
                print(f"   Rotation: yaw={transform.rotation.yaw:.1f}°")
                print(f"   Speed: {speed:.2f} m/s ({speed * 3.6:.1f} km/h)")
                print("")

        if not args.detailed and len(vehicles) > 0:
            print("Tip: Use --detailed flag to see full vehicle state")

    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
