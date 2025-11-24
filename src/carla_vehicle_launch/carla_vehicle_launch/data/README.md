# Vehicle Actuation Maps - NOT CURRENTLY USED

## Status: ARCHIVED / REFERENCE ONLY

This directory contains CSV lookup tables for converting desired vehicle dynamics (acceleration, steering) to actuator commands (throttle, brake, steering wheel). These files are **NOT currently used** for CARLA simulation.

## Files

### accel_map.csv
Maps (velocity, desired_acceleration) → throttle pedal position [0-1]

**Format**:
```
velocity_mps, acceleration_mps2, throttle_0_to_1
```

### brake_map.csv
Maps (velocity, desired_deceleration) → brake pedal position [0-1]

**Format**:
```
velocity_mps, deceleration_mps2, brake_0_to_1
```

### steer_map.csv
Maps (velocity, desired_tire_angle) → steering wheel angle [rad]

**Format**:
```
velocity_mps, tire_angle_rad, steering_wheel_angle_rad
```

## Why Not Used with CARLA?

The bridge converts ControlCommand to CARLA VehicleControl using simple linear formulas:

```rust
throttle = acceleration.max(0.0) / max_accel;  // Normalize to [0,1]
brake = (-acceleration).max(0.0) / max_decel;
steer = steering_tire_angle / max_steer_angle;
```

CARLA's vehicle physics handle the non-linearities internally, so lookup tables are unnecessary.

## When Would These Be Used?

These maps would be needed if:
- **Hardware-in-the-loop** - Matching real vehicle pedal calibration
- **Actuation testing** - Testing with ActuationCommand interface
- **Realistic modeling** - Modeling specific vehicle's pedal response curves

## Data Source

These maps are from awsim_labs_vehicle_launch and represent a Lexus-like vehicle's actuation characteristics. For CARLA-specific vehicles, these would need to be regenerated from CARLA vehicle data or real vehicle calibration.

## How to Use (if needed)

1. Enable raw_vehicle_cmd_converter in `launch/vehicle_interface.launch.xml`
2. Configure paths in `config/raw_vehicle_cmd_converter/raw_vehicle_cmd_converter.param.yaml`:
   ```yaml
   csv_path_accel_map: $(find-pkg-share carla_vehicle_launch)/data/accel_map.csv
   csv_path_brake_map: $(find-pkg-share carla_vehicle_launch)/data/brake_map.csv
   csv_path_steer_map: $(find-pkg-share carla_vehicle_launch)/data/steer_map.csv
   convert_accel_cmd: true
   convert_brake_cmd: true
   convert_steer_cmd_method: "steer_map"  # or "vgr"
   ```

## Reference

- **Original source**: awsim_labs_vehicle_launch
- **Autoware package**: autoware_raw_vehicle_cmd_converter
- **Documentation**: See Autoware documentation for vehicle actuation mapping
