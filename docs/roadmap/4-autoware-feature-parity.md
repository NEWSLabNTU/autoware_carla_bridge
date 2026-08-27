# Autoware Feature Parity Roadmap

Track progress toward feature parity with TUMFTM Carla-Autoware-Bridge while maintaining our superior architecture (single Rust process vs multi-process Python).

**Reference Documents**:
- `docs/archive/architecture-comparison.md` - Feature comparison with TUMFTM
- `docs/archive/tumftm-bridge-analysis.md` - Detailed TUMFTM analysis
- `docs/design/sensor-configuration-strategy.md` - Sensor config design

**Current Status**: 🟢 **MOSTLY COMPLETE** - Vehicle control and status publishers implemented, calibration pending

---

## Progress Overview

### ✅ Completed Features

**Infrastructure** (Superior to TUMFTM):
- ✅ Direct CARLA integration via carla-rust (no carla_ros_bridge dependency)
- ✅ Single Rust process (vs TUMFTM's 5+ Python processes)
- ✅ Native ROS 2 DDS communication (rclrs)
- ✅ URDF-based sensor configuration (vs TUMFTM's static JSON)
- ✅ Automatic sensor spawning from Autoware `/robot_description`
- ✅ TF2 tree traversal for sensor positions
- ✅ CARLA sensor parameter configuration system (hybrid URDF + YAML)

**Localization** (Complete):
- ✅ `/localization/kinematic_state` publisher (Odometry)
- ✅ `/sensing/gnss/pose_with_covariance` (PoseWithCovarianceStamped) - bypasses gnss_poser for local projector maps

**Sensors** (Complete):
- ✅ Camera (RGB) - `/sensing/camera/*/image_raw`
- ✅ LiDAR - `/sensing/lidar/*/pointcloud`
- ✅ IMU - `/sensing/imu/imu_raw`
- ✅ GNSS - `/sensing/gnss/*/nav_sat_fix`

**Clock**:
- ✅ Simulator clock synchronization via `/clock`

**Vehicle Control** (Complete):
- ✅ Control command subscriber (`/control/command/actuation_cmd`)
- ✅ Velocity status publisher (`/vehicle/status/velocity_status`)
- ✅ Steering status publisher (`/vehicle/status/steering_status`)
- ✅ Control mode publisher (`/vehicle/status/control_mode`)
- ✅ Gear status publisher (`/vehicle/status/gear_status`)
- ✅ GNSS pose with covariance publisher (`/sensing/gnss/pose_with_covariance`)

---

## Vehicle Control Integration

**Objective**: Implement bidirectional vehicle control between Autoware and CARLA

**Status**: ✅ **COMPLETE**

**Implementation**: `src/autoware_carla_bridge/src/vehicle_control.rs`

### 4.1 Control Command Subscriber

**Status**: ✅ **COMPLETE**

**Implementation**:
- Subscribes to `/control/command/actuation_cmd` (ActuationCommandStamped)
- Converts steer_cmd, accel_cmd, brake_cmd to CARLA VehicleControl
- Steering clamped to [-1.0, 1.0], throttle/brake to [0.0, 1.0]
- Dead zone at 0.01 for accel/brake to avoid jitter
- Applied via `vehicle.apply_control()`

**Design Choice**: Uses ActuationCommandStamped (direct throttle/brake/steer) rather than AckermannControlCommand (speed/steering_angle). This matches Autoware's raw_vehicle_cmd_converter output.

---

### 4.2 Vehicle Status Publishers

**Status**: ✅ **COMPLETE**

**Implementation** (`VehicleControlBridge::publish_status()`):

**a) Velocity Status** (`/vehicle/status/velocity_status`):
- VelocityReport with longitudinal_velocity (3D magnitude), lateral_velocity, heading_rate
- Reads directly from `vehicle.velocity()` and `vehicle.angular_velocity()`

**b) Steering Status** (`/vehicle/status/steering_status`):
- SteeringReport with steering_tire_angle (converted from CARLA's -1..1 to radians)
- Max steering angle: 1.22 rad (~70 degrees)

**c) Control Mode** (`/vehicle/status/control_mode`):
- ControlModeReport with mode=AUTONOMOUS (1) always

**d) Gear Status** (`/vehicle/status/gear_status`):
- GearReport with report=DRIVE always
- Added beyond original plan for Autoware compatibility

All published every tick (~20Hz) with simulation timestamps.

---

### 4.3 Pose with Covariance Publisher

**Status**: ✅ **COMPLETE**

**Implementation**: `src/autoware_carla_bridge/src/autoware.rs`

- Publishes PoseWithCovarianceStamped to `/sensing/gnss/pose_with_covariance`
- Enabled when `auto_initialize_localization` is active
- Bypasses gnss_poser (which fails with local projector type maps)
- Covariance diagonal: [0.25, 0.25, 0.25, 0.01, 0.01, 0.01] (position + orientation)
- Converts CARLA transform to ROS pose with coordinate system conversion

---

## Vehicle Calibration

**Objective**: Add vehicle-specific calibration for different CARLA models

**Status**: 🔴 **NOT STARTED**

**Priority**: 🟡 **MEDIUM** - Improves control accuracy

**Duration**: 3-5 days

### 4.4 Calibration Config File

**Objective**: Support per-vehicle calibration parameters

**Tasks**:
- [ ] Design YAML schema for vehicle calibration
- [ ] Add steering angle multiplier
- [ ] Add throttle/brake calibration curves
- [ ] Add wheelbase and vehicle dimensions
- [ ] Load calibration based on vehicle blueprint ID

**Example Structure**:
```yaml
# config/vehicle_calibration.yaml
vehicles:
  "vehicle.tesla.model3":
    steering_multiplier: 1.2
    wheelbase: 2.875
    max_steer_angle: 70.0

  "vehicle.audi.etron":
    steering_multiplier: 1.1
    wheelbase: 2.928
    max_steer_angle: 75.0

  default:  # Fallback for unknown vehicles
    steering_multiplier: 1.0
    wheelbase: 2.7
    max_steer_angle: 70.0
```

**Deliverables**:
- Calibration config system implemented
- CLI parameter for calibration file path
- Calibration applied to control commands

**Testing**:
- [ ] Test with multiple vehicle models
- [ ] Validate steering response matches Autoware expectations
- [ ] Compare with TUMFTM control behavior

---

## Advanced Features (Low Priority)

**Status**: ⏳ **FUTURE**

**Priority**: 🟢 **LOW** - Nice to have, not critical

### 4.5 Manual/Autonomous Mode Switching

**Objective**: Support switching between manual and autonomous control

**Why Low Priority**: Simulation is always autonomous, manual mode not needed

**Tasks** (if needed):
- [ ] Subscribe to mode switch topic
- [ ] Disable Autoware control in manual mode
- [ ] Allow keyboard/joystick control
- [ ] Update `/vehicle/status/control_mode` accordingly

---

### 4.6 Emergency Stop

**Objective**: Implement emergency stop functionality

**Tasks**:
- [x] Subscribe to emergency stop topic -- `/control/command/emergency_cmd`, which had one
      publisher and zero subscribers until 2026-08-28; see docs/issues/021
- [x] Immediately apply full brakes on trigger -- 2.10 m/s to a standstill inside a second,
      held with the handbrake so CARLA's idle creep cannot restart it
- [x] Override Autoware control commands -- decided before any pedal-map work, since the
      emergency travels on its own topic exactly so it holds when the control command does not
- [ ] Publish emergency state to `/vehicle/status/control_mode` -- not done deliberately.
      `ControlModeReport` has no emergency value (NO_COMMAND, AUTONOMOUS, the two partial
      autonomies, MANUAL, DISENGAGED, NOT_READY), and Autoware's own vehicle interfaces keep
      reporting AUTONOMOUS through an emergency. Hazard lights carry the state instead.

---

## Progress Tracking

### Metrics

| Category                               | TUMFTM      | Our Bridge          | Status    |
|----------------------------------------|-------------|---------------------|-----------|
| **Publishers**                         |             |                     |           |
| Sensor data (camera, lidar, imu, gnss) | ✅          | ✅                  | Done      |
| Kinematic state / Odometry             | ✅          | ✅                  | Done      |
| Velocity status                        | ✅          | ✅                  | Done      |
| Steering status                        | ✅          | ✅                  | Done      |
| Control mode                           | ✅          | ✅                  | Done      |
| Gear status                            | ❌          | ✅                  | Done      |
| Pose with covariance                   | ✅          | ✅                  | Done      |
| **Subscribers**                        |             |                     |           |
| Control command                        | ✅          | ✅                  | Done      |
| **Performance**                        |             |                     |           |
| Process count                          | 5+          | 1                   | Better    |
| Memory usage                           | ~500-800 MB | ~50-100 MB          | Better    |
| Latency                                | ~10-20 ms   | ~1-5 ms             | Better    |
| **Integration**                        |             |                     |           |
| Sensor config                          | Static JSON | URDF + YAML         | Better    |
| Vehicle spawning                       | Manual      | Automatic from RViz | Better    |

### Completion Status

**Overall Progress**: 90% complete

- ✅ Infrastructure: 100% (superior to TUMFTM)
- ✅ Sensor integration: 100%
- ✅ Localization: 100%
- ✅ Vehicle control: 100% (Phase 4.1)
- ✅ Vehicle status: 100% (Phase 4.2)
- ✅ GNSS pose: 100% (Phase 4.3)
- 🔴 Vehicle calibration: 0% (Phase 4.4)

**Estimated Remaining Work**: 3-5 days (calibration only)

---

## Dependencies

### Message Types Used

In workspace:
- `autoware_vehicle_msgs::msg::VelocityReport` ✅
- `autoware_vehicle_msgs::msg::SteeringReport` ✅
- `autoware_vehicle_msgs::msg::ControlModeReport` ✅
- `autoware_vehicle_msgs::msg::GearReport` ✅
- `tier4_vehicle_msgs::msg::ActuationCommandStamped` ✅ (for control_cmd)
- `geometry_msgs::msg::PoseWithCovarianceStamped` ✅ (for GNSS pose)

### CARLA APIs Used

- `vehicle.apply_control(VehicleControl)` ✅ - Apply control commands
- `vehicle.control()` ✅ - Get current control state (for steering status)
- `vehicle.velocity()` ✅ - Get velocity vector
- `vehicle.angular_velocity()` ✅ - Get angular velocity

---

## Next Steps

**Remaining (Phase 4.4)**:
1. Design YAML schema for vehicle calibration
2. Add per-vehicle steering angle multiplier
3. Test with multiple vehicle models
4. Tune calibration parameters
