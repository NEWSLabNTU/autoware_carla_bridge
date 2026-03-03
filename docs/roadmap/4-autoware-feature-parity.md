# Autoware Feature Parity Roadmap

Track progress toward feature parity with TUMFTM Carla-Autoware-Bridge while maintaining our superior architecture (single Rust process vs multi-process Python).

**Reference Documents**:
- `docs/architecture-comparison.md` - Feature comparison with TUMFTM
- `docs/tumftm-bridge-analysis.md` - Detailed TUMFTM analysis
- `docs/sensor-configuration-strategy.md` - Sensor config design

**Current Status**: 🟡 **IN PROGRESS** - Core infrastructure complete, vehicle integration pending

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

**Localization** (Partially Complete):
- ✅ `/localization/kinematic_state` publisher (Odometry) - **Our approach**
- ⏳ `/sensing/gnss/pose_with_covariance` (PoseWithCovarianceStamped) - **TUMFTM approach** (verify if needed)

**Sensors** (Complete):
- ✅ Camera (RGB) - `/sensing/camera/*/image_raw`
- ✅ LiDAR - `/sensing/lidar/*/pointcloud`
- ✅ IMU - `/sensing/imu/imu_raw`
- ✅ GNSS - `/sensing/gnss/*/nav_sat_fix`

**Clock**:
- ✅ Simulator clock synchronization via `/clock`

### 🔴 Missing Critical Features

Priority order based on Autoware integration requirements.

---

## Vehicle Control Integration

**Objective**: Implement bidirectional vehicle control between Autoware and CARLA

**Status**: 🔴 **NOT STARTED**

**Priority**: 🔴 **HIGH** - Required for Autoware to control vehicle

**Duration**: 1-2 weeks

**Reference**: TUMFTM's `aw_bridge.py` (lines 82-134)

### 4.1 Control Command Subscriber

**Objective**: Subscribe to Autoware control commands and apply to CARLA vehicle

**Tasks**:
- [ ] Subscribe to `/control/command/control_cmd` (AckermannControlCommand)
- [ ] Parse lateral control (steering_tire_angle, steering_tire_rotation_rate)
- [ ] Parse longitudinal control (speed, acceleration, jerk)
- [ ] Convert to CARLA VehicleControl
- [ ] Apply control to vehicle using `vehicle.apply_control()`
- [ ] Add control rate limiting (if needed)

**TUMFTM Findings**:
- Uses 1.2x steering angle multiplier (vehicle calibration)
- Directly forwards speed, acceleration, jerk
- No throttle/brake conversion (uses high-level control)

**Our Approach**:
- Direct CARLA `VehicleControl` API
- Optional calibration config file per vehicle model
- Support both high-level (speed) and low-level (throttle/brake) control

**Deliverables**:
- Control command subscription working
- Vehicle responds to Autoware planning commands
- Smooth control without oscillations

**Testing**:
- [ ] Verify vehicle follows Autoware trajectory
- [ ] Test emergency stops
- [ ] Validate steering response
- [ ] Check control loop timing

---

### 4.2 Vehicle Status Publishers

**Objective**: Publish vehicle state to Autoware for monitoring and feedback

**Tasks**:

**a) Velocity Status** (`/vehicle/status/velocity_status`):
- [ ] Create VelocityReport message
- [ ] Publish longitudinal velocity (from CARLA vehicle)
- [ ] Publish lateral velocity
- [ ] Publish heading rate
- [ ] Set timestamp from simulator clock

**b) Steering Status** (`/vehicle/status/steering_status`):
- [ ] Create SteeringReport message
- [ ] Publish steering_tire_angle (from CARLA vehicle)
- [ ] Set timestamp from simulator clock

**c) Control Mode** (`/vehicle/status/control_mode`):
- [ ] Create ControlModeReport message
- [ ] Set mode to AUTONOMOUS (mode = 1) - always autonomous in simulation
- [ ] Publish at 10 Hz

**TUMFTM Implementation**:
```python
# From TUMFTM aw_bridge.py
velocity_status.longitudinal_velocity = odometry.twist.twist.linear.x
velocity_status.lateral_velocity = odometry.twist.twist.linear.y
velocity_status.heading_rate = odometry.twist.twist.angular.z

control_mode.mode = 1  # Always autonomous
```

**Our Approach**:
- Get state directly from CARLA vehicle API
- Use `vehicle.velocity()`, `vehicle.angular_velocity()`
- Extract steering from `vehicle.get_control()`
- More efficient than TUMFTM (no intermediate odometry conversion)

**Deliverables**:
- Three status publishers working
- Autoware receives vehicle feedback
- Status updates at appropriate rates

**Testing**:
- [ ] Verify Autoware sees vehicle status
- [ ] Check timing of status updates
- [ ] Validate values match CARLA ground truth

---

### 4.3 Pose with Covariance Publisher (Optional)

**Objective**: Publish pose with covariance for Autoware localization (if needed)

**Status**: ⏳ **INVESTIGATE** - May not be needed if Odometry is sufficient

**Tasks**:
- [ ] Check if Autoware 2025.02 requires this topic
- [ ] If yes: Convert from kinematic_state to PoseWithCovarianceStamped
- [ ] Set covariance matrix (use TUMFTM values: 0.1 diagonal)
- [ ] Publish to `/sensing/gnss/pose_with_covariance`

**TUMFTM Covariance**:
```python
covariance = [
    0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
]
```

**Decision Point**:
- First verify if Autoware works with `/localization/kinematic_state` alone
- Only add this if Autoware explicitly requires it

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
- [ ] Subscribe to emergency stop topic
- [ ] Immediately apply full brakes on trigger
- [ ] Override Autoware control commands
- [ ] Publish emergency state to `/vehicle/status/control_mode`

---

## Progress Tracking

### Metrics

| Category                               | TUMFTM      | Our Bridge          | Status                |
|----------------------------------------|-------------|---------------------|-----------------------|
| **Publishers**                         |             |                     |                       |
| Sensor data (camera, lidar, imu, gnss) | ✅          | ✅                  | Done                  |
| Kinematic state / Odometry             | ✅          | ✅                  | Done                  |
| Velocity status                        | ✅          | ❌                  | TODO Phase 4.2a       |
| Steering status                        | ✅          | ❌                  | TODO Phase 4.2b       |
| Control mode                           | ✅          | ❌                  | TODO Phase 4.2c       |
| Pose with covariance                   | ✅          | ⏳                  | Investigate Phase 4.3 |
| **Subscribers**                        |             |                     |                       |
| Control command                        | ✅          | ❌                  | TODO Phase 4.1        |
| **Performance**                        |             |                     |                       |
| Process count                          | 5+          | 1                   | ⚡ Better             |
| Memory usage                           | ~500-800 MB | ~50-100 MB          | ⚡ Better             |
| Latency                                | ~10-20 ms   | ~1-5 ms             | ⚡ Better             |
| **Integration**                        |             |                     |                       |
| Sensor config                          | Static JSON | URDF + YAML         | ⚡ Better             |
| Vehicle spawning                       | Manual      | Automatic from RViz | ⚡ Better             |

### Completion Status

**Overall Progress**: 60% complete

- ✅ Infrastructure: 100% (superior to TUMFTM)
- ✅ Sensor integration: 100%
- ✅ Localization: 100% (Odometry approach)
- 🔴 Vehicle control: 0% (Phase 4 TODO)
- 🔴 Vehicle status: 0% (Phase 4 TODO)

**Estimated Remaining Work**: 2-3 weeks

---

## Dependencies

### Message Types Needed

Already in workspace (verify availability):
- `autoware_vehicle_msgs::msg::VelocityReport`
- `tier4_vehicle_msgs::msg::SteeringReport`
- `autoware_vehicle_msgs::msg::ControlModeReport`
- `autoware_control_msgs::msg::Control` (for control_cmd)

If missing, add to `src/interface/` symlinks.

### CARLA APIs Needed

- `vehicle.apply_control(VehicleControl)` - Apply control commands
- `vehicle.get_control()` - Get current control state (for steering status)
- `vehicle.velocity()` - Get velocity vector ✅ (already using)
- `vehicle.angular_velocity()` - Get angular velocity ✅ (already using)

---

## Risk Assessment

### High Risk

1. **Control loop timing**: Autoware expects specific control rates
   - **Mitigation**: Measure and adjust publish rates, add timing diagnostics

2. **Steering calibration**: Different vehicles need different multipliers
   - **Mitigation**: Implement calibration system early (Phase 5)

### Medium Risk

1. **Message compatibility**: Autoware 2025.02 may have changed message definitions
   - **Mitigation**: Test early with running Autoware instance

2. **Covariance values**: Incorrect covariance may confuse Autoware
   - **Mitigation**: Use TUMFTM's proven values, tune if needed

### Low Risk

1. **Performance**: Control loop may be too slow
   - **Mitigation**: Unlikely with Rust's performance, monitor anyway

---

## Next Steps

**Immediate (Phase 4.1)**:
1. Check that control message types are available in workspace
2. Implement control command subscriber
3. Test vehicle responds to Autoware commands
4. Validate control loop timing

**Short Term (Phase 4.2)**:
1. Implement three status publishers
2. Verify Autoware sees vehicle feedback
3. Test complete control loop

**Medium Term (Phase 5)**:
1. Add vehicle calibration system
2. Test with multiple vehicle models
3. Tune calibration parameters

---

**Last Updated**: 2025-11-08
**Next Review**: After Phase 4 completion
