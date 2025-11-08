# Architecture Comparison: TUMFTM vs Our Bridge

Quick reference comparing the TUMFTM Carla-Autoware-Bridge with our Rust-based implementation.

**See also**: `tumftm-bridge-analysis.md` for detailed analysis.

---

## High-Level Architecture

### TUMFTM Approach (Python-based)

```
Autoware
   ↕ Autoware topics
carla_autoware_bridge (Python adapter - TUMFTM code)
   ↕ CARLA topics
carla_ros_bridge (Python - External dependency)
   ↕ CARLA Python API
CARLA Simulator

Processes: 5+ Python processes
```

### Our Approach (Rust-based)

```
Autoware
   ↕ ROS 2 topics (native DDS)
autoware_carla_bridge (Rust - Direct implementation)
   ↕ carla-rust crate
CARLA Simulator

Processes: 1 Rust process
```

---

## Key Differences Table

| Aspect              | TUMFTM Bridge                                       | Our Bridge                | Winner    |
|---------------------|-----------------------------------------------------|---------------------------|-----------|
| **Language**        | Python                                              | Rust                      | ⚡ Rust   |
| **Performance**     | Python overhead, GIL                                | Zero-cost abstractions    | ⚡ Rust   |
| **Type Safety**     | Runtime checking                                    | Compile-time checking     | ⚡ Rust   |
| **Memory Usage**    | High (multiple Python processes)                    | Low (single Rust process) | ⚡ Rust   |
| **Process Count**   | 5+ processes                                        | 1 process                 | ⚡ Rust   |
| **CARLA Interface** | Indirect (via carla_ros_bridge)                     | Direct (carla-rust)       | ⚡ Rust   |
| **Latency**         | Higher (multi-process IPC)                          | Lower (single process)    | ⚡ Rust   |
| **Dependencies**    | High (carla_ros_bridge, ackermann controller, etc.) | Low (rclrs, carla-rust)   | ⚡ Rust   |
| **Sensor Config**   | Static JSON (objects.json)                          | Dynamic URDF parsing      | ⚡ Rust   |
| **Config Sync**     | Manual (JSON must match URDF)                       | Automatic (single source) | ⚡ Rust   |
| **Deployment**      | Complex (5+ processes to manage)                    | Simple (single binary)    | ⚡ Rust   |
| **Maturity**        | Published (IEEE IV 2024)                            | In development            | 📄 TUMFTM |
| **Maps**            | Pre-converted available                             | Need conversion           | 📄 TUMFTM |
| **Community**       | Academic (TUM)                                      | NEWSLab NTU               | 🤝 Both   |

---

## Component Comparison

### Sensor Management

| Feature               | TUMFTM                                | Our Bridge                              |
|-----------------------|---------------------------------------|-----------------------------------------|
| Configuration source  | Static JSON file                      | Dynamic URDF from Autoware              |
| Sensor parameters     | Explicit in JSON                      | Parsed from URDF + TF                   |
| Example               | `objects.json` with hardcoded sensors | `/robot_description` topic subscription |
| Flexibility           | Must edit JSON for changes            | Auto-adapts to Autoware config          |
| CARLA-specific params | ✅ Explicit (fov, points_per_second)  | ⚠️ Not captured (future enhancement)     |

**TUMFTM objects.json example**:
```json
{
  "type": "sensor.lidar.ray_cast",
  "id": "sensor/lidar/front",
  "spawn_point": {"x": 1.5, "y": 0.0, "z": 2.1},
  "range": 50,
  "channels": 64,
  "points_per_second": 480000
}
```

**Our URDF parsing**:
- Parse `/robot_description` topic
- Extract sensor links and types
- Lookup transforms from TF2
- Spawn with URDF poses + TF transforms

### Localization

| Aspect                | TUMFTM                               | Our Bridge                             |
|-----------------------|--------------------------------------|----------------------------------------|
| Input                 | `Odometry` from carla_ros_bridge     | Direct CARLA `Vehicle.get_transform()` |
| Output topic          | `/sensing/gnss/pose_with_covariance` | `/localization/kinematic_state`        |
| Message type          | `PoseWithCovarianceStamped`          | `Odometry`                             |
| Covariance            | Hardcoded `[0.1, 0.0, ...]`          | Not set (future: dynamic)              |
| Frame                 | `map`                                | `map`                                  |
| Coordinate conversion | None (carla_ros_bridge handles)      | Explicit CARLA → ROS conversion        |

### Vehicle Control

| Aspect               | TUMFTM                          | Our Bridge                  |
|----------------------|---------------------------------|-----------------------------|
| Input topic          | `/control/command/control_cmd`  | ❌ **Not implemented yet**  |
| Input type           | `AckermannControlCommand`       | N/A                         |
| Processing           | Convert to `AckermannDrive`     | N/A                         |
| Steering multiplier  | 1.2x (hardcoded calibration)    | N/A                         |
| Output               | Via carla_ackermann_control     | N/A                         |
| Direct CARLA control | ❌ No (via separate controller) | ✅ **Planned** (direct API) |

### Vehicle Status

| Topic                             | TUMFTM                          | Our Bridge   | Status         |
|-----------------------------------|---------------------------------|--------------|----------------|
| `/vehicle/status/velocity_status` | ✅ Published                    | ❌ Missing   | 🔴 Need to add |
| `/vehicle/status/steering_status` | ✅ Published                    | ❌ Missing   | 🔴 Need to add |
| `/vehicle/status/control_mode`    | ✅ Published (always mode=1)    | ❌ Missing   | 🔴 Need to add |
| `/localization/kinematic_state`   | ❌ (uses pose_with_cov instead) | ✅ Published | 🟢 Done        |

---

## Message Flow Comparison

### TUMFTM Message Flow

```
CARLA Simulator
    ↓ (CARLA Python API)
carla_ros_bridge
    ↓ /carla/ego_vehicle/odometry
    ↓ /carla/ego_vehicle/vehicle_status
carla_autoware_bridge (converter)
    ↓ /vehicle/status/velocity_status
    ↓ /vehicle/status/steering_status
    ↓ /sensing/gnss/pose_with_covariance
Autoware
    ↓ /control/command/control_cmd
carla_autoware_bridge (converter)
    ↓ /carla/ego_vehicle/ackermann_cmd
carla_ackermann_control
    ↓ (CARLA Python API)
CARLA Simulator
```

**Latency sources**:
- Python GIL contention
- Inter-process communication (5+ processes)
- Message serialization overhead
- carla_ros_bridge → carla_autoware_bridge conversion

### Our Message Flow

```
CARLA Simulator
    ↓ (carla-rust async callbacks)
autoware_carla_bridge
    ↓ /localization/kinematic_state (Odometry)
    ↓ /sensing/lidar/*/pointcloud (PointCloud2)
    ↓ /sensing/camera/*/image_raw (Image)
    ↓ /sensing/imu/imu_raw (Imu)
    ↓ /sensing/gnss/*/nav_sat_fix (NavSatFix)
    ↓ (Future: /vehicle/status/* publishers)
Autoware
    ↓ (Future: /control/command/control_cmd)
autoware_carla_bridge
    ↓ (carla-rust direct API)
CARLA Simulator
```

**Latency advantages**:
- Single process (no IPC)
- Rust's zero-cost abstractions
- Direct CARLA API calls
- Async sensor callbacks

---

## Missing Features in Our Bridge

Based on TUMFTM analysis, we're missing these Autoware integration topics:

### High Priority

1. **Control Command Subscription** 🔴
   - Topic: `/control/command/control_cmd`
   - Type: `AckermannControlCommand`
   - Purpose: Receive control commands from Autoware planning
   - Implementation: Subscribe + apply to CARLA vehicle

2. **Vehicle Status Publishers** 🔴
   - `/vehicle/status/velocity_status` (VelocityReport)
   - `/vehicle/status/steering_status` (SteeringReport)
   - `/vehicle/status/control_mode` (ControlModeReport)
   - Purpose: Provide vehicle state to Autoware

3. **Pose with Covariance** 🟡
   - Topic: `/sensing/gnss/pose_with_covariance`
   - Type: `PoseWithCovarianceStamped`
   - Purpose: Alternative to odometry (check if Autoware prefers this)
   - Note: We currently publish `/localization/kinematic_state` (Odometry)

### Medium Priority

4. **Sensor Parameter Configuration** 🟡
   - CARLA-specific params not in URDF (fov, points_per_second, etc.)
   - Consider JSON config overlay on URDF
   - Example: Hybrid approach (URDF poses + JSON params)

5. **Traffic Generation** 🟢
   - Port TUMFTM's `generate_traffic.py` to Rust
   - Spawn NPC vehicles and pedestrians
   - CLI params: `--traffic-vehicles N --traffic-walkers N`

6. **Control Calibration** 🟡
   - Vehicle-specific steering multiplier (TUMFTM uses 1.2x)
   - Configurable calibration file
   - Per-vehicle tuning

---

## Performance Comparison (Estimated)

| Metric | TUMFTM | Our Bridge | Improvement |
|--------|--------|------------|-------------|
| Memory (RSS) | ~500-800 MB (5 Python processes) | ~50-100 MB (1 Rust process) | **5-8x** |
| Latency (sensor→ROS) | ~10-20 ms (multi-process) | ~1-5 ms (single process) | **2-10x** |
| CPU Usage | ~15-25% (Python overhead) | ~5-10% (Rust efficiency) | **2-3x** |
| Startup Time | ~5-10 sec (launch 5 processes) | ~1-2 sec (single binary) | **3-5x** |

*Note: These are estimates. Actual benchmarking needed for precise numbers.*

---

## Design Philosophy Comparison

### TUMFTM Design

**Philosophy**: Thin adapter layer on existing infrastructure

**Strengths**:
- ✅ Leverages mature carla_ros_bridge
- ✅ Quick development (Python)
- ✅ Proven in academic research (IEEE IV 2024)
- ✅ Pre-converted maps available

**Trade-offs**:
- ⚠️ Performance limited by Python
- ⚠️ Dependent on carla_ros_bridge updates
- ⚠️ Complex multi-process architecture
- ⚠️ Manual sensor configuration sync

**Target**: Fast prototyping and academic research

### Our Design

**Philosophy**: Direct, efficient, single-process bridge

**Strengths**:
- ✅ Maximum performance (Rust + direct API)
- ✅ Single source of truth (URDF)
- ✅ Compile-time safety
- ✅ Simple deployment (one binary)
- ✅ Full control over CARLA integration

**Trade-offs**:
- ⚠️ Longer development time
- ⚠️ Need to implement all features ourselves
- ⚠️ Less mature than TUMFTM

**Target**: Production-grade, high-performance simulation

---

## Integration Strategy

### What to Adopt from TUMFTM

1. **Message Patterns** ✅
   - Reference for Autoware topic structure
   - Covariance values (0.1 diagonal)
   - Control mode semantics

2. **Pre-Converted Maps** ✅
   - Use their Lanelet2 maps for testing
   - Download from their hosting service
   - Verify with our bridge

3. **Sensor Config Format** 🤔
   - Consider JSON overlay for CARLA-specific params
   - Hybrid: URDF (poses) + JSON (CARLA params)
   - Optional, not required

4. **Traffic Generation** 🤔
   - Port to Rust if needed
   - Or use CARLA's built-in traffic manager
   - Lower priority

### What to Keep Different

1. **Direct CARLA Integration** ✅
   - Don't add carla_ros_bridge dependency
   - Maintain single-process architecture
   - Keep Rust implementation

2. **URDF Parsing** ✅
   - Keep as primary config source
   - Auto-sync with Autoware
   - Vehicle-agnostic approach

3. **Performance Focus** ✅
   - Optimize for latency and throughput
   - Leverage Rust's zero-cost abstractions
   - Async/concurrent design

---

## Roadmap Alignment

### TUMFTM's Stated Goals

From their README:
> "We aim to enhance future efficiency by ensuring that the bridge is Python-free, utilizing native DDS connection with the CARLA simulator"

**Status**: ✅ **Our bridge already achieves this goal**

### Our Next Steps

**Phase 4: Vehicle Control** (High Priority)
- [ ] Add `/control/command/control_cmd` subscriber
- [ ] Implement vehicle actuation (throttle/brake/steering)
- [ ] Add `/vehicle/status/velocity_status` publisher
- [ ] Add `/vehicle/status/steering_status` publisher
- [ ] Add `/vehicle/status/control_mode` publisher

**Phase 5: Advanced Features** (Medium Priority)
- [ ] Test with TUMFTM pre-converted maps
- [ ] Optional JSON sensor configuration
- [ ] Traffic generation (Rust port)
- [ ] Vehicle calibration config

---

## Conclusion

### Summary Table

| Category | TUMFTM | Our Bridge |
|----------|--------|------------|
| **Maturity** | Production (IEEE 2024) | Development |
| **Performance** | Python baseline | Rust optimized ⚡ |
| **Architecture** | Multi-process adapter | Single-process direct |
| **Flexibility** | Static configuration | Dynamic URDF |
| **Maintenance** | Depends on carla_ros_bridge | Self-contained |
| **Learning Value** | High (reference impl) | High (clean design) |

### Key Takeaway

**TUMFTM provides**:
- Validated Autoware integration patterns
- Pre-converted maps for testing
- Reference message conversions
- Proof of concept

**Our bridge achieves**:
- Their stated performance goals (Python-free, native DDS)
- Superior efficiency and latency
- Cleaner architecture
- Better maintainability

**Best path forward**:
1. Implement missing Autoware topics (using TUMFTM as reference)
2. Test with their pre-converted maps
3. Benchmark performance vs TUMFTM
4. Contribute findings back to community

---

**Last Updated**: 2025-11-08
**Related Documents**:
- `tumftm-bridge-analysis.md` - Detailed analysis of TUMFTM bridge
- `autoware-integration-design.md` - Our bridge architecture
- `carla-autoware-map-integration.md` - Map conversion guide
- `roadmap.md` - Development plan
