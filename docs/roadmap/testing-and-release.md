# Testing and Release

This document covers integration testing, performance validation, documentation, and release preparation for the autoware_carla_bridge project.

**Status**: ⏳ **PENDING** - Awaiting completion of Phases 3-6

---

## Phase 9: Integration Testing

**Objective**: End-to-end testing with complete Autoware integration workflow.

**Status**: ⏳ **PENDING**

**Duration**: 1-2 weeks

**Prerequisites**:
- Phases 3-6 complete (full integration)

**Testing Workflow**:
1. Start CARLA simulator
2. Start bridge (waits for Autoware)
3. Start Autoware planning simulator
4. Bridge detects Autoware, spawns vehicle
5. Compare data between CARLA and Autoware topics

### 9.1 Autoware Detection Testing

**Objective**: Verify Autoware instance detection and lifecycle management.

**Tasks**:
- [ ] Test bridge detects Autoware correctly
  - Start Autoware before bridge
  - Start bridge before Autoware
  - Verify detection logs
- [ ] Test handles Autoware disappearance
  - Stop Autoware while bridge running
  - Verify vehicle cleanup
  - Check state transition logs
- [ ] Test multiple start/stop cycles
  - Start/stop Autoware 10 times
  - Verify no memory leaks
  - Check for resource cleanup
- [ ] Test timeout behavior (Autoware not started)
  - Start bridge without Autoware
  - Verify timeout message after 60s
  - Check graceful error handling
- [ ] Test rapid restart scenarios
  - Stop and immediately restart Autoware
  - Verify detection recovers
  - Check for race conditions

**Deliverables**:
- [ ] Test script: `scripts/test_autoware_detection.sh`
- [ ] Test report documenting all scenarios

**Success Criteria**:
- All detection scenarios pass
- No crashes or hangs
- Clear log messages for all state transitions
- Proper cleanup in all scenarios

---

### 9.2 Sensor Data Validation

**Objective**: Verify sensor data accuracy and consistency between CARLA and Autoware.

**Tasks**:
- [ ] Compare `/sensing/camera/*/image_raw` topics
  - Capture images from CARLA and ROS
  - Verify resolution, encoding, frame rate
  - Check timestamp synchronization
  - Visual comparison (no corruption)
- [ ] Compare `/sensing/lidar/*/pointcloud` topics
  - Capture point clouds from CARLA and ROS
  - Verify point count, format, fields
  - Check coordinate transformation
  - Measure publishing rate
- [ ] Compare `/sensing/imu/imu_data`
  - Record IMU data from CARLA and ROS
  - Verify acceleration, angular velocity values
  - Check orientation quaternion
  - Validate coordinate conversion
- [ ] Compare `/sensing/gnss/pose`
  - Record GNSS data from CARLA and ROS
  - Verify lat/lon/alt values
  - Check fix status and service type
  - Measure accuracy
- [ ] Verify data formats match Autoware expectations
  - Use `ros2 interface show` to check message types
  - Validate required fields present
  - Check data ranges and units
- [ ] Validate coordinate transformations (check in RViz)
  - Visualize sensor data in RViz
  - Verify transforms align correctly
  - Check for any axis flips or rotations
  - Compare with expected TF tree
- [ ] Measure publishing rates vs. expected rates
  - Use `ros2 topic hz` to measure rates
  - Compare with configured sensor tick rates
  - Check for dropped messages
  - Verify consistent timing

**Deliverables**:
- [ ] Test script: `scripts/verify_sensor_data.py`
- [ ] Data validation script: `scripts/compare_carla_ros_data.py`
- [ ] Sensor data comparison report
- [ ] RViz configuration for visualization

**Success Criteria**:
- All sensor data published correctly
- Data matches CARLA output (with coordinate conversion)
- Publishing rates within ±5% of expected
- Timestamps synchronized within 10ms
- No data corruption or missing fields

---

### 9.3 Vehicle Control Testing

**Objective**: Verify bidirectional vehicle control integration.

**Tasks**:
- [ ] Send control commands from Autoware
  - Enable Autoware control mode
  - Set navigation goal
  - Verify control commands published
- [ ] Verify vehicle moves correctly in CARLA
  - Observe vehicle motion in CARLA spectator
  - Check throttle, brake, steering response
  - Verify smooth control (no jitter)
- [ ] Check status feedback matches CARLA state
  - Compare velocity from CARLA and `/vehicle/status/velocity_status`
  - Compare steering angle
  - Verify actuation status
- [ ] Test steering, throttle, brake commands
  - Manual control commands via CLI
  - Test full range (0.0 to 1.0)
  - Verify value clamping
- [ ] Test emergency stop
  - Trigger emergency stop in Autoware
  - Verify vehicle stops immediately
  - Check brake application
- [ ] Validate control loop timing
  - Measure end-to-end latency
  - Check command to motion delay
  - Verify real-time performance

**Deliverables**:
- [ ] Control test script: `scripts/test_vehicle_control.py`
- [ ] Control loop latency measurements
- [ ] Control response validation report

**Success Criteria**:
- Vehicle responds to all control commands
- Control loop latency < 50ms
- Status feedback accurate within ±2%
- Emergency stop works correctly
- Smooth vehicle motion (no oscillations)

---

### 9.4 Lifecycle Testing

**Objective**: Verify complete vehicle lifecycle management.

**Tasks**:
- [ ] Test vehicle spawn on initial pose from RViz
  - Start system without initial pose
  - Set initial pose in RViz (2D Pose Estimate)
  - Verify vehicle spawns at correct location
  - Check all sensors attached
- [ ] Test vehicle cleanup when Autoware stops
  - Spawn vehicle normally
  - Stop Autoware
  - Verify vehicle and sensors destroyed
  - Check bridge returns to waiting state
- [ ] Test vehicle respawn when Autoware restarts
  - After cleanup, restart Autoware
  - Set new initial pose
  - Verify vehicle respawns correctly
  - Check sensor reattachment
- [ ] Test teleportation via RViz pose updates
  - Spawn vehicle
  - Set new pose in RViz
  - Verify vehicle teleports
  - Test multiple teleportations
- [ ] Test sensor attachment/detachment
  - Verify all URDF sensors attached
  - Check sensor positions match TF
  - Test with different sensor configurations

**Deliverables**:
- [ ] Lifecycle test script: `scripts/test_vehicle_lifecycle.py`
- [ ] Lifecycle state machine diagram
- [ ] Test report for all scenarios

**Success Criteria**:
- Vehicle spawns correctly with initial pose
- Cleanup removes all actors from CARLA
- Respawn works after cleanup
- Teleportation accurate
- All sensors properly attached

---

### 9.5 Performance Testing

**Objective**: Measure and validate system performance.

**Tasks**:
- [ ] Measure sensor publishing rates
  - Camera: Target 30 Hz
  - LiDAR: Target 10 Hz
  - IMU: Target 100 Hz
  - GNSS: Target 1 Hz
  - Use `ros2 topic hz` for measurement
- [ ] Check control loop latency (<50ms target)
  - Timestamp control command
  - Measure time until vehicle responds
  - Calculate end-to-end latency
  - Identify bottlenecks if needed
- [ ] Monitor CPU usage (< 30% target for single vehicle)
  - Use `htop` or `top` to monitor
  - Measure during full operation
  - Identify high-CPU threads
  - Profile if exceeding target
- [ ] Monitor memory usage (< 500MB target)
  - Check RSS and virtual memory
  - Monitor for memory leaks
  - Measure over extended run (1 hour)
  - Verify stable usage
- [ ] Test with all sensors active simultaneously
  - Enable all sensor types
  - 4 cameras + 4 LiDARs + IMU + GNSS
  - Verify no dropped messages
  - Check for performance degradation
- [ ] Profile bottlenecks if needed
  - Use flamegraph or perf
  - Identify hot paths
  - Optimize if necessary
  - Re-test after optimization

**Deliverables**:
- [ ] Performance benchmark script: `scripts/benchmark_performance.sh`
- [ ] Performance report with metrics
- [ ] Profiling data (if bottlenecks found)
- [ ] Optimization recommendations

**Success Criteria**:
- All publishing rates meet targets (±10%)
- Control latency < 50ms (99th percentile)
- CPU usage < 30% (single vehicle, all sensors)
- Memory usage < 500MB and stable
- No message drops under normal load

---

### Phase 9 Summary

**Deliverables**:
- [ ] Test scripts for all scenarios
- [ ] Validation scripts for data comparison
- [ ] Performance benchmarks and reports
- [ ] Test documentation
- [ ] Integration with CI/CD (if applicable)

**Success Criteria**:
- [ ] All integration tests pass
- [ ] Sensor data validated against CARLA
- [ ] Vehicle control working bidirectionally
- [ ] Lifecycle management robust
- [ ] Performance targets met
- [ ] No known bugs or issues

**Testing Checklist**:
```
□ Autoware detection (all scenarios)
□ URDF parsing (sample_sensor_kit)
□ TF transform lookup (all sensors)
□ Coordinate conversion (all data)
□ Vehicle spawning (with sensors)
□ Vehicle cleanup (on Autoware loss)
□ Vehicle teleportation (RViz pose)
□ Camera data (4 cameras)
□ LiDAR data (4 LiDARs)
□ IMU data
□ GNSS data
□ Control commands (throttle/brake/steer)
□ Status feedback (velocity/steering)
□ Performance benchmarks
□ Memory leak test (1+ hour run)
```

---

## Phase 10: Documentation and Release

**Objective**: Complete documentation and prepare v0.13.0 release.

**Status**: ⏳ **PENDING**

**Duration**: 3-5 days

**Prerequisites**:
- Phase 9 complete (testing)

### 10.1 Update README.md

**Objective**: Update main project README with Autoware integration.

**Tasks**:
- [ ] Add Autoware integration section
  - Overview of integration approach
  - Benefits of new architecture
  - Link to detailed guides
- [ ] Document prerequisites (Autoware 2025.02)
  - System requirements
  - Dependency versions
  - Installation links
- [ ] Add quick start guide for Autoware integration
  ```markdown
  ## Quick Start: Autoware Integration

  1. Start CARLA simulator:
     ```bash
     systemctl --user start carla-0.9.16@3000
     ```

  2. Start bridge (waits for Autoware):
     ```bash
     ros2 run autoware_carla_bridge autoware_carla_bridge
     ```

  3. Start Autoware planning simulator:
     ```bash
     cd ~/autoware
     ros2 launch autoware_launch planning_simulator.launch.xml \
       map_path:=$HOME/autoware_map/sample-map-planning \
       vehicle_model:=sample_vehicle \
       sensor_model:=sample_sensor_kit
     ```

  4. Set initial pose in RViz (2D Pose Estimate tool)

  5. Vehicle spawns in CARLA with sensors!
  ```
- [ ] Update CLI arguments documentation
  - Document removed arguments (tick, slowdown)
  - Document new arguments (--sensor-config)
  - Update examples
- [ ] Add configuration options (sensor mappings, map origin)
  - Sensor mapping file format
  - ROS parameter configuration
  - Example configurations
- [ ] Update build instructions if needed
  - Verify instructions current
  - Add troubleshooting section
  - Update dependency list

**Deliverables**:
- [ ] Updated README.md
- [ ] Quick start guide
- [ ] CLI reference

---

### 10.2 Create Integration Guide

**Objective**: Create comprehensive Autoware integration guide.

**Tasks**:
- [ ] Create `docs/autoware-integration-guide.md`
- [ ] Step-by-step Autoware setup instructions
  - Installing Autoware 2025.02
  - Setting up sample map
  - Configuring ROS environment
  - Building workspace
- [ ] Bridge configuration (carla_sensor_mappings.yaml)
  - File format explanation
  - Sensor type mappings
  - Parameter descriptions
  - Example configurations (sample_sensor_kit)
- [ ] Workflow: CARLA → Bridge → Autoware → Test
  - Detailed step-by-step workflow
  - Expected log output
  - Verification steps
  - Common pitfalls
- [ ] Troubleshooting section (common issues)
  - "Autoware not detected" → Check /robot_description
  - "Sensors not attached" → Check TF tree
  - "Wrong vehicle pose" → Check map origin
  - "Control not working" → Check control topics
  - Debug logging options
- [ ] Example workflows for different scenarios
  - Basic integration (Town01 + sample_sensor_kit)
  - Custom sensor configuration
  - Multi-vehicle setup (ROS_DOMAIN_ID)
  - Map origin configuration

**Deliverables**:
- [ ] `docs/autoware-integration-guide.md` (comprehensive guide)
- [ ] Troubleshooting flowcharts
- [ ] Example configurations

---

### 10.3 API Documentation

**Objective**: Document all modules and APIs.

**Tasks**:
- [ ] Document `autoware_detection.rs` module
  - AutowareDetector API
  - State machine diagram
  - Usage examples
- [ ] Document `tf_bridge.rs` module
  - TFBuffer API
  - Transform lookup
  - Usage examples
- [ ] Document `vehicle_lifecycle.rs` module
  - VehicleSpawner API
  - Lifecycle states
  - Usage examples
- [ ] Document `coordinate_conversion.rs` module
  - Conversion functions
  - Coordinate system diagrams
  - Usage examples
- [ ] Update existing module docs (sensor_bridge, vehicle_bridge)
  - Update for new architecture
  - Add examples
  - Document changes from v0.12.0
- [ ] Add code examples for each module
  - Self-contained examples
  - Common use cases
  - Best practices

**Deliverables**:
- [ ] Module documentation (rustdoc comments)
- [ ] API reference guide
- [ ] Code examples

---

### 10.4 Create Examples

**Objective**: Provide example configurations and scenarios.

**Tasks**:
- [ ] Example 1: Basic Autoware integration (Town01 + sample_sensor_kit)
  - Complete working example
  - Step-by-step instructions
  - Expected output
- [ ] Example 2: Custom sensor configuration
  - Custom sensor mapping file
  - URDF modifications
  - Integration steps
- [ ] Example 3: Multi-vehicle setup with ROS domains (deferred to future)
  - ROS_DOMAIN_ID usage
  - Multiple bridge instances
  - Coordination considerations
- [ ] Add example configs for common scenarios
  - Different CARLA maps
  - Different sensor kits
  - Different vehicle models

**Deliverables**:
- [ ] `examples/` directory with examples
- [ ] Example configurations
- [ ] Example README files

---

### 10.5 Release Preparation

**Objective**: Prepare and publish v0.13.0 release.

**Tasks**:
- [ ] Update CHANGELOG.md with all changes since v0.12.0
  ```markdown
  ## [0.13.0] - 2025-11-XX

  ### Added
  - Autoware integration via /robot_description and /tf_static
  - Automatic sensor configuration from URDF
  - Vehicle lifecycle management tied to Autoware
  - Initial pose subscription (/initialpose)
  - 1-to-1 bridge-vehicle architecture
  - Root namespace topics (standard Autoware names)
  - Vehicle selection CLI (--vehicle-name, --vehicle-id)
  - Coordinate system conversion (ROS ↔ CARLA)

  ### Removed
  - Simulation control (tick thread, --tick, --slowdown)
  - Vehicle prefix filtering ("autoware_" prefix)
  - Multi-vehicle auto-discovery
  - Mode enum (RmwZenoh/ROS2/DDS)

  ### Changed
  - Topics now use root namespace (no vehicle prefix)
  - Bridge is passive adapter (doesn't control simulation)
  - Vehicle must be explicitly selected
  - carla dependency updated to 0.12.0 (local path)

  ### Fixed
  - Various bug fixes from testing
  ```
- [ ] Version bump to v0.13.0 in Cargo.toml
  - Update workspace Cargo.toml
  - Update package Cargo.toml
  - Update documentation versions
- [ ] Create GitHub release notes
  - Highlight major features
  - Link to migration guide
  - Include breaking changes
  - Add screenshots/demos
- [ ] Tag release: `git tag v0.13.0`
  - Create annotated tag
  - Include release summary
  - Sign tag if applicable
- [ ] Build and test release binaries
  - Test on clean system
  - Verify all features work
  - Check binary size
- [ ] Publish release on GitHub
  - Upload binaries (if applicable)
  - Publish release notes
  - Announce on relevant channels

**Deliverables**:
- [ ] CHANGELOG.md updated
- [ ] Git tag v0.13.0
- [ ] GitHub release published
- [ ] Release announcement

---

### Phase 10 Summary

**Deliverables**:
- [ ] Updated README.md
- [ ] Autoware integration guide
- [ ] API documentation
- [ ] Example configurations and scenarios
- [ ] CHANGELOG.md
- [ ] v0.13.0 release published

**Success Criteria**:
- [ ] All documentation complete and accurate
- [ ] Examples tested and working
- [ ] Release notes comprehensive
- [ ] Version tagged and published
- [ ] Community notified

**Release Checklist**:
```
□ README.md updated
□ Integration guide complete
□ API docs written
□ Examples tested
□ CHANGELOG.md updated
□ Version bumped (Cargo.toml)
□ Git tag created
□ Release notes written
□ Binaries built and tested
□ GitHub release published
□ Announcement posted
```

---

**Document Version**: 1.0
**Last Updated**: 2025-11-04
**Related Documents**:
- [roadmap.md](../roadmap.md) - Main roadmap index
- [infrastructure.md](infrastructure.md) - Infrastructure setup (Phases 0, 1, 7)
- [bridge.md](bridge.md) - Data bridge implementation (Phases 2, 5, 6, 8)
- [integration.md](integration.md) - Autoware integration (Phases 3-4)
- [../architecture.md](../architecture.md) - Architecture design and ADRs
