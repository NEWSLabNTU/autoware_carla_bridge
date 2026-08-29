# Testing

Integration testing and performance validation for the project.

**Status**: ⏳ **PENDING** - Phases 1-4 complete; formal test scripts not yet created

**Note**: Core functionality has been informally verified via end-to-end autonomous driving (CARLA -> Bridge -> Autoware -> Vehicle follows route to goal). Formal test scripts and benchmarks are the remaining work.

**Prerequisites**:
- ✅ Phases 1-4 complete (full integration working)
- Phase 6 complete (package renaming)

---

## 7.1 Autoware Detection Testing

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

## 7.2 Sensor Data Validation

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
- Publishing rates within +/-5% of expected
- Timestamps synchronized within 10ms
- No data corruption or missing fields

---

## 7.3 Vehicle Control Testing

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
- Status feedback accurate within +/-2%
- Emergency stop works correctly
- Smooth vehicle motion (no oscillations)

---

## 7.4 Lifecycle Testing

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

## 7.5 Performance Testing

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
- All publishing rates meet targets (+/-10%)
- Control latency < 50ms (99th percentile)
- CPU usage < 30% (single vehicle, all sensors)
- Memory usage < 500MB and stable
- No message drops under normal load

---

## Summary

**Deliverables**:
- [ ] Test scripts for all scenarios
- [ ] Validation scripts for data comparison
- [ ] Performance benchmarks and reports
- [ ] Test documentation

**Success Criteria**:
- [ ] All integration tests pass
- [ ] Sensor data validated against CARLA
- [ ] Vehicle control working bidirectionally
- [ ] Lifecycle management robust
- [ ] Performance targets met
- [ ] No known bugs or issues

**Testing Checklist**:
```
[] Autoware detection (all scenarios)
[] URDF parsing (sample_sensor_kit)
[] TF transform lookup (all sensors)
[] Coordinate conversion (all data)
[] Vehicle spawning (with sensors)
[] Vehicle cleanup (on Autoware loss)
[] Vehicle teleportation (RViz pose)
[] Camera data (4 cameras)
[] LiDAR data (4 LiDARs)
[] IMU data
[] GNSS data
[] Control commands (throttle/brake/steer)
[] Status feedback (velocity/steering)
[] Performance benchmarks
[] Memory leak test (1+ hour run)
```

## Acceptance harness (2026-08-29)

`scripts/acceptance.py`, or `just acceptance [scenario] [runs]`, runs a scenario against a
live stack and judges it. It does not build a stack: `just run` and `just ego-av` come first.

What it checks, and the failure each one is for:

| check | the failure it catches |
|---|---|
| the scenario's own `result.junit.xml` | what SSv2 says happened |
| the ego reached a peak speed and travelled a distance | a run that passes while the ego barely moves |
| no node's stderr holds a fatal error | `traffic_light_multi_camera_fusion` aborting at startup in every run for a fortnight |
| CARLA answers a real RPC before starting | a server that is alive, listening, and serving nobody (issue 017) |
| non-OK diagnostics are reported | the fifteen-node cascade behind issue 016 |

Diagnostics are reported but do not fail a run. A healthy stack carries one or two permanently
(`vehicle_door: The door status is unknown`), and a check that cries wolf gets switched off.

### Validated against a real regression

The dead-node check was run against the play_log of a run from before the play_launch quoting
fix, and against one from after:

```
current run,  dead nodes: 0
pre-fix run,  dead nodes: 1
   traffic_light_multi_camera_fusion: terminate called after throwing an instance of
     'rclcpp::exceptions::InvalidTopicNameError'
```

So it would have caught the bug that hid for two weeks, in the run where it first appeared.

### Traps it avoids, each having produced a wrong answer already

- **A despawned CARLA actor keeps reporting `is_alive`** and then returns garbage: position
  (0, 0), steering 5.7e28, handbrake true. That reads exactly like a stalled ego, and completed
  runs were scored as stalls because of it. Nothing here holds an actor handle across ticks.
- **A fresh CARLA client cannot read a synchronous world.** Its first `GetWorld()` waits for a
  snapshot that only a tick delivers. Everything that can come from ROS does.
- **`DiagnosticStatus.level` is a byte** in Humble's Python bindings, not an int. Formatting it
  with `%d` raises, after the run.

### What it does not do yet

- Lateral and longitudinal tracking quality are measured by `score_tracking.py` and
  `diagnose_gain.py`, and are not folded in. They need a baseline before they can be a
  threshold, and inventing one would make the suite fail on noise.
- It judges one stack configuration. The unmanaged ego (phase 013) needs its own domain and
  goal file, which the `--domain` flag allows but nothing exercises automatically.
- Nothing runs it on a schedule. It is a command, not CI.
