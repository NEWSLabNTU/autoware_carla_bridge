# Enhancements (Phases 7-8)

This document covers enhancement phases for carla-rust integration and architecture refactoring.

---

## Phase 7: carla-rust Integration and Enhancements

**Objective**: Leverage new APIs from local carla-rust repository to improve bridge functionality and robustness.

**Status**: 🔄 **IN PROGRESS** - Local carla-rust integrated (2025-10-29)

**Duration**: 1-2 weeks

### 7.1 Implement Proper Actor Cleanup

**Priority**: High

- [ ] Add `ActorBase::destroy()` to SensorBridge Drop implementation
  ```rust
  impl Drop for SensorBridge {
      fn drop(&mut self) {
          if self._actor.destroy() {
              log::info!("Destroyed sensor: {}", self._sensor_name);
          }
      }
  }
  ```
- [ ] Add `ActorBase::destroy()` to VehicleBridge Drop implementation
- [ ] Test actor cleanup with rapid spawn/despawn cycles
- [ ] Verify no resource leaks with CARLA monitor tools
- [ ] Document actor lifecycle management

**Benefits**:
- Explicit resource cleanup instead of relying on CARLA GC
- Faster actor removal from simulation
- Better memory management
- Clearer resource lifecycle

### 7.2 Implement Efficient World Loading

**Priority**: Medium

- [ ] Add world loading utility with `Client::load_world_if_different()`
  ```rust
  pub fn load_world_smart(client: &Client, map_name: &str) -> World {
      #[cfg(carla_0916)]
      {
          client.load_world_if_different(map_name)
      }
      #[cfg(not(carla_0916))]
      {
          client.load_world(map_name)
      }
  }
  ```
- [ ] Add CLI option for initial map selection
- [ ] Support map switching without bridge restart
- [ ] Test with different map combinations
- [ ] Benchmark loading time improvement vs. reload

**Benefits**:
- Avoids unnecessary map reloads (saves 5-10 seconds per switch)
- Smoother testing workflow
- Less CARLA server restarts needed

### 7.3 Add Debug Data Recording

**Priority**: Low

- [ ] Add `--debug-record` CLI flag
- [ ] Implement sensor data recording with `Sensor::save_to_disk()`
  ```rust
  if args.debug_record {
      sensor.save_to_disk(&format!("debug/{}_{}", vehicle_name, sensor_name));
  }
  ```
- [ ] Create debug output directory structure
- [ ] Add recording controls (start/stop via ROS service)
- [ ] Document debug recording usage

**Benefits**:
- Capture sensor data for offline debugging
- Reproduce issues without running full simulation
- Compare sensor outputs across CARLA versions

### 7.4 Multi-Version CARLA Support

**Priority**: Medium

- [ ] Add `CARLA_VERSION` variable to Makefile
  ```makefile
  CARLA_VERSION ?= 0.9.16
  ```
- [ ] Document version selection in README.md
- [ ] Create test scripts for each CARLA version
  - [ ] `scripts/test_carla_0914.sh`
  - [ ] `scripts/test_carla_0915.sh`
  - [ ] `scripts/test_carla_0916.sh`
- [ ] Test bridge with CARLA 0.9.14
- [ ] Test bridge with CARLA 0.9.15
- [ ] Test bridge with CARLA 0.9.16 (current)
- [ ] Document version-specific features and limitations
- [ ] Add CI/CD matrix testing for multiple versions

**Benefits**:
- Support users with different CARLA installations
- Test backward compatibility
- Identify version-specific issues early

### 7.5 Explore Advanced carla-rust APIs

**Priority**: Low (Research)

Investigate and document potential uses for:

- [ ] **Walker Control APIs**
  - Walker bone control for pedestrian simulation
  - Walker AI controller integration
- [ ] **Batch Operations**
  - Batch spawn/destroy for performance
  - Bulk command execution
- [ ] **Debug Visualization**
  - Add ROS markers for CARLA debug shapes
  - Visualize waypoints, bounding boxes in RViz
- [ ] **Recording/Playback**
  - CARLA native recording integration
  - Playback for deterministic testing
- [ ] **Traffic Light Extensions**
  - Traffic light group control
  - Traffic light state synchronization
- [ ] **Vehicle Telemetry**
  - Failure state monitoring
  - Advanced telemetry publishing

**Deliverables**:
- [ ] Research document: `docs/advanced-carla-apis.md`
- [ ] Proof-of-concept implementations in separate branch
- [ ] Recommendations for future integration

### 7.6 Update Documentation

- [ ] Update README.md with carla-rust integration notes
- [ ] Update CLAUDE.md with new API patterns
- [ ] Add troubleshooting section for multi-version builds
- [ ] Create examples for new features
- [ ] Update migration guide with carla-rust benefits

**Deliverables**:
- Updated documentation across all docs
- Example code demonstrating new APIs
- Version compatibility matrix

**Success Criteria**:
- ✅ Actor cleanup implemented and tested
- ✅ Smart world loading reduces unnecessary reloads
- ✅ Debug recording available for troubleshooting
- ✅ Bridge tested with at least 2 CARLA versions
- ✅ Documentation updated with new features
- ✅ No performance regression from new features

**Risks**:
- Version-specific APIs may require conditional compilation
- Debug recording may impact performance
- Multi-version testing increases CI/CD complexity

---

## Phase 8: Architecture Refactoring - 1-to-1 Design

**Objective**: Refactor bridge architecture to implement Autoware-centric 1-to-1 design where one bridge instance manages exactly one CARLA vehicle.

**Status**: ⏳ **PENDING** - Depends on Phase 2/3 runtime validation

**Duration**: 1-2 weeks

**Prerequisites**:
- Phase 2 runtime testing complete
- Phase 3 sensor testing complete (at least Camera, LiDAR, IMU, GNSS)
- Understanding of current multi-vehicle architecture
- Review [architecture.md](architecture.md) for design details

### 8.1 Remove Simulation Control

**Priority**: High

**Objective**: Remove all simulation control logic from bridge. Scenario scripts will control CARLA ticking.

**Tasks**:
- [ ] Remove tick thread spawn (main.rs:97-106)
- [ ] Remove `tick` CLI parameter from Opts struct
- [ ] Remove `slowdown` CLI parameter from Opts struct
- [ ] Remove synchronous mode configuration (main.rs:83-86)
  ```rust
  // DELETE:
  carla_settings.synchronous_mode = true;
  carla_settings.fixed_delta_seconds = Some(tick_ms * 0.001);
  world.apply_settings(&carla_settings);
  ```
- [ ] Keep SimulatorClock but remove tick publishing responsibility
- [ ] Update SimulatorClock to only publish on world.wait_for_tick()
- [ ] Remove thread spawning and sleep logic
- [ ] Update error messages to not reference tick parameters

**Testing**:
- [ ] Verify bridge compiles without tick parameters
- [ ] Create test scenario script that controls ticking
- [ ] Verify bridge waits for external ticks

**Benefits**:
- Cleaner separation of concerns
- Bridge becomes passive adapter
- Easier to coordinate simulation with external scenarios
- No risk of bridge tick conflicts with scenario logic

### 8.2 Add Vehicle Selection

**Priority**: High

**Objective**: Add CLI arguments to select specific vehicle by name or ID.

**Tasks**:
- [ ] Add `--vehicle-name` parameter to Opts
  ```rust
  /// Vehicle role_name to bridge (e.g., "ego_vehicle")
  #[clap(long, conflicts_with = "vehicle_id")]
  pub vehicle_name: Option<String>,
  ```
- [ ] Add `--vehicle-id` parameter to Opts
  ```rust
  /// Vehicle actor ID to bridge
  #[clap(long, conflicts_with = "vehicle_name")]
  pub vehicle_id: Option<u32>,
  ```
- [ ] Add `--vehicle-wait-timeout` parameter
  ```rust
  /// Timeout in seconds to wait for vehicle (0 = wait forever)
  #[clap(long, default_value_t = 30)]
  pub vehicle_wait_timeout: u64,
  ```
- [ ] Validate that at least one of vehicle_name or vehicle_id is provided
- [ ] Create `find_target_vehicle()` function
  ```rust
  fn find_target_vehicle(
      world: &World,
      opts: &Opts,
      timeout: Duration,
  ) -> Result<Vehicle>
  ```
- [ ] Implement polling loop with timeout
- [ ] Add logging for vehicle search progress

**Testing**:
- [ ] Test `--vehicle-name` with exact match
- [ ] Test `--vehicle-id` with actor ID
- [ ] Test timeout behavior (vehicle not found)
- [ ] Test vehicle appearing after bridge starts
- [ ] Verify error messages are clear

**Benefits**:
- Explicit vehicle selection
- Support both role_name and actor ID matching
- Flexible timing (vehicle can spawn before or after bridge)

### 8.3 Simplify Actor Management

**Priority**: High

**Objective**: Remove multi-vehicle discovery and management. Track only the selected ego vehicle.

**Tasks**:
- [ ] Replace `HashMap<ActorId, Box<dyn ActorBridge>>` with `EgoBridges` struct
  ```rust
  struct EgoBridges {
      vehicle: VehicleBridge,
      sensors: Vec<SensorBridge>,
  }
  ```
- [ ] Replace `HashMap<String, Autoware>` with single `Autoware` instance
- [ ] Remove actor discovery diff logic (added_ids, deleted_ids)
- [ ] Remove "autoware_" prefix filtering from vehicle_bridge.rs
- [ ] Remove "autoware_" prefix filtering from sensor_bridge.rs
- [ ] Implement exact vehicle matching in `is_target_vehicle()`
- [ ] Filter sensors by parent vehicle ID
  ```rust
  fn is_ego_sensor(sensor: &Sensor, ego_id: ActorId) -> bool {
      sensor.parent().map_or(false, |p| p.id() == ego_id)
  }
  ```
- [ ] Update get_bridge_type() to not require prefix matching
- [ ] Remove BridgeError::Npc variant (no longer needed)

**Testing**:
- [ ] Verify only ego vehicle is bridged
- [ ] Verify only ego's sensors are bridged
- [ ] Verify other vehicles are ignored
- [ ] Test with multiple vehicles in CARLA (should only bridge one)

**Benefits**:
- Simpler code (no multi-vehicle tracking)
- Clear single-vehicle focus
- No NPC filtering needed

### 8.4 Root Namespace Topics

**Priority**: High

**Objective**: Publish to standard Autoware topic names without vehicle prefixes.

**Tasks**:
- [ ] Modify `Autoware::new()` to remove vehicle_name parameter
  ```rust
  impl Autoware {
      pub fn new() -> Autoware {
          Autoware {
              topic_actuation_status: "/vehicle/status/actuation_status".to_string(),
              topic_velocity_status: "/vehicle/status/velocity_status".to_string(),
              // ... all topics with root namespace
          }
      }
  }
  ```
- [ ] Update all vehicle status topics to root namespace
- [ ] Update all control command topics to root namespace
- [ ] Update sensor topics to root namespace pattern
  ```rust
  format!("/sensing/camera/{}/image_raw", sensor_name)
  format!("/sensing/lidar/{}/pointcloud", sensor_name)
  format!("/sensing/imu/{}/imu_raw", sensor_name)
  format!("/sensing/gnss/{}/nav_sat_fix", sensor_name)
  ```
- [ ] Update clock topic to "/clock" (no prefix)
- [ ] Remove vehicle_name field from Autoware struct
- [ ] Update all topic creation in vehicle_bridge.rs
- [ ] Update all topic creation in sensor_bridge.rs

**Testing**:
- [ ] Verify topics with `ros2 topic list`
- [ ] Should see `/vehicle/status/...` not `/autoware_v1/vehicle/status/...`
- [ ] Should see `/sensing/...` not `/autoware_v1/sensing/...`
- [ ] Test with unmodified Autoware launch files

**Benefits**:
- Works with standard Autoware topic names
- No launch file modifications needed
- Clean, predictable topic structure

### 8.5 Vehicle Respawn Handling

**Priority**: Medium

**Objective**: Detect when ego vehicle is destroyed and wait for it to respawn.

**Tasks**:
- [ ] Add vehicle alive check in main loop
  ```rust
  if !ego_vehicle.is_alive() {
      log::warn!("Ego vehicle destroyed, waiting for respawn...");
      ego_vehicle = find_target_vehicle(&world, &opts, timeout)?;
      ego_bridges = create_ego_bridges(&ego_vehicle, &node)?;
      log::info!("Ego vehicle respawned, bridges recreated");
  }
  ```
- [ ] Implement `Actor::is_alive()` check
- [ ] Drop old bridges before creating new ones
- [ ] Re-register sensor callbacks after respawn
- [ ] Add logging for vehicle lifecycle events
- [ ] Test with rapid destroy/respawn cycles

**Testing**:
- [ ] Manually destroy vehicle in CARLA
- [ ] Verify bridge detects destruction
- [ ] Respawn vehicle with same role_name
- [ ] Verify bridge recreates connections
- [ ] Test sensor data flows after respawn

**Benefits**:
- Robust to vehicle destruction
- Supports scenario resets
- No need to restart bridge for vehicle respawn

### 8.6 Main Loop Refactoring

**Priority**: High

**Objective**: Simplify main loop to focus on single ego vehicle.

**Tasks**:
- [ ] Remove actor discovery loop
- [ ] Remove bridge_list HashMap iteration
- [ ] Simplify to single vehicle step:
  ```rust
  loop {
      // Check vehicle alive
      if !ego_vehicle.is_alive() {
          handle_respawn();
      }

      // Step ego vehicle
      ego_bridges.vehicle.step(timestamp)?;
      // Sensors use callbacks, no step needed

      // Wait for next tick
      world.wait_for_tick();
  }
  ```
- [ ] Remove added_ids/deleted_ids computation
- [ ] Keep timeout detection logic
- [ ] Update error handling for simpler flow
- [ ] Add clear logging for main loop events

**Testing**:
- [ ] Verify main loop is simpler and clearer
- [ ] Verify no performance regression
- [ ] Test with long-running scenarios
- [ ] Monitor CPU/memory usage

**Benefits**:
- Cleaner, easier to understand code
- Fewer moving parts
- More predictable behavior

### 8.7 Create Scenario Scripts

**Priority**: Medium

**Objective**: Create example scenario scripts that demonstrate simulation control.

**Tasks**:
- [ ] Create `scripts/scenario/` directory
- [ ] Create `scripts/scenario/README.md`
- [ ] Create `scripts/scenario/simple_drive.py`:
  - Configure synchronous mode
  - Spawn vehicle with role_name
  - Run tick loop at 20 Hz
  - Handle Ctrl-C cleanup
- [ ] Create `scripts/scenario/multi_vehicle.py`:
  - Spawn multiple vehicles with different role_names
  - Demonstrate multiple bridge instances with domains
- [ ] Create `scripts/scenario/vehicle_respawn.py`:
  - Test vehicle destruction and respawn
  - Demonstrate bridge resilience
- [ ] Document scenario script API patterns
- [ ] Add scenario templates for common testing patterns

**Testing**:
- [ ] Run each scenario script
- [ ] Verify CARLA responds correctly
- [ ] Test with bridge in different states
- [ ] Document expected behavior

**Benefits**:
- Clear examples of simulation control
- Testing templates
- Documentation for users

### 8.8 Testing & Validation

**Priority**: High

**Objective**: Comprehensive testing of refactored architecture.

**Tasks**:
- [ ] Test vehicle selection by name with exact match
- [ ] Test vehicle selection by actor ID
- [ ] Test vehicle wait timeout (vehicle not spawned)
- [ ] Test vehicle appearing after bridge starts
- [ ] Test vehicle respawn after destruction
- [ ] Test root namespace topics with Autoware
- [ ] Test multi-vehicle simulation:
  - Run two bridges in different domains
  - Verify topic isolation
  - Test with two Autoware instances
- [ ] Performance testing:
  - Measure CPU usage (single vehicle)
  - Measure memory usage
  - Compare with old multi-vehicle code
- [ ] Update `scripts/run_test_env.sh` for new CLI arguments
- [ ] Update test documentation
- [ ] Add integration tests for new features

**Expected Results**:
- Bridge connects to specific vehicle only
- External scenario controls ticking
- Topics use root namespace
- Vehicle respawn works smoothly
- Multiple bridges work in separate domains
- No performance regression

**Documentation Updates**:
- [ ] Update README.md with new CLI arguments
- [ ] Update scripts/README.md with new patterns
- [ ] Document multi-vehicle setup with domains
- [ ] Update CLAUDE.md with architecture changes
- [ ] Create migration guide for users of old version

### 8.9 TF Publisher Implementation (Optional)

**Priority**: Low

**Objective**: Publish TF tree for sensor transforms.

**Tasks**:
- [ ] Create `src/tf_publisher.rs`
- [ ] Read sensor transforms from CARLA
- [ ] Publish base_link → sensor_link transforms
- [ ] Update on vehicle/sensor changes
- [ ] Test with Autoware localization

**Benefits**:
- Required for proper Autoware sensor fusion
- Correct spatial relationships between sensors

**Deliverables**:
- [ ] Refactored bridge with 1-to-1 architecture
- [ ] Vehicle selection CLI arguments
- [ ] Root namespace topic publishing
- [ ] Vehicle respawn handling
- [ ] Scenario script examples
- [ ] Updated documentation
- [ ] Test results and validation

**Success Criteria**:
- ✅ Bridge manages exactly one vehicle
- ✅ External scenario script controls simulation
- ✅ Topics use root namespace (standard Autoware)
- ✅ Vehicle selection works by name and ID
- ✅ Vehicle respawn is handled gracefully
- ✅ Multiple bridges run successfully in separate domains
- ✅ Code is simpler and easier to maintain
- ✅ No performance regression
- ✅ Documentation is updated and clear

**Risks**:
- Breaking changes require migration guide
- Users need to update launch workflows
- Testing with actual Autoware may reveal edge cases

**Mitigation**:
- Comprehensive testing before merging
- Clear migration documentation
- Maintain backward compatibility flag if needed

---

**Document Version**: 1.0
**Last Updated**: 2025-11-02
**Related Documents**: [roadmap.md](../roadmap.md), [architecture.md](architecture.md), [meta.md](meta.md)
