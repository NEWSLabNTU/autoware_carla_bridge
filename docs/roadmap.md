# autoware_carla_bridge Migration Roadmap

This document outlines the phased approach for migrating `zenoh_carla_bridge` to `autoware_carla_bridge` using rclrs (ROS 2 Rust client library).

## Table of Contents

- [Project Overview](#project-overview)
- [Migration Goals](#migration-goals)
- [Architecture Design Decisions](#architecture-design-decisions)
- [Prerequisites](#prerequisites)
- [Phase 0: Preparation](#phase-0-preparation)
- [Phase 1: Core Infrastructure](#phase-1-core-infrastructure)
- [Phase 2: Clock and Simple Publishers](#phase-2-clock-and-simple-publishers)
- [Phase 3: Sensor Bridge Migration](#phase-3-sensor-bridge-migration)
- [Phase 4: Vehicle Bridge Migration](#phase-4-vehicle-bridge-migration)
- [Phase 5: Testing and Optimization](#phase-5-testing-and-optimization)
- [Phase 6: Documentation and Release](#phase-6-documentation-and-release)
- [Phase 7: carla-rust Integration and Enhancements](#phase-7-carla-rust-integration-and-enhancements)
- [Phase 8: Architecture Refactoring - 1-to-1 Design](#phase-8-architecture-refactoring---1-to-1-design)
- [Architecture Decision Record](#architecture-decision-record)
- [Success Criteria](#success-criteria)
- [Risk Management](#risk-management)

## Project Overview

**Current State**: `zenoh_carla_bridge` uses Zenoh to bridge CARLA simulator data to ROS 2 systems through multiple modes (DDS, ROS2, RmwZenoh).

**Target State**: `autoware_carla_bridge` will use rclrs to publish CARLA data directly as native ROS 2 topics, eliminating the need for intermediate bridges.

**Timeline**: Estimated 4-6 weeks for complete migration and testing.

## Migration Goals

- ✅ Replace Zenoh with native ROS 2 communication (rclrs)
- ✅ Maintain compatibility with Autoware 2025.22
- ✅ Simplify codebase by removing bridge-specific logic
- ✅ Improve type safety with compile-time message type checking
- ✅ Achieve comparable or better performance
- ✅ Provide comprehensive testing and documentation

## Prerequisites

Before starting the migration, ensure:

- [ ] CARLA 0.9.15 is installed and working
- [ ] ROS 2 Humble is installed
- [ ] Autoware 2025.22 is installed (for ROS message types)
- [ ] Rust toolchain is configured (rustc, cargo)
- [ ] LLVM/Clang 12 is installed for CARLA Rust bindings
- [ ] `carla_agent` vehicle spawning tool is working
- [ ] `external/autoware` symlink points to Autoware installation
- [ ] Familiarity with both Zenoh and rclrs APIs (see `docs/zenoh-to-rclrs-api-comparison.md`)

---

## Architecture Design Decisions

### 1-to-1 Autoware-Centric Design Philosophy

**Decision Date**: 2025-10-31

The bridge architecture follows a **1-to-1 mapping** between CARLA vehicles and Autoware instances:

```
CARLA Vehicle ↔ Bridge Instance ↔ Autoware Instance
```

This design choice aligns with Autoware's single-vehicle autonomous driving architecture and provides clean isolation between multiple vehicles in simulation.

### Core Principles

#### 1. One Bridge Per Vehicle

Each bridge instance manages exactly one CARLA vehicle (the "ego vehicle") and all its attached sensors. To simulate multiple vehicles:

- Spawn multiple bridge processes (one per vehicle)
- Use `ROS_DOMAIN_ID` for isolation (prevents topic name conflicts)
- Each bridge connects to the same CARLA server but manages different vehicles

**Benefits**:
- ✅ Aligns with Autoware's single-vehicle design (no multi-vehicle awareness needed)
- ✅ Standard ROS 2 domain isolation (no hacks or workarounds)
- ✅ No topic naming conflicts (domains are isolated)
- ✅ Works with Autoware's hardcoded absolute topic names
- ✅ Fault isolation (one bridge crash doesn't affect others)
- ✅ Simple to understand, test, and debug
- ✅ Natural horizontal scaling (add vehicles = add bridges)

#### 2. Bridge is Passive Adapter

The bridge does **NOT** control simulation execution. Simulation control is the responsibility of external scenario scripts.

**Bridge Responsibilities**:
- ✅ Connect to CARLA (passive mode)
- ✅ Find and track assigned ego vehicle
- ✅ Bridge sensor data (CARLA → ROS 2)
- ✅ Bridge vehicle control (ROS 2 → CARLA)
- ✅ Publish TF tree for sensor transforms
- ✅ Wait for and respond to simulation ticks

**NOT Bridge Responsibilities**:
- ❌ Configure CARLA synchronous mode settings
- ❌ Call `world.tick()` to advance simulation
- ❌ Control simulation time or speed
- ❌ Manage scenario logic or vehicle spawning

**External Scenario Script Responsibilities**:
- Configure CARLA synchronous_mode and fixed_delta_seconds
- Spawn vehicles with appropriate role_name attributes
- Control simulation ticking (call world.tick() in loop)
- Manage scenario progression and vehicle lifecycle

#### 3. Root Namespace Topics

The bridge publishes to **standard Autoware topic names** without vehicle-specific prefixes:

```
/vehicle/status/velocity_status
/vehicle/status/actuation_status
/sensing/camera/traffic_light/image_raw
/sensing/lidar/top/pointcloud
/sensing/imu/tamagawa/imu_raw
/sensing/gnss/ublox/nav_sat_fix
/control/command/actuation_cmd
/clock
```

This allows Autoware to run unmodified with its hardcoded absolute topic names. When running multiple vehicles:
- Each bridge runs in a separate ROS domain (`ROS_DOMAIN_ID=0`, `ROS_DOMAIN_ID=1`, etc.)
- Topics within each domain use standard names
- Domains provide isolation, preventing conflicts

#### 4. Vehicle Discovery and Selection

The bridge must be told which CARLA vehicle to manage:

```bash
# Select by role_name
ros2 run autoware_carla_bridge autoware_carla_bridge --vehicle-name ego_vehicle

# Or select by actor ID
ros2 run autoware_carla_bridge autoware_carla_bridge --vehicle-id 42
```

**Vehicle Lifecycle**:
1. **Startup**: Bridge polls CARLA for target vehicle (with configurable timeout)
2. **Not Found**: Wait up to `--vehicle-wait-timeout` seconds (default: 30s)
3. **Found**: Create bridges for vehicle + all attached sensors
4. **Running**: Monitor vehicle existence each tick
5. **Destroyed**: Detect vehicle destruction, wait for respawn
6. **Respawn**: Recreate bridges when vehicle reappears

This design allows flexible scenario scripting:
- Scenario can spawn vehicle before or after bridge starts
- Vehicle can be destroyed and recreated (testing reset scenarios)
- Bridge remains robust to vehicle lifecycle changes

### Startup Sequence

**Revised sequence** for 1-to-1 Autoware-centric design:

1. **Parse Command Line Arguments**
   - Vehicle selector: `--vehicle-name` (role_name) or `--vehicle-id` (actor ID)
   - Wait timeout: `--vehicle-wait-timeout` (default: 30s, 0 = wait forever)
   - CARLA connection: `--carla-host`, `--carla-port`

2. **Initialize ROS 2 Infrastructure**
   - Create rclrs Context, Executor, Node
   - Node name: `autoware_carla_bridge`
   - No ROS domain configuration (use `ROS_DOMAIN_ID` environment variable)

3. **Connect to CARLA (Passive Mode)**
   - Connect client to CARLA server
   - Get world handle
   - **Do NOT configure synchronous mode** (scenario script's responsibility)
   - **Do NOT spawn tick thread** (scenario script controls ticking)

4. **Find Target Ego Vehicle**
   - Search CARLA world for vehicle matching selector
   - If not found: poll every 1 second until found or timeout
   - Error and exit on timeout
   - Log when ego vehicle is found

5. **Create Vehicle Bridge**
   - Create single `VehicleBridge` for ego vehicle
   - Subscribe to Autoware control commands (root namespace)
   - Publish vehicle status (root namespace)

6. **Discover and Bridge Sensors**
   - Find all sensors attached to ego vehicle (filter by parent ID)
   - Create `SensorBridge` for each (Camera, LiDAR, IMU, GNSS)
   - Register CARLA sensor callbacks
   - Publish to root namespace sensor topics

7. **Publish TF Tree** (TODO - Phase 8)
   - Read sensor transforms from CARLA
   - Publish base_link → sensor_link transforms
   - Required for Autoware localization and sensor fusion

8. **Publish /clock Topic**
   - Create clock publisher for CARLA simulation time
   - Even though we don't control ticking, Autoware needs sim time

9. **Enter Main Loop**
   - Check if ego vehicle still exists each iteration
   - If destroyed: wait for respawn (go back to step 4)
   - If alive: call `ego_vehicle_bridge.step(timestamp)`
   - Wait for next tick: `world.wait_for_tick()`
   - No multi-vehicle actor discovery needed

### Multi-Vehicle Simulation Example

```bash
# Terminal 1: CARLA simulator
systemctl --user start carla-0.9.16@3000

# Terminal 2: Scenario script (controls simulation)
python scripts/scenario/multi_vehicle_scenario.py
# - Configures synchronous_mode = True
# - Spawns vehicle_1 (role_name="ego_vehicle_1")
# - Spawns vehicle_2 (role_name="ego_vehicle_2")
# - Runs tick loop at 20 Hz

# Terminal 3: Bridge for vehicle 1 (domain 0)
ROS_DOMAIN_ID=0 ros2 run autoware_carla_bridge autoware_carla_bridge \
  --vehicle-name ego_vehicle_1 \
  --carla-port 3000

# Terminal 4: Bridge for vehicle 2 (domain 1)
ROS_DOMAIN_ID=1 ros2 run autoware_carla_bridge autoware_carla_bridge \
  --vehicle-name ego_vehicle_2 \
  --carla-port 3000

# Terminal 5: Autoware instance 1
ROS_DOMAIN_ID=0 ros2 launch autoware_launch autoware.launch.xml

# Terminal 6: Autoware instance 2
ROS_DOMAIN_ID=1 ros2 launch autoware_launch autoware.launch.xml

# Terminal 7: Monitor vehicle 1 topics
ROS_DOMAIN_ID=0 ros2 topic list

# Terminal 8: Monitor vehicle 2 topics
ROS_DOMAIN_ID=1 ros2 topic list
```

Each domain is completely isolated - topics, nodes, and services don't leak between domains.

### Alternative Designs Considered

#### Option 1: Multi-Vehicle Bridge with Domain Workarounds (❌ Rejected)

One bridge instance publishes topics for all vehicles into multiple ROS domains.

**Problems**:
- ❌ No standard way for one ROS node to exist in multiple domains
- ❌ Would require rclrs hacks or multiple context instances (non-standard)
- ❌ Debugging nightmare (one process doing multiple things)
- ❌ Tight coupling between vehicle lifecycles
- ❌ Resource contention between vehicles
- ❌ Complex implementation for minimal benefit

#### Option 2: Topic Namespacing Instead of Domains (❌ Rejected)

Use topic prefixes (`/vehicle_1/...`, `/vehicle_2/...`) instead of domain isolation.

**Problems**:
- ❌ Requires modifying Autoware launch files (brittle, maintenance burden)
- ❌ Autoware has many hardcoded absolute topic names
- ❌ No isolation at node/service level (only topics)
- ❌ More complex configuration
- ❌ Less clean than standard domain separation

#### Option 3: 1-to-1 Bridge-Vehicle Design (✅ Selected)

**Advantages**:
- ✅ Simple and robust
- ✅ Standard ROS 2 patterns (domain isolation)
- ✅ Minimal code changes needed
- ✅ Mirrors real-world deployment (one Autoware per vehicle)
- ✅ Easy to test and debug
- ✅ Works with unmodified Autoware
- ✅ Natural fault isolation

### Consequences and Trade-offs

**Consequences of 1-to-1 Design**:

**Positive**:
- Clean separation of concerns
- Simple mental model (one bridge = one vehicle)
- Standard ROS tooling works as expected
- Easy to add/remove vehicles dynamically
- No cross-talk between vehicles
- Independent resource management

**Trade-offs**:
- Need multiple bridge processes for multi-vehicle
  - Mitigation: This is how real-world deployment works anyway
- Slightly more resource usage (N processes vs 1)
  - Mitigation: Rust bridges are lightweight (~10-20 MB RAM each)
- Need to manage ROS_DOMAIN_ID environment variable
  - Mitigation: Standard ROS 2 practice, well-documented

**Overall**: Trade-offs are minimal, benefits are substantial.

---

## Phase 0: Preparation

**Objective**: Set up the development environment and understand the current codebase.

**Duration**: 3-5 days

### Tasks

- [x] ✅ Study Zenoh API usage in current codebase
- [x] ✅ Study rclrs API and examples
- [x] ✅ Document API differences and migration strategy
- [x] ✅ Set up Autoware environment
  - Symlink created: `src/external/autoware` → `/home/aeon/repos/autoware/2025.02-ws`
  - Autoware workspace configured and accessible
- [x] ✅ Review and understand all bridge types:
  - [x] ✅ `sensor_bridge.rs` (Camera, LiDAR, IMU, GNSS)
  - [x] ✅ `vehicle_bridge.rs` (Control, status, velocity)
  - [x] ✅ `trafficlight_bridge.rs`
  - [x] ✅ `trafficsign_bridge.rs`
  - [x] ✅ `other_bridge.rs`
- [x] ✅ Set up colcon workspace and three-stage build system
- [x] ✅ Create package.xml and launch file for ROS 2 integration

**Deliverables**:
- [x] ✅ `docs/zenoh-to-rclrs-api-comparison.md`
- [x] ✅ `docs/roadmap.md` (this document)
- [x] ✅ `docs/message-type-migration.md`
- [x] ✅ `docs/carla-rust-integration.md`
- [x] ✅ Autoware environment configured (symlink created)
- [x] ✅ Colcon workspace structure created
- [x] ✅ Three-stage build system implemented

**Success Criteria**:
- ✅ All team members understand migration approach
- ✅ Build system configured and working
- ✅ Autoware workspace accessible

---

## Phase 1: Core Infrastructure

**Objective**: Replace Zenoh session with rclrs context and node, update dependencies.

**Duration**: 1 week

### 1.1 Understand ROS Message Type Provision

**Reference**: See `docs/message-type-migration.md` for comprehensive guide on migrating from zenoh-ros-type to rclrs interface packages.

- [x] ✅ Study how rclrs provides ROS message types:
  - ✅ rclrs generates `.cargo/config.toml` to link ROS interface packages
  - ✅ Message types (sensor_msgs, std_msgs, etc.) are sourced from colcon workspace
  - ✅ Three-stage build system implemented with rosidl_generator_rs
  - ✅ direnv configured for automatic environment sourcing

- [x] ✅ Verify ROS message availability:
  ```bash
  # Source Autoware environment
  source external/autoware/install/setup.bash

  # Check available message packages
  ros2 interface package sensor_msgs
  ros2 interface package autoware_vehicle_msgs
  ros2 interface package tier4_vehicle_msgs

  # Check specific message definitions
  ros2 interface show sensor_msgs/msg/Image
  ros2 interface show autoware_vehicle_msgs/msg/VelocityReport
  ```
  ✅ Message availability verified through colcon build system

- [x] ✅ Test rclrs message type access:
  - ✅ Created `examples/test_message_types.rs` for verification
  - ✅ Three-stage build generates all required message crates

- [x] ✅ Examine generated `.cargo/config.toml`:
  - ✅ Generated at project root with ROS message patches
  - ✅ Contains paths to all required message crates (50+ packages)
  - ✅ Verified patches include: std_msgs, sensor_msgs, geometry_msgs, autoware_vehicle_msgs, tier4_vehicle_msgs, tier4_control_msgs

### 1.2 Update Dependencies

- [x] ✅ Update `Cargo.toml`:
  - ✅ Removed all zenoh dependencies (zenoh, zenoh-ros-type)
  - ✅ Added rclrs with wildcard version
  - ✅ Added ROS 2 message package dependencies (wildcards resolved via .cargo/config.toml)
  - ✅ Updated carla to 0.12.0 with local path dependency: `{ version = "0.12.0", path = "../../../carla-rust/carla" }`
  - ✅ Kept all utility crates (arc-swap, atomic_float, clap, etc.)

- [x] ✅ Remove Zenoh-specific crates from dependencies
  - ✅ All zenoh and zenoh-ros-type references removed
  - ✅ Only repository URL contains "zenoh" (harmless)

- [x] ✅ Build with colcon build system:
  - ✅ Three-stage build implemented in Makefile
  - ✅ direnv configured for automatic environment
  - ✅ Successfully built on 2025-10-29

- [x] ✅ Verify `.cargo/config.toml` is generated with ROS message paths
  - ✅ Generated with 50+ message package patches
  - ✅ All required message types accessible

### 1.3 Update Main Entry Point

File: `src/main.rs`

- [x] ✅ Remove Zenoh imports:
  - ✅ All `zenoh::` imports removed
  - ✅ Replaced with `use rclrs::CreateBasicExecutor;`

- [x] ✅ Add rclrs imports:
  - ✅ `use rclrs::CreateBasicExecutor;`
  - ✅ Standard Arc and threading imports retained

- [x] ✅ Remove `Mode` enum - no longer needed
  - ✅ Mode enum completely removed
  - ✅ No mode-specific logic remains

- [x] ✅ Remove Zenoh config and listen endpoints from CLI arguments
  - ✅ Removed: `zenoh_listen`, `mode`, `zenoh_config`
  - ✅ Kept: `carla_address`, `carla_port`, `tick`, `slowdown`

- [x] ✅ Update `Opts` struct:
  - ✅ Simplified to CARLA-only parameters
  - ✅ Clean CLI interface with no Zenoh references

- [x] ✅ Replace Zenoh session initialization with rclrs:
  ```rust
  // Implemented:
  let ctx = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?;
  let executor = ctx.create_basic_executor();
  let node = executor.create_node("autoware_carla_bridge")?;
  ```

- [x] ✅ Update function signatures to pass `rclrs::Node` (not Arc - Node is Arc internally)

- [x] ✅ Add ROS 2 executor:
  - ✅ Basic executor created with `create_basic_executor()`
  - ✅ Node created from executor

### 1.4 Update Bridge Infrastructure

Files: `src/bridge/actor_bridge.rs`, `src/bridge/mod.rs`

- [x] ✅ Update `create_bridge()` signature to accept `rclrs::Node`
  - ✅ Changed from `Arc<Session>` to `rclrs::Node`
  - ✅ Node is `Arc<NodeState>` internally, cheap to clone

- [x] ✅ Update all bridge constructors to accept `rclrs::Node`:
  - ✅ SensorBridge::new()
  - ✅ VehicleBridge::new()
  - ✅ TrafficLightBridge::new()
  - ✅ TrafficSignBridge::new()
  - ✅ OtherActorBridge::new()

- [x] ✅ Remove `mode` parameter from all bridge-related functions
  - ✅ No mode parameter anywhere in codebase
  - ✅ Simplified bridge creation logic

- [x] ✅ Update `ActorBridge` trait:
  - ✅ Trait interfaces updated for rclrs
  - ✅ No breaking changes to external API

### 1.5 Remove Mode-Specific Logic

Files: `src/autoware.rs`, `src/utils.rs`

- [x] ✅ Delete `Mode` enum - completely removed
- [x] ✅ Remove `setup_topics()` function - deleted
- [x] ✅ Remove `declare_node_liveliness()` - deleted
- [x] ✅ Remove `declare_topic_liveliness()` - deleted
- [x] ✅ Remove `undeclare_all_liveliness()` - deleted
- [x] ✅ Remove `format_topic_key()` - replaced with simple string concatenation
- [x] ✅ Remove `generate_attachment()` function - deleted
- [x] ✅ Remove `put_with_attachment!` macro - deleted

**Code Reduction**:
- ✅ autoware.rs: 336 → 134 lines (-60% reduction)
- ✅ sensor_bridge.rs: 773 → 629 lines (-19% reduction)
- ✅ ~300 lines removed, ~500 lines modified
- ✅ All Zenoh complexity eliminated

**Deliverables**:
- [x] ✅ Understanding of rclrs ROS message type provision mechanism
- [x] ✅ Updated `Cargo.toml` with rclrs dependencies
- [x] ✅ Updated `main.rs` with rclrs initialization
- [x] ✅ Cleaned up mode-specific logic
- [x] ✅ Generated `.cargo/config.toml` with ROS message paths
- [x] ✅ Code compiles and passes lint checks

**Success Criteria**:
- ✅ `make build` succeeds with direnv environment
- ✅ No Zenoh dependencies remain (only in repo URL)
- ✅ rclrs node created successfully
- ✅ ROS message types accessible from colcon workspace
- ✅ Binary built: 9.3 MB at `install/autoware_carla_bridge/lib/`

---

## Phase 2: Clock and Simple Publishers

**Objective**: Migrate the simplest publisher (clock) as a proof of concept.

**Status**: ✅ **COMPLETE** - Code migration (2025-10-22), Runtime verified (2025-10-31)

**Duration**: N/A (integrated with Phase 1, testing completed separately)

### 2.1 Migrate Clock Publisher

File: `src/clock.rs`

- [x] ✅ Review current Zenoh implementation
- [x] ✅ Refactor to use rclrs:
  ```rust
  pub struct SimulatorClock {
      publisher_clock: Arc<rclrs::Publisher<builtin_interfaces::msg::Time>>,
  }
  ```
- [x] ✅ Update `new()` function:
  ```rust
  pub fn new(node: rclrs::Node) -> Result<SimulatorClock> {
      let publisher_clock = node.create_publisher("/clock")?;
      Ok(SimulatorClock {
          publisher_clock: Arc::new(publisher_clock),
      })
  }
  ```
- [x] ✅ Update `publish_clock()` function:
  ```rust
  pub fn publish_clock(&self, timestamp: Option<f64>) -> Result<()> {
      let time = if let Some(sec) = timestamp {
          builtin_interfaces::msg::Time {
              sec: sec.floor() as i32,
              nanosec: (sec.fract() * 1_000_000_000_f64) as u32,
          }
      } else {
          let now = SystemTime::now()
              .duration_since(UNIX_EPOCH)
              .expect("Unable to get current time");
          builtin_interfaces::msg::Time {
              sec: now.as_secs() as i32,
              nanosec: now.subsec_nanos(),
          }
      };
      self.publisher_clock.publish(time)?;
      Ok(())
  }
  ```
- [x] ✅ Remove CDR serialization
- [x] ✅ Remove attachment logic
- [x] ✅ Remove mode handling

### 2.2 Update Utility Functions

File: `src/utils.rs`

- [x] ✅ Keep `is_bigendian()` function
- [x] ✅ Update `create_ros_header()` to work with rclrs message types:
  ```rust
  pub fn create_ros_header(timestamp: Option<f64>) -> std_msgs::msg::Header {
      let time = if let Some(sec) = timestamp {
          builtin_interfaces::msg::Time {
              sec: sec.floor() as i32,
              nanosec: (sec.fract() * 1_000_000_000_f64) as u32,
          }
      } else {
          let now = SystemTime::now()
              .duration_since(UNIX_EPOCH)
              .expect("Unable to get current time");
          builtin_interfaces::msg::Time {
              sec: now.as_secs() as i32,
              nanosec: now.subsec_nanos(),
          }
      };
      std_msgs::msg::Header {
          stamp: time,
          frame_id: "".to_string(),
      }
  }
  ```

### 2.3 Test Clock Publisher

- [x] ✅ Build the project: `make build`
- [x] ✅ Start CARLA simulator (port 3000)
- [x] ✅ Run the bridge: `make run` (connects to CARLA on port 3000)
- [x] ✅ In another terminal, verify clock topic:
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 topic list | grep clock
  ros2 topic echo /clock
  ros2 topic hz /clock
  ```
- [x] ✅ Verify clock publishes successfully *(verified 2025-10-31)*

**Deliverables**:
- [x] ✅ Working clock publisher using rclrs
- [x] ✅ Updated utility functions
- [x] ✅ **Verification test results - PASSED** *(2025-10-31)*

**Success Criteria**:
- ✅ Clock publisher compiles and integrates with bridge
- ✅ No Zenoh dependencies or CDR serialization
- ✅ Clock topic appears in `ros2 topic list` - **VERIFIED**
- ✅ Clock messages publish successfully - **VERIFIED**
- ✅ Bridge connects to CARLA and runs - **VERIFIED**

---

## Phase 3: Sensor Bridge Migration

**Objective**: Migrate all sensor publishers (camera, LiDAR, IMU, GNSS).

**Duration**: 1-2 weeks

### 3.1 Define Message Type Mappings

- [ ] Document message type mappings:
  | Sensor Type | ROS Message Type                | QoS Profile               |
  |-------------|---------------------------------|---------------------------|
  | Camera RGB  | `sensor_msgs::msg::Image`       | `QOS_PROFILE_SENSOR_DATA` |
  | Camera Info | `sensor_msgs::msg::CameraInfo`  | `QOS_PROFILE_SENSOR_DATA` |
  | LiDAR       | `sensor_msgs::msg::PointCloud2` | `QOS_PROFILE_SENSOR_DATA` |
  | IMU         | `sensor_msgs::msg::Imu`         | `QOS_PROFILE_DEFAULT`     |
  | GNSS        | `sensor_msgs::msg::NavSatFix`   | `QOS_PROFILE_DEFAULT`     |

### 3.2 Refactor Sensor Bridge Structure

File: `src/bridge/sensor_bridge.rs`

- [ ] Update `SensorBridge` struct:
  ```rust
  pub struct SensorBridge {
      _vehicle_name: String,
      sensor_type: SensorType,
      _actor: Sensor,
      sensor_name: String,
      // Remove: tx: Sender<(MessageType, Vec<u8>)>,
      // Publishers will be stored per sensor type
  }
  ```
- [ ] Remove channel-based threading (`tx`, `rx`, `MessageType` enum)
- [ ] Publishers will be stored in individual sensor structs or as trait objects

### 3.3 Migrate Camera RGB Sensor

- [ ] Update `register_camera_rgb()` signature:
  ```rust
  fn register_camera_rgb(
      node: Arc<rclrs::Node>,
      actor: &Sensor,
      topic_name: String,
  ) -> Result<Arc<rclrs::Publisher<sensor_msgs::msg::Image>>>
  ```
- [ ] Create publisher:
  ```rust
  let image_publisher = node.create_publisher::<sensor_msgs::msg::Image>(
      &format!("{}/image_raw", topic_name),
      rclrs::QOS_PROFILE_SENSOR_DATA,
  )?;
  let info_publisher = node.create_publisher::<sensor_msgs::msg::CameraInfo>(
      &format!("{}/camera_info", topic_name),
      rclrs::QOS_PROFILE_SENSOR_DATA,
  )?;
  ```
- [ ] Update `camera_callback()` to publish directly:
  ```rust
  fn camera_callback(
      publisher: Arc<rclrs::Publisher<sensor_msgs::msg::Image>>,
      header: std_msgs::msg::Header,
      image: CarlaImage,
  ) -> Result<()> {
      let image_msg = sensor_msgs::msg::Image {
          header,
          height: image.height() as u32,
          width: image.width() as u32,
          encoding: "bgra8".to_string(),
          is_bigendian: utils::is_bigendian() as u8,
          step: (image.width() * 4) as u32,
          data: image.as_slice()
              .iter()
              .flat_map(|&Color { b, g, r, a }| [b, g, r, a])
              .collect(),
      };
      publisher.publish(&image_msg)?;
      Ok(())
  }
  ```
- [ ] Remove CDR serialization
- [ ] Remove channel send
- [ ] Update `camera_info_callback()` similarly
- [ ] Test camera topic:
  ```bash
  ros2 topic list | grep camera
  ros2 topic echo /autoware_v1/sensing/camera/traffic_light/image_raw --no-arr
  ros2 topic hz /autoware_v1/sensing/camera/traffic_light/image_raw
  ```

### 3.4 Migrate LiDAR Sensors

- [ ] Update `register_lidar_raycast()`:
  ```rust
  fn register_lidar_raycast(
      node: Arc<rclrs::Node>,
      actor: &Sensor,
      topic_name: String,
  ) -> Result<Arc<rclrs::Publisher<sensor_msgs::msg::PointCloud2>>>
  ```
- [ ] Create publisher with `QOS_PROFILE_SENSOR_DATA`
- [ ] Update `lidar_callback()` to publish directly
- [ ] Remove CDR serialization
- [ ] Update `register_lidar_raycast_semantic()` similarly
- [ ] Update `sematic_lidar_callback()` similarly
- [ ] Test LiDAR topics:
  ```bash
  ros2 topic echo /autoware_v1/sensing/lidar/top/pointcloud --no-arr
  ros2 topic hz /autoware_v1/sensing/lidar/top/pointcloud
  ```

### 3.5 Migrate IMU Sensor

- [ ] Update `register_imu()`:
  ```rust
  fn register_imu(
      node: Arc<rclrs::Node>,
      actor: &Sensor,
      topic_name: String,
  ) -> Result<Arc<rclrs::Publisher<sensor_msgs::msg::Imu>>>
  ```
- [ ] Create publisher with `QOS_PROFILE_DEFAULT`
- [ ] Update `imu_callback()` to publish directly
- [ ] Remove CDR serialization
- [ ] Test IMU topic:
  ```bash
  ros2 topic echo /autoware_v1/sensing/imu/tamagawa/imu_raw
  ros2 topic hz /autoware_v1/sensing/imu/tamagawa/imu_raw
  ```

### 3.6 Migrate GNSS Sensor

- [ ] Update `register_gnss()`:
  ```rust
  fn register_gnss(
      node: Arc<rclrs::Node>,
      actor: &Sensor,
      topic_name: String,
  ) -> Result<Arc<rclrs::Publisher<sensor_msgs::msg::NavSatFix>>>
  ```
- [ ] Create publisher with `QOS_PROFILE_DEFAULT`
- [ ] Update `gnss_callback()` to publish directly
- [ ] Remove CDR serialization
- [ ] Test GNSS topic:
  ```bash
  ros2 topic echo /autoware_v1/sensing/gnss/ublox/nav_sat_fix
  ros2 topic hz /autoware_v1/sensing/gnss/ublox/nav_sat_fix
  ```

### 3.7 Update SensorBridge::new()

- [ ] Refactor to create appropriate publisher based on sensor type
- [ ] Store publishers (may need to use trait objects or enum variants)
- [ ] Remove attachment and mode parameters
- [ ] Update error handling

### 3.8 Clean up SensorBridge::Drop

- [ ] Remove thread stopping logic
- [ ] Publishers will be automatically cleaned up when dropped

**Deliverables**:
- [ ] All sensor publishers migrated to rclrs
- [ ] No CDR serialization code remains
- [ ] All sensor topics verified with ROS 2 tools

**Success Criteria**:
- All sensor types publish correctly
- Topics appear in `ros2 topic list`
- Message data is correct (verify with `ros2 topic echo`)
- Topic rates match expected values

---

## Phase 4: Vehicle Bridge Migration

**Objective**: Migrate vehicle control subscribers and status publishers.

**Duration**: 1-2 weeks

### 4.1 Verify Autoware Message Types

- [ ] Verify Autoware message types are accessible through rclrs:
  ```bash
  # Source Autoware environment
  source external/autoware/install/setup.bash

  # Check Autoware message packages
  ros2 interface package autoware_vehicle_msgs
  ros2 interface package tier4_vehicle_msgs
  ros2 interface package tier4_control_msgs

  # List specific message types
  ros2 interface show autoware_vehicle_msgs/msg/VelocityReport
  ros2 interface show tier4_vehicle_msgs/msg/ActuationCommandStamped
  ```

- [ ] Verify `.cargo/config.toml` includes Autoware message paths:
  ```bash
  # After running make build, check generated config
  cat .cargo/config.toml | grep autoware
  ```

- [ ] Test Autoware message imports in Rust:
  ```rust
  use autoware_vehicle_msgs::msg::VelocityReport;
  use tier4_vehicle_msgs::msg::ActuationCommandStamped;
  use tier4_control_msgs::msg::GateMode;
  ```

**Note**: If message types are not available through Autoware workspace:
  - Ensure Autoware 2025.22 is fully built with all message packages
  - Check that `external/autoware` symlink points to correct installation
  - Verify ROS 2 environment is properly sourced in Makefile

### 4.2 Migrate Vehicle Status Publishers

File: `src/bridge/vehicle_bridge.rs`

- [ ] Update `VehicleBridge` struct:
  ```rust
  pub struct VehicleBridge {
      vehicle_name: String,
      actor: Vehicle,
      // Subscribers
      _subscription_actuation_cmd: Arc<rclrs::Subscription<ActuationCommandStamped>>,
      _subscription_gear_cmd: Arc<rclrs::Subscription<GearCommand>>,
      _subscription_gate_mode: Arc<rclrs::Subscription<GateMode>>,
      // Publishers
      publisher_actuation: Arc<rclrs::Publisher<ActuationStatusStamped>>,
      publisher_velocity: Arc<rclrs::Publisher<VelocityReport>>,
      publisher_steer: Arc<rclrs::Publisher<SteeringReport>>,
      publisher_gear: Arc<rclrs::Publisher<GearReport>>,
      publisher_control: Arc<rclrs::Publisher<ControlModeReport>>,
      publisher_turnindicator: Arc<rclrs::Publisher<TurnIndicatorsReport>>,
      publisher_hazardlight: Arc<rclrs::Publisher<HazardLightsReport>>,
      // State
      velocity: Arc<AtomicF32>,
      current_actuation_cmd: Arc<ArcSwap<ActuationCommandStamped>>,
      current_gear: Arc<ArcSwap<u8>>,
      current_gate_mode: Arc<ArcSwap<GateMode>>,
      // Remove: attachment, mode
      // Control parameters
      tau: f32,
      prev_timestamp: Option<f64>,
      prev_steer_output: f32,
  }
  ```

- [ ] Update `VehicleBridge::new()` to create publishers:
  ```rust
  let publisher_actuation = node.create_publisher::<ActuationStatusStamped>(
      &format!("/{}/vehicle/status/actuation_status", vehicle_name),
      rclrs::QOS_PROFILE_DEFAULT,
  )?;
  // ... create other publishers
  ```

- [ ] Create subscribers:
  ```rust
  let cloned_cmd = current_actuation_cmd.clone();
  let subscription_actuation_cmd = node.create_subscription::<ActuationCommandStamped, _>(
      &format!("/{}/control/command/actuation_cmd", vehicle_name),
      rclrs::QOS_PROFILE_DEFAULT,
      move |msg: ActuationCommandStamped| {
          cloned_cmd.store(Arc::new(msg));
      }
  )?;
  // ... create other subscriptions
  ```

### 4.3 Update Status Publishing Functions

- [ ] Update `pub_current_actuation()`:
  ```rust
  fn pub_current_actuation(&mut self, timestamp: f64) -> Result<()> {
      let control = self.actor.control();
      let mut header = utils::create_ros_header(Some(timestamp));
      header.frame_id = String::from("base_link");
      let actuation_msg = ActuationStatusStamped {
          header,
          status: ActuationStatus {
              accel_status: control.throttle as f64,
              brake_status: control.brake as f64,
              steer_status: -control.steer as f64,
          },
      };
      self.publisher_actuation.publish(&actuation_msg)?;
      Ok(())
  }
  ```
- [ ] Remove CDR serialization from all publishing functions:
  - [ ] `pub_current_velocity()`
  - [ ] `pub_current_steer()`
  - [ ] `pub_current_gear()`
  - [ ] `pub_current_control()`
  - [ ] `pub_current_indicator()`
  - [ ] `pub_hazard_light()`
- [ ] Remove attachment publishing logic
- [ ] Keep `update_carla_control()` logic unchanged (it reads from subscriptions)

### 4.4 Test Vehicle Bridge

- [ ] Start CARLA simulator
- [ ] Spawn test vehicle
- [ ] Run autoware_carla_bridge
- [ ] Verify vehicle status topics:
  ```bash
  ros2 topic list | grep vehicle
  ros2 topic echo /autoware_v1/vehicle/status/velocity_status
  ros2 topic echo /autoware_v1/vehicle/status/steering_status
  ```
- [ ] Test vehicle control by publishing commands:
  ```bash
  # Terminal 1: Monitor velocity
  ros2 topic echo /autoware_v1/vehicle/status/velocity_status

  # Terminal 2: Send control command
  ros2 topic pub /autoware_v1/control/command/actuation_cmd \
    tier4_vehicle_msgs/msg/ActuationCommandStamped \
    "{actuation: {accel_cmd: 0.3, brake_cmd: 0.0, steer_cmd: 0.0}}"
  ```
- [ ] Verify vehicle moves in CARLA

**Deliverables**:
- [ ] Vehicle publishers migrated to rclrs
- [ ] Vehicle subscribers migrated to rclrs
- [ ] Control loop functional
- [ ] All vehicle topics verified

**Success Criteria**:
- Vehicle status topics publish correctly
- Vehicle responds to control commands
- No crashes or errors during operation

---

## Phase 5: Testing and Optimization

**Objective**: Comprehensive testing and performance optimization.

**Duration**: 1-2 weeks

### 5.1 Integration Testing

- [ ] Create test scenarios:
  - [ ] Single vehicle spawn and control
  - [ ] Multiple vehicles (v1, v2)
  - [ ] All sensor types enabled
  - [ ] Long-running stability test (30+ minutes)
  - [ ] Rapid vehicle spawn/despawn
- [ ] Document test procedures in `docs/testing.md`
- [ ] Create test scripts in `tests/` directory

### 5.2 ROS 2 Executor Integration

File: `src/main.rs`

- [ ] Add executor spin to main loop:
  ```rust
  let executor = rclrs::SingleThreadedExecutor::new();

  while running.load(Ordering::SeqCst) {
      // Bridge management logic
      {
          // Actor spawn/despawn
      }

      // Update bridges
      let sec = world.snapshot().timestamp().elapsed_seconds;
      bridge_list.values_mut().try_for_each(|bridge| bridge.step(sec))?;

      // Spin ROS callbacks
      executor.spin_once(Some(std::time::Duration::from_millis(10)))?;

      world.wait_for_tick();
  }
  ```
- [ ] Test that callbacks are processed
- [ ] Adjust spin timeout if needed
- [ ] Consider `MultiThreadedExecutor` if performance requires it

### 5.3 Performance Testing

- [ ] Measure topic publication rates:
  ```bash
  ros2 topic hz /autoware_v1/sensing/lidar/top/pointcloud
  ros2 topic hz /autoware_v1/sensing/camera/traffic_light/image_raw
  ros2 topic hz /autoware_v1/vehicle/status/velocity_status
  ```
- [ ] Compare with Zenoh version benchmarks
- [ ] Monitor CPU and memory usage:
  ```bash
  top -p $(pgrep autoware_carla_bridge)
  ```
- [ ] Profile with `flamegraph` or `perf`:
  ```bash
  # Build the project
  make build

  # Profile with flamegraph
  cargo flamegraph --profile release-with-debug --bin autoware_carla_bridge

  # Or use perf directly
  perf record -g ./target/release-with-debug/autoware_carla_bridge
  perf report
  ```
- [ ] Optimize hot paths if needed

### 5.4 Error Handling and Robustness

- [ ] Test error scenarios:
  - [ ] CARLA disconnection during operation
  - [ ] Invalid sensor data
  - [ ] Rapid topic subscription/unsubscription
  - [ ] ROS 2 node termination
- [ ] Improve error messages
- [ ] Add graceful degradation where possible
- [ ] Ensure proper cleanup on Ctrl-C

### 5.5 Code Quality

- [ ] Format and lint code:
  ```bash
  make format  # Format code
  make lint    # Check formatting and run clippy
  ```
- [ ] Add documentation comments to public APIs
- [ ] Remove dead code and unused imports
- [ ] Update code comments referencing Zenoh

### 5.6 Autoware Integration Test

- [ ] Launch Autoware 2025.22
- [ ] Run autoware_carla_bridge with spawned vehicles
- [ ] Verify Autoware receives all expected topics:
  ```bash
  ros2 node info /autoware_node
  ros2 topic list
  ```
- [ ] Test basic Autoware functionality:
  - [ ] Localization with GNSS
  - [ ] Perception with LiDAR/Camera
  - [ ] Planning with vehicle control
- [ ] Document any Autoware-specific configuration needed

**Deliverables**:
- [ ] Test suite and documentation
- [ ] Performance benchmarks
- [ ] Autoware integration validated
- [ ] Code quality improvements

**Success Criteria**:
- All tests pass consistently
- Performance meets or exceeds Zenoh version
- No memory leaks or resource issues
- Autoware integration works correctly

---

## Phase 6: Documentation and Release

**Objective**: Complete documentation and prepare for release.

**Duration**: 3-5 days

### 6.1 Update README.md

- [ ] Update project description to reflect rclrs usage
- [ ] Remove references to Zenoh bridges
- [ ] Update execution instructions:
  - [ ] Remove zenoh router step
  - [ ] Simplify to just CARLA + autoware_carla_bridge
- [ ] Update topic verification examples
- [ ] Update prerequisites (remove Zenoh installation)
- [ ] Update roadmap status

### 6.2 Write Migration Guide

File: `docs/migration-from-zenoh.md`

- [ ] Document breaking changes
- [ ] Provide topic name mapping (if changed)
- [ ] Explain QoS profile changes
- [ ] Document command-line argument changes
- [ ] Provide troubleshooting section

### 6.3 API Documentation

- [ ] Add rustdoc comments to all public items:
  ```rust
  /// A bridge for CARLA sensor data to ROS 2 topics.
  ///
  /// This bridge handles camera, LiDAR, IMU, and GNSS sensors,
  /// publishing their data to appropriate ROS 2 topics.
  pub struct SensorBridge {
      // ...
  }
  ```
- [ ] Generate documentation:
  ```bash
  cargo doc --no-deps --open
  ```
- [ ] Review generated docs for completeness

### 6.4 Examples and Tutorials

- [ ] Create example launch files (if using launch system)
- [ ] Write tutorial: "Getting Started with autoware_carla_bridge"
- [ ] Write tutorial: "Integrating with Autoware"
- [ ] Create example ROS 2 bag recording script

### 6.5 Final Testing

- [ ] Fresh installation test on clean Ubuntu 22.04
- [ ] Follow README instructions exactly
- [ ] Verify all examples work
- [ ] Test with different ROS 2 distributions (if supported)

### 6.6 Release Preparation

- [ ] Update `Cargo.toml` version to `0.13.0`
- [ ] Create CHANGELOG.md:
  ```markdown
  # Changelog

  ## [0.13.0] - 2025-XX-XX

  ### Changed
  - Migrated from Zenoh to native ROS 2 (rclrs)
  - Removed bridge mode selection (DDS, ROS2, RmwZenoh)
  - Simplified codebase and improved type safety

  ### Removed
  - Zenoh dependencies
  - Liveliness token management
  - Attachment metadata handling
  - Mode-specific logic

  ### Fixed
  - [List any bugs fixed during migration]
  ```
- [ ] Create git tag:
  ```bash
  git tag -a v0.13.0 -m "Release v0.13.0: rclrs migration"
  ```
- [ ] Merge feature branch to main:
  ```bash
  git checkout main
  git merge feature/rclrs-migration
  ```
- [ ] Push to repository:
  ```bash
  git push origin main --tags
  ```

**Deliverables**:
- [ ] Updated README.md
- [ ] Migration guide
- [ ] API documentation
- [ ] Examples and tutorials
- [ ] CHANGELOG.md
- [ ] Git release tag

**Success Criteria**:
- Documentation is clear and comprehensive
- New users can get started without external help
- All examples work correctly

---

## Success Criteria

The migration is considered successful when:

### Functional Requirements
- ✅ All sensor types publish to ROS 2 topics (Camera, LiDAR, IMU, GNSS)
- ✅ Vehicle control works bidirectionally (status out, commands in)
- ✅ Multiple vehicles can be bridged simultaneously
- ✅ Clock synchronization works correctly
- ✅ Integration with Autoware 2025.22 is functional

### Technical Requirements
- ✅ Zero Zenoh dependencies remain
- ✅ All ROS 2 topics use correct message types
- ✅ QoS profiles are appropriate for each topic type
- ✅ No CDR serialization code remains
- ✅ Code passes `make lint` with no warnings
- ✅ Code is formatted with `make format`

### Performance Requirements
- ✅ Topic publication rates match or exceed Zenoh version
- ✅ CPU usage is comparable or better
- ✅ Memory usage is stable (no leaks)
- ✅ Latency is acceptable for real-time control

### Documentation Requirements
- ✅ README.md accurately describes the project
- ✅ API documentation is complete
- ✅ Migration guide helps Zenoh users transition
- ✅ Examples and tutorials are working

### Testing Requirements
- ✅ All integration tests pass
- ✅ Autoware integration is verified
- ✅ Long-running stability test completes
- ✅ Error handling is robust

---

## Risk Management

### Identified Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Autoware message types unavailable in Rust | High | Medium | Generate from `.msg` files or use CDR temporarily |
| Performance regression vs. Zenoh | Medium | Low | Profile and optimize hot paths |
| rclrs API limitations | Medium | Low | Engage with ros2-rust community |
| Breaking changes during development | Medium | Medium | Use feature branch, frequent testing |
| CARLA API incompatibilities | Low | Low | Keep CARLA version pinned |

### Contingency Plans

- **If Autoware messages unavailable**:
  - Option 1: Generate Rust bindings using rosidl
  - Option 2: Define structs manually and use CDR
  - Option 3: Contribute message generation to ros2-rust project

- **If performance issues**:
  - Profile with flamegraph
  - Consider `MultiThreadedExecutor`
  - Optimize message copying
  - Use `Arc` more aggressively

- **If rclrs bugs found**:
  - Report to ros2-rust GitHub
  - Fork and patch if critical
  - Consider temporary workarounds

---

## Progress Tracking

Track progress by marking tasks complete in this document:

- `[ ]` = Not started
- `[~]` = In progress
- `[x]` = Complete
- `[!]` = Blocked

### Current Phase: Phase 7 (carla-rust Integration)

**Last Updated**: 2025-10-31

### Phase Completion Status

**✅ Phase 0: Preparation** - COMPLETE (2025-10-27)
- ✅ All documentation created (4 docs, 2,713 lines)
- ✅ Autoware environment configured
- ✅ Colcon workspace structure created
- ✅ Three-stage build system implemented
- ✅ All bridge types reviewed and understood

**✅ Phase 1: Core Infrastructure** - COMPLETE (2025-10-22)
- ✅ Zenoh → rclrs migration complete (12 files, ~800 lines changed)
- ✅ All Zenoh dependencies removed
- ✅ Mode enum and liveliness tokens removed
- ✅ Code compiles and passes lint checks (zero warnings)
- ✅ Binary built: 9.3 MB
- ✅ .cargo/config.toml generated with 50+ message packages

**✅ Phase 2: Clock and Simple Publishers** - COMPLETE (Code: 2025-10-22, Runtime: 2025-10-31)
- ✅ Clock publisher migrated to rclrs (src/clock.rs)
- ✅ Utility functions updated (src/utils.rs)
- ✅ No CDR serialization or Zenoh dependencies
- ✅ Code compiles successfully
- ✅ **Runtime testing PASSED** - Clock publishes to `/clock` topic successfully

**🔄 Phase 7: carla-rust Integration** - IN PROGRESS (Started 2025-10-29)
- ✅ Local carla-rust path dependency configured
- ✅ Build system verified
- ✅ Documentation created (`docs/carla-rust-integration.md`)
- ✅ Roadmap updated with enhancement tasks (Phase 2 marked complete)
- ⏳ Actor cleanup implementation (pending)
- ⏳ Multi-version CARLA testing (pending)

**📋 Phase 8: Architecture Refactoring - 1-to-1 Design** - PLANNED (2025-10-31)
- ✅ Architecture design documented
- ✅ 1-to-1 Autoware-centric design specified
- ✅ 9 subsections with detailed work items (8.1-8.9)
- ✅ 3 Architecture Decision Records created
- ✅ Multi-vehicle simulation pattern documented
- ⏳ Implementation pending (depends on Phase 2/3 testing)
- ⏳ ~100 tasks defined across all subsections
- **Estimated Duration**: 1-2 weeks
- **Prerequisites**: Phase 2/3 runtime validation

**⏳ Phase 3-6: Sensor/Vehicle Bridge Testing and Further Development** - PENDING
- Phase 3: Sensor bridge runtime verification (Camera, LiDAR, IMU, GNSS)
- Phase 4: Vehicle bridge runtime verification (control subscribers, status publishers)
- Phase 5: Testing and optimization
- Phase 6: Documentation and release
- Awaiting CARLA simulator testing
- Integration testing with Autoware
- Performance optimization

### Completed Items (All Phases)
- [x] ✅ Study Zenoh API usage
- [x] ✅ Study rclrs API
- [x] ✅ Create API comparison document (647 lines)
- [x] ✅ Create roadmap document (1,198 lines)
- [x] ✅ Create message type migration guide (482 lines)
- [x] ✅ Complete Phase 1: Core Infrastructure migration
- [x] ✅ Remove all Zenoh dependencies
- [x] ✅ Remove Mode enum and liveliness logic
- [x] ✅ Update all 5 bridge types to rclrs
- [x] ✅ Complete Phase 2: Clock and Simple Publishers migration
- [x] ✅ Migrate clock publisher to rclrs (src/clock.rs)
- [x] ✅ Update utility functions (src/utils.rs)
- [x] ✅ Remove CDR serialization from clock publisher
- [x] ✅ Runtime testing: Clock publisher verified with CARLA (2025-10-31)
- [x] ✅ Confirmed /clock topic publishes successfully with ros2 topic echo
- [x] ✅ Integrate local carla-rust repository
- [x] ✅ Switch to path dependency for carla crate
- [x] ✅ Create carla-rust integration documentation (386 lines)
- [x] ✅ Configure direnv for automatic environment
- [x] ✅ Simplify Makefile (removed manual sourcing)
- [x] ✅ Fix Makefile typo (build-packages → build-bridge)

### Current Focus (Phase 7)
1. ⏳ Implement actor cleanup with `ActorBase::destroy()`
2. ⏳ Add efficient world loading with `Client::load_world_if_different()`
3. ⏳ Multi-version CARLA support testing (0.9.14, 0.9.15, 0.9.16)
4. ⏳ Documentation updates for new APIs

### Next Steps
1. **Immediate**: Begin Phase 3 runtime testing with CARLA simulator
   - Verify clock publisher (Phase 2) publishes to `/clock` topic
   - Test sensor bridges (Camera, LiDAR, IMU, GNSS) with spawned vehicles
   - Validate vehicle control and status publishing
2. Add Drop implementation with `actor.destroy()` to SensorBridge and VehicleBridge
3. Add world loading utility with version-conditional compilation
4. Test bridge with multiple CARLA versions (0.9.14, 0.9.15, 0.9.16)
5. Explore advanced carla-rust APIs (walker control, batch operations, debug visualization)

### Metrics
- **Documentation**: 5 guides (includes roadmap), 3,600+ total lines
- **Phases Total**: 8 phases (0-7 plus new Phase 8)
- **Phases Complete**: 3 of 8 (Phase 0, 1, 2) - **Phase 2 runtime verified**
- **Phases In Progress**: Phase 7 (carla-rust Integration)
- **Phases Pending**: Phase 3-6, 8 (Architecture Refactoring)
- **Code Changes**: 15 files modified, ~800 lines changed, ~300 lines removed
- **Build Time**: ~5.5 minutes (first build), ~3 minutes (incremental)
- **Binary Size**: 9.3 MB
- **Lint Warnings**: 0
- **Compilation Status**: ✅ Success
- **Runtime Testing**: ✅ **Phase 2 PASSED** - Clock publisher verified (2025-10-31)
- **Architecture Design**: ✅ **DOCUMENTED** - 1-to-1 design, 3 ADRs (2025-10-31)

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
- Review Architecture Design Decisions section above

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

## Architecture Decision Record

### ADR-001: 1-to-1 Bridge-Vehicle Mapping

**Status**: Proposed (2025-10-31)

**Context**:
We need to decide how to handle multiple vehicles in CARLA simulation when integrating with Autoware. Autoware is designed for single-vehicle autonomous driving and has hardcoded absolute topic names.

**Decision**:
Adopt a 1-to-1 mapping where each bridge instance manages exactly one CARLA vehicle. Multiple vehicles require multiple bridge processes running in separate ROS domains.

**Alternatives Considered**:

1. **Multi-Vehicle Bridge with Domain Workarounds** (Rejected)
   - Pros: Single process
   - Cons: Complex implementation, non-standard ROS patterns, tight coupling, debugging difficulty

2. **Topic Namespacing** (Rejected)
   - Pros: Works in single domain
   - Cons: Requires Autoware modification, brittle, maintenance burden

3. **1-to-1 Mapping** (Selected)
   - Pros: Simple, robust, standard patterns, fault isolation, matches real-world deployment
   - Cons: Multiple processes needed (minimal trade-off)

**Consequences**:
- **Positive**: Clean architecture, standard ROS 2, easy testing, matches production
- **Negative**: Need N processes for N vehicles (acceptable trade-off)
- **Neutral**: ROS_DOMAIN_ID management (standard practice)

**Related**:
- See [Architecture Design Decisions](#architecture-design-decisions) section
- See Phase 8 implementation tasks

### ADR-002: Bridge Does Not Control Simulation

**Status**: Proposed (2025-10-31)

**Context**:
Original bridge implementation controls CARLA simulation by configuring synchronous mode and running a tick thread. This creates tight coupling and makes coordination with external scenarios difficult.

**Decision**:
Bridge will not control simulation. External scenario scripts configure CARLA and control ticking. Bridge is a passive adapter that responds to ticks.

**Rationale**:
- Scenario scripts need full control of simulation for testing
- Bridge should not interfere with scenario timing
- Clearer separation of concerns
- More flexible for different testing patterns

**Consequences**:
- **Positive**: Cleaner separation, flexible scenarios, no tick conflicts
- **Negative**: Users must write scenario scripts (acceptable, provides examples)
- **Migration**: Existing users need to add scenario script to their workflow

**Related**:
- Phase 8.1: Remove Simulation Control
- Phase 8.7: Create Scenario Scripts

### ADR-003: Root Namespace Topics

**Status**: Proposed (2025-10-31)

**Context**:
Original bridge used vehicle-prefixed topics (`/autoware_v1/vehicle/status/...`). Autoware has hardcoded absolute topic names that don't use prefixes.

**Decision**:
Publish to root namespace using standard Autoware topic names. Use ROS domains for isolation when running multiple vehicles.

**Rationale**:
- Works with unmodified Autoware
- No launch file modifications needed
- Standard topic structure
- ROS domains provide clean isolation

**Consequences**:
- **Positive**: Standard Autoware topics, no configuration needed, clear structure
- **Negative**: Must use ROS_DOMAIN_ID for multiple vehicles (standard practice)
- **Migration**: Topic names change, but standard names easier to remember

**Related**:
- Phase 8.4: Root Namespace Topics
- Architecture Design Decisions: Core Principles

---

**Document Version**: 1.4
**Last Updated**: 2025-10-31 (Architecture design documented, Phase 8 added)
**Maintained By**: Autoware CARLA Bridge Team
