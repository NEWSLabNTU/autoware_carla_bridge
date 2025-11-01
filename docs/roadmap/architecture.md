# Architecture Design Decisions

This document describes the architecture design decisions for the autoware_carla_bridge project.

## 1-to-1 Autoware-Centric Design Philosophy

**Decision Date**: 2025-10-31

The bridge architecture follows a **1-to-1 mapping** between CARLA vehicles and Autoware instances:

```
CARLA Vehicle ↔ Bridge Instance ↔ Autoware Instance
```

This design choice aligns with Autoware's single-vehicle autonomous driving architecture and provides clean isolation between multiple vehicles in simulation.

## Core Principles

### 1. One Bridge Per Vehicle

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

### 2. Bridge is Passive Adapter

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

### 3. Root Namespace Topics

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

### 4. Vehicle Discovery and Selection

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

## Startup Sequence

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

## Multi-Vehicle Simulation Example

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

## Alternative Designs Considered

### Option 1: Multi-Vehicle Bridge with Domain Workarounds (❌ Rejected)

One bridge instance publishes topics for all vehicles into multiple ROS domains.

**Problems**:
- ❌ No standard way for one ROS node to exist in multiple domains
- ❌ Would require rclrs hacks or multiple context instances (non-standard)
- ❌ Debugging nightmare (one process doing multiple things)
- ❌ Tight coupling between vehicle lifecycles
- ❌ Resource contention between vehicles
- ❌ Complex implementation for minimal benefit

### Option 2: Topic Namespacing Instead of Domains (❌ Rejected)

Use topic prefixes (`/vehicle_1/...`, `/vehicle_2/...`) instead of domain isolation.

**Problems**:
- ❌ Requires modifying Autoware launch files (brittle, maintenance burden)
- ❌ Autoware has many hardcoded absolute topic names
- ❌ No isolation at node/service level (only topics)
- ❌ More complex configuration
- ❌ Less clean than standard domain separation

### Option 3: 1-to-1 Bridge-Vehicle Design (✅ Selected)

**Advantages**:
- ✅ Simple and robust
- ✅ Standard ROS 2 patterns (domain isolation)
- ✅ Minimal code changes needed
- ✅ Mirrors real-world deployment (one Autoware per vehicle)
- ✅ Easy to test and debug
- ✅ Works with unmodified Autoware
- ✅ Natural fault isolation

## Consequences and Trade-offs

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

# Architecture Decision Records

## ADR-001: 1-to-1 Bridge-Vehicle Mapping

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
- See [Core Principles](#core-principles) section
- See [enhancements.md](enhancements.md) Phase 8 implementation tasks

## ADR-002: Bridge Does Not Control Simulation

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
- Phase 8.1: Remove Simulation Control (see [enhancements.md](enhancements.md))
- Phase 8.7: Create Scenario Scripts (see [enhancements.md](enhancements.md))

## ADR-003: Root Namespace Topics

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
- Phase 8.4: Root Namespace Topics (see [enhancements.md](enhancements.md))
- [Core Principles](#core-principles): Root Namespace Topics

---

**Document Version**: 1.0
**Last Updated**: 2025-11-02
**Related Documents**: [roadmap.md](../roadmap.md), [enhancements.md](enhancements.md)
