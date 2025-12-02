# Autoware CARLA Bridge

## Project Overview

Native ROS 2 bridge between CARLA and Autoware, written in Rust using rclrs.

**Repository**: https://github.com/NEWSLabNTU/ros_zenoh_bridge

**Status**: ✅ Phases 0-3 Complete + Phase 4 Vehicle Spawning (55%) - Core migration from Zenoh to rclrs complete with modern Autoware API integration.

**Recent**: ✅ Modern Autoware localization API integration (2025-11-23) - Bridge now spawns vehicles automatically when localization initializes, enabling full end-to-end autonomous driving.

---

## Current State

### What Works

**Rust Bridge (rclrs)**:
- ✅ Native ROS 2 publishers/subscribers (no Zenoh/CDR serialization)
- ✅ Clock publisher and utility functions
- ✅ All 5 bridge types migrated (Sensor, Vehicle, TrafficLight, TrafficSign, OtherActor)
- ✅ Autoware integration foundation:
  - Autoware instance detection via `/robot_description`
  - URDF parsing (26 sensors from sample_sensor_kit)
  - TF2 transform buffer with multi-hop chain traversal
  - ROS ↔ CARLA coordinate conversion
  - **Modern Autoware localization API integration**:
    - Subscribes to `/localization/initialization_state` (monitors INITIALIZED state)
    - Subscribes to `/localization/kinematic_state` (receives vehicle pose)
    - Spawns vehicle when localization becomes INITIALIZED (state 3)
    - No backward compatibility with legacy `/initialpose` topic
- ✅ Responsive shutdown (100ms Ctrl-C exit)
- ✅ Runtime verified with live Autoware + CARLA

**Autonomous Driving Scripts (Python/rclpy)**:
- ✅ **`drive_in_autoware.py`** - Full autonomous driving workflow
  - Uses modern Autoware 2024/2025 API services
  - Initializes localization via `/api/localization/initialize`
  - Sets route via `/api/routing/set_route_points`
  - Engages autonomous mode via `/api/operation_mode/change_to_autonomous`
  - Monitors progress until goal reached
  - **Reusable**: Can run multiple times in single Autoware session
- ✅ **`read_poses.py`** - Captures poses from RViz
  - Subscribes to `/initialpose` and `/planning/mission_planning/goal`
  - Saves to `scripts/poses.json` for autonomous driving
- ✅ **`get_carla_spawn_points.py`** - Get valid CARLA spawn points
  - Connects to CARLA and retrieves spawn points
  - Suggests valid pose pairs for autonomous driving

### What's Next (Phase 4)
- [x] Vehicle spawning with initial pose (via modern Autoware localization API) ✅ 2025-11-23
- [ ] Sensor attachment with TF2 transforms (partially working, needs refinement)
- [ ] Vehicle cleanup on Autoware loss (known issue: vehicle destroyed after spawn)
- [ ] Pose teleportation updates
- [ ] Sensor parameter configuration

---

## Build & Run System

### Build (Rust Bridge)

Uses colcon-cargo-ros2 for seamless Rust + ROS 2 integration:

```bash
just build  # Standard colcon build
```

No manual staging or configuration required - builds like any ROS 2 package.

### Runtime Management (justfile)

**Autoware Management**:
```bash
just autoware start     # Start Autoware planning simulator
just autoware restart   # Restart Autoware
just autoware stop      # Stop Autoware
just autoware logs      # View logs
just autoware status    # Check status
```

**CARLA Management**:
```bash
just carla start 0.9.16 2000   # Start CARLA (version, port)
just carla stop 0.9.16 2000    # Stop CARLA
just carla logs 0.9.16 2000    # View logs
just carla status 0.9.16 2000  # Check status
```

**Bridge Management**:
```bash
just bridge start [port]  # Start bridge (default: 2000)
just bridge stop          # Stop bridge
just bridge logs          # View logs
just bridge status        # Check status
```

**Demo (All-in-One)**:
```bash
just demo start    # Start CARLA + Autoware + Bridge
just demo stop     # Stop all services
just demo status   # Check all services
just demo logs     # View all logs
```

The demo workflow:
1. **Phase 1**: Starts CARLA and Autoware in parallel using GNU Parallel
2. **Phase 2**: Starts bridge after both are ready (5s initialization wait)

**Autonomous Driving**:
```bash
# Start all services
just demo start

# Capture poses from RViz (if not already captured)
./scripts/read_poses.py

# Run autonomous driving
just drive

# Watch bridge logs (in another terminal)
just bridge logs -f

# Stop all services
just demo stop
```

The `just drive` command runs `./scripts/drive_in_autoware.py` with automatic poses.json validation.

**Environment Variables**: The justfile automatically passes `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID`, and `ROS_LOCALHOST_ONLY` to Autoware if set.

---

## Environment Setup

**CRITICAL**: Always source `.envrc` before working on this project.

The `.envrc` file sets up the required environment:
- ROS 2 Humble
- Autoware workspace (provides Autoware message packages)
- Local install directory

### Using direnv (Recommended)

```bash
# Install direnv if not already installed
sudo apt install direnv

# Add to ~/.bashrc
eval "$(direnv hook bash)"

# Allow .envrc in this directory (one-time)
cd /path/to/autoware_carla_bridge
direnv allow
```

Environment is automatically sourced when entering the directory.

### Manual Sourcing

If not using direnv, source manually each session:

```bash
source /opt/ros/humble/setup.sh
source third_party/autoware/autoware_repo/install/setup.sh
source install/setup.sh  # After first build
```

### Verification

Check that Autoware packages are available:

```bash
# Should list Autoware message packages
colcon list | grep autoware_vehicle_msgs
```

**Note for Claude Code**: When starting a new session, always verify the environment is properly sourced. Build failures with "package not found" errors typically indicate missing environment setup.

---

## Repository Structure

```
.
├── src/
│   ├── autoware_carla_bridge/     # Main bridge (rclrs)
│   └── external/
│       ├── autoware@              # Symlink to Autoware workspace
│       ├── carla-rust/            # CARLA Rust bindings
│       └── zenoh_carla_bridge/    # Reference implementation
├── docs/                          # Technical documentation
│   ├── carla-map-acquisition-guide.md
│   ├── sensor-configuration-strategy.md
│   └── roadmap.md
├── scripts/                       # Python utilities & autonomous driving
│   ├── drive_in_autoware.py      # ⭐ Main autonomous driving script
│   ├── read_poses.py              # Capture poses from RViz
│   ├── get_carla_spawn_points.py # Get valid CARLA spawn points
│   ├── setup_carla.py             # Configure CARLA
│   ├── set_initial_pose.py        # Set initial pose
│   ├── poses.json                 # Current poses (auto-generated)
│   └── README.md                  # Scripts documentation
├── third_party/
│   ├── autoware@                  # Symlink for just commands
│   └── carla/                     # CARLA run scripts
├── data/
│   └── carla-autoware-bridge/     # Pre-converted maps (Town01-10)
├── justfile                       # Service management commands
└── build/, install/, log/         # Colcon artifacts
```

---

## Key Technical Decisions

### Architecture
- **Single client**: One CARLA connection vs two separate clients
- **Stateless CarlaVehicle**: No lifecycle state machine, immediate spawning
- **Linear workflow**: Sequential steps (detect Autoware → parse URDF → wait pose → spawn)
- **Direct publishing**: `Arc<Publisher>` from CARLA callbacks (no threading/channels)

### Dependencies
- **rclrs**: Native ROS 2 Rust bindings
- **tracing**: Structured logging (replaced `log`)
- **color-eyre**: Enhanced error reporting (replaced `anyhow`)
- **nalgebra**: Math for transforms (must match carla-rust version)
- **roxmltree**: Lightweight URDF XML parsing
- **autoware_adapi_v1_msgs**: Modern Autoware API message types (for localization state)
- **nav_msgs**: Standard ROS navigation messages (for odometry/kinematic state)

### Coordinate Systems
- Position: meters ↔ cm, Y-axis flip
- Rotation: radians ↔ degrees, roll/yaw sign flips
- Transform chain: Multi-hop TF2 traversal (max depth: 20)

---

## Key Learnings

### rclrs API (Rust)
- Builder pattern for QoS: `"topic".sensor_data_qos()`, `"topic".reliable()`
- Node is `Arc<NodeState>` internally (cheap clone)
- Publishers need `Arc<Publisher>` for thread sharing
- Automatic serialization (no CDR)
- **Subscription callbacks**: Use `Arc<std::sync::Mutex<T>>` for shared state between callbacks
  - Clone the Arc before moving into closures
  - Example: State monitoring (localization_init_state) and data capture (kinematic_state) in separate callbacks
  - Subscriptions must be kept alive as struct fields (e.g., `_localization_init_state_sub`)

### rclpy API (Python - Autonomous Driving Scripts)
- Use `call_async()` for service calls, followed by `spin_until_future_complete()`
- Don't instantiate service classes directly (raises `NotImplementedError`)
- Use `ServiceClass.Request()` to create request objects
- QoS profiles: `TRANSIENT_LOCAL` for latched topics, `VOLATILE` for streaming data

### Modern Autoware API (2024/2025)
**Service Endpoints** (used by Python scripts):
- `/api/localization/initialize` - Initialize localization (not `/initialpose` topic)
- `/api/routing/set_route_points` - Set route to goal
- `/api/routing/clear_route` - Clear existing route
- `/api/operation_mode/change_to_autonomous` - Engage autonomous mode (not `change_operation_mode`)

**Topic Endpoints** (used by Rust bridge):
- `/localization/initialization_state` (autoware_adapi_v1_msgs/LocalizationInitializationState)
  - State values: 0=UNKNOWN, 1=UNINITIALIZED, 2=INITIALIZING, 3=INITIALIZED
  - Bridge monitors state changes to trigger vehicle spawning
- `/localization/kinematic_state` (nav_msgs/Odometry)
  - Provides current vehicle pose in map frame
  - Bridge uses this as initial pose when state becomes INITIALIZED

**Bridge Integration**:
- Bridge subscribes to both topics on startup
- When localization state transitions to INITIALIZED (3), bridge extracts pose from kinematic state
- Vehicle spawns at this pose in CARLA
- No backward compatibility with legacy `/initialpose` topic (removed as of 2025-11-23)

**Important**:
- Route planning requires poses on **connected lanes** in lanelet2 map
- Use RViz "2D Pose Estimate" and "2D Goal Pose" for guaranteed valid poses
- Service calls may fail if Autoware not fully initialized (wait 10-15s after launch)
- **Timing for automated workflows**:
  - 15s wait after localization initialization (vehicle spawn + diagnostics stabilization)
  - 10s wait after route is set (planning/control pipeline activation)
  - These waits are critical for autonomous mode engagement

### CARLA Integration
- Sensor callbacks run in separate threads
- Shutdown requires 1s tick timeout + flag checks
- Executor must spin in wait loops for callbacks
- carla-rust uses nalgebra types: `.to_na()` / `Transform::from_na()`

### Dependency Synchronization
When using local carla-rust (`path = "..."`), match critical dependency versions (nalgebra, ndarray) to avoid type incompatibility errors.

### URDF/Xacro XML Comments - CRITICAL

**NEVER use colons in URDF/Xacro XML comments**

ROS 2 launch validates the robot_description parameter as YAML, and colons in XML comments trigger YAML key-value syntax interpretation, causing parsing failures.

**Failure Example**:
```xml
<!-- NOTE: This causes an error -->
<!-- Example: This also fails -->
<!-- Position: center at wheelbase/2 -->
```

**Error Message**:
```
mapping values are not allowed here
  in "<unicode string>", line XX, column YY:
      NOTE: This causes an error ...
          ^
```

**Correct Usage**:
```xml
<!-- NOTE - This works fine -->
<!-- Example - This also works -->
<!-- Position - center at wheelbase/2 -->
```

**Files Affected**:
- `src/carla_vehicle_launch/carla_vehicle_description/urdf/vehicle.xacro`
- `src/carla_sensor_kit_launch/carla_sensor_kit_description/urdf/sensor_kit.xacro`

**Impact**: robot_state_publisher crashes → no /robot_description published → bridge can't detect Autoware → vehicle fails to spawn → localization can't initialize → "The vehicle is not stopped" error

**Root Cause**: ROS 2 launch system parses robot_description as YAML before passing to robot_state_publisher. Colons are interpreted as YAML mapping syntax regardless of being inside XML comments.

---

## Coding Practices

### Temporary Files

**Always use project's tmp/ directory for temporary files**

```bash
# Correct - use project tmp/
$project/tmp/test_file.txt

# Incorrect - don't use system /tmp/
/tmp/test_file.txt
```

The project's `tmp/` directory is gitignored and provides better isolation for project-specific temporary files.

### Error Handling

**Never silence `Result` types without justification**

```rust
// Preferred: Propagate errors
blueprint.set_attribute("fov", &value.to_string())?;

// Alternative: Explicit handling
if let Err(e) = operation() {
    tracing::error!("Failed: {}", e);
    return Err(e);
}

// Last resort: Document why ignoring is safe
// SAFETY: Best-effort operation, failure is non-critical
let _ = optional_operation();
```

### Unused Code

**Delete unused code unless documented**

Keep only when:
- Resource management (e.g., subscriptions kept alive): Document with `/// NOTE:`
- Planned for future phases: `/// TODO(Phase X):`
- Module-level: `#![allow(dead_code)]` with explanation

Delete:
- Experimental code
- Old implementations
- Helpers with no clear use

---

## Using Local carla-rust

**Location**: `~/repos/carla-rust/`

Provides latest APIs and multi-version support (0.9.14-0.9.16).

**Integration**: Update `src/autoware_carla_bridge/Cargo.toml`:
```toml
carla = { version = "0.12.0", path = "../../carla-rust/carla" }
```

**Important**: Synchronize dependency versions (nalgebra, ndarray) with carla-rust's `Cargo.toml`.

---

## Autonomous Driving Workflow (Python Scripts)

### Quick Start

1. **Start all services**:
   ```bash
   just demo start
   # Starts CARLA + Autoware + Bridge
   ```

2. **Capture Poses in RViz** (if not already captured):
   ```bash
   # In RViz:
   # - Click "2D Pose Estimate" → set initial position
   # - Click "2D Goal Pose" → set goal position

   # Capture poses:
   ./scripts/read_poses.py
   # Saves to scripts/poses.json
   ```

3. **Run Autonomous Driving**:
   ```bash
   just drive
   ```

4. **Monitor** (optional, in another terminal):
   ```bash
   just bridge logs -f
   ```

5. **Stop all services**:
   ```bash
   just demo stop
   ```

### Script Details

**`drive_in_autoware.py`** performs these steps:
1. Load poses from `poses.json`
2. Initialize localization via `/api/localization/initialize` service
3. Clear existing route via `/api/routing/clear_route`
4. Set new route via `/api/routing/set_route_points`
5. Engage autonomous mode via `/api/operation_mode/change_to_autonomous`
6. Monitor progress (velocity, distance, route state) until goal reached

**Reusability**: Can be run multiple times in a single Autoware session without restarting.

**Troubleshooting**: See `scripts/README.md` for common issues and solutions.

---

## Documentation

**In-repo**:
- `scripts/README.md` - **Autonomous driving scripts guide** ⭐
- `docs/carla-map-acquisition-guide.md` - Map conversion guide
- `docs/sensor-configuration-strategy.md` - Sensor config & gap analysis
- `docs/roadmap.md` - Phase breakdown
- `README.md` - Setup & quick start

**External**:
- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/)
- [Autoware AD API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/)
- [carla-rust](https://github.com/jerry73204/carla-rust)
- [rclrs](https://github.com/ros2-rust/ros2_rust)
- [CARLA Simulator](https://carla.org/)

---

**Last Updated**: 2025-11-23 (Session: Modern Autoware API integration)
**Migration Status**: Phases 0-3 Complete + Phase 4 Vehicle Spawning (55%)
**Autonomous Driving**: ✅ End-to-end working (Python scripts + Rust bridge with modern Autoware APIs)
**Recent Changes**:
- ✅ Bridge now uses modern Autoware localization API exclusively
- ✅ Removed backward compatibility with legacy `/initialpose` topic
- ✅ Vehicle spawning triggered by localization state transitions
- ✅ End-to-end autonomous driving test confirmed working (1.64m final distance to goal)
