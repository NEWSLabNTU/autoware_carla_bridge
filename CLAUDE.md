# Autoware CARLA Bridge

## Project Overview

Native ROS 2 bridge between CARLA and Autoware, written in Rust using rclrs.

**Repository**: https://github.com/NEWSLabNTU/ros_zenoh_bridge

**Status**: ✅ Phases 1-4 Complete - End-to-end autonomous driving working (CARLA + Bridge + Autoware). Remaining: vehicle calibration, map automation, formal testing.

**Recent**: ✅ Modern Autoware localization API integration (2025-11-23) - Bridge now spawns vehicles automatically when localization initializes, enabling full end-to-end autonomous driving.

---

## Current State

### What Works

**Rust Bridge (rclrs)**:
- ✅ Native ROS 2 publishers/subscribers (no Zenoh/CDR serialization)
- ✅ Clock publisher and utility functions
- ✅ Autoware integration:
  - Autoware instance detection via `/robot_description`
  - TF2 transform buffer with multi-hop chain traversal
  - ROS ↔ CARLA coordinate conversion
  - Modern Autoware localization API integration
  - Config-driven vehicle and sensor spawning (`vehicle_config.yaml`)
- ✅ Sensor data publishing (all verified via autonomous driving):
  - Camera (Image + CameraInfo)
  - LiDAR (PointCloud2 with PointXYZIRC format for NDT)
  - IMU (acceleration, angular velocity, orientation)
  - GNSS (NavSatFix)
- ✅ Vehicle control:
  - Subscribes to `/control/command/actuation_cmd` (ActuationCommandStamped)
  - Publishes VelocityReport, SteeringReport, ControlModeReport, GearReport at ~20Hz
  - GNSS PoseWithCovarianceStamped for localization (bypasses gnss_poser)
- ✅ Connection robustness:
  - Infinite retry loops for CARLA connection with panic catching
  - Infinite wait for Autoware detection with executor spinning
  - Graceful Ctrl-C handling during retry/wait phases
- ✅ Vehicle cleanup on Autoware loss (respawn requires bridge restart)
- ✅ Responsive shutdown (100ms Ctrl-C exit)

**Autonomous Driving (standalone scripts)**:
- ✅ **`scripts/auto_drive.py`** - Full autonomous driving workflow
  - 6-step sequence: localize → wait → route → wait → engage → wait for arrival
  - Uses Autoware AD API v1 services with retry logic and state monitoring
  - Options: `--poses`, `--timeout`, `--no-wait-arrival`
- ✅ **`scripts/capture_poses.py`** - Captures poses from RViz
  - Subscribes to `/initialpose`, `/planning/mission_planning/goal`, `/rviz/routing/rough_goal`
  - Saves to YAML with flat format `{x, y, z, qx, qy, qz, qw}` + covariance
  - Partial save on Ctrl-C

**Pre-converted Maps**:
- ✅ Town01, Town02, Town03, Town05, Town10 in `data/carla-autoware-bridge/`
- ✅ Each with lanelet2_map.osm, pointcloud_map.pcd, map_config.yaml, map_projector_info.yaml

### What's Next
- [ ] Vehicle calibration per CARLA model (steering multiplier, wheelbase)
- [ ] Vehicle respawn after Autoware reconnection (currently requires bridge restart)
- [ ] Formal test scripts and performance benchmarks
- [ ] Map conversion automation (custom maps beyond pre-converted TUMFTM)

---

## Build & Run System

### Build (Rust Bridge)

Uses colcon-cargo-ros2 for seamless Rust + ROS 2 integration:

```bash
just build  # Standard colcon build
just clean  # Clean build artifacts (optional, for fresh rebuild)
```

**IMPORTANT**: Always use `just build` (and optionally `just clean`) to rebuild the project. Do NOT use `colcon build` directly with specific packages, as `just build` ensures proper environment setup and consistent build configuration.

No manual staging or configuration required - builds like any ROS 2 package.

### Runtime Management (justfile)

**CARLA Management** (systemd service):
```bash
just carla-start    # Start CARLA as background service
just carla-stop     # Stop CARLA service
just carla-status   # Check CARLA service status
just carla-logs     # View CARLA service logs
```

**Foreground Commands**:
```bash
just run-autoware   # Start Autoware planning simulator (foreground)
just run-bridge     # Start CARLA-Autoware bridge (foreground)
just run-demo       # Start Autoware + bridge + scenario + pilot + monitor (foreground)
just run-monitor    # Start vehicle monitor GUI (foreground)
just run-scenario   # Start demo scenario (foreground)
just run-pilot      # Start autonomous driving pilot (foreground)
```

**Demo Workflow** (two-step):
```bash
# 1. Start CARLA (background service, takes ~30s)
just carla-start

# 2. Start everything else (Autoware + bridge + scenario + pilot + monitor)
just run-demo

# Check CARLA status
just carla-status

# Stop CARLA when done
just carla-stop
```

CARLA is started separately because it takes a long time to initialize. The bridge and scenario have retry loops for CARLA connection.

**Autonomous Driving**:
```bash
# Start CARLA
just carla-start

# Start demo
just run-demo

# (Optional) Capture poses from RViz
just run-capture-poses /path/to/output.yaml

# (Optional) Run pilot separately
just run-pilot /path/to/poses.yaml

# Stop CARLA
just carla-stop
```

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
│   ├── README.md                  # Docs index
│   ├── design/                    # Architecture & design docs
│   ├── guides/                    # How-to guides
│   ├── roadmap/                   # Phase 1-6 roadmap docs
│   └── archive/                   # Historical migration docs
├── scripts/                       # Python utilities
│   ├── auto_drive.py              # Autonomous driving sequence
│   ├── capture_poses.py           # Capture poses from RViz
│   ├── get_carla_spawn_points.py  # Get valid CARLA spawn points
│   ├── poses.yaml                 # Current poses (auto-generated)
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

CARLA uses a left-handed system (Y = right), ROS uses right-handed (Y = left). The conversion is a Y-axis flip on positions and roll/yaw sign flips on rotations (see `coordinate_conversion.rs`).

**The Y-flip is NOT applied uniformly.** LiDAR point data stays in CARLA's native coordinates, while poses and velocities are converted to ROS coordinates:

| Data | Coordinate System | Y-Flip? | Why |
|------|-------------------|---------|-----|
| LiDAR points (live scan) | CARLA native | No | Must match PCD map for NDT |
| PCD map (carla_pcd_gen) | CARLA native | No | Must match live LiDAR scans |
| Vehicle pose, GNSS | ROS (right-handed) | Yes | Autoware planning expects ROS coords |
| Velocities | ROS (right-handed) | Yes | Autoware control expects ROS coords |
| Lanelet2 map | Local coordinates | No | Generated from OpenDRIVE, consistent with poses |

NDT scan matching is purely geometric -- it aligns point cloud shapes. As long as the live LiDAR scans and PCD map are in the same coordinate system, NDT works regardless of handedness convention. The raw numbers are unchanged; handedness only affects interpretation.

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

### use_sim_time Parameter Propagation - CRITICAL

**Problem**: ROS 2 launch files don't automatically propagate `use_sim_time` through nested includes. Autoware's `global_params.launch.py` uses `SetParameter` but this only affects nodes launched in the same context.

**Symptom**: Auto button disabled in RViz despite valid route. Check logs for:
```
Lookup would require extrapolation into the past. Requested time 527.290747 but earliest data is at time 1765365855.014771
```

**Root Cause Chain**:
```
Localization nodes have use_sim_time=false
  → TF transforms published with wall clock (~1.77 billion seconds)
  → Sensor data uses simulation time (~527 seconds)
  → MessageFilter drops all pointclouds (TF lookup fails)
  → occupancy_grid_map produces no output
  → behavior_path_planner blocked
  → No trajectory/control commands
  → Diagnostic rate checks fail
  → is_autonomous_mode_available = false
```

**Fix**: Add global `<set_parameter>` in top-level launch file:
```xml
<launch>
  <arg name="use_sim_time" default="true"/>

  <!-- CRITICAL: Set use_sim_time globally for ALL nodes -->
  <set_parameter name="use_sim_time" value="$(var use_sim_time)"/>

  <!-- Include Autoware launch files after set_parameter -->
  <include file="$(find-pkg-share autoware_launch)/launch/autoware.launch.xml">
    ...
  </include>
</launch>
```

**Verification**:
```bash
# Check localization nodes have use_sim_time=true
ros2 param get /localization/pose_estimator/ndt_scan_matcher use_sim_time
ros2 param get /localization/ekf_localizer use_sim_time
```

**File**: `src/carla_autoware_launch/launch/carla_simulator.launch.xml` (fixed 2025-12-11)

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

**Fixed** (2025-12-03): Removed problematic commented-out sections (lines 232-265 in sensor_kit.xacro) that contained colon-heavy documentation about default macro invocation. Even commented XML sections can cause YAML parsing failures.

---

## Coding Practices

### Background Commands for Demo

**When starting demo, use simple background commands with run_in_background=true**

```bash
# Correct - simple command in background
just run-demo 2>&1
# (with run_in_background=true)

# Incorrect - don't add sleep/echo/wait patterns
just run-demo 2>&1 &
sleep 80
echo "=== Done ==="
```

The Bash tool's `run_in_background` parameter handles background execution properly. Adding shell backgrounding (`&`), sleep/wait patterns clutters the command and makes output harder to track.

**Checking status**: Use `just carla-status` or `BashOutput` tool to check background job output.

### Systemd Service Commands

**`just bridge start`, `just bridge stop`, etc. start/stop systemd services and return immediately**

```bash
# Correct - run directly, no background needed
just bridge start
just bridge stop

# Incorrect - don't put in background
just bridge start &
```

These commands control systemd services that run in the background themselves. They complete immediately after starting/stopping the service.

### Build Commands - CRITICAL

**Always use `just build` instead of direct `colcon build`**

```bash
# Correct - use just build
just build

# Incorrect - don't use colcon build directly
colcon build --packages-select some_package
```

**Why**: The justfile passes `--symlink-install` to colcon, which creates symbolic links in the `install/` directory instead of file copies. Running `colcon build` directly creates file copies that conflict with previously created symlinks, causing unpredictable behavior.

**If you accidentally ran `colcon build` directly**:
```bash
just clean && just build
```

This removes all build artifacts and rebuilds cleanly with proper symlinks.

**If install/ files are broken for a specific package** (e.g., stale launch files):
```bash
# Clean specific package and rebuild
rm -rf build/autoware_carla_bridge install/autoware_carla_bridge
just build
```

Never manually fix symlinks or copy files to install/. Always use `just build` after removing the broken package directories.

### ROS 2 Topic Monitoring Commands

**Use Bash tool's timeout parameter instead of shell patterns**

When running streaming ROS 2 commands like `ros2 topic hz` or `ros2 topic echo`, use the Bash tool's `timeout` parameter:

```bash
# Correct - use timeout parameter on Bash tool
timeout 5 ros2 topic hz /some/topic --window 3

# Incorrect - don't use sleep/kill patterns
ros2 topic hz /some/topic --window 3 &
sleep 3
kill %1 2>/dev/null
```

**Why**: The Bash tool's timeout parameter handles process cleanup properly. Shell backgrounding with manual kill creates race conditions and cluttered output.

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

1. **Start CARLA** (background service):
   ```bash
   just carla-start
   ```

2. **Start demo** (Autoware + bridge + scenario + pilot + monitor):
   ```bash
   just run-demo
   ```

3. **Capture Poses in RViz** (if not already captured):
   ```bash
   # In RViz:
   # - Click "2D Pose Estimate" → set initial position
   # - Click "2D Goal Pose" → set goal position

   # Capture poses:
   just run-capture-poses /path/to/output.yaml
   ```

4. **Monitor** (optional, in another terminal):
   ```bash
   just carla-logs -f
   ```

5. **Stop CARLA**:
   ```bash
   just carla-stop
   ```

### Script Details

**`scripts/auto_drive.py`** performs a 6-step sequence:
1. Initialize localization (publishes to `/initialpose` + calls `/api/localization/initialize`)
2. Wait for localization convergence (monitors `/api/operation_mode/state`)
3. Set route via `/api/routing/set_route_points` (with retry)
4. Wait for route state to become SET
5. Engage autonomous mode via `/api/operation_mode/change_to_autonomous` (with retry)
6. Wait for arrival (route state becomes ARRIVED)

**Options**: `--poses <file>`, `--timeout <secs>`, `--no-wait-arrival`

**`scripts/capture_poses.py`** captures poses from RViz to a YAML file.
**Options**: `-o <file>`

---

## Debugging Autoware

### Autoware Node Logs (play_launch)

When Autoware is running via `just autoware start`, individual node logs are stored at:
```
third_party/autoware/autoware_repo/play_log/<timestamp>/node/<node-name>/{out,err}
```

Example - check NDT scan matcher logs:
```bash
# List available log sessions
ls third_party/autoware/autoware_repo/play_log/

# Check specific node logs
cat third_party/autoware/autoware_repo/play_log/2025-12-08_03-24-49/node/ndt_scan_matcher/err
cat third_party/autoware/autoware_repo/play_log/2025-12-08_03-24-49/node/autoware_pose_initializer_node-78/out
```

### CARLA-Specific System Configurations

The `carla_autoware_launch` package provides CARLA-optimized configurations that override Autoware defaults:

**MRM Handler** (`config/system/mrm_handler/mrm_handler.param.yaml`):
- `timeout_operation_mode_availability: 2.0` (default: 0.5s) - Relaxed for simulation timing variations
- `timeout_call_mrm_behavior: 0.5` (default: 0.01s) - Allow more time for service responses
- `timeout_cancel_mrm_behavior: 0.5` (default: 0.01s)
- `use_pull_over: false`, `use_comfortable_stop: false` - Disabled for simulation

**Component State Monitor Topics** (`config/system/component_state_monitor/topics.yaml`):
- Excludes traffic light recognition topic monitoring (not available in CARLA)

**Launch File** (`launch/carla_simulator.launch.xml`):
- Sets `use_sim_time` globally via `<set_parameter>`
- Sets `system_run_mode=logging_simulation` to disable pose_initializer stop check
- Disables traffic light recognition (`use_traffic_light_recognition=false`)
- Uses CARLA-optimized localization config path
- Launches system component with CARLA-specific topics and MRM handler configs

### Pose Initializer Stop Check

The pose_initializer requires the vehicle to be stopped before accepting initialization:

- **Topic**: `/sensing/vehicle_velocity_converter/twist_with_covariance`
- **Message type**: `geometry_msgs::msg::TwistWithCovarianceStamped`
- **Parameters** (on `/localization/util/pose_initializer`):
  - `stop_check_enabled`: Whether stop check is required (default: true in `online` mode)
  - `stop_check_duration`: How long vehicle must be stopped (default: 3.0s)

**CRITICAL**: The `stop_check_enabled` parameter is controlled by `system_run_mode` at launch time:
- `system_run_mode=online` → `stop_check_enabled=true` (default)
- `system_run_mode=logging_simulation` → `stop_check_enabled=false`

Our `carla_simulator.launch.xml` sets `system_run_mode=logging_simulation` to disable stop check because:
1. CARLA simulation has timing issues with `use_sim_time`
2. Localization nodes have `use_sim_time=False` due to parameter propagation issues
3. This causes timestamp mismatch that makes stop check always fail

**Verification**: `ros2 param get /localization/util/pose_initializer stop_check_enabled` should return `False`.

---

## Documentation

**In-repo**:
- `scripts/README.md` - **Autonomous driving scripts guide** ⭐
- `docs/README.md` - Docs index (design, guides, roadmap, archive)
- `docs/guides/automated-map-generation.md` - Map conversion guide
- `docs/design/sensor-configuration-strategy.md` - Sensor config & gap analysis
- `README.md` - Setup & quick start

**External**:
- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/)
- [Autoware AD API](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/)
- [carla-rust](https://github.com/jerry73204/carla-rust)
- [rclrs](https://github.com/ros2-rust/ros2_rust)
- [CARLA Simulator](https://carla.org/)

---

**Last Updated**: 2026-03-03 (Roadmap docs updated to reflect actual implementation status)
**Status**: Phases 1-4 Complete - End-to-end autonomous driving working
**Remaining**: Vehicle calibration (Phase 4.4), map automation (Phase 5), formal testing (Phase 6)
**Recent Changes**:
- ✅ **MRM handler timeout configuration for CARLA** (2025-12-12):
  - Fixed MRM oscillation causing EMERGENCY_STOP flickering and autonomous mode unavailability
  - Root cause: Default MRM handler timeouts too aggressive for simulation (0.5s availability, 0.01s behavior calls)
  - Created CARLA-specific MRM handler config: `src/carla_autoware_launch/config/system/mrm_handler/mrm_handler.param.yaml`
  - Relaxed timeouts: `timeout_operation_mode_availability: 2.0s`, `timeout_call_mrm_behavior: 0.5s`
  - Updated `carla_simulator.launch.xml` to use CARLA-specific MRM handler config
  - Result: MRM state stable at NORMAL (1), autonomous mode can now be engaged successfully
- ✅ **use_sim_time parameter propagation fix** (2025-12-11):
  - Fixed Auto button disabled in RViz despite valid route
  - Root cause: Localization nodes had `use_sim_time=false` causing TF timestamp mismatch
  - TF transforms used wall clock time (~1.77B sec) while sensors used sim time (~527 sec)
  - MessageFilter dropped all pointclouds → cascading failure through perception/planning/control
  - Fix: Added `<set_parameter name="use_sim_time" value="$(var use_sim_time)"/>` in carla_simulator.launch.xml
- ✅ **Localization stop_check fix** (2025-12-10):
  - Fixed "vehicle is not stopped" error during pose initialization
  - Updated `carla_simulator.launch.xml` to set `system_run_mode=logging_simulation`
  - This disables `stop_check_enabled` in pose_initializer at launch time
  - Root cause: `use_sim_time=False` on localization nodes caused timestamp mismatch
- ✅ **ROS parameters migration** (2025-12-08):
  - Replaced clap command-line args with native ROS 2 parameters
  - Single config file: `src/autoware_carla_bridge/config/vehicle_config.yaml`
  - Parameters set via `--ros-args -p param:=value` (e.g., `carla_port`, `vehicle_config`)
  - Removed duplicate config files, consolidated to single source of truth
- ✅ **CARLA localization configuration** (2025-12-05):
  - Created `carla_autoware_launch` package with CARLA-optimized NDT parameters
  - Disabled `stop_check_enabled` in pose_initializer (CARLA timing issues caused false "vehicle not stopped" errors)
  - Custom NDT params: resolution=1.0m, max_iterations=50, particles_num=500, map_radius=200m
- ✅ **Connection robustness improvements** (2025-12-03):
  - Infinite retry loops for CARLA connection with panic catching (main.rs:167-196)
  - Infinite wait for Autoware detection with executor spinning (main.rs:223-259)
  - Graceful Ctrl-C handling during all wait phases
  - Progress logging every 2-5 seconds
- ✅ **URDF/YAML parsing fixes** (2025-12-03):
  - Fixed duplicate link error in sensor_kit.xacro
  - Removed colon-containing XML comments causing YAML parsing failures
  - Demo environment stability verified (all services running)
- ✅ Bridge now uses modern Autoware localization API exclusively (2025-11-23)
- ✅ Removed backward compatibility with legacy `/initialpose` topic (2025-11-23)
- ✅ Vehicle spawning triggered by localization state transitions (2025-11-23)
- ✅ End-to-end autonomous driving test confirmed working (2025-11-23)
