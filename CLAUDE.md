# Autoware CARLA Bridge

## Project Overview

Native ROS 2 bridge between CARLA and Autoware, written in Rust using rclrs.

**Repository**: https://github.com/NEWSLabNTU/ros_zenoh_bridge

**Status**: ✅ Phases 0-3 Complete (50%) - Core migration from Zenoh to rclrs complete with Autoware integration foundation.

---

## Current State

### What Works
- ✅ Native ROS 2 publishers/subscribers (no Zenoh/CDR serialization)
- ✅ Clock publisher and utility functions
- ✅ All 5 bridge types migrated (Sensor, Vehicle, TrafficLight, TrafficSign, OtherActor)
- ✅ Autoware integration foundation:
  - Autoware instance detection via `/robot_description`
  - URDF parsing (26 sensors from sample_sensor_kit)
  - TF2 transform buffer with multi-hop chain traversal
  - ROS ↔ CARLA coordinate conversion
- ✅ Responsive shutdown (100ms Ctrl-C exit)
- ✅ Runtime verified with live Autoware + CARLA

### What's Next (Phase 4)
- [ ] Vehicle spawning with initial pose from RViz
- [ ] Sensor attachment with TF2 transforms
- [ ] Vehicle cleanup on Autoware loss
- [ ] Pose teleportation updates
- [ ] Sensor parameter configuration

---

## Build System

Uses colcon-cargo-ros2 for seamless Rust + ROS 2 integration:

```bash
just build  # Standard colcon build
```

No manual staging or configuration required - builds like any ROS 2 package.

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
├── docs/                          # Migration guides
├── scripts/                       # Utilities
├── third_party/
│   ├── autoware@                  # Symlink for just commands
│   └── carla/                     # CARLA run scripts
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

### Coordinate Systems
- Position: meters ↔ cm, Y-axis flip
- Rotation: radians ↔ degrees, roll/yaw sign flips
- Transform chain: Multi-hop TF2 traversal (max depth: 20)

---

## Key Learnings

### rclrs API
- Builder pattern for QoS: `"topic".sensor_data_qos()`, `"topic".reliable()`
- Node is `Arc<NodeState>` internally (cheap clone)
- Publishers need `Arc<Publisher>` for thread sharing
- Automatic serialization (no CDR)

### CARLA Integration
- Sensor callbacks run in separate threads
- Shutdown requires 1s tick timeout + flag checks
- Executor must spin in wait loops for callbacks
- carla-rust uses nalgebra types: `.to_na()` / `Transform::from_na()`

### Dependency Synchronization
When using local carla-rust (`path = "..."`), match critical dependency versions (nalgebra, ndarray) to avoid type incompatibility errors.

---

## Coding Practices

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

## Documentation

**In-repo**:
- `docs/sensor-configuration-strategy.md` - Sensor config & gap analysis
- `docs/carla-autoware-map-integration.md` - Map conversion guide
- `docs/roadmap.md` - Phase breakdown
- `README.md` - Setup & quick start

**External**:
- [carla-rust](https://github.com/jerry73204/carla-rust)
- [rclrs](https://github.com/ros2-rust/ros2_rust)
- [CARLA Simulator](https://carla.org/)
- [Autoware](https://autowarefoundation.github.io/autoware-documentation/)

---

**Last Updated**: 2025-11-18
**Migration Status**: Phases 0-3 Complete (50%)
