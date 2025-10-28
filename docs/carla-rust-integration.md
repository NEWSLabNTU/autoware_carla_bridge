# Using Local carla-rust Repository

## Overview

This document describes how to integrate the locally-developed carla-rust library (from `~/repos/carla-rust/`) into the autoware_carla_bridge project.

**Key Benefits**:
- Access to latest API additions and improvements
- Support for multiple CARLA versions (0.9.14, 0.9.15, 0.9.16)
- Version selection via `CARLA_VERSION` environment variable
- Ability to contribute improvements back to carla-rust
- Faster iteration with local changes

---

## Current State

**Current dependency** (in `src/autoware_carla_bridge/Cargo.toml`):
```toml
carla = "0.12.0"  # From crates.io
```

This uses the published carla-0.12.0 crate from crates.io, which:
- Has a fixed CARLA version (0.9.16 by default)
- Doesn't include recent API additions (e.g., `ActorBase::destroy()`, `Client::load_world_if_different()`)
- Can't be modified for local development

---

## Proposed Change

**Switch to path dependency**:
```toml
carla = { version = "0.12.0", path = "../../carla-rust/carla" }
```

This uses the local carla-rust repository, which:
- Supports CARLA 0.9.14, 0.9.15, 0.9.16 (selectable via `CARLA_VERSION`)
- Includes latest API additions and bug fixes
- Allows local modifications and contributions
- Maintains same version number (0.12.0)

---

## CARLA Version Selection

The carla-rust repository supports multiple CARLA versions through the `CARLA_VERSION` environment variable:

### Default (0.9.16)
```bash
make build  # Uses CARLA 0.9.16 by default
```

### CARLA 0.9.15
```bash
CARLA_VERSION=0.9.15 make build
```

### CARLA 0.9.14
```bash
CARLA_VERSION=0.9.14 make build
```

### How It Works

The carla-rust build system:
1. Checks `CARLA_VERSION` environment variable (default: "0.9.16")
2. Downloads appropriate prebuilt `libcarla_client` for that version
3. Uses version-specific headers from `carla-sys/headers/{VERSION}/`
4. Enables conditional compilation with `#[cfg(carla_0916)]`, `#[cfg(carla_0915)]`, etc.

**Storage efficiency**: Multiple versions coexist in build cache (~2GB total for all 3 versions)

---

## New APIs Available

Recent additions to carla-rust (not in crates.io 0.12.0):

### Actor Management
```rust
// Destroy an actor
let success = actor.destroy();  // Returns bool
```

### Client Operations
```rust
// Load world only if different (avoids unnecessary reload)
let world = client.load_world_if_different("Town03");

// With custom settings
let world = client.load_world_if_different_opt("Town03", true);
```

### Sensor Operations
```rust
// Save sensor data to disk
sensor.save_to_disk("/path/to/output");
```

### Map Operations
```rust
// Get map topologies
let topologies = map.topologies();
```

### Traffic Light Operations
```rust
// Reset traffic light group
traffic_light.reset_group();
```

### And Many More
- Optical flow and DVS event API
- Walker bone control API
- Walker AI controller API
- Batched command API
- Recording and playback API
- Debug visualization API
- Vehicle failure state and telemetry API

**Full commit history**: See `~/repos/carla-rust/` git log since 2024-10-20

---

## Integration Steps

### 1. Update Cargo.toml

Edit `src/autoware_carla_bridge/Cargo.toml`:

```toml
[dependencies]
# CARLA and utilities
carla = { version = "0.12.0", path = "../../carla-rust/carla" }
# ... rest of dependencies
```

**Note**: The path is relative to the Cargo.toml file location:
- Cargo.toml location: `src/autoware_carla_bridge/Cargo.toml`
- Target path: `../../carla-rust/carla` → `~/repos/carla-rust/carla`

### 2. Update Makefile (Optional)

Add CARLA_VERSION support to Makefile:

```makefile
# At top of Makefile
CARLA_VERSION ?= 0.9.16

# In build targets
build:
	@# ... existing setup.sh sourcing ...
	CARLA_VERSION=$(CARLA_VERSION) colcon build --symlink-install \
		--packages-up-to autoware_carla_bridge \
		--cargo-args --release
```

### 3. Clean and Rebuild

```bash
make clean
make build
```

Or with specific CARLA version:
```bash
CARLA_VERSION=0.9.15 make build
```

### 4. Verify Integration

Check that the local carla crate is being used:

```bash
# Should show path to local carla-rust
grep -A5 "\[dependencies\]" src/autoware_carla_bridge/Cargo.toml

# Check build uses local version
CARLA_VERSION=0.9.16 cargo build --manifest-path src/autoware_carla_bridge/Cargo.toml -vv 2>&1 | grep "carla v0.12.0"
# Should show: Compiling carla v0.12.0 (/home/aeon/repos/carla-rust/carla)
```

---

## API Migration Opportunities

With the local carla-rust, we can leverage new APIs to improve our bridge:

### 1. Proper Actor Cleanup

**Before** (current):
```rust
// Actors are not explicitly destroyed, rely on CARLA's garbage collection
```

**After** (with new API):
```rust
impl Drop for SensorBridge {
    fn drop(&mut self) {
        if self._actor.destroy() {
            log::info!("Successfully destroyed sensor actor");
        } else {
            log::warn!("Failed to destroy sensor actor");
        }
    }
}
```

### 2. Efficient World Loading

**Before** (current):
```rust
// No way to avoid reloading same map
world.reload_world();
```

**After** (with new API):
```rust
// Only reloads if map is different
let world = client.load_world_if_different("Town03");
```

### 3. Sensor Data Recording

**New capability**:
```rust
// Save sensor data to disk for debugging
if debug_mode {
    sensor.save_to_disk(&format!("debug/{}_sensor_data", sensor_name));
}
```

---

## Testing Across CARLA Versions

With multi-version support, we can test against different CARLA versions:

```bash
# Test with CARLA 0.9.14
CARLA_VERSION=0.9.14 make build
./scripts/test_with_carla_0914.sh

# Test with CARLA 0.9.15
CARLA_VERSION=0.9.15 make build
./scripts/test_with_carla_0915.sh

# Test with CARLA 0.9.16 (default)
make build
./scripts/test_with_carla_0916.sh
```

---

## Version-Specific APIs

Some APIs are only available in specific CARLA versions:

```rust
// Only available in CARLA 0.9.16
#[cfg(carla_0916)]
let world = client.load_world_if_different("Town03");

// Fallback for older versions
#[cfg(not(carla_0916))]
let world = client.load_world("Town03");
```

**Current conditional compilation flags**:
- `carla_0914` - CARLA 0.9.14 features
- `carla_0915` - CARLA 0.9.15 features
- `carla_0916` - CARLA 0.9.16 features

---

## Troubleshooting

### "cannot find crate `carla`"

**Cause**: Path is incorrect

**Fix**: Verify relative path from Cargo.toml to carla-rust:
```bash
ls ../../carla-rust/carla/Cargo.toml  # From src/autoware_carla_bridge/
```

### "undefined reference to `carla_client_XXX`"

**Cause**: Wrong CARLA version selected

**Fix**: Ensure CARLA_VERSION matches your CARLA installation:
```bash
# Check your CARLA version
~/carla/CARLA_0.9.15/CarlaUE4.sh --version

# Build with matching version
CARLA_VERSION=0.9.15 make build
```

### Build takes a long time on first use

**Expected**: First build downloads prebuilt `libcarla_client` (~50-80MB depending on version) and compiles Rust bindings (~30-40 seconds)

**Subsequent builds**: Only recompile changed crates (fast)

### Multiple LLVM versions conflict

**Cause**: System has LLVM 14+ but carla-rust requires LLVM 11-13

**Fix**: Install compatible LLVM version:
```bash
sudo apt install clang-12 libclang-12-dev
export LLVM_CONFIG_PATH=/usr/bin/llvm-config-12
export LIBCLANG_PATH=/usr/lib/llvm-12/lib
```

---

## Development Workflow

### Making Changes to carla-rust

1. Make changes in `~/repos/carla-rust/`
2. Test changes:
   ```bash
   cd ~/repos/carla-rust
   make build
   cargo run --example spawn_vehicle --profile dev-release
   ```
3. Changes automatically available to autoware_carla_bridge (path dependency)
4. Rebuild bridge:
   ```bash
   cd ~/repos/ros_zenoh_bridge
   make build
   ```

### Contributing Back

1. Create feature branch in carla-rust:
   ```bash
   cd ~/repos/carla-rust
   git checkout -b feature/my-improvement
   ```
2. Make changes and test
3. Commit and push:
   ```bash
   git commit -m "Add new feature"
   git push origin feature/my-improvement
   ```
4. Create pull request to carla-rust repository

---

## Recommendations

### Immediate Actions

1. ✅ **Switch to path dependency** - Enables access to latest APIs
2. ✅ **Test build with CARLA 0.9.15** - Verify multi-version support works
3. ✅ **Add ActorBase::destroy()** - Proper resource cleanup in bridge
4. ✅ **Update CLAUDE.md** - Document the integration

### Future Enhancements

1. **Leverage new APIs**: Use `load_world_if_different()`, `save_to_disk()`, etc.
2. **Multi-version testing**: Test bridge with all 3 CARLA versions
3. **Contribute improvements**: Report issues and contribute fixes back to carla-rust
4. **Version-specific features**: Use conditional compilation for version-specific optimizations

---

## Summary

Switching to the local carla-rust repository provides:
- ✅ Latest API features and bug fixes
- ✅ Multi-version CARLA support (0.9.14, 0.9.15, 0.9.16)
- ✅ Flexibility for local development
- ✅ Contribution pathway to carla-rust
- ✅ Better resource management with new APIs

**Recommended**: Make this change now to benefit from ongoing carla-rust improvements and prepare for future CARLA version support.

---

**Last Updated**: 2025-10-28
