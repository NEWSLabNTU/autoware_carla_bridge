# Autoware CARLA Bridge

ROS 2 bridge between Autoware and CARLA simulator, written in Rust.

## Overview

This project provides a bridge between CARLA simulator and ROS 2, allowing CARLA vehicle data to be consumed by ROS-based autonomous driving stacks like Autoware. It is built as a native ROS 2 node using `rclrs`, the Rust client library for ROS 2.

**Current Status**: Under active development. The bridge is being migrated from zenoh-based communication to native ROS 2 using rclrs. The [zenoh_carla_bridge](https://github.com/evshary/zenoh_carla_bridge) reference implementation is included for comparison.

### Key Features

- Native ROS 2 node (composable, discoverable)
- Exposes CARLA vehicle data as ROS 2 topics
- Compatible with CARLA 0.9.15
- Works with ROS 2 Humble on Ubuntu 22.04
- Designed to integrate with Autoware 2025.22
- Built with Rust for performance and safety

## Prerequisites

- **Operating System**: Ubuntu 22.04 LTS
- **ROS Version**: ROS 2 Humble
- **CARLA Version**: 0.9.15
- **Autoware**: Built Autoware 2025.22 workspace
- **Rust**: Stable toolchain
- **Python**: 3.8+ with uv package manager

## Initial Setup

### 1. Set up Autoware symlink

Create a symlink to your Autoware workspace:

```bash
# Adjust the path to point to your Autoware workspace
ln -s /path/to/your/autoware/workspace src/external/autoware
```

Verify the symlink points to a built Autoware workspace with an `install/` directory.

### 2. Initialize git submodules

```bash
git submodule update --init --recursive
```

This will clone:
- `src/ros2_rust/rosidl_rust` - Rust message generator for ROS 2
- `src/ros2_rust/rosidl_runtime_rs` - Runtime support for Rust messages
- `src/external/zenoh_carla_bridge` - Reference implementation (not built)

The repository also includes symlinks to required Autoware message packages in `src/interface/`:
- `autoware_vehicle_msgs` - Vehicle control and status messages
- `tier4_vehicle_msgs` - Tier4-specific vehicle messages
- `tier4_control_msgs` - Tier4 control messages

### 3. Install dependencies

```bash
make install-deps
```

This installs:
- `colcon-cargo` and `colcon-ros-cargo` - Colcon plugins for Rust
- `cargo-ament-build` - Cargo plugin for ROS 2 integration
- `libclang-dev` - Required for FFI bindings generation
- `python3-vcstool` - Version control tool

Note: The required Autoware message packages are already symlinked in `src/interface/` and tracked in git.

## Building

Build the workspace using colcon:

```bash
make build          # Debug build
# OR
make build-release  # Release build
```

This will:
1. Generate Rust bindings for all ROS 2 messages (standard + Autoware)
2. Create `.cargo/config.toml` with paths to generated crates
3. Build `autoware_carla_bridge` as a ROS 2 node
4. Install to `install/` directory

## Running

### Using ros2 launch (recommended)

```bash
make launch

# Or with custom parameters:
. install/setup.sh
ros2 launch autoware_carla_bridge autoware_carla_bridge.launch.xml \
  carla_host:=localhost \
  carla_port:=2000
```

### Using ros2 run

```bash
make run

# Or directly:
. install/setup.sh
ros2 run autoware_carla_bridge autoware_carla_bridge
```

## Development

### Format code
```bash
make format
```

### Run linter
```bash
make lint
```

### Clean build artifacts
```bash
make clean
```

## Troubleshooting

### "autoware symlink not found"
Create the symlink: `ln -s /path/to/autoware src/external/autoware`

### "colcon-cargo not found"
Run: `make install-deps`

### "failed to resolve patches"
Clean and rebuild: `make clean && make build`

### Message types not found
1. Ensure Autoware is built: `ls src/external/autoware/install/`
2. Verify symlinks: `ls -la src/interface/`
3. Rebuild: `make clean && make build`

## Additional Setup (Optional)

The following sections describe additional setup needed for running CARLA and spawning test vehicles.

### Install ROS 2 Humble

Follow the official ROS 2 Humble installation guide [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

### Install Rust Development Environment

```bash
# Install rustup
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Configure current shell
source "$HOME/.cargo/env"

# Verify installation
rustc --version
cargo --version
```

### Install Uv (Python Package Manager)

```bash
# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# Configure current shell
source "$HOME/.cargo/env"

# Verify installation
uv --version
```

### Download and Install CARLA 0.9.15

```bash
# Create CARLA directory
mkdir -p ~/carla
cd ~/carla

# Download CARLA 0.9.15 (Ubuntu version)
wget https://carla-releases.b-cdn.net/Linux/CARLA_0.9.15.tar.gz
wget https://carla-releases.b-cdn.net/Linux/AdditionalMaps_0.9.15.tar.gz

# Extract
tar -xzf CARLA_0.9.15.tar.gz
tar -xzf AdditionalMaps_0.9.15.tar.gz

# Install Python dependencies
pip3 install carla==0.9.15
```

### Configure LLVM/Clang for Rust CARLA Bindings

The Rust CARLA bindings (used by `carla-rust`) require LLVM/Clang to build. Without this configuration, the build process will hang on the CARLA dependency.

Reference: [carla-rust](https://github.com/jerry73204/carla-rust)

```bash
# Install Clang 12 and development libraries
sudo apt install clang-12 libclang-12-dev

# Set environment variables for LLVM/Clang
export LLVM_CONFIG_PATH=/usr/bin/llvm-config-12
export LIBCLANG_PATH=/usr/lib/llvm-12/lib
export LIBCLANG_STATIC_PATH=/usr/lib/llvm-12/lib
export CLANG_PATH=/usr/bin/clang-12
```

### Setup carla_agent (Vehicle Spawning Tool)

```bash
# Navigate to carla_agent directory
cd ~/repos/autoware_carla_bridge/carla_agent

# Install dependencies using uv
uv sync

# Verify installation
uv run python simple_spawn.py --help
```

## Testing with CARLA

### Testing with zenoh_carla_bridge (Current)

This section describes how to run the reference zenoh_carla_bridge implementation for initial testing.

#### Step 1: Start CARLA Simulator

```bash
# Navigate to CARLA directory
cd ~/carla/CARLA_0.9.15

# Launch CARLA server
./CarlaUE4.sh

# Optional: Launch with specific settings
# ./CarlaUE4.sh -quality-level=Low -RenderOffScreen
```

#### Step 2: Spawn Vehicles in CARLA

```bash
# In a new terminal
cd ~/repos/autoware_carla_bridge/carla_agent

# Spawn vehicles with sensors using simple_spawn.py
uv run python simple_spawn.py

# This will spawn Tesla Model 3 vehicles with:
# - GNSS sensor (ublox)
# - IMU sensor (tamagawa)
# - LiDAR sensor (top)
# - RGB Camera (traffic_light)
```

**Note**: The `simple_spawn.py` script spawns two vehicles (v1 and v2) at predefined positions with all necessary sensors. Leave this terminal running to keep the vehicles active.

#### Step 3: Start Zenoh Router

```bash
# In a new terminal
zenohd
```

#### Step 4: Run `zenoh_carla_bridge`

```bash
# In a new terminal
cd ~/repos/autoware_carla_bridge/external/zenoh_carla_bridge

# Run the bridge
cargo run --release
```

#### Step 5: Verify Zenoh Topics

```bash
# In a new terminal
# Install Zenoh tools if not already installed
cargo install zenoh --features unstable --all-targets

# List Zenoh topics
z_get "/**"

# You should see topics from the spawned vehicles, such as:
# - /autoware_v1/sensing/gnss/ublox/...
# - /autoware_v1/sensing/imu/tamagawa/...
# - /autoware_v1/sensing/lidar/top/...
# - /autoware_v1/sensing/camera/traffic_light/...
# - /autoware_v2/sensing/...
```

#### Step 6: Run Autoware with Zenoh (Optional)

```bash
# In a new terminal, launch Autoware 2025.22
# Configure Autoware to use rmw_zenoh_cpp
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# Follow Autoware's specific launch instructions for your setup
```

---

### Testing with `autoware_carla_bridge` (ROS 2 Native)

This is the main implementation using native ROS 2 communication via rclrs.

#### Step 1: Start CARLA Simulator

```bash
# Navigate to CARLA directory
cd ~/carla/CARLA_0.9.15

# Launch CARLA server
./CarlaUE4.sh
```

#### Step 2: Spawn Vehicles in CARLA

```bash
# In a new terminal
cd ~/repos/autoware_carla_bridge/carla_agent

# Spawn vehicles with sensors
uv run python simple_spawn.py
```

#### Step 3: Run autoware_carla_bridge

```bash
# In a new terminal
cd ~/repos/autoware_carla_bridge

# Source ROS environment
source /opt/ros/humble/setup.bash

# Run the bridge
make run
```

#### Step 4: Verify ROS Topics

```bash
# In a new terminal
source /opt/ros/humble/setup.bash

# List all ROS topics
ros2 topic list

# You should see topics from the spawned vehicles, such as:
# - /autoware_v1/sensing/gnss/ublox/...
# - /autoware_v1/sensing/imu/tamagawa/...
# - /autoware_v1/sensing/lidar/top/pointcloud
# - /autoware_v1/sensing/camera/traffic_light/image_raw
# - /autoware_v2/sensing/...

# Monitor a specific topic
ros2 topic echo /autoware_v1/sensing/gnss/ublox/nav_sat_fix
```

#### Step 5: Run Autoware (Optional)

```bash
# In a new terminal, launch Autoware 2025.22
# Follow Autoware's specific launch instructions for your setup
```

## Project Structure

```
.
├── src/
│   ├── autoware_carla_bridge/     # Main bridge package
│   │   ├── Cargo.toml             # Rust dependencies
│   │   ├── package.xml            # ROS 2 package manifest
│   │   ├── launch/                # Launch files
│   │   │   └── autoware_carla_bridge.launch.xml
│   │   └── src/                   # Rust source code
│   ├── interface/                 # Symlinked message packages (generated)
│   ├── ros2_rust/                 # Rust code generators (git submodules)
│   │   ├── rosidl_rust/
│   │   └── rosidl_runtime_rs/
│   └── external/
│       ├── autoware@              # Symlink to Autoware workspace
│       └── zenoh_carla_bridge/    # Reference implementation
├── scripts/
│   └── install_deps.sh            # Install colcon plugins
├── carla_agent/                   # Python tool for spawning vehicles
│   ├── simple_spawn.py
│   └── pyproject.toml
├── build/                         # Colcon build artifacts
├── install/                       # Colcon install directory
├── log/                           # Colcon build logs
├── Makefile
└── README.md
```

## Roadmap

### Phase 1: Setup and Reference Testing
- [x] Initial project setup
- [x] Add zenoh_carla_bridge as submodule
- [x] Convert carla_agent from Poetry to Uv
- [ ] Test vehicle spawning with carla_agent
- [ ] Test zenoh_carla_bridge with CARLA 0.9.15
- [ ] Validate Zenoh topics and data flow

### Phase 2: ROS Bridge Implementation
- [ ] Translate zenoh_carla_bridge to autoware_carla_bridge
- [ ] Implement basic CARLA connection
- [ ] Publish vehicle state as ROS topics
- [ ] Publish sensor data as ROS topics (camera, lidar, etc.)

### Phase 3: Integration and Migration
- [ ] Test autoware_carla_bridge with Autoware 2025.22
- [ ] Migrate to CARLA 0.9.16 support

## References

- [CARLA Simulator](https://carla.org/)
- [zenoh_carla_bridge](https://github.com/evshary/zenoh_carla_bridge) - Reference implementation using Zenoh
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [Autoware](https://autowarefoundation.github.io/autoware-documentation/)

## License

TBD

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.
