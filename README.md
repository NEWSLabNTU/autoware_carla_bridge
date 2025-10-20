# Autoware CARLA Bridge

A ROS 2 bridge for the CARLA autonomous driving simulator that publishes vehicle sensor data and state information as ROS topics for Autoware.

## Overview

This project provides a bridge between CARLA simulator and ROS 2, allowing CARLA vehicle data to be consumed by ROS-based autonomous driving stacks like Autoware. It is inspired by and builds upon the [zenoh_carla_bridge](https://github.com/evshary/zenoh_carla_bridge) project, but publishes data directly as ROS topics instead of Zenoh topics.

**Current Status**: This project is under active development. The autoware_carla_bridge is being developed as a translation of the zenoh_carla_bridge reference implementation. Currently, the zenoh_carla_bridge (included as a submodule) is used for initial testing and validation.

### Key Features

- Exposes CARLA vehicle data as ROS 2 topics
- Compatible with CARLA 0.9.15 (migration to 0.9.16 planned)
- Works with ROS 2 Humble on Ubuntu 22.04
- Designed to integrate with Autoware 2025.22

## Prerequisites

- **Operating System**: Ubuntu 22.04 LTS
- **ROS Version**: ROS 2 Humble
- **CARLA Version**: 0.9.15 (0.9.16 support coming soon)
- **Programming Languages**: Rust, Python 3.8+
- **Build Tools**: LLVM/Clang 12 (required for Rust CARLA bindings)

## Environment Setup

### 1. Install ROS 2 Humble

Follow the official ROS 2 Humble installation guide [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

### 2. Install Zenoh

Follow the official installation guide to install Zenoh [here](https://zenoh.io/docs/getting-started/installation/).

### 3. Install `rmw_zenoh_cpp`

```bash
sudo apt install ros-humble-rmw-zenoh-cpp
```

### 4. Install Rust Development Environment

```bash
# Install rustup
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Configure current shell
source "$HOME/.cargo/env"

# Verify installation
rustc --version
cargo --version
```

### 4.5. Install Uv (Python Package Manager)

```bash
# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# Configure current shell
source "$HOME/.cargo/env"

# Verify installation
uv --version
```

### 5. Download and Install CARLA 0.9.15

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

### 5.5. Configure LLVM/Clang for Rust CARLA Bindings

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

### 6. Setup carla_agent (Vehicle Spawning Tool)

```bash
# Navigate to carla_agent directory
cd ~/repos/autoware_carla_bridge/carla_agent

# Install dependencies using uv
uv sync

# Verify installation
uv run python simple_spawn.py --help
```

### 7. Clone and Build zenoh_carla_bridge

```bash
# Navigate back to project root
cd ~/repos/autoware_carla_bridge

# Download submodules
git submodule update --init --recursive

# Build zenoh_carla_bridge (reference implementation)
cd external/zenoh_carla_bridge
cargo build --release
cd ../..
```

### 8. Build autoware_carla_bridge (Work in Progress)

```bash
# Navigate to project root
cd ~/repos/autoware_carla_bridge

# Using Makefile (recommended)
make build      # Build with release-with-debug profile
make help       # Show all available targets

# Or using cargo directly
cargo build --profile release-with-debug
```

## Execution

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

### Testing with `autoware_carla_bridge` (Future)

This section will be applicable once autoware_carla_bridge implementation is complete.

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
autoware_carla_bridge/
├── carla_agent/               # Python tool for spawning vehicles in CARLA
│   ├── main.py               # Interactive manual control
│   ├── simple_spawn.py       # Automatic vehicle spawning
│   ├── simulation/           # Simulation utilities and sensor definitions
│   └── pyproject.toml        # Python dependencies (managed by uv)
├── external/
│   └── zenoh_carla_bridge/   # Reference implementation (submodule)
├── src/                       # ROS bridge source code (to be implemented)
├── Cargo.toml                 # Rust dependencies
└── README.md                  # This file
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
