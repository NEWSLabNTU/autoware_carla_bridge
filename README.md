# ROS CARLA Bridge

A ROS 2 bridge for the CARLA autonomous driving simulator that publishes vehicle sensor data and state information as ROS topics.

## Overview

This project provides a bridge between CARLA simulator and ROS 2, allowing CARLA vehicle data to be consumed by ROS-based autonomous driving stacks like Autoware. It is inspired by and builds upon the [zenoh_carla_bridge](https://github.com/evshary/zenoh_carla_bridge) project, but publishes data directly as ROS topics instead of Zenoh topics.

**Current Status**: This project is under active development. The ros_carla_bridge is being developed as a translation of the zenoh_carla_bridge reference implementation. Currently, the zenoh_carla_bridge (included as a submodule) is used for initial testing and validation.

### Key Features

- Exposes CARLA vehicle data as ROS 2 topics
- Compatible with CARLA 0.9.15 (migration to 0.9.16 planned)
- Works with ROS 2 Humble on Ubuntu 22.04
- Designed to integrate with Autoware 2025.22

## Prerequisites

- **Operating System**: Ubuntu 22.04 LTS
- **ROS Version**: ROS 2 Humble
- **CARLA Version**: 0.9.15 (0.9.16 support coming soon)
- **Programming Language**: Rust

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

### 6. Clone and Build zenoh_carla_bridge

```bash
# Download submodules
git submodule update --init --recursive

# Build zenoh_carla_bridge (reference implementation)
cd external/zenoh_carla_bridge
cargo build --release
cd ../..
```

### 7. Build ros_carla_bridge (Work in Progress)

<!-- TODO -->

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

#### Step 2: Start Zenoh Router

```bash
# In a new terminal
zenohd
```

#### Step 3: Run `zenoh_carla_bridge`

```bash
# In a new terminal
cd ~/repos/ros_carla_bridge/external/zenoh_carla_bridge

# Run the bridge
cargo run --release
```

#### Step 4: Verify Zenoh Topics

```bash
# In a new terminal
# Install Zenoh tools if not already installed
cargo install zenoh --features unstable --all-targets

# List Zenoh topics
z_get "/**"
```

#### Step 5: Run Autoware with Zenoh (Optional)

```bash
# In a new terminal, launch Autoware 2025.22
# Configure Autoware to use rmw_zenoh_cpp
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# Follow Autoware's specific launch instructions for your setup
```

---

### Testing with `ros_carla_bridge` (Future)

This section will be applicable once ros_carla_bridge implementation is complete.

#### Step 1: Start CARLA Simulator

```bash
# Navigate to CARLA directory
cd ~/carla/CARLA_0.9.15

# Launch CARLA server
./CarlaUE4.sh
```

#### Step 2: Run ros_carla_bridge

```bash
# In a new terminal
cd ~/repos/ros_carla_bridge

# Source ROS environment
source /opt/ros/humble/setup.bash

# Run the bridge
cargo run --release
```

#### Step 3: Verify ROS Topics

```bash
# In a new terminal
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /carla/<topic_name>
```

#### Step 4: Run Autoware (Optional)

```bash
# In a new terminal, launch Autoware 2025.22
# Follow Autoware's specific launch instructions for your setup
```

## Project Structure

```
ros_carla_bridge/
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
- [ ] Test zenoh_carla_bridge with CARLA 0.9.15
- [ ] Validate Zenoh topics and data flow

### Phase 2: ROS Bridge Implementation
- [ ] Translate zenoh_carla_bridge to ros_carla_bridge
- [ ] Implement basic CARLA connection
- [ ] Publish vehicle state as ROS topics
- [ ] Publish sensor data as ROS topics (camera, lidar, etc.)

### Phase 3: Integration and Migration
- [ ] Test ros_carla_bridge with Autoware 2025.22
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
