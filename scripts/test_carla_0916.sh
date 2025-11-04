#!/bin/bash
# Test bridge with CARLA 0.9.16 (default version)

set -e

echo "Testing autoware_carla_bridge with CARLA 0.9.16"
echo "================================================"
echo

# Set CARLA version
export CARLA_VERSION=0.9.16
echo "CARLA_VERSION=$CARLA_VERSION"
echo

# Clean previous builds
echo "Cleaning previous builds..."
make clean-bridge
echo

# Build with CARLA 0.9.16
echo "Building bridge for CARLA 0.9.16..."
make build-bridge
echo

# Run basic tests
echo "Running tests..."
make test
echo

echo "✓ CARLA 0.9.16 compatibility test passed"
