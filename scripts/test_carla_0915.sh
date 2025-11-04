#!/bin/bash
# Test bridge with CARLA 0.9.15

set -e

echo "Testing autoware_carla_bridge with CARLA 0.9.15"
echo "================================================"
echo

# Set CARLA version
export CARLA_VERSION=0.9.15
echo "CARLA_VERSION=$CARLA_VERSION"
echo

# Clean previous builds
echo "Cleaning previous builds..."
make clean-bridge
echo

# Build with CARLA 0.9.15
echo "Building bridge for CARLA 0.9.15..."
make build-bridge
echo

# Run basic tests
echo "Running tests..."
make test
echo

echo "✓ CARLA 0.9.15 compatibility test passed"
