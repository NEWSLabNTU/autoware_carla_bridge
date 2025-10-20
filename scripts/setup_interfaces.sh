#!/bin/bash
set -e

# Check if autoware symlink exists
if [ ! -L "src/external/autoware" ]; then
    echo "ERROR: src/external/autoware symlink not found!"
    echo "Please create it first: ln -s /path/to/autoware/workspace src/external/autoware"
    exit 1
fi

AUTOWARE_INSTALL="src/external/autoware/install"
INTERFACE_DIR="src/interface"

# Check if autoware install exists
if [ ! -d "$AUTOWARE_INSTALL" ]; then
    echo "ERROR: $AUTOWARE_INSTALL not found!"
    echo "Please ensure Autoware is built and the symlink points to the correct workspace"
    exit 1
fi

# Create interface directory
mkdir -p "$INTERFACE_DIR"

# Symlink all Autoware and Tier4 message packages
echo "Symlinking Autoware interface packages..."
for pkg in autoware_*_msgs tier4_*_msgs; do
    if [ -d "$AUTOWARE_INSTALL/$pkg" ]; then
        ln -sf "../../external/autoware/install/$pkg/share/$pkg" "$INTERFACE_DIR/$pkg"
        echo "  ✓ Linked $pkg"
    fi
done

echo "Interface packages linked successfully!"
