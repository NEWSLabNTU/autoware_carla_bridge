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
count=0

# Find all *_msgs directories in the Autoware install
for pkg_path in "$AUTOWARE_INSTALL"/*_msgs; do
    if [ -d "$pkg_path" ]; then
        pkg=$(basename "$pkg_path")
        # Only link Autoware and Tier4 message packages
        if [[ "$pkg" == autoware_*_msgs ]] || [[ "$pkg" == tier4_*_msgs ]]; then
            # Remove existing symlink if present
            rm -f "$INTERFACE_DIR/$pkg"
            # Create symlink (relative path from src/interface/ to src/external/autoware/...)
            ln -s "../external/autoware/install/$pkg/share/$pkg" "$INTERFACE_DIR/$pkg"
            echo "  ✓ Linked $pkg"
            count=$((count + 1))
        fi
    fi
done

if [ $count -eq 0 ]; then
    echo "  ⚠ No Autoware/Tier4 message packages found in $AUTOWARE_INSTALL"
    exit 1
fi

echo "Interface packages linked successfully! ($count packages)"
