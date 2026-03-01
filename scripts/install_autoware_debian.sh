#!/usr/bin/env bash
set -e

echo "=== Installing Autoware 1.5.0 via Debian packages ==="

# Check if already installed
if [ -f /opt/autoware/1.5.0/setup.bash ]; then
    echo "Autoware 1.5.0 is already installed at /opt/autoware/1.5.0/"
    echo "To reinstall, run: sudo apt install --reinstall autoware-full-1-5-0"
    exit 0
fi

REPO_URL_BASE="https://github.com/NEWSLabNTU/autoware-localrepo/releases/download/1.5.0-1"

# Checksums
SHA256SUM_UBUNTU2204="9f433f7ae4642c9501b9b1f53853a3d724627cc2e85088ac6a2e2e21775898ea"
SHA256SUM_JETPACK62="5c50148e9d9ad5426e92fdee68d2ab22ca23f37d9de83fe9e7fd06db330f0ae0"

ARCH=$(uname -m)

if [[ "$ARCH" == "x86_64" ]]; then
    echo "Detected architecture: amd64 (x86_64)"
    DEB_FILE="autoware-localrepo-1-5-0_1.5.0-1ubuntu2204_all.deb"
    SHA256SUM="${SHA256SUM_UBUNTU2204}"
elif [[ "$ARCH" == "aarch64" ]]; then
    echo "Detected architecture: arm64 (aarch64) - Assuming JetPack 6.2"
    DEB_FILE="autoware-localrepo-1-5-0_1.5.0-1jetpack62_all.deb"
    SHA256SUM="${SHA256SUM_JETPACK62}"
else
    echo "Error: Unsupported architecture: $ARCH"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DEB_DOWNLOAD_DIR="${PROJECT_DIR}/data/autoware-debian"
mkdir -p "${DEB_DOWNLOAD_DIR}"

DOWNLOAD_URL="${REPO_URL_BASE}/${DEB_FILE}"
TEMP_DEB="${DEB_DOWNLOAD_DIR}/${DEB_FILE}"

# Install aria2c if not already installed
if ! command -v aria2c &> /dev/null; then
    echo "Installing aria2c for parallel downloads..."
    sudo apt update
    sudo apt install -y aria2
fi

# Download or verify existing file
DOWNLOAD_REQUIRED=false
if [[ -f "$TEMP_DEB" ]]; then
    echo "File ${DEB_FILE} already exists. Verifying checksum..."
    ACTUAL_SHA256SUM=$(sha256sum "$TEMP_DEB" | awk '{print $1}')
    if [[ "$ACTUAL_SHA256SUM" == "$SHA256SUM" ]]; then
        echo "Checksum matches. Skipping download."
    else
        echo "ERROR: Checksum mismatch for existing file: ${TEMP_DEB}"
        echo "  Expected: ${SHA256SUM}"
        echo "  Actual:   ${ACTUAL_SHA256SUM}"
        echo "Remove the file and re-run to download fresh."
        exit 1
    fi
else
    echo "Downloading ${DEB_FILE}..."
    DOWNLOAD_REQUIRED=true
fi

if "$DOWNLOAD_REQUIRED"; then
    aria2c "${DOWNLOAD_URL}" \
        --dir="${DEB_DOWNLOAD_DIR}" \
        --out="${DEB_FILE}" \
        --checksum=sha-256="${SHA256SUM}" \
        -x 10 -s 10 -k 1M
fi

echo "Installing Autoware localrepo..."
sudo apt update
sudo apt install -y "$TEMP_DEB"

# Run setup-prerequisites.sh
if [ -f /usr/share/autoware/setup-prerequisites.sh ]; then
    echo "Running setup-prerequisites.sh..."
    sudo /usr/share/autoware/setup-prerequisites.sh
else
    echo "Warning: /usr/share/autoware/setup-prerequisites.sh not found. Skipping."
fi

echo "Updating apt cache..."
sudo apt update

# Fix: time-daemon dependency may not be satisfiable; install chrony as fallback
if ! apt-cache search --names-only '^time-daemon$' | grep -q 'time-daemon'; then
    echo "'time-daemon' not found. Installing 'chrony' as a replacement."
    sudo apt install -y chrony
fi

echo "Installing autoware-full-1-5-0..."
sudo apt install -y autoware-full-1-5-0

echo ""
echo "=== Autoware 1.5.0 installed successfully ==="
echo "Setup file: /opt/autoware/1.5.0/setup.bash"
