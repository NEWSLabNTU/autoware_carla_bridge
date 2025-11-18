#!/usr/bin/env bash
set -e

URL="https://syncandshare.lrz.de/dl/fiBgYSNkmsmRB28meoX3gZ/.dir"
SHA256="6e582242198ae50aa1d1fd410b53609c8b7ecfc6b9696294c498646783ed4838"
DIR="$(cd "$(dirname "$0")/.." && pwd)/data"
ZIP="$DIR/carla-autoware-bridge.zip"
OUT="$DIR/carla-autoware-bridge"

# Skip if already extracted
[ -d "$OUT" ] && [ "$(ls -A "$OUT")" ] && exit 0

mkdir -p "$DIR"

# Download with resume
aria2c -c -x4 -s4 --file-allocation=none --auto-file-renaming=false \
    -d "$DIR" -o "$(basename "$ZIP")" "$URL"

# Verify checksum
echo "$SHA256  $ZIP" | sha256sum -c --quiet

# Extract
unzip -q -o "$ZIP" -d "$OUT"
