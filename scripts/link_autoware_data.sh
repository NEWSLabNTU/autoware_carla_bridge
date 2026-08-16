#!/usr/bin/env bash
# Mirror Autoware's packaged model directory into a writable one.
#
# TensorRT nodes build their engine from the ONNX on first use and then write the result
# *next to the ONNX*. The packaged data directory is root-owned, so that write fails --
# and it fails after a successful 33 s build, with a one-line
#
#     [E] [TRT] Fail to open engine file
#     Component constructor threw an exception: Failed to setup TensorRT engine
#
# which the composable-node loader swallows. The node never appears, the rest of the
# pipeline comes up fine, and the only outward symptom is a stage that publishes empty
# results forever. That is how traffic light classification was silently absent.
#
# Upstream's own default for data_path is $HOME/autoware_data for exactly this reason.
# This mirrors the packaged tree there: real directories, symlinked files. Engines are
# then written beside the symlinks, in the writable copy, and survive to the next launch
# instead of being rebuilt every time.
set -euo pipefail

SRC="${1:-/opt/autoware/1.5.0/data}"
DST="${2:-$HOME/autoware_data}"

if [ ! -d "$SRC" ]; then
    echo "No packaged data directory at $SRC" >&2
    exit 1
fi

echo "Mirroring $SRC -> $DST (real dirs, symlinked files)"
linked=0
while IFS= read -r -d '' path; do
    rel="${path#"$SRC"/}"
    if [ -d "$path" ]; then
        mkdir -p "$DST/$rel"
    else
        mkdir -p "$DST/$(dirname "$rel")"
        # Skip anything already materialised as a real file, and never clobber a built
        # engine with a link.
        if [ -e "$DST/$rel" ] && [ ! -L "$DST/$rel" ]; then
            continue
        fi
        ln -sfn "$path" "$DST/$rel"
        linked=$((linked + 1))
    fi
done < <(find "$SRC" -mindepth 1 -print0)

echo "  $linked file(s) linked"
echo "  existing engines kept: $(find "$DST" -name '*.engine' -type f 2>/dev/null | wc -l)"
