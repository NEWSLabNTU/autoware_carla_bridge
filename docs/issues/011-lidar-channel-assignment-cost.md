# 011 — LiDAR channel assignment is O(points x channels) per scan

**Severity**: Low
**Component**: `src/acb_bridge/src/bridge/sensor_bridge.rs`, `publish_lidar`
**Status**: Fixed

## What is wrong

Each point's `channel` field was found by scanning the channel-boundary table linearly:

```rust
let channel = channel_boundaries
    .windows(2)
    .position(|w| idx >= w[0] && idx < w[1])
    .unwrap_or(0) as u16;
```

`channel_boundaries` is monotonically increasing and `idx` walks it in order, so this is
a linear scan of a sorted array, per point. The configured sensor is 128 channels at
2,621,440 points/s — 131,072 points per scan at 20 Hz — giving on the order of 8.4
million comparisons per scan, 168 million per second, on the CARLA callback thread.

## Why it matters

It is not fatal — the runs pass — but it is spent inside the sensor callback, which is
also the thread CARLA needs back promptly, and it scales with exactly the two knobs
(channels, points per second) that get raised for better perception. It also competes
with the two Autoware stacks that already push this host's LiDAR to 10 Hz, per
`docs/roadmap/README.md`.

## Fix

Two changes, both mechanical:

- The boundary table is sorted, so `partition_point` finds the channel in
  `log2(128) = 7` comparisons instead of an average of 64.
- The scan is in channel order, so the search can start from the previous point's
  channel; in practice this makes it a single comparison for all but 128 points per
  scan.

The second subsumes the first, but `partition_point` is kept as the general path because
CARLA does not promise the ordering.
