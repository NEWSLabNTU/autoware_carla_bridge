# 012 — An unrecognised sensor blueprint is silently classified as a camera

**Severity**: Low
**Component**: `src/acb_bridge/src/sensor_config.rs`, `SensorDefinition::sensor_type`
**Status**: Fixed

## What is wrong

```rust
} else {
    SensorType::Camera // Default fallback
}
```

Any blueprint in `vehicle_config.yaml` that does not contain `lidar`, `camera`, `imu`,
`gnss` or `radar` was classified as a camera. The sensor was spawned, handed to
`register_camera_rgb`, and died there on the missing `image_size_x` attribute — reported
as:

```
Failed to create sensor bridge for '<link>': CarlaIssue("no image_size_x")
```

which says nothing about the actual mistake, a wrong `blueprint:` line.

There was a second, quieter case. `sensor.lidar.ray_cast_semantic` contains `lidar`, so
it was classified `Lidar` and mapped to the plain ray-cast publisher. Semantic and plain
LiDAR are different measurement structs, so every frame failed to decode and the run
logged `Failed to transform lidar data` at the sensor's rate while publishing nothing. A
`LidarRayCastSemantic` publisher existed in `sensor_bridge.rs` the whole time and was
unreachable.

## Why it matters

This repository's roadmap opens with a phase called *Honest Failures* — "stop returning
success for work not done". A misconfigured sensor that reports a confusing error from
three layers down, or one that logs a decode failure forever while a working publisher
sits unused, is the same class of problem in a smaller place.

## Fix

`SensorType` gains `SemanticLidar` and `Unsupported`. The classifier checks `semantic`
before falling through to plain LiDAR, and unrecognised blueprints become `Unsupported`
rather than `Camera`.

`main.rs` grows one `bridge_sensor_type()` helper that both passes (topic registration
and bridge creation) now share, so they cannot disagree about what is supported. An
unsupported sensor logs, once, what is actually wrong:

```
Sensor '<link>' has a blueprint this bridge does not recognise; it will publish
nothing. Check its `blueprint` in vehicle_config.yaml
```

The sensor is still spawned in CARLA — it exists in the world and is destroyed with the
rest on cleanup — it simply has no publisher, which is what "unsupported" means.
