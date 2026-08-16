# 013 — The sensor calibration package is unreachable from Autoware's launch

**Severity**: High
**Component**: `src/individual_params` (was `src/acb_individual_params`)
**Status**: Fixed

## What is wrong

`autoware.launch.xml` builds the vehicle's URDF with a hardcoded package name:

```xml
<arg name="config_dir" value="$(find-pkg-share individual_params)/config/$(var vehicle_id)/$(var sensor_model)"/>
```

There is no argument for it — the package name `individual_params` is baked in. Every
Autoware sensor-kit vendor is expected to *overlay* that package with its own
calibration, which is why upstream's own repository is named `individual_params` and why
this workspace's copy carries the `sample_sensor_kit`, `awsim_sensor_kit` and
`single_lidar_sensor_kit` configs alongside `acb_sensor_kit` — it is a fork of it.

Phase 6 of the roadmap renamed it to `acb_individual_params` for consistency with the
other `acb_*` packages (`docs/roadmap/6-renaming.md`, "Renamed `individual_params/…`").
That rename put the calibration somewhere `find-pkg-share individual_params` can never
look. The launch resolves the *system* Autoware's `individual_params`, which knows
nothing about `acb_sensor_kit`, and dies while building `robot_description`:

```
error: [Errno 2] No such file or directory:
  '/opt/ros/humble/share/individual_params/config/default/acb_sensor_kit/sensors_calibration.yaml'
when evaluating expression 'xacro.load_yaml('$(arg config_dir)/sensors_calibration.yaml')'
when instantiating macro: sensor_kit_macro
```

## What it resolves to instead

Checked on the host where this surfaced:

- The Autoware 1.5.0 Debian at `/opt/autoware/1.5.0` ships **no** `individual_params`
  package at all. This workspace is the only intended provider.
- That host also had `autoware-full 2025.2` installed into `/opt/ros/humble`, which
  *does* ship an `individual_params` — carrying `sample_sensor_kit`,
  `awsim_sensor_kit`, `awsim_labs_sensor_kit` and `single_lidar_sensor_kit`, but of
  course not `acb_sensor_kit`.

So the lookup did not fail loudly. It found a real package that simply did not have our
kit in it, and reported a missing YAML file three layers down inside a xacro macro. With
only Autoware 1.5.0 installed it would instead fail outright with `PackageNotFoundError`.

The blast radius is total, not partial: no `robot_description` means no `/tf_static`, so
acb_bridge never detects Autoware, never attaches sensors, and the whole stack waits
forever with nothing to say about why.

## Fix

Rename the package back to `individual_params` (directory `src/individual_params`,
`package.xml` `<name>`, `CMakeLists.txt` `project()`). The workspace overlay then takes
precedence over whatever the Autoware install ships, which is the mechanism Autoware
intends and the one every other sensor kit uses.

This is a deliberate exception to the `acb_*` naming convention from phase 6, for the
same reason `autoware_launch` and `tier4_perception_component` are not renamed: the name
is an interface, not a label.

## Verified

`play_launch dump acb_launch carla_simulator.launch.xml` now resolves
`robot_description` — `ros2 pkg prefix individual_params` points at the workspace copy,
and the xacro expands.
