# Issues

One file per defect or gap found in `acb_bridge`, numbered `NNN-slug.md`. Each states
what is wrong, why it matters to Autoware, how it was confirmed, and what closing it
requires. Roadmap phases live in `../roadmap/`; this directory is for specific,
verifiable defects.

Opened by the audit of 2026-08-16, which read every publisher and subscriber in
`acb_bridge` against Autoware 1.5.0's vehicle interface and CARLA 0.9.16's sensor
conventions.

| # | Title | Severity | Status |
|---|-------|----------|--------|
| [001](001-imu-orientation-is-constant.md) | IMU orientation is a constant, not the heading | High | Fixed |
| [002](002-imu-gyroscope-sign-convention.md) | IMU angular velocity has the wrong sign on x and y | Medium | Fixed |
| [003](003-velocity-report-frame.md) | VelocityReport is world-frame and unsigned | High | Fixed |
| [004](004-gear-command-ignored.md) | Gear command ignored; GearReport hardcoded to DRIVE | High | Fixed |
| [005](005-turn-indicators-and-hazards.md) | Turn indicators and hazard lights go nowhere | Medium | Fixed |
| [006](006-hardcoded-max-steer-angle.md) | Max steer angle hardcoded instead of read from CARLA | Medium | Fixed |
| [007](007-ground-truth-twist-frame.md) | Ground-truth odometry twist is world-frame under `base_link` | Low | Fixed |
| [008](008-angular-velocity-units.md) | `Actor::angular_velocity()` units undocumented in CARLA's own API | Medium | Investigated |
| [009](009-steering-report-echoes-command.md) | SteeringReport echoes the command instead of the measured angle | Low | Fixed |
| [010](010-actuation-status-not-published.md) | `/vehicle/status/actuation_status` is never published | Low | Fixed |
| [011](011-lidar-channel-assignment-cost.md) | LiDAR channel assignment is O(points x channels) per scan | Low | Fixed |
| [012](012-unknown-blueprint-becomes-a-camera.md) | An unrecognised sensor blueprint is silently classified as a camera | Low | Fixed |
| [013](013-individual-params-is-unreachable.md) | The sensor calibration package is unreachable from Autoware's launch | High | Fixed |
| [014](014-a-paused-simulation-looks-like-a-running-one.md) | A paused simulation is indistinguishable from a running one | High | Fixed |
| [015](015-sensors-destroyed-while-still-listening.md) | Sensors are destroyed while still listening | High | Partly fixed; root cause open |

## Severity

- **High** — wrong data reaches Autoware's control or localization path, or a
  documented Autoware feature cannot work at all.
- **Medium** — wrong data reaches a path Autoware does not currently depend on in this
  configuration, or a feature is silently absent.
- **Low** — fidelity, cost, or hygiene; nothing downstream breaks today.
