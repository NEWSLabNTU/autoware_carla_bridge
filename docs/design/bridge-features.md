# Bridge Features

Feature overview of the Autoware CARLA Bridge and how it improves on the TUMFTM Carla-Autoware-Bridge.

---

## Architecture

| | TUMFTM | Ours |
|--|--------|------|
| Language | Python | Rust |
| Process model | 5+ processes (carla_ros_bridge, carla_autoware_bridge, carla_ackermann_control, spawn_ego_vehicle, carla_manual_control) | Single process |
| ROS 2 transport | Python rclpy via carla_ros_bridge | Native rclrs (direct DDS) |
| CARLA API access | Indirect, through carla_ros_bridge | Direct, via carla-rust bindings |
| Serialization | Python CDR overhead | Zero-copy ROS 2 message construction |
| Dependency | Requires unmaintained carla_ros_bridge fork | Standalone, no external bridge dependency |

## Sensor Pipeline

| | TUMFTM | Ours |
|--|--------|------|
| Configuration source | Static JSON (`objects.json`) | YAML config + Autoware TF tree (URDF-derived positions) |
| Sensor discovery | Hardcoded per-sensor Python nodes | Bridge reads `vehicle_config.yaml`, looks up TF for positions at runtime |
| Supported sensors | Camera, LiDAR, GNSS, IMU, collision, lane invasion | Camera, LiDAR, GNSS, IMU |
| LiDAR format | PointCloud2 | PointCloud2 with PointXYZIRC (NDT-compatible, 6 fields) |
| Perception | Ground-truth bounding boxes from CARLA | Real LiDAR perception via lidar_centerpoint |

Using real perception is more realistic -- it exercises Autoware's full sensing-to-planning pipeline rather than bypassing it with simulator ground truth.

## Localization

| | TUMFTM | Ours |
|--|--------|------|
| Initialization | Manual 2D Pose Estimate in RViz | Automatic via GNSS -> gnss_poser -> NDT align |
| Map projector | Local coordinates | TransverseMercator (matches CARLA's `+proj=tmerc`) |
| Pipeline | Odometry from carla_ros_bridge -> direct pose publish | GNSS NavSatFix -> gnss_poser -> autoware_automatic_pose_initializer -> NDT scan matching |

Automatic localization eliminates a manual step that blocks unattended operation.

## Vehicle Control

| | TUMFTM | Ours |
|--|--------|------|
| Control input | `AckermannControlCommand` (converted from Autoware Control) | `autoware_control_msgs/Control` (direct) |
| Steering conversion | `tire_angle * 1.2` (hardcoded multiplier for Volkswagen T2) | `tire_angle / max_steer_angle` (linear, configurable via `vehicle_info.param.yaml`) |
| Throttle/brake | Forwarded to separate `carla_ackermann_control` process | Direct: `acceleration / max_accel` split into throttle or brake |
| Status publishers | 3 (velocity, steering, control_mode) | 4 (velocity, steering, control_mode, gear) |
| Control pipeline | Autoware -> Python converter -> Ackermann controller -> carla_ros_bridge -> CARLA | Autoware -> Rust bridge -> CARLA (single hop) |

## Simulation Mode

| | TUMFTM | Ours |
|--|--------|------|
| CARLA mode | Synchronous (coupled to bridge process) | Synchronous with dedicated ticker (`demo_scenario.py`) |
| Tick rate | Variable | Fixed 20 Hz (deterministic) |
| Ticker design | Bridge is the ticker | Scenario script is the sole ticker; bridge passively waits |
| Multi-client | Bridge must tick | Any number of clients can read; only ticker calls `world.tick()` |

Separating the ticker from the bridge allows multiple bridges (one per vehicle) to share a single CARLA world without tick conflicts.

## Robustness

| | TUMFTM | Ours |
|--|--------|------|
| CARLA connection | Fails on disconnect | Infinite retry with 5s backoff, panic catching |
| Autoware detection | Assumes running | Polls `/robot_description` with infinite wait |
| Ctrl-C handling | Standard Python signal | Graceful shutdown within 100ms at any phase |
| Sensor cleanup | Manual | Automatic on Autoware loss (sensors destroyed, vehicle preserved) |
| Stale actor handling | None | Hero vehicle stability check (survives 60 ticks before attaching) |
| MRM tuning | Default Autoware timeouts | CARLA-specific MRM handler (30s availability timeout) and component monitor (3s topic timeouts) |
| Diagnostic errors | Not addressed | 4 known non-critical errors documented; none in autonomous mode path |

## Companion Tools

Tools included in the repository beyond the bridge itself:

- **Vehicle monitor GUI** (`carla_manual_control`) -- Rust/macroquad chase camera with HUD (speed, heading, IMU, GNSS, collision detection)
- **Lanelet2 map generator** (`carla_map_gen`) -- generates Autoware-compatible lanelet2 maps from CARLA OpenDRIVE
- **Pointcloud map generator** (`carla_pcd_gen`) -- generates PCD maps from CARLA world geometry
- **Autonomous driving pilot** (`acb_pilot`) -- auto_drive (route + engage), demo_scenario (CARLA ticker + vehicle spawner), capture_poses (RViz pose recorder)
- **Demo orchestration** (`acb_demo_launch`) -- single command (`just run-demo`) launches Autoware + bridge + scenario + pilot + monitor

## Remaining Gaps

See [docs/roadmap/9-gap-analysis.md](../roadmap/9-gap-analysis.md) for details. Summary:

- Vehicle physics param extraction script (for switching CARLA blueprints)
- Optional steering calibration multiplier
- Optional ground-truth object publisher (for debugging without GPU perception)
- Multi-vehicle documentation

All low priority. The bridge is functionally complete.
