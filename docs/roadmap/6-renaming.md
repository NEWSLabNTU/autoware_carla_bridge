# Renaming

Rename all packages to the `acb_*` (Autoware Carla Bridge) prefix for clarity and to avoid ambiguity with pure CARLA packages.

**Status**: ✅ **COMPLETE** (2026-03-29)

**Prerequisites**:
- ✅ Phases 1-4 complete (end-to-end autonomous driving working)

---

## Naming Convention

- **`acb_*`** — packages that integrate Autoware with CARLA (core product)
- **`carla_*`** — standalone CARLA utilities (no Autoware runtime dependency)

---

## 6.1 Rename Map

| Current Name                   | New Name                     | Type                               |
|--------------------------------|------------------------------|------------------------------------|
| `autoware_carla_bridge`        | `acb_bridge`                 | Rust binary (ROS 2 node)           |
| `carla_autoware_launch`        | `acb_launch`                 | Launch + config (ament_cmake)      |
| `carla_sensor_kit_launch`      | `acb_sensor_kit_launch`      | Launch (ament_cmake)               |
| `carla_sensor_kit_description` | `acb_sensor_kit_description` | URDF/xacro (ament_cmake)           |
| `carla_vehicle_launch`         | `acb_vehicle_launch`         | Launch (ament_cmake)               |
| `carla_vehicle_description`    | `acb_vehicle_description`    | URDF/xacro (ament_cmake)           |
| `carla_demo_launch`            | `acb_demo_launch`            | Launch (ament_cmake)               |
| `carla_pilot`                  | `acb_pilot`                  | Python (ament_python)              |
| `individual_params`            | `acb_individual_params`      | Config (ament_cmake)               |
| `manual_control`               | `carla_manual_control`       | Rust GUI (standalone)              |
| `carla_pcd_gen`                | `carla_pcd_gen`              | Rust CLI (standalone, unchanged)   |
| `carla_map_gen`                | `carla_map_gen`              | Python CLI (standalone, unchanged) |

---

## 6.2 Directory Structure After Rename

```
src/
├── acb_bridge/                        # Core bridge (Rust)
├── acb_launch/                        # Autoware launch + config
├── acb_demo_launch/                   # Demo orchestration
├── acb_pilot/                         # Autonomous driving pilot (Python)
├── acb_individual_params/             # Sensor calibrations
├── acb_sensor_kit_launch/
│   ├── acb_sensor_kit_description/
│   └── acb_sensor_kit_launch/
├── acb_vehicle_launch/
│   ├── acb_vehicle_description/
│   └── acb_vehicle_launch/
├── carla_pcd_gen/                     # Standalone CARLA tools
├── carla_map_gen/
└── carla_manual_control/
```

---

## 6.3 Rename Tasks

- [ ] Rename directories under `src/`
- [ ] Update `package.xml` names in all packages
- [ ] Update `Cargo.toml` package names for Rust crates
- [ ] Update `setup.py` / `setup.cfg` for Python packages
- [ ] Update all `<include>` and `find-pkg-share` references in launch files
- [ ] Update cross-package dependencies in `package.xml` files
- [ ] Update justfile recipes
- [ ] Update `CLAUDE.md` and documentation references
- [ ] Verify `just build` succeeds after rename
- [ ] Verify `just run-demo` works end-to-end
