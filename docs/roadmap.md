# autoware_carla_bridge Roadmap

This document provides the main roadmap index and project overview for the autoware_carla_bridge migration project.

---

## Project Overview

**Current State**: `zenoh_carla_bridge` uses Zenoh to bridge CARLA simulator data to ROS 2 systems through multiple modes (DDS, ROS2, RmwZenoh).

**Target State**: `autoware_carla_bridge` uses rclrs to publish CARLA data directly as native ROS 2 topics, with automatic Autoware integration and sensor configuration.

**Timeline**:
- Core migration (Phases 0-2): ✅ Complete (3 weeks, Oct 2025)
- Architecture refactoring (Phase 8): ✅ Complete (1 week, Nov 2025)
- Autoware integration (Phases 3-6): ⏳ Pending (4-6 weeks estimated)
- Testing and release (Phases 9-10): ⏳ Pending (2-3 weeks estimated)
- **Total Remaining**: 6-9 weeks estimated for full Autoware integration

---

## Migration Goals

### Core Migration (✅ Complete)
- ✅ Replace Zenoh with native ROS 2 communication (rclrs)
- ✅ Maintain compatibility with Autoware 2025.02
- ✅ Simplify codebase by removing bridge-specific logic
- ✅ Improve type safety with compile-time message type checking
- ✅ Implement 1-to-1 Autoware-centric architecture

### Autoware Integration (⏳ Pending)
- ⏳ Implement Autoware instance detection via `/robot_description`
- ⏳ Parse URDF to extract sensor configuration automatically
- ⏳ Handle vehicle lifecycle tied to Autoware lifecycle
- ⏳ Support dynamic sensor configuration (no hardcoding)
- ⏳ Validate sensor data accuracy against Autoware topics
- ⏳ Achieve comparable or better performance
- ⏳ Provide comprehensive testing and documentation

---

## Success Criteria

### Functional Requirements (Core Migration) - ✅ Complete
- ✅ All sensor types publish to ROS 2 topics (Camera, LiDAR, IMU, GNSS)
- ✅ Vehicle control works bidirectionally (status out, commands in)
- ✅ Multiple vehicles can be bridged simultaneously (via ROS domains)
- ✅ Clock synchronization works correctly
- ✅ 1-to-1 bridge-vehicle architecture implemented

### Functional Requirements (Autoware Integration) - ⏳ Pending
- ⏳ Bridge detects running Autoware instance via `/robot_description`
- ⏳ Bridge parses URDF to extract sensor configuration
- ⏳ Bridge spawns CARLA vehicle with correct sensors based on URDF
- ⏳ Vehicle lifecycle tied to Autoware lifecycle (spawn/despawn)
- ⏳ Initial pose from `/initialpose` correctly places vehicle in CARLA
- ⏳ Sensor data from CARLA matches Autoware topic expectations
- ⏳ Full integration with Autoware 2025.02 planning simulator

### Technical Requirements
- ✅ Zero Zenoh dependencies remain
- ✅ All ROS 2 topics use correct message types
- ✅ QoS profiles are appropriate for each topic type
- ✅ No CDR serialization code remains
- ✅ Code passes `just lint` with no warnings
- ✅ Code is formatted with `just format`

### Performance Requirements
- ⏳ Topic publication rates match or exceed Zenoh version
- ⏳ CPU usage is comparable or better
- ⏳ Memory usage is stable (no leaks)
- ⏳ Latency is acceptable for real-time control

---

## Quick Navigation

### 📋 Functional Areas

#### Infrastructure
**[infrastructure.md](roadmap/infrastructure.md)** - Phases 0, 1, 7
- Environment setup and colcon workspace
- Zenoh → rclrs core migration
- carla-rust integration (local dependency)
- Build system (three-stage colcon)

#### Data Bridge
**[bridge.md](roadmap/bridge.md)** - Phases 2, 5, 6, 8
- Clock publisher (simple publisher proof-of-concept)
- Sensor data publishing (Camera, LiDAR, IMU, GNSS)
- Vehicle control integration (commands, status)
- 1-to-1 architecture refactoring (vehicle selection, root namespace)

#### Autoware Integration
**[integration.md](roadmap/integration.md)** - Phases 3-4
- Autoware instance detection (`/robot_description`, `/tf_static`)
- URDF parsing and sensor configuration
- TF2 transform parsing and coordinate conversion
- Vehicle lifecycle management (spawning, cleanup, teleportation)

#### Testing & Release
**[testing-and-release.md](roadmap/testing-and-release.md)** - Phases 9-10
- Integration testing (detection, sensors, control, lifecycle)
- Performance testing and benchmarks
- Documentation (README, integration guide, API docs)
- Release preparation (v0.13.0)

#### Feature Parity & Map Integration
**[autoware-feature-parity.md](roadmap/autoware-feature-parity.md)** - TUMFTM Comparison
- Vehicle control integration (control_cmd, status publishers)
- Feature comparison with TUMFTM bridge
- Performance benchmarks and improvements
- Vehicle calibration system

**[map-integration.md](roadmap/map-integration.md)** - CARLA Map Integration
- TUMFTM pre-converted maps (quick start)
- Automated point cloud generation
- OpenDRIVE to Lanelet2 conversion
- Traffic light integration
- Map validation and management

### 🏛️ Architecture & Design
- **[architecture.md](architecture.md)** - 1-to-1 design philosophy, core principles, ADRs
- **[autoware-integration-design.md](autoware-integration-design.md)** - Detailed Autoware integration design

---

## Current Status

**Overall Progress**: 5 of 10 phases complete, 2 in progress (50-70% estimated)

**Completed** (Phases 0-3, 7-8):
- ✅ Phase 0: Preparation (2025-10-27)
- ✅ Phase 1: Core Infrastructure (2025-10-22)
- ✅ Phase 2: Clock and Simple Publishers - Runtime verified (2025-10-31)
- ✅ Phase 3: Autoware Integration Foundation (2025-11-05)
- ✅ Phase 7: carla-rust Integration (2025-10-29 to 2025-11-04)
- ✅ Phase 8: Architecture Refactoring - 1-to-1 Design (2025-11-04)

**Current Phases**:
- 🔧 Phase 4: Vehicle Lifecycle Management (core module complete, integration pending)
- 🔧 Phase 5: Sensor Data Publishing (publishing code complete 80%, integration pending)
- ✅ Sensor Configuration System: CARLA sensor parameters (YAML config) - **NEW** (2025-11-08)

**Pending** (Phases 4-6, 9-10):
- 🔧 Phase 4: Vehicle Lifecycle Management (core module complete, integration pending)
- 🔧 Phase 5: Sensor Data Publishing (publishing code complete 80%, integration pending)
- ⏳ Phase 6: Vehicle Control Integration
- ⏳ Phase 9: Integration Testing
- ⏳ Phase 10: Documentation and Release

---

## Key Milestones

| Milestone                          | Status      | Date       |
|------------------------------------|-------------|------------|
| Preparation and Documentation      | ✅ Complete | 2025-10-27 |
| Core Zenoh → rclrs Migration       | ✅ Complete | 2025-10-22 |
| Clock Publisher Runtime Verified   | ✅ Complete | 2025-10-31 |
| Architecture Design Documented     | ✅ Complete | 2025-10-31 |
| Local carla-rust Integration       | ✅ Complete | 2025-10-29 |
| Roadmap Documentation Restructured | ✅ Complete | 2025-11-04 |
| 1-to-1 Architecture Implemented    | ✅ Complete | 2025-11-04 |
| Autoware Integration Design        | ✅ Complete | 2025-11-04 |
| Autoware Integration Foundation    | ✅ Complete | 2025-11-05 |
| Vehicle Lifecycle Management       | 🔧 In Progress (core module complete) | 2025-11-05 (started) |
| Sensor Data Publishing             | 🔧 In Progress (80% complete) | 2025-11-05 (documented) |
| CARLA Sensor Configuration System  | ✅ Complete | 2025-11-08 |
| TUMFTM Feature Parity Roadmap      | ✅ Complete | 2025-11-08 |
| Map Integration Roadmap            | ✅ Complete | 2025-11-08 |
| Vehicle Control Integration        | ⏳ Pending  | TBD        |
| Integration Testing                | ⏳ Pending  | TBD        |
| v0.13.0 Release                    | ⏳ Pending  | TBD        |

---

## Progress Metrics

**Project Completion**: 50% (Phases 0-3, 7-8 complete)

**Documentation**:
- 6 roadmap documents: infrastructure.md, bridge.md, integration.md, testing-and-release.md, **autoware-feature-parity.md**, **map-integration.md**
- 1 architecture document: architecture.md
- 1 integration design: autoware-integration-design.md
- 6 technical guides: API comparison, message migration, carla-rust integration, URDF integration, **sensor-configuration-strategy.md**, **automated-map-generation.md**
- 2 comparison documents: **architecture-comparison.md**, **tumftm-bridge-analysis.md**
- **Total**: 16 documents, ~8,000+ lines

**Phase Progress**:
- **Phases Total**: 10 phases (0-2, 3-6, 7-8, 9-10)
- **Phases Complete**: 5 of 10 (Phases 0, 1, 2, 3, 7, 8) - **50% complete**
- **Phases In Progress**: 2 of 10 (Phases 4, 5) - **~18% estimated** (Phase 4: ~10%, Phase 5: ~8%)
- **Phases Pending**: 3 of 10 (Phases 6, 9, 10) - **30% remaining**
- **Current Phases**:
  - Phase 4 (Vehicle Lifecycle - core module complete, integration pending)
  - Phase 5 (Sensor Publishing - code complete 80%, integration pending)

**Code Quality**:
- **Files Modified**: 17 files (~1,100 lines changed, ~300 lines removed)
  - New in Phase 4: vehicle_lifecycle.rs (~290 lines), vehicle_config.yaml (80 lines)
- **Build Time**: ~5.5 minutes (first build), ~3 minutes (incremental)
- **Binary Size**: 9.3 MB
- **Lint Warnings**: 0
- **Compilation Status**: ✅ Success (as of 2025-11-05)

**Testing Status**:
- **Runtime Testing**: ✅ Phase 2 PASSED - Clock publisher verified (2025-10-31)
- **Integration Testing**: ✅ Phase 3 PASSED - Autoware detection, URDF parsing, TF2, coordinate conversion (2025-11-05)
- **Full Integration Testing**: ⏳ Pending Phase 9
- **Performance Testing**: ⏳ Pending Phase 9

**Architecture**:
- ✅ 1-to-1 bridge-vehicle design implemented
- ✅ 3 Architecture Decision Records (ADRs) documented
- ✅ Autoware integration design documented
- ⏳ Autoware integration implementation pending

---

## Risk Management

### Identified Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Autoware message types unavailable in Rust | High | Low | Three-stage colcon build generates Rust bindings |
| Performance regression vs. Zenoh | Medium | Low | Profile and optimize hot paths |
| rclrs API limitations | Medium | Low | Engage with ros2-rust community |
| TF transform composition complexity | Medium | Medium | Extensive testing with sample_sensor_kit |
| Coordinate conversion edge cases | Medium | Medium | Comprehensive unit tests (100+ cases) |
| URDF parsing robustness | Medium | Medium | Handle malformed input gracefully |
| CARLA spawn failures | Low | Low | Detailed error messages, retry logic |

### Contingency Plans

**If Autoware messages unavailable**:
- Three-stage build system implemented (rosidl_generator_rs)
- .cargo/config.toml generation tested
- 50+ message packages successfully generated

**If performance issues**:
- Profile with flamegraph
- Consider `MultiThreadedExecutor`
- Optimize message copying
- Use `Arc` more aggressively

**If rclrs bugs found**:
- Report to ros2-rust GitHub
- Fork and patch if critical
- Consider temporary workarounds

**If TF/coordinate conversion issues**:
- Extensive unit testing (100+ test cases)
- Compare with known good transforms
- Visual verification in RViz
- Reference Autoware coordinate system

---

## Related Documentation

### Architecture & Design
- [architecture.md](architecture.md) - 1-to-1 design philosophy, ADRs
- [autoware-integration-design.md](autoware-integration-design.md) - Autoware integration detailed design

### Migration Guides
- [zenoh-to-rclrs-api-comparison.md](zenoh-to-rclrs-api-comparison.md) - API comparison guide
- [message-type-migration.md](message-type-migration.md) - Message type migration guide
- [carla-rust-integration.md](carla-rust-integration.md) - carla-rust local integration guide

### Project Documentation
- [README.md](../README.md) - Main project documentation
- [CLAUDE.md](../CLAUDE.md) - Development session history
- [scripts/README.md](../scripts/README.md) - Testing workflows and script documentation
- [scripts/autoware/README.md](../scripts/autoware/README.md) - Autoware launch scripts

---

## Quick Links

**For New Contributors**:
1. Read this roadmap for project overview
2. Review [architecture.md](architecture.md) for design philosophy
3. Check [infrastructure.md](roadmap/infrastructure.md) to understand what's been done
4. See [integration.md](roadmap/integration.md) for current work (Autoware integration)

**For Testing**:
- See [scripts/README.md](../scripts/README.md) for test environment setup
- See [testing-and-release.md](roadmap/testing-and-release.md) for testing plans
- Run `just test-env` for automated testing

**For Development**:
- See [CLAUDE.md](../CLAUDE.md) for session history and learnings
- Check this roadmap for current phase status
- Review [autoware-integration-design.md](autoware-integration-design.md) for implementation details

---

## Prerequisites

Before starting the Autoware integration, ensure:

- [ ] CARLA 0.9.14, 0.9.15, or 0.9.16 is installed and working
- [ ] ROS 2 Humble is installed
- [ ] Autoware 2025.02 is installed and configured
- [ ] Rust toolchain is configured (rustc, cargo)
- [ ] LLVM/Clang 12 is installed for CARLA Rust bindings
- [ ] `external/autoware` symlink points to Autoware installation
- [ ] Familiarity with both Zenoh and rclrs APIs (see API comparison guide)
- [ ] Understanding of URDF and TF2 concepts
- [ ] Sample map available at `$HOME/autoware_map/sample-map-planning/`

---

**Document Version**: 4.3 (Phases 4-5 In Progress)
**Last Updated**: 2025-11-05
**Migration Status**: Phases 0-3, 7-8 Complete (50%), Phases 4-5 In Progress (68% estimated)
**Next Milestone**: Phase 4-5 Integration (sensor spawning + lifecycle)
