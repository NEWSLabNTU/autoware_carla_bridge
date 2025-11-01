# Project Meta Information

This document provides an overview of the autoware_carla_bridge project, including goals, success criteria, risk management, and progress tracking.

---

## Project Overview

**Current State**: `zenoh_carla_bridge` uses Zenoh to bridge CARLA simulator data to ROS 2 systems through multiple modes (DDS, ROS2, RmwZenoh).

**Target State**: `autoware_carla_bridge` will use rclrs to publish CARLA data directly as native ROS 2 topics, eliminating the need for intermediate bridges.

**Timeline**: Estimated 4-6 weeks for complete migration and testing.

## Migration Goals

- ✅ Replace Zenoh with native ROS 2 communication (rclrs)
- ✅ Maintain compatibility with Autoware 2025.22
- ✅ Simplify codebase by removing bridge-specific logic
- ✅ Improve type safety with compile-time message type checking
- ✅ Achieve comparable or better performance
- ✅ Provide comprehensive testing and documentation

## Prerequisites

Before starting the migration, ensure:

- [ ] CARLA 0.9.15 is installed and working
- [ ] ROS 2 Humble is installed
- [ ] Autoware 2025.22 is installed (for ROS message types)
- [ ] Rust toolchain is configured (rustc, cargo)
- [ ] LLVM/Clang 12 is installed for CARLA Rust bindings
- [ ] `carla_agent` vehicle spawning tool is working
- [ ] `external/autoware` symlink points to Autoware installation
- [ ] Familiarity with both Zenoh and rclrs APIs (see `docs/zenoh-to-rclrs-api-comparison.md`)

---

## Success Criteria

The migration is considered successful when:

### Functional Requirements
- ✅ All sensor types publish to ROS 2 topics (Camera, LiDAR, IMU, GNSS)
- ✅ Vehicle control works bidirectionally (status out, commands in)
- ✅ Multiple vehicles can be bridged simultaneously
- ✅ Clock synchronization works correctly
- ✅ Integration with Autoware 2025.22 is functional

### Technical Requirements
- ✅ Zero Zenoh dependencies remain
- ✅ All ROS 2 topics use correct message types
- ✅ QoS profiles are appropriate for each topic type
- ✅ No CDR serialization code remains
- ✅ Code passes `make lint` with no warnings
- ✅ Code is formatted with `make format`

### Performance Requirements
- ✅ Topic publication rates match or exceed Zenoh version
- ✅ CPU usage is comparable or better
- ✅ Memory usage is stable (no leaks)
- ✅ Latency is acceptable for real-time control

### Documentation Requirements
- ✅ README.md accurately describes the project
- ✅ API documentation is complete
- ✅ Migration guide helps Zenoh users transition
- ✅ Examples and tutorials are working

### Testing Requirements
- ✅ All integration tests pass
- ✅ Autoware integration is verified
- ✅ Long-running stability test completes
- ✅ Error handling is robust

---

## Risk Management

### Identified Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Autoware message types unavailable in Rust | High | Medium | Generate from `.msg` files or use CDR temporarily |
| Performance regression vs. Zenoh | Medium | Low | Profile and optimize hot paths |
| rclrs API limitations | Medium | Low | Engage with ros2-rust community |
| Breaking changes during development | Medium | Medium | Use feature branch, frequent testing |
| CARLA API incompatibilities | Low | Low | Keep CARLA version pinned |

### Contingency Plans

- **If Autoware messages unavailable**:
  - Option 1: Generate Rust bindings using rosidl
  - Option 2: Define structs manually and use CDR
  - Option 3: Contribute message generation to ros2-rust project

- **If performance issues**:
  - Profile with flamegraph
  - Consider `MultiThreadedExecutor`
  - Optimize message copying
  - Use `Arc` more aggressively

- **If rclrs bugs found**:
  - Report to ros2-rust GitHub
  - Fork and patch if critical
  - Consider temporary workarounds

---

## Progress Tracking

Track progress by marking tasks complete in this document:

- `[ ]` = Not started
- `[~]` = In progress
- `[x]` = Complete
- `[!]` = Blocked

### Current Phase: Phase 7 (carla-rust Integration)

**Last Updated**: 2025-11-02

### Phase Completion Status

**✅ Phase 0: Preparation** - COMPLETE (2025-10-27)
- ✅ All documentation created (4 docs, 2,713 lines)
- ✅ Autoware environment configured
- ✅ Colcon workspace structure created
- ✅ Three-stage build system implemented
- ✅ All bridge types reviewed and understood

**✅ Phase 1: Core Infrastructure** - COMPLETE (2025-10-22)
- ✅ Zenoh → rclrs migration complete (12 files, ~800 lines changed)
- ✅ All Zenoh dependencies removed
- ✅ Mode enum and liveliness tokens removed
- ✅ Code compiles and passes lint checks (zero warnings)
- ✅ Binary built: 9.3 MB
- ✅ .cargo/config.toml generated with 50+ message packages

**✅ Phase 2: Clock and Simple Publishers** - COMPLETE (Code: 2025-10-22, Runtime: 2025-10-31)
- ✅ Clock publisher migrated to rclrs (src/clock.rs)
- ✅ Utility functions updated (src/utils.rs)
- ✅ No CDR serialization or Zenoh dependencies
- ✅ Code compiles successfully
- ✅ **Runtime testing PASSED** - Clock publishes to `/clock` topic successfully

**🔄 Phase 7: carla-rust Integration** - IN PROGRESS (Started 2025-10-29)
- ✅ Local carla-rust path dependency configured
- ✅ Build system verified
- ✅ Documentation created (`docs/carla-rust-integration.md`)
- ✅ Roadmap updated with enhancement tasks (Phase 2 marked complete)
- ⏳ Actor cleanup implementation (pending)
- ⏳ Multi-version CARLA testing (pending)

**📋 Phase 8: Architecture Refactoring - 1-to-1 Design** - PLANNED (2025-10-31)
- ✅ Architecture design documented
- ✅ 1-to-1 Autoware-centric design specified
- ✅ 9 subsections with detailed work items (8.1-8.9)
- ✅ 3 Architecture Decision Records created
- ✅ Multi-vehicle simulation pattern documented
- ⏳ Implementation pending (depends on Phase 2/3 testing)
- ⏳ ~100 tasks defined across all subsections
- **Estimated Duration**: 1-2 weeks
- **Prerequisites**: Phase 2/3 runtime validation

**⏳ Phase 3-6: Sensor/Vehicle Bridge Testing and Further Development** - PENDING
- Phase 3: Sensor bridge runtime verification (Camera, LiDAR, IMU, GNSS)
- Phase 4: Vehicle bridge runtime verification (control subscribers, status publishers)
- Phase 5: Testing and optimization
- Phase 6: Documentation and release
- Awaiting CARLA simulator testing
- Integration testing with Autoware
- Performance optimization

### Completed Items (All Phases)
- [x] ✅ Study Zenoh API usage
- [x] ✅ Study rclrs API
- [x] ✅ Create API comparison document (647 lines)
- [x] ✅ Create roadmap document (now split into multiple files)
- [x] ✅ Create message type migration guide (482 lines)
- [x] ✅ Complete Phase 1: Core Infrastructure migration
- [x] ✅ Remove all Zenoh dependencies
- [x] ✅ Remove Mode enum and liveliness logic
- [x] ✅ Update all 5 bridge types to rclrs
- [x] ✅ Complete Phase 2: Clock and Simple Publishers migration
- [x] ✅ Migrate clock publisher to rclrs (src/clock.rs)
- [x] ✅ Update utility functions (src/utils.rs)
- [x] ✅ Remove CDR serialization from clock publisher
- [x] ✅ Runtime testing: Clock publisher verified with CARLA (2025-10-31)
- [x] ✅ Confirmed /clock topic publishes successfully with ros2 topic echo
- [x] ✅ Integrate local carla-rust repository
- [x] ✅ Switch to path dependency for carla crate
- [x] ✅ Create carla-rust integration documentation (386 lines)
- [x] ✅ Configure direnv for automatic environment
- [x] ✅ Simplify Makefile (removed manual sourcing)
- [x] ✅ Fix Makefile typo (build-packages → build-bridge)

### Current Focus (Phase 7)
1. ⏳ Implement actor cleanup with `ActorBase::destroy()`
2. ⏳ Add efficient world loading with `Client::load_world_if_different()`
3. ⏳ Multi-version CARLA support testing (0.9.14, 0.9.15, 0.9.16)
4. ⏳ Documentation updates for new APIs

### Next Steps
1. **Immediate**: Begin Phase 3 runtime testing with CARLA simulator
   - Verify clock publisher (Phase 2) publishes to `/clock` topic
   - Test sensor bridges (Camera, LiDAR, IMU, GNSS) with spawned vehicles
   - Validate vehicle control and status publishing
2. Add Drop implementation with `actor.destroy()` to SensorBridge and VehicleBridge
3. Add world loading utility with version-conditional compilation
4. Test bridge with multiple CARLA versions (0.9.14, 0.9.15, 0.9.16)
5. Explore advanced carla-rust APIs (walker control, batch operations, debug visualization)

### Metrics
- **Documentation**: 5 guides (includes roadmap), 3,600+ total lines
- **Phases Total**: 8 phases (0-7 plus Phase 8)
- **Phases Complete**: 3 of 8 (Phase 0, 1, 2) - **Phase 2 runtime verified**
- **Phases In Progress**: Phase 7 (carla-rust Integration)
- **Phases Pending**: Phase 3-6, 8 (Architecture Refactoring)
- **Code Changes**: 15 files modified, ~800 lines changed, ~300 lines removed
- **Build Time**: ~5.5 minutes (first build), ~3 minutes (incremental)
- **Binary Size**: 9.3 MB
- **Lint Warnings**: 0
- **Compilation Status**: ✅ Success
- **Runtime Testing**: ✅ **Phase 2 PASSED** - Clock publisher verified (2025-10-31)
- **Architecture Design**: ✅ **DOCUMENTED** - 1-to-1 design, 3 ADRs (2025-10-31)

---

**Document Version**: 1.0
**Last Updated**: 2025-11-02
**Related Documents**:
- [roadmap.md](../roadmap.md) - Index and navigation
- [architecture.md](architecture.md) - Design decisions and ADRs
- [core-migration.md](core-migration.md) - Phases 0-2 (completed)
- [feature-implementation.md](feature-implementation.md) - Phases 3-6 (pending)
- [enhancements.md](enhancements.md) - Phases 7-8 (in progress/planned)
