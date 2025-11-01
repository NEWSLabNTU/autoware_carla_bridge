# autoware_carla_bridge Migration Roadmap

This document provides an index and navigation for the migration roadmap documentation.

---

## Quick Navigation

### 📋 Project Information
- **[Meta Information](roadmap/meta.md)** - Overview, goals, success criteria, risk management, and progress tracking

### 🏛️ Architecture & Design
- **[Architecture Design](roadmap/architecture.md)** - 1-to-1 design philosophy, core principles, startup sequence, and Architecture Decision Records (ADRs)

### 📂 Implementation Phases

#### Completed Phases
- **[Core Migration (Phases 0-2)](roadmap/core-migration.md)** - ✅ Preparation, core infrastructure, clock publisher
  - Phase 0: Preparation
  - Phase 1: Core Infrastructure
  - Phase 2: Clock and Simple Publishers

#### Pending Phases
- **[Feature Implementation (Phases 3-6)](roadmap/feature-implementation.md)** - ⏳ Sensor bridges, vehicle control, testing, release
  - Phase 3: Sensor Bridge Migration
  - Phase 4: Vehicle Bridge Migration
  - Phase 5: Testing and Optimization
  - Phase 6: Documentation and Release

#### Enhancement Phases
- **[Enhancements (Phases 7-8)](roadmap/enhancements.md)** - 🔄 carla-rust integration, architecture refactoring
  - Phase 7: carla-rust Integration and Enhancements (IN PROGRESS)
  - Phase 8: Architecture Refactoring - 1-to-1 Design (PLANNED)

---

## Document Overview

### [Meta Information](roadmap/meta.md)
- Project overview and target state
- Migration goals
- Prerequisites
- Success criteria (functional, technical, performance, documentation, testing)
- Risk management and contingency plans
- Progress tracking and metrics

### [Architecture Design](roadmap/architecture.md)
- 1-to-1 Autoware-centric design philosophy
- Core principles:
  1. One bridge per vehicle
  2. Bridge is passive adapter
  3. Root namespace topics
  4. Vehicle discovery and selection
- Revised startup sequence (9 steps)
- Multi-vehicle simulation example
- Alternative designs considered
- Consequences and trade-offs
- **Architecture Decision Records (ADRs)**:
  - ADR-001: 1-to-1 Bridge-Vehicle Mapping
  - ADR-002: Bridge Does Not Control Simulation
  - ADR-003: Root Namespace Topics

### [Core Migration](roadmap/core-migration.md) - ✅ COMPLETE
Completed phases covering the foundation of the migration:
- **Phase 0**: Development environment setup, codebase analysis, documentation
- **Phase 1**: Zenoh to rclrs migration, dependency updates, mode removal
- **Phase 2**: Clock publisher migration and runtime verification

### [Feature Implementation](roadmap/feature-implementation.md) - ⏳ PENDING
Pending phases for completing bridge functionality:
- **Phase 3**: Sensor bridges (Camera, LiDAR, IMU, GNSS)
- **Phase 4**: Vehicle control subscribers and status publishers
- **Phase 5**: Integration testing, performance optimization, code quality
- **Phase 6**: Documentation updates, migration guide, release preparation

### [Enhancements](roadmap/enhancements.md) - 🔄 IN PROGRESS / PLANNED
Enhancement phases for improved functionality and architecture:
- **Phase 7** (IN PROGRESS): carla-rust integration
  - Actor cleanup with destroy()
  - Efficient world loading
  - Debug data recording
  - Multi-version CARLA support
  - Advanced API exploration
- **Phase 8** (PLANNED): Architecture refactoring for 1-to-1 design
  - Remove simulation control
  - Add vehicle selection CLI
  - Simplify actor management
  - Root namespace topics
  - Vehicle respawn handling
  - Main loop refactoring
  - Scenario scripts
  - Testing and validation
  - TF publisher (optional)

---

## Current Status

**Overall Progress**: 3 of 8 phases complete

**Completed**:
- ✅ Phase 0: Preparation (2025-10-27)
- ✅ Phase 1: Core Infrastructure (2025-10-22)
- ✅ Phase 2: Clock and Simple Publishers - Runtime verified (2025-10-31)

**In Progress**:
- 🔄 Phase 7: carla-rust Integration (Started 2025-10-29)

**Planned**:
- 📋 Phase 8: Architecture Refactoring - 1-to-1 Design (Documented 2025-10-31)

**Pending**:
- ⏳ Phase 3: Sensor Bridge Migration
- ⏳ Phase 4: Vehicle Bridge Migration
- ⏳ Phase 5: Testing and Optimization
- ⏳ Phase 6: Documentation and Release

---

## Key Milestones

| Milestone | Status | Date |
|-----------|--------|------|
| Preparation and Documentation | ✅ Complete | 2025-10-27 |
| Core Zenoh → rclrs Migration | ✅ Complete | 2025-10-22 |
| Clock Publisher Runtime Verified | ✅ Complete | 2025-10-31 |
| Architecture Design Documented | ✅ Complete | 2025-10-31 |
| Local carla-rust Integration | ✅ Complete | 2025-10-29 |
| Roadmap Documentation Split | ✅ Complete | 2025-11-02 |
| Sensor Bridges Migrated | ⏳ Pending | TBD |
| Vehicle Control Migrated | ⏳ Pending | TBD |
| Testing and Optimization | ⏳ Pending | TBD |
| Documentation and Release | ⏳ Pending | TBD |
| carla-rust Enhancements | 🔄 In Progress | TBD |
| 1-to-1 Architecture Refactoring | 📋 Planned | TBD |

---

## Related Documentation

### Migration Guides
- [Zenoh to rclrs API Comparison](zenoh-to-rclrs-api-comparison.md)
- [Message Type Migration Guide](message-type-migration.md)
- [carla-rust Integration Guide](carla-rust-integration.md)

### Project Documentation
- [README.md](../README.md) - Main project documentation
- [CLAUDE.md](../CLAUDE.md) - Development session history
- [scripts/README.md](../scripts/README.md) - Testing workflows and script documentation

---

## Quick Links

**For New Contributors**:
1. Read [Meta Information](roadmap/meta.md) for project overview
2. Review [Architecture Design](roadmap/architecture.md) for design philosophy
3. Check [Core Migration](roadmap/core-migration.md) to understand what's been done
4. See [Feature Implementation](roadmap/feature-implementation.md) for current work

**For Testing**:
- See [scripts/README.md](../scripts/README.md) for test environment setup
- Run `make test-env` for automated testing

**For Development**:
- See [CLAUDE.md](../CLAUDE.md) for session history and learnings
- Check [Meta Information](roadmap/meta.md) for current phase status

---

**Document Version**: 2.0 (Reorganized)
**Last Updated**: 2025-11-02
**Migration Status**: Phase 2 Complete (Runtime Verified), Phase 7 In Progress
