//! CARLA vehicle and sensor configuration module
//!
//! This module provides CARLA-specific configuration for vehicles and sensors: which sensors
//! to spawn and their CARLA blueprints/parameters. The file is the car's `carla_sensors.yaml`,
//! generated from its vehicle spec by sim_vehicle_spec (roadmap 051); the spec is the single
//! source of truth, and `spec_hash` names the spec it came from.
//!
//! ## Design Philosophy
//! - **Explicit over implicit**: Sensors are explicitly listed in the config file
//! - **No name-based inference**: We don't guess sensor types from URDF link names
//! - **TF for positions only**: URDF/TF is only used for sensor positions, not types
//!
//! ## Config File Format (carla_sensors.yaml)
//! ```yaml
//! spec_hash: "d5d4..."                # the vehicle spec this was generated from
//! vehicle:
//!   blueprint: "vehicle.tesla.model3"
//! sensors:
//!   velodyne_top:                     # Link name (must exist in TF)
//!     blueprint: "sensor.lidar.ray_cast"
//!     parameters:
//!       channels: "128"
//!       range: "200.0"
//! ```

use serde::{Deserialize, Serialize};
use std::{collections::HashMap, fs, path::Path};

use crate::error::{BridgeError, Result};

// ============================================================================
// VehicleConfig - the sensors to spawn, generated from the vehicle spec
// ============================================================================

/// Vehicle and sensor configuration loaded from vehicle_config.yaml
///
/// This is the **single source of truth** for which sensors to spawn.
/// Each sensor entry defines:
/// - Link name (key) - must exist in TF tree for position lookup
/// - CARLA blueprint - the sensor type to spawn
/// - Parameters - CARLA-specific sensor attributes
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VehicleConfig {
    /// The vehicle spec this file was generated from (sim_vehicle_spec's spec_hash); empty for
    /// a hand-written file.
    #[serde(default)]
    pub spec_hash: String,

    /// Vehicle configuration
    #[serde(default)]
    pub vehicle: VehicleSettings,

    /// Sensor definitions (link_name -> sensor config)
    #[serde(default)]
    pub sensors: HashMap<String, SensorDefinition>,

    /// Retired: the map origin is per map, not per vehicle, and lives in the map dir's
    /// `carla/map_origin.yaml` (`map_origin_path`). Parsed only to refuse it by name.
    #[serde(default, skip_serializing)]
    pub map_origin: Option<serde_yaml::Value>,
}

/// Vehicle blueprint configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleSettings {
    /// CARLA vehicle blueprint ID (e.g., "vehicle.tesla.model3")
    pub blueprint: String,
}

impl Default for VehicleSettings {
    fn default() -> Self {
        Self {
            blueprint: "vehicle.tesla.model3".to_string(),
        }
    }
}

/// Sensor definition from config file
///
/// Defines a sensor to spawn in CARLA with its blueprint and parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorDefinition {
    /// CARLA sensor blueprint (e.g., "sensor.lidar.ray_cast", "sensor.camera.rgb")
    pub blueprint: String,

    /// CARLA sensor parameters as string key-value pairs
    #[serde(default)]
    pub parameters: HashMap<String, String>,
}

impl SensorDefinition {
    /// Classify sensor type from blueprint ID
    ///
    /// Anything unrecognised is `Unsupported`, not a camera. Guessing "camera" for a
    /// blueprint nobody recognised meant a typo in `vehicle_config.yaml` surfaced as
    /// `no image_size_x` from deep inside the camera publisher.
    pub fn sensor_type(&self) -> SensorType {
        // Semantic first: its blueprint id also contains "lidar", and the two decode
        // different measurement structs.
        if self.blueprint.contains("lidar") {
            if self.blueprint.contains("semantic") {
                SensorType::SemanticLidar
            } else {
                SensorType::Lidar
            }
        } else if self.blueprint.contains("camera") {
            SensorType::Camera
        } else if self.blueprint.contains("imu") {
            SensorType::Imu
        } else if self.blueprint.contains("gnss") {
            SensorType::Gnss
        } else if self.blueprint.contains("radar") {
            SensorType::Radar
        } else {
            SensorType::Unsupported
        }
    }

    /// Apply parameters to a CARLA blueprint
    pub fn apply_to_blueprint(&self, blueprint: &mut carla::client::ActorBlueprint) -> Result<()> {
        for (key, value) in &self.parameters {
            if !blueprint.set_attribute(key, value) {
                return Err(BridgeError::ConfigError(format!(
                    "Failed to set sensor attribute '{}' to '{}'",
                    key, value
                )));
            }
        }
        Ok(())
    }
}

impl VehicleConfig {
    /// Load configuration from a YAML file
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self> {
        let path = path.as_ref();
        tracing::info!("Loading vehicle config from: {}", path.display());

        let contents = fs::read_to_string(path).map_err(|e| {
            BridgeError::ConfigError(format!(
                "Failed to read config file {}: {}",
                path.display(),
                e
            ))
        })?;

        let config: VehicleConfig = serde_yaml::from_str(&contents).map_err(|e| {
            BridgeError::ConfigError(format!(
                "Failed to parse YAML config {}: {}",
                path.display(),
                e
            ))
        })?;

        if config.map_origin.is_some() {
            return Err(BridgeError::ConfigError(format!(
                "{}: `map_origin` in the vehicle config is no longer read; put the map's \
                 origin in <map dir>/carla/map_origin.yaml (x, y, z; no yaw) instead",
                path.display()
            )));
        }

        tracing::info!(
            "✓ Loaded vehicle config: {} sensors defined",
            config.sensors.len()
        );

        for (link_name, sensor_def) in &config.sensors {
            tracing::info!(
                "  - '{}': {} ({:?})",
                link_name,
                sensor_def.blueprint,
                sensor_def.sensor_type()
            );
        }

        Ok(config)
    }

    /// Get sensor definitions as an iterator
    #[allow(dead_code)]
    pub fn sensor_iter(&self) -> impl Iterator<Item = (&String, &SensorDefinition)> {
        self.sensors.iter()
    }
}

/// Sensor type classification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorType {
    Camera,
    Lidar,
    /// `sensor.lidar.ray_cast_semantic`. A different measurement struct from
    /// `Lidar` -- decoding one as the other fails on every frame.
    SemanticLidar,
    Gnss,
    Imu,
    /// Radar sensor support - planned for future phases
    #[allow(dead_code)]
    Radar,
    /// A blueprint this bridge does not know how to publish.
    ///
    /// This used to fall back to `Camera`, so a typo'd or unsupported blueprint was
    /// spawned and then handed to the camera publisher, which failed on the missing
    /// `image_size_x` attribute -- a message that says nothing about the real mistake.
    Unsupported,
}

/// The spec_hash an Autoware vehicle package records (`<car>_description/config/spec_hash.yaml`,
/// autoware_sim_vehicle, roadmap 051).
pub fn autoware_spec_hash(path: &Path) -> Result<String> {
    #[derive(Deserialize)]
    struct File {
        spec_hash: String,
    }
    let text = fs::read_to_string(path)
        .map_err(|e| BridgeError::ConfigError(format!("reading {}: {e}", path.display())))?;
    let file: File = serde_yaml::from_str(&text)
        .map_err(|e| BridgeError::ConfigError(format!("parsing {}: {e}", path.display())))?;
    Ok(file.spec_hash)
}

/// Why the CARLA sensors and Autoware's vehicle package came from different specs, `None`
/// when they agree or either names none.
pub fn spec_disagreement(carla_sensors: &str, autoware: &str) -> Option<String> {
    (!carla_sensors.is_empty() && !autoware.is_empty() && carla_sensors != autoware).then(|| {
        format!(
            "vehicle_config was generated from spec {}, Autoware's vehicle package from {}",
            &carla_sensors[..carla_sensors.len().min(12)],
            &autoware[..autoware.len().min(12)]
        )
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_spec_check_names_both_hashes_only_when_they_differ() {
        assert_eq!(
            spec_disagreement("d5d4b266c7fd1fb4", "d5d4b266c7fd1fb4"),
            None
        );
        assert_eq!(spec_disagreement("", "d5d4"), None);
        assert_eq!(spec_disagreement("d5d4", ""), None);
        assert_eq!(
            spec_disagreement("d5d4b266c7fd1fb4", "0123456789abcdef").as_deref(),
            Some(
                "vehicle_config was generated from spec d5d4b266c7fd, Autoware's vehicle \
                 package from 0123456789ab"
            )
        );
    }
}
