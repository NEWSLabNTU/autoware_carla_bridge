//! CARLA vehicle and sensor configuration module
//!
//! This module provides CARLA-specific configuration for vehicles and sensors.
//! The vehicle_config.yaml file is the **single source of truth** for which
//! sensors to spawn and their CARLA blueprints/parameters.
//!
//! ## Design Philosophy
//! - **Explicit over implicit**: Sensors are explicitly listed in the config file
//! - **No name-based inference**: We don't guess sensor types from URDF link names
//! - **TF for positions only**: URDF/TF is only used for sensor positions, not types
//!
//! ## Config File Format (vehicle_config.yaml)
//! ```yaml
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
// VehicleConfig - Single source of truth for sensor spawning
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
    /// Vehicle configuration
    #[serde(default)]
    pub vehicle: VehicleSettings,

    /// Sensor definitions (link_name -> sensor config)
    #[serde(default)]
    pub sensors: HashMap<String, SensorDefinition>,

    /// Map origin offset (CARLA coordinates relative to ROS map frame)
    #[serde(default)]
    pub map_origin: MapOrigin,
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

/// Map origin offset configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MapOrigin {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub yaw: f64,
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
    pub fn sensor_type(&self) -> SensorType {
        if self.blueprint.contains("lidar") {
            SensorType::Lidar
        } else if self.blueprint.contains("camera") {
            SensorType::Camera
        } else if self.blueprint.contains("imu") {
            SensorType::Imu
        } else if self.blueprint.contains("gnss") {
            SensorType::Gnss
        } else if self.blueprint.contains("radar") {
            SensorType::Radar
        } else {
            SensorType::Camera // Default fallback
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

// ============================================================================
// Legacy CarlaConfig - kept for backward compatibility but deprecated
// ============================================================================

/// Top-level CARLA sensor configuration (DEPRECATED)
///
/// **DEPRECATED**: Use `VehicleConfig` instead.
/// This struct is kept for backward compatibility but may be removed.
#[allow(dead_code)]
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CarlaConfig {
    /// Type-level defaults for each sensor category
    #[serde(default)]
    pub defaults: SensorTypeDefaults,

    /// Sensor-specific overrides (keyed by link name from URDF)
    #[serde(default)]
    pub sensors: HashMap<String, SensorParams>,
}

#[allow(dead_code)]
impl CarlaConfig {
    /// Load configuration from a YAML file
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self> {
        let path = path.as_ref();
        tracing::info!("Loading CARLA sensor config from: {}", path.display());

        let contents = fs::read_to_string(path).map_err(|e| {
            BridgeError::ConfigError(format!(
                "Failed to read config file {}: {}",
                path.display(),
                e
            ))
        })?;

        let config: CarlaConfig = serde_yaml::from_str(&contents).map_err(|e| {
            BridgeError::ConfigError(format!(
                "Failed to parse YAML config {}: {}",
                path.display(),
                e
            ))
        })?;

        tracing::info!(
            "✓ Loaded config: {} type defaults, {} sensor overrides",
            Self::count_type_defaults(&config.defaults),
            config.sensors.len()
        );

        Ok(config)
    }

    /// Get merged parameters for a specific sensor
    ///
    /// This merges type defaults with sensor-specific overrides.
    /// Sensor-specific parameters take precedence.
    pub fn get_sensor_params(&self, link_name: &str, sensor_type: SensorType) -> SensorParams {
        let type_defaults = self.defaults.get_defaults(sensor_type);

        // If no sensor-specific override exists, return type defaults
        let Some(override_params) = self.sensors.get(link_name) else {
            tracing::debug!(
                "No override for '{}', using {:?} type defaults",
                link_name,
                sensor_type
            );
            return type_defaults;
        };

        // Merge: sensor-specific params override type defaults
        tracing::debug!(
            "Merging override for '{}' with {:?} defaults",
            link_name,
            sensor_type
        );
        type_defaults.merge(override_params)
    }

    fn count_type_defaults(defaults: &SensorTypeDefaults) -> usize {
        let mut count = 0;
        if defaults.camera.is_some() {
            count += 1;
        }
        if defaults.lidar.is_some() {
            count += 1;
        }
        if defaults.gnss.is_some() {
            count += 1;
        }
        if defaults.imu.is_some() {
            count += 1;
        }
        if defaults.radar.is_some() {
            count += 1;
        }
        count
    }
}

/// Type-level defaults for each sensor category (DEPRECATED)
#[allow(dead_code)]
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SensorTypeDefaults {
    #[serde(default)]
    pub camera: Option<SensorParams>,

    #[serde(default)]
    pub lidar: Option<SensorParams>,

    #[serde(default)]
    pub gnss: Option<SensorParams>,

    #[serde(default)]
    pub imu: Option<SensorParams>,

    #[serde(default)]
    pub radar: Option<SensorParams>,
}

#[allow(dead_code)]
impl SensorTypeDefaults {
    /// Get defaults for a specific sensor type
    pub fn get_defaults(&self, sensor_type: SensorType) -> SensorParams {
        match sensor_type {
            SensorType::Camera => self.camera.clone().unwrap_or_default(),
            SensorType::Lidar => self.lidar.clone().unwrap_or_default(),
            SensorType::Gnss => self.gnss.clone().unwrap_or_default(),
            SensorType::Imu => self.imu.clone().unwrap_or_default(),
            SensorType::Radar => self.radar.clone().unwrap_or_default(),
        }
    }
}

/// Sensor type classification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorType {
    Camera,
    Lidar,
    Gnss,
    Imu,
    /// Radar sensor support - planned for future phases
    #[allow(dead_code)]
    Radar,
}

/// CARLA sensor parameters (DEPRECATED)
///
/// **DEPRECATED**: Use `SensorDefinition.parameters` instead.
/// This struct is kept for backward compatibility but may be removed.
#[allow(dead_code)]
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SensorParams {
    // Common parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sensor_tick: Option<f32>,

    // Camera parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub image_size_x: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub image_size_y: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fov: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gamma: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub shutter_speed: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub iso: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fstop: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enable_postprocess_effects: Option<bool>,

    // LiDAR parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub channels: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub range: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub points_per_second: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rotation_frequency: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub upper_fov: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lower_fov: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub atmosphere_attenuation_rate: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dropoff_general_rate: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dropoff_intensity_limit: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dropoff_zero_intensity: Option<f32>,

    // GNSS parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_alt_bias: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_alt_stddev: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_lat_bias: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_lat_stddev: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_lon_bias: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_lon_stddev: Option<f32>,

    // IMU parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_accel_stddev_x: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_accel_stddev_y: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_accel_stddev_z: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_gyro_stddev_x: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_gyro_stddev_y: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_gyro_stddev_z: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_gyro_bias_x: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_gyro_bias_y: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub noise_gyro_bias_z: Option<f32>,

    // Radar parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub horizontal_fov: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vertical_fov: Option<f32>,
}

#[allow(dead_code)]
impl SensorParams {
    /// Merge this set of parameters with another, preferring `other`'s values
    ///
    /// This is used to merge type defaults with sensor-specific overrides.
    /// Any field set in `other` will override the corresponding field in `self`.
    pub fn merge(&self, other: &SensorParams) -> SensorParams {
        SensorParams {
            // Common
            sensor_tick: other.sensor_tick.or(self.sensor_tick),

            // Camera
            image_size_x: other.image_size_x.or(self.image_size_x),
            image_size_y: other.image_size_y.or(self.image_size_y),
            fov: other.fov.or(self.fov),
            gamma: other.gamma.or(self.gamma),
            shutter_speed: other.shutter_speed.or(self.shutter_speed),
            iso: other.iso.or(self.iso),
            fstop: other.fstop.or(self.fstop),
            enable_postprocess_effects: other
                .enable_postprocess_effects
                .or(self.enable_postprocess_effects),

            // LiDAR
            channels: other.channels.or(self.channels),
            range: other.range.or(self.range),
            points_per_second: other.points_per_second.or(self.points_per_second),
            rotation_frequency: other.rotation_frequency.or(self.rotation_frequency),
            upper_fov: other.upper_fov.or(self.upper_fov),
            lower_fov: other.lower_fov.or(self.lower_fov),
            atmosphere_attenuation_rate: other
                .atmosphere_attenuation_rate
                .or(self.atmosphere_attenuation_rate),
            dropoff_general_rate: other.dropoff_general_rate.or(self.dropoff_general_rate),
            dropoff_intensity_limit: other
                .dropoff_intensity_limit
                .or(self.dropoff_intensity_limit),
            dropoff_zero_intensity: other.dropoff_zero_intensity.or(self.dropoff_zero_intensity),

            // GNSS
            noise_alt_bias: other.noise_alt_bias.or(self.noise_alt_bias),
            noise_alt_stddev: other.noise_alt_stddev.or(self.noise_alt_stddev),
            noise_lat_bias: other.noise_lat_bias.or(self.noise_lat_bias),
            noise_lat_stddev: other.noise_lat_stddev.or(self.noise_lat_stddev),
            noise_lon_bias: other.noise_lon_bias.or(self.noise_lon_bias),
            noise_lon_stddev: other.noise_lon_stddev.or(self.noise_lon_stddev),

            // IMU
            noise_accel_stddev_x: other.noise_accel_stddev_x.or(self.noise_accel_stddev_x),
            noise_accel_stddev_y: other.noise_accel_stddev_y.or(self.noise_accel_stddev_y),
            noise_accel_stddev_z: other.noise_accel_stddev_z.or(self.noise_accel_stddev_z),
            noise_gyro_stddev_x: other.noise_gyro_stddev_x.or(self.noise_gyro_stddev_x),
            noise_gyro_stddev_y: other.noise_gyro_stddev_y.or(self.noise_gyro_stddev_y),
            noise_gyro_stddev_z: other.noise_gyro_stddev_z.or(self.noise_gyro_stddev_z),
            noise_gyro_bias_x: other.noise_gyro_bias_x.or(self.noise_gyro_bias_x),
            noise_gyro_bias_y: other.noise_gyro_bias_y.or(self.noise_gyro_bias_y),
            noise_gyro_bias_z: other.noise_gyro_bias_z.or(self.noise_gyro_bias_z),

            // Radar
            horizontal_fov: other.horizontal_fov.or(self.horizontal_fov),
            vertical_fov: other.vertical_fov.or(self.vertical_fov),
        }
    }

    /// Apply these parameters to a CARLA sensor blueprint
    ///
    /// This sets all non-None attributes on the blueprint.
    /// Returns an error if any attribute fails to set.
    pub fn apply_to_blueprint(&self, blueprint: &mut carla::client::ActorBlueprint) -> Result<()> {
        // Helper macro to set attribute from a value and propagate errors
        macro_rules! set_attr {
            ($name:expr, $value:expr) => {
                if !blueprint.set_attribute($name, &$value.to_string()) {
                    return Err(BridgeError::ConfigError(format!(
                        "Failed to set sensor attribute '{}' to '{}'",
                        $name, $value
                    )));
                }
            };
        }

        // Common
        if let Some(v) = self.sensor_tick {
            set_attr!("sensor_tick", v);
        }

        // Camera
        if let Some(v) = self.image_size_x {
            set_attr!("image_size_x", v);
        }
        if let Some(v) = self.image_size_y {
            set_attr!("image_size_y", v);
        }
        if let Some(v) = self.fov {
            set_attr!("fov", v);
        }
        if let Some(v) = self.gamma {
            set_attr!("gamma", v);
        }
        if let Some(v) = self.shutter_speed {
            set_attr!("shutter_speed", v);
        }
        if let Some(v) = self.iso {
            set_attr!("iso", v);
        }
        if let Some(v) = self.fstop {
            set_attr!("fstop", v);
        }
        if let Some(v) = self.enable_postprocess_effects {
            set_attr!("enable_postprocess_effects", v);
        }

        // LiDAR
        if let Some(v) = self.channels {
            set_attr!("channels", v);
        }
        if let Some(v) = self.range {
            set_attr!("range", v);
        }
        if let Some(v) = self.points_per_second {
            set_attr!("points_per_second", v);
        }
        if let Some(v) = self.rotation_frequency {
            set_attr!("rotation_frequency", v);
        }
        if let Some(v) = self.upper_fov {
            set_attr!("upper_fov", v);
        }
        if let Some(v) = self.lower_fov {
            set_attr!("lower_fov", v);
        }
        if let Some(v) = self.atmosphere_attenuation_rate {
            set_attr!("atmosphere_attenuation_rate", v);
        }
        if let Some(v) = self.dropoff_general_rate {
            set_attr!("dropoff_general_rate", v);
        }
        if let Some(v) = self.dropoff_intensity_limit {
            set_attr!("dropoff_intensity_limit", v);
        }
        if let Some(v) = self.dropoff_zero_intensity {
            set_attr!("dropoff_zero_intensity", v);
        }

        // GNSS
        if let Some(v) = self.noise_alt_bias {
            set_attr!("noise_alt_bias", v);
        }
        if let Some(v) = self.noise_alt_stddev {
            set_attr!("noise_alt_stddev", v);
        }
        if let Some(v) = self.noise_lat_bias {
            set_attr!("noise_lat_bias", v);
        }
        if let Some(v) = self.noise_lat_stddev {
            set_attr!("noise_lat_stddev", v);
        }
        if let Some(v) = self.noise_lon_bias {
            set_attr!("noise_lon_bias", v);
        }
        if let Some(v) = self.noise_lon_stddev {
            set_attr!("noise_lon_stddev", v);
        }

        // IMU
        if let Some(v) = self.noise_accel_stddev_x {
            set_attr!("noise_accel_stddev_x", v);
        }
        if let Some(v) = self.noise_accel_stddev_y {
            set_attr!("noise_accel_stddev_y", v);
        }
        if let Some(v) = self.noise_accel_stddev_z {
            set_attr!("noise_accel_stddev_z", v);
        }
        if let Some(v) = self.noise_gyro_stddev_x {
            set_attr!("noise_gyro_stddev_x", v);
        }
        if let Some(v) = self.noise_gyro_stddev_y {
            set_attr!("noise_gyro_stddev_y", v);
        }
        if let Some(v) = self.noise_gyro_stddev_z {
            set_attr!("noise_gyro_stddev_z", v);
        }
        if let Some(v) = self.noise_gyro_bias_x {
            set_attr!("noise_gyro_bias_x", v);
        }
        if let Some(v) = self.noise_gyro_bias_y {
            set_attr!("noise_gyro_bias_y", v);
        }
        if let Some(v) = self.noise_gyro_bias_z {
            set_attr!("noise_gyro_bias_z", v);
        }

        // Radar
        if let Some(v) = self.horizontal_fov {
            set_attr!("horizontal_fov", v);
        }
        if let Some(v) = self.vertical_fov {
            set_attr!("vertical_fov", v);
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_merge() {
        let defaults = SensorParams {
            fov: Some(90.0),
            image_size_x: Some(800),
            image_size_y: Some(600),
            ..Default::default()
        };

        let override_params = SensorParams {
            fov: Some(50.0), // Override FOV
            // Don't override image sizes
            ..Default::default()
        };

        let merged = defaults.merge(&override_params);

        assert_eq!(merged.fov, Some(50.0)); // Overridden
        assert_eq!(merged.image_size_x, Some(800)); // From defaults
        assert_eq!(merged.image_size_y, Some(600)); // From defaults
    }
}
