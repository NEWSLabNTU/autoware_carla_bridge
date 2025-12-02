//! Bridge configuration module
//!
//! This module provides configuration for the Autoware-CARLA bridge,
//! including the mandatory initial spawn pose for the vehicle.

use serde::{Deserialize, Serialize};
use std::{fs, path::Path};

use crate::error::{BridgeError, Result};

/// Bridge configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BridgeConfig {
    /// Initial vehicle spawn pose (mandatory)
    pub spawn_pose: SpawnPose,

    /// Publish pose directly to /localization/kinematic_state (bypasses Autoware localization)
    /// Ground truth is always published to /carla/ground_truth/* for debug/evaluation
    /// Set to true for testing without Autoware localization
    #[serde(default)]
    pub publish_direct_localization: bool,
}

/// Vehicle spawn pose configuration (CARLA coordinates)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpawnPose {
    /// Position in meters (CARLA coordinates)
    pub position: Position,

    /// Orientation in degrees (CARLA coordinates)
    pub orientation: Orientation,
}

/// 3D position (CARLA coordinates: meters)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Position {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Orientation (CARLA coordinates: degrees)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Orientation {
    /// Yaw rotation around Z-axis in degrees
    pub yaw: f64,
}

impl BridgeConfig {
    /// Load configuration from a YAML file
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self> {
        let path = path.as_ref();
        tracing::info!("Loading bridge config from: {}", path.display());

        let contents = fs::read_to_string(path).map_err(|e| {
            BridgeError::ConfigError(format!(
                "Failed to read config file {}: {}",
                path.display(),
                e
            ))
        })?;

        let config: BridgeConfig = serde_yaml::from_str(&contents).map_err(|e| {
            BridgeError::ConfigError(format!(
                "Failed to parse YAML config {}: {}",
                path.display(),
                e
            ))
        })?;

        tracing::info!(
            "Loaded spawn pose: x={}, y={}, z={}, yaw={}°",
            config.spawn_pose.position.x,
            config.spawn_pose.position.y,
            config.spawn_pose.position.z,
            config.spawn_pose.orientation.yaw
        );
        tracing::info!(
            "Direct localization publishing: {}",
            if config.publish_direct_localization {
                "enabled"
            } else {
                "disabled"
            }
        );

        Ok(config)
    }

    /// Convert spawn pose to nalgebra Isometry3 for CARLA
    pub fn to_isometry(&self) -> nalgebra::Isometry3<f32> {
        let yaw_rad = self.spawn_pose.orientation.yaw.to_radians();
        let rotation = nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, yaw_rad as f32);
        let translation = nalgebra::Translation3::new(
            self.spawn_pose.position.x as f32,
            self.spawn_pose.position.y as f32,
            self.spawn_pose.position.z as f32,
        );
        nalgebra::Isometry3::from_parts(translation, rotation)
    }
}
