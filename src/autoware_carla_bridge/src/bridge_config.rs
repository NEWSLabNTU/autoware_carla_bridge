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

    /// Automatically initialize localization via /api/localization/initialize service
    /// Uses spawn_pose as the initial pose for NDT localization
    #[serde(default)]
    pub auto_initialize_localization: bool,
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
        tracing::info!(
            "Auto-initialize localization: {}",
            if config.auto_initialize_localization {
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

    /// Convert spawn pose to ROS PoseWithCovarianceStamped for localization init
    ///
    /// NOTE: The pointcloud map was created with CARLA's native coordinate system,
    /// so we pass the CARLA coordinates directly (no Y-axis flip or yaw sign flip).
    /// This ensures the initial pose matches the map's coordinate convention.
    pub fn to_ros_pose_with_covariance(&self) -> geometry_msgs::msg::PoseWithCovarianceStamped {
        use crate::coordinate_conversion;

        // Use CARLA coordinates directly (no conversion)
        // The pointcloud map was created with CARLA's native coordinate system
        let pos_x = self.spawn_pose.position.x;
        let pos_y = self.spawn_pose.position.y;
        let pos_z = self.spawn_pose.position.z;

        // Use CARLA yaw directly (no sign flip)
        let yaw_rad = self.spawn_pose.orientation.yaw.to_radians();

        // Convert to quaternion
        let quat = coordinate_conversion::euler_to_quaternion(0.0, 0.0, yaw_rad);

        // Build message
        let mut msg = geometry_msgs::msg::PoseWithCovarianceStamped::default();
        msg.header.frame_id = "map".to_string();
        // Timestamp will be set by caller

        msg.pose.pose.position.x = pos_x;
        msg.pose.pose.position.y = pos_y;
        msg.pose.pose.position.z = pos_z;

        msg.pose.pose.orientation.w = quat.w;
        msg.pose.pose.orientation.x = quat.i;
        msg.pose.pose.orientation.y = quat.j;
        msg.pose.pose.orientation.z = quat.k;

        // Set covariance (small values indicate high confidence)
        // Same as drive_in_autoware.py
        msg.pose.covariance[0] = 0.25; // x variance
        msg.pose.covariance[7] = 0.25; // y variance
        msg.pose.covariance[14] = 0.25; // z variance
        msg.pose.covariance[21] = 0.01; // roll variance
        msg.pose.covariance[28] = 0.01; // pitch variance
        msg.pose.covariance[35] = 0.06853891909122467; // yaw variance

        msg
    }
}
