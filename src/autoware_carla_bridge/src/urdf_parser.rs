//! URDF parsing utilities
//!
//! **DEPRECATED**: This module is no longer used for sensor spawning.
//! Sensor configuration now comes from `vehicle_config.yaml` (single source of truth).
//! TF transforms are obtained from the `/tf` topic published by robot_state_publisher.
//!
//! This module is kept for reference but may be removed in a future version.

#![allow(dead_code)]

use crate::error::{BridgeError, Result};
use nalgebra::{Quaternion, Vector3};

/// Sensor configuration extracted from URDF (DEPRECATED)
///
/// **DEPRECATED**: Use `VehicleConfig` from `sensor_config.rs` instead.
/// This struct is no longer used for sensor spawning.
#[derive(Debug, Clone)]
pub struct SensorConfig {
    /// Sensor link name from URDF
    pub link_name: String,

    /// Position relative to base_link (x, y, z)
    pub position: Vector3<f64>,

    /// Orientation relative to base_link (quaternion)
    pub orientation: Quaternion<f64>,

    /// Parent frame name
    pub parent_frame: String,
}

/// Parse URDF and extract link configurations (DEPRECATED)
///
/// **DEPRECATED**: This function is no longer used for sensor spawning.
/// Use `VehicleConfig::from_file()` instead.
///
/// # Arguments
/// * `urdf_xml` - URDF XML string
///
/// # Returns
/// Vector of sensor configurations found in URDF
pub fn parse_urdf_links(urdf_xml: &str) -> Result<Vec<SensorConfig>> {
    // Parse URDF using urdf-rs
    let robot = urdf_rs::read_from_string(urdf_xml)
        .map_err(|e| BridgeError::AutowareIssue(format!("Failed to parse URDF: {}", e)))?;

    tracing::info!("Parsed URDF for robot: {}", robot.name);
    tracing::debug!(
        "URDF has {} links and {} joints",
        robot.links.len(),
        robot.joints.len()
    );

    let mut configs = Vec::new();

    // Iterate through all links
    for link in &robot.links {
        // Find joint connecting this link to its parent
        let joint = robot.joints.iter().find(|j| j.child.link == link.name);

        if let Some(joint) = joint {
            let origin = &joint.origin;

            // Extract position (xyz)
            let position = Vector3::new(origin.xyz[0], origin.xyz[1], origin.xyz[2]);

            // Extract orientation (rpy -> quaternion)
            let orientation = rpy_to_quaternion(origin.rpy[0], origin.rpy[1], origin.rpy[2]);

            configs.push(SensorConfig {
                link_name: link.name.clone(),
                position,
                orientation,
                parent_frame: joint.parent.link.clone(),
            });
        }
    }

    tracing::info!("Found {} links in URDF", configs.len());

    Ok(configs)
}

/// Convert roll-pitch-yaw to quaternion
///
/// # Arguments
/// * `roll` - Roll angle in radians
/// * `pitch` - Pitch angle in radians
/// * `yaw` - Yaw angle in radians
///
/// # Returns
/// Quaternion representing the rotation
fn rpy_to_quaternion(roll: f64, pitch: f64, yaw: f64) -> Quaternion<f64> {
    // Using Tait-Bryan angles (ZYX convention)
    let cy = (yaw * 0.5).cos();
    let sy = (yaw * 0.5).sin();
    let cp = (pitch * 0.5).cos();
    let sp = (pitch * 0.5).sin();
    let cr = (roll * 0.5).cos();
    let sr = (roll * 0.5).sin();

    Quaternion::new(
        cr * cp * cy + sr * sp * sy, // w
        sr * cp * cy - cr * sp * sy, // x
        cr * sp * cy + sr * cp * sy, // y
        cr * cp * sy - sr * sp * cy, // z
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rpy_to_quaternion_identity() {
        let q = rpy_to_quaternion(0.0, 0.0, 0.0);
        // Identity quaternion is [1, 0, 0, 0]
        assert!((q.w - 1.0).abs() < 1e-10);
        assert!(q.i.abs() < 1e-10);
        assert!(q.j.abs() < 1e-10);
        assert!(q.k.abs() < 1e-10);
    }

    #[test]
    fn test_rpy_to_quaternion_90deg_yaw() {
        let q = rpy_to_quaternion(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        // 90 degree yaw rotation
        assert!((q.w - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-10);
        assert!(q.i.abs() < 1e-10);
        assert!(q.j.abs() < 1e-10);
        assert!((q.k - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-10);
    }

    #[test]
    fn test_parse_urdf_minimal() {
        let urdf_xml = r#"<?xml version="1.0"?>
<robot name="test_robot">
    <link name="base_link"/>
    <link name="camera_link"/>
    <joint name="camera_joint" type="fixed">
        <parent link="base_link"/>
        <child link="camera_link"/>
        <origin xyz="1.0 0.0 0.5" rpy="0 0 0"/>
    </joint>
</robot>"#;

        let result = parse_urdf_links(urdf_xml);
        if let Err(e) = &result {
            eprintln!("Parse error: {:?}", e);
        }
        assert!(result.is_ok(), "Failed to parse URDF: {:?}", result);

        let configs = result.unwrap();
        // Now returns all links including base_link (1 with joint = camera_link)
        assert!(!configs.is_empty());
        let camera = configs.iter().find(|c| c.link_name == "camera_link");
        assert!(camera.is_some());
        let camera = camera.unwrap();
        assert_eq!(camera.parent_frame, "base_link");
        assert!((camera.position.x - 1.0).abs() < 1e-10);
        assert!((camera.position.y - 0.0).abs() < 1e-10);
        assert!((camera.position.z - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_parse_urdf_malformed() {
        let urdf_xml = "this is not valid XML";
        let result = parse_urdf_links(urdf_xml);
        assert!(result.is_err());
    }
}
