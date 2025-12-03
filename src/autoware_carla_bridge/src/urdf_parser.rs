use crate::{
    bridge::sensor_bridge::SensorType,
    error::{BridgeError, Result},
};
use nalgebra::{Quaternion, Vector3};

/// Sensor configuration extracted from URDF
#[derive(Debug, Clone)]
pub struct SensorConfig {
    /// Sensor link name from URDF
    pub link_name: String,

    /// Classified sensor type
    pub sensor_type: SensorType,

    /// Position relative to base_link (x, y, z)
    pub position: Vector3<f64>,

    /// Orientation relative to base_link (quaternion)
    pub orientation: Quaternion<f64>,

    /// Parent frame name
    pub parent_frame: String,
}

/// Parse URDF and extract sensor configurations
///
/// # Arguments
/// * `urdf_xml` - URDF XML string
///
/// # Returns
/// Vector of sensor configurations found in URDF
pub fn parse_urdf_sensors(urdf_xml: &str) -> Result<Vec<SensorConfig>> {
    // Parse URDF using urdf-rs
    let robot = urdf_rs::read_from_string(urdf_xml)
        .map_err(|e| BridgeError::AutowareIssue(format!("Failed to parse URDF: {}", e)))?;

    tracing::info!("Parsed URDF for robot: {}", robot.name);
    tracing::debug!(
        "URDF has {} links and {} joints",
        robot.links.len(),
        robot.joints.len()
    );

    let mut sensors = Vec::new();

    // Iterate through all links
    for link in &robot.links {
        // Try to classify sensor type from link name
        if let Some(sensor_type) = classify_sensor_type(&link.name) {
            tracing::debug!("Found sensor link: {} (type: {:?})", link.name, sensor_type);

            // Find joint connecting this link to its parent
            let joint = robot.joints.iter().find(|j| j.child.link == link.name);

            if let Some(joint) = joint {
                let origin = &joint.origin;

                // Extract position (xyz)
                let position = Vector3::new(origin.xyz[0], origin.xyz[1], origin.xyz[2]);

                // Extract orientation (rpy -> quaternion)
                let orientation = rpy_to_quaternion(origin.rpy[0], origin.rpy[1], origin.rpy[2]);

                sensors.push(SensorConfig {
                    link_name: link.name.clone(),
                    sensor_type,
                    position,
                    orientation,
                    parent_frame: joint.parent.link.clone(),
                });

                tracing::info!(
                    "Sensor '{}' configured: type={:?}, parent={}, pos=[{:.3}, {:.3}, {:.3}]",
                    link.name,
                    sensor_type,
                    joint.parent.link,
                    position.x,
                    position.y,
                    position.z
                );
            } else {
                tracing::warn!(
                    "Sensor link '{}' has no joint connecting to parent, skipping",
                    link.name
                );
            }
        }
    }

    if sensors.is_empty() {
        tracing::warn!("No sensors found in URDF");
    } else {
        tracing::info!("Found {} sensors in URDF", sensors.len());
    }

    Ok(sensors)
}

/// Classify sensor type from URDF link name
///
/// Uses common naming patterns to identify sensor types:
/// - camera, rgb -> CameraRgb
/// - lidar, velodyne, pointcloud -> LidarRayCast
/// - imu, inertial -> Imu
/// - gnss, gps -> Gnss
///
/// # Arguments
/// * `link_name` - URDF link name
///
/// # Returns
/// Some(SensorType) if recognized, None otherwise
fn classify_sensor_type(link_name: &str) -> Option<SensorType> {
    let name_lower = link_name.to_lowercase();

    // Camera patterns
    if name_lower.contains("camera") || name_lower.contains("rgb") {
        return Some(SensorType::CameraRgb);
    }

    // LiDAR patterns
    // Note: "top", "left", "right" match Autoware's standard LiDAR naming convention
    // (e.g., /sensing/lidar/top/, /sensing/lidar/left/, /sensing/lidar/right/)
    if name_lower.contains("lidar")
        || name_lower.contains("velodyne")
        || name_lower.contains("pointcloud")
        || name_lower.contains("laser")
        || name_lower == "top"
        || name_lower == "top_base_link"
        || name_lower == "left"
        || name_lower == "left_base_link"
        || name_lower == "right"
        || name_lower == "right_base_link"
    {
        return Some(SensorType::LidarRayCast);
    }

    // IMU patterns
    if name_lower.contains("imu") || name_lower.contains("inertial") {
        return Some(SensorType::Imu);
    }

    // GNSS patterns
    if name_lower.contains("gnss") || name_lower.contains("gps") {
        return Some(SensorType::Gnss);
    }

    // No match
    None
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
    fn test_classify_sensor_type_camera() {
        assert_eq!(
            classify_sensor_type("front_camera"),
            Some(SensorType::CameraRgb)
        );
        assert_eq!(
            classify_sensor_type("rgb_camera"),
            Some(SensorType::CameraRgb)
        );
        assert_eq!(
            classify_sensor_type("traffic_light_camera"),
            Some(SensorType::CameraRgb)
        );
    }

    #[test]
    fn test_classify_sensor_type_lidar() {
        assert_eq!(
            classify_sensor_type("velodyne_top"),
            Some(SensorType::LidarRayCast)
        );
        assert_eq!(
            classify_sensor_type("lidar_front"),
            Some(SensorType::LidarRayCast)
        );
        assert_eq!(
            classify_sensor_type("pointcloud_sensor"),
            Some(SensorType::LidarRayCast)
        );
    }

    #[test]
    fn test_classify_sensor_type_imu() {
        assert_eq!(classify_sensor_type("tamagawa_imu"), Some(SensorType::Imu));
        assert_eq!(classify_sensor_type("imu_link"), Some(SensorType::Imu));
    }

    #[test]
    fn test_classify_sensor_type_gnss() {
        assert_eq!(classify_sensor_type("gnss_link"), Some(SensorType::Gnss));
        assert_eq!(classify_sensor_type("ublox_gps"), Some(SensorType::Gnss));
    }

    #[test]
    fn test_classify_sensor_type_unknown() {
        assert_eq!(classify_sensor_type("base_link"), None);
        assert_eq!(classify_sensor_type("wheel_left"), None);
        assert_eq!(classify_sensor_type("unknown_sensor"), None);
    }

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

        let result = parse_urdf_sensors(urdf_xml);
        if let Err(e) = &result {
            eprintln!("Parse error: {:?}", e);
        }
        assert!(result.is_ok(), "Failed to parse URDF: {:?}", result);

        let sensors = result.unwrap();
        assert_eq!(sensors.len(), 1);
        assert_eq!(sensors[0].link_name, "camera_link");
        assert!(matches!(sensors[0].sensor_type, SensorType::CameraRgb));
        assert_eq!(sensors[0].parent_frame, "base_link");
        assert!((sensors[0].position.x - 1.0).abs() < 1e-10);
        assert!((sensors[0].position.y - 0.0).abs() < 1e-10);
        assert!((sensors[0].position.z - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_parse_urdf_malformed() {
        let urdf_xml = "this is not valid XML";
        let result = parse_urdf_sensors(urdf_xml);
        assert!(result.is_err());
    }
}
