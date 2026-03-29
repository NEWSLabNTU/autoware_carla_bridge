/// Coordinate system conversion between ROS and CARLA
///
/// # Coordinate System Differences
///
/// ## ROS (REP 103/105) - Right-handed
/// - **X-axis**: Forward (red)
/// - **Y-axis**: Left (green)
/// - **Z-axis**: Up (blue)
/// - **Units**: meters
/// - **Rotation**: Radians
///
/// ## CARLA - Left-handed (Unreal Engine)
/// - **X-axis**: Forward
/// - **Y-axis**: Right
/// - **Z-axis**: Up
/// - **Units**: **meters** (both Rust and Python APIs use meters)
/// - **Rotation**: Degrees
///
/// ## Key Transformations
///
/// ### Position (ROS → CARLA)
/// ```text
/// CARLA_x = ROS_x       // meters (no unit conversion)
/// CARLA_y = -ROS_y      // left-handed conversion (Y-axis flip)
/// CARLA_z = ROS_z       // meters (no unit conversion)
/// ```
///
/// ### Position (CARLA → ROS)
/// ```text
/// ROS_x = CARLA_x       // meters (no unit conversion)
/// ROS_y = -CARLA_y      // left-handed conversion (Y-axis flip)
/// ROS_z = CARLA_z       // meters (no unit conversion)
/// ```
///
/// ### Rotation (ROS → CARLA)
/// ```text
/// CARLA_roll = -ROS_roll * 180.0 / π   // radians to degrees, sign flip
/// CARLA_pitch = ROS_pitch * 180.0 / π  // radians to degrees
/// CARLA_yaw = -ROS_yaw * 180.0 / π     // radians to degrees, sign flip
/// ```
///
/// ### Rotation (CARLA → ROS)
/// ```text
/// ROS_roll = -CARLA_roll * π / 180.0   // degrees to radians, sign flip
/// ROS_pitch = CARLA_pitch * π / 180.0  // degrees to radians
/// ROS_yaw = -CARLA_yaw * π / 180.0     // degrees to radians, sign flip
/// ```
use nalgebra::{Quaternion, Vector3};
use std::f64::consts::PI;

/// Convert position from ROS (meters, right-handed) to CARLA (meters, left-handed)
///
/// # Arguments
/// * `ros_position` - Position in ROS coordinate system (meters)
///
/// # Returns
/// Position in CARLA coordinate system (meters)
///
/// # Example
/// ```
/// use acb_bridge::coordinate_conversion::ros_to_carla_position;
/// use nalgebra::Vector3;
///
/// let ros_pos = Vector3::new(1.0, 2.0, 3.0); // 1m forward, 2m left, 3m up
/// let carla_pos = ros_to_carla_position(&ros_pos);
/// assert_eq!(carla_pos.x, 1.0); // Forward: no change
/// assert_eq!(carla_pos.y, -2.0); // 2m left → -2m right (Y-axis flip)
/// assert_eq!(carla_pos.z, 3.0); // Up: no change
/// ```
pub fn ros_to_carla_position(ros_position: &Vector3<f64>) -> Vector3<f64> {
    Vector3::new(
        ros_position.x,  // Forward: meters (no unit conversion)
        -ros_position.y, // Left → Right: Y-axis flip
        ros_position.z,  // Up: meters (no unit conversion)
    )
}

/// Convert position from CARLA (meters, left-handed) to ROS (meters, right-handed)
///
/// # Arguments
/// * `carla_position` - Position in CARLA coordinate system (meters)
///
/// # Returns
/// Position in ROS coordinate system (meters)
///
/// # Example
/// ```
/// use acb_bridge::coordinate_conversion::carla_to_ros_position;
/// use nalgebra::Vector3;
///
/// let carla_pos = Vector3::new(1.0, -2.0, 3.0); // 1m forward, 2m right, 3m up
/// let ros_pos = carla_to_ros_position(&carla_pos);
/// assert_eq!(ros_pos.x, 1.0); // Forward: no change
/// assert_eq!(ros_pos.y, 2.0); // -2m right → 2m left (Y-axis flip)
/// assert_eq!(ros_pos.z, 3.0); // Up: no change
/// ```
pub fn carla_to_ros_position(carla_position: &Vector3<f64>) -> Vector3<f64> {
    Vector3::new(
        carla_position.x,  // Forward: meters (no unit conversion)
        -carla_position.y, // Right → Left: Y-axis flip
        carla_position.z,  // Up: meters (no unit conversion)
    )
}

/// Convert rotation from ROS Euler angles (radians) to CARLA Euler angles (degrees)
///
/// # Arguments
/// * `roll` - Roll angle in radians
/// * `pitch` - Pitch angle in radians
/// * `yaw` - Yaw angle in radians
///
/// # Returns
/// Tuple of (roll, pitch, yaw) in degrees for CARLA
///
/// # Example
/// ```
/// use acb_bridge::coordinate_conversion::ros_to_carla_rotation;
/// use std::f64::consts::PI;
///
/// let (carla_roll, carla_pitch, carla_yaw) = ros_to_carla_rotation(0.0, 0.0, PI / 2.0);
/// assert!((carla_roll - 0.0).abs() < 1e-10);
/// assert!((carla_pitch - 0.0).abs() < 1e-10);
/// assert!((carla_yaw - (-90.0)).abs() < 1e-6); // 90° counterclockwise → -90° in CARLA
/// ```
///
/// NOTE: Kept for API completeness and potential future use in rotation conversions.
#[allow(dead_code)]
pub fn ros_to_carla_rotation(roll: f64, pitch: f64, yaw: f64) -> (f64, f64, f64) {
    (
        -roll * 180.0 / PI, // Sign flip for left-handed system
        pitch * 180.0 / PI,
        -yaw * 180.0 / PI, // Sign flip for left-handed system
    )
}

/// Convert rotation from CARLA Euler angles (degrees) to ROS Euler angles (radians)
///
/// # Arguments
/// * `roll` - Roll angle in degrees
/// * `pitch` - Pitch angle in degrees
/// * `yaw` - Yaw angle in degrees
///
/// # Returns
/// Tuple of (roll, pitch, yaw) in radians for ROS
///
/// # Example
/// ```
/// use acb_bridge::coordinate_conversion::carla_to_ros_rotation;
/// use std::f64::consts::PI;
///
/// let (ros_roll, ros_pitch, ros_yaw) = carla_to_ros_rotation(0.0, 0.0, -90.0);
/// assert!((ros_roll - 0.0).abs() < 1e-10);
/// assert!((ros_pitch - 0.0).abs() < 1e-10);
/// assert!((ros_yaw - PI / 2.0).abs() < 1e-6); // -90° in CARLA → 90° counterclockwise
/// ```
pub fn carla_to_ros_rotation(roll: f64, pitch: f64, yaw: f64) -> (f64, f64, f64) {
    (
        -roll * PI / 180.0, // Sign flip for right-handed system
        pitch * PI / 180.0,
        -yaw * PI / 180.0, // Sign flip for right-handed system
    )
}

/// Convert ROS quaternion to CARLA Euler angles (degrees)
///
/// # Arguments
/// * `quaternion` - Rotation as quaternion (x, y, z, w)
///
/// # Returns
/// Tuple of (roll, pitch, yaw) in degrees for CARLA
/// NOTE: Kept for API completeness and potential future use in quaternion conversions.
#[allow(dead_code)]
pub fn ros_quaternion_to_carla_euler(quaternion: &Quaternion<f64>) -> (f64, f64, f64) {
    let (roll, pitch, yaw) = quaternion_to_euler(quaternion);
    ros_to_carla_rotation(roll, pitch, yaw)
}

/// Convert CARLA Euler angles (degrees) to ROS quaternion
///
/// # Arguments
/// * `roll` - Roll angle in degrees
/// * `pitch` - Pitch angle in degrees
/// * `yaw` - Yaw angle in degrees
///
/// # Returns
/// Rotation as quaternion (x, y, z, w)
///
/// NOTE: Kept for API completeness and potential future use in sensor data conversion.
#[allow(dead_code)]
pub fn carla_euler_to_ros_quaternion(roll: f64, pitch: f64, yaw: f64) -> Quaternion<f64> {
    let (ros_roll, ros_pitch, ros_yaw) = carla_to_ros_rotation(roll, pitch, yaw);
    euler_to_quaternion(ros_roll, ros_pitch, ros_yaw)
}

/// Convert quaternion to Euler angles (roll, pitch, yaw) in radians
///
/// Uses the ZYX (yaw-pitch-roll) convention, which is standard in ROS.
///
/// # Arguments
/// * `q` - Quaternion (x, y, z, w)
///
/// # Returns
/// Tuple of (roll, pitch, yaw) in radians
///
/// # Example
/// ```
/// use acb_bridge::coordinate_conversion::quaternion_to_euler;
/// use nalgebra::Quaternion;
///
/// // Identity quaternion (no rotation)
/// let q = Quaternion::new(1.0, 0.0, 0.0, 0.0);
/// let (roll, pitch, yaw) = quaternion_to_euler(&q);
/// assert!((roll - 0.0).abs() < 1e-10);
/// assert!((pitch - 0.0).abs() < 1e-10);
/// assert!((yaw - 0.0).abs() < 1e-10);
/// ```
pub fn quaternion_to_euler(q: &Quaternion<f64>) -> (f64, f64, f64) {
    // Extract quaternion components
    let (w, x, y, z) = (q.w, q.i, q.j, q.k);

    // Roll (x-axis rotation)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    // Pitch (y-axis rotation)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        sinp.signum() * PI / 2.0 // Use 90 degrees if out of range
    } else {
        sinp.asin()
    };

    // Yaw (z-axis rotation)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    (roll, pitch, yaw)
}

/// Convert Euler angles (roll, pitch, yaw) to quaternion
///
/// Uses the ZYX (yaw-pitch-roll) convention, which is standard in ROS.
///
/// # Arguments
/// * `roll` - Roll angle in radians
/// * `pitch` - Pitch angle in radians
/// * `yaw` - Yaw angle in radians
///
/// # Returns
/// Quaternion (x, y, z, w)
///
/// # Example
/// ```
/// use acb_bridge::coordinate_conversion::{euler_to_quaternion, quaternion_to_euler};
///
/// // Test round-trip conversion
/// let (roll, pitch, yaw) = (0.1, 0.2, 0.3);
/// let q = euler_to_quaternion(roll, pitch, yaw);
/// let (r2, p2, y2) = quaternion_to_euler(&q);
/// assert!((roll - r2).abs() < 1e-10);
/// assert!((pitch - p2).abs() < 1e-10);
/// assert!((yaw - y2).abs() < 1e-10);
/// ```
pub fn euler_to_quaternion(roll: f64, pitch: f64, yaw: f64) -> Quaternion<f64> {
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

/// Convert CARLA Transform to ROS Isometry3 (high-level pose conversion)
///
/// Converts a complete CARLA pose (position + orientation) to ROS coordinate system.
/// This is the preferred function for converting poses between systems.
///
/// # Arguments
/// * `carla_transform` - CARLA transform (meters, degrees, left-handed)
///
/// # Returns
/// ROS transform as Isometry3<f32> (meters, radians, right-handed)
///
/// # Example
/// ```ignore
/// use carla::geom::{Transform, Location, Rotation};
///
/// let carla_tf = Transform {
///     location: Location { x: 100.0, y: -200.0, z: 50.0 },
///     rotation: Rotation { roll: 0.0, pitch: 0.0, yaw: -90.0 },
/// };
/// let ros_iso = carla_transform_to_ros_isometry(&carla_tf);
/// ```
///
/// NOTE: Reserved for future use when converting CARLA states back to ROS
#[allow(dead_code)]
pub fn carla_transform_to_ros_isometry(
    carla_transform: &carla::geom::Transform,
) -> nalgebra::Isometry3<f32> {
    // Convert position: Y-axis flip, no unit conversion (meters to meters)
    let ros_position = nalgebra::Translation3::new(
        carla_transform.location.x,  // Forward: no change
        -carla_transform.location.y, // Right → Left: Y-axis flip
        carla_transform.location.z,  // Up: no change
    );

    // Convert rotation: degrees to radians, sign flips for right-handed system
    let (ros_roll, ros_pitch, ros_yaw) = carla_to_ros_rotation(
        carla_transform.rotation.roll as f64,
        carla_transform.rotation.pitch as f64,
        carla_transform.rotation.yaw as f64,
    );

    // Create quaternion from euler angles
    let ros_quat_f64 = euler_to_quaternion(ros_roll, ros_pitch, ros_yaw);
    let ros_rotation = nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
        ros_quat_f64.w as f32,
        ros_quat_f64.i as f32,
        ros_quat_f64.j as f32,
        ros_quat_f64.k as f32,
    ));

    nalgebra::Isometry3::from_parts(ros_position, ros_rotation)
}

/// Convert ROS Isometry3 to CARLA Transform (high-level pose conversion)
///
/// Converts a complete ROS pose (position + orientation) to CARLA coordinate system.
/// This is the preferred function for converting poses between systems.
///
/// # Arguments
/// * `ros_isometry` - ROS transform as Isometry3<f32> (meters, radians, right-handed)
///
/// # Returns
/// CARLA transform (meters, degrees, left-handed)
///
/// # Example
/// ```ignore
/// use nalgebra::{Isometry3, Translation3, UnitQuaternion};
///
/// let ros_iso = Isometry3::from_parts(
///     Translation3::new(1.0, 2.0, 0.5),
///     UnitQuaternion::from_euler_angles(0.0, 0.0, 1.57),
/// );
/// let carla_tf = ros_isometry_to_carla_transform(&ros_iso);
/// ```
pub fn ros_isometry_to_carla_transform(
    ros_isometry: &nalgebra::Isometry3<f32>,
) -> carla::geom::Transform {
    // Convert position: Y-axis flip, no unit conversion (meters to meters)
    let carla_location = carla::geom::Location {
        x: ros_isometry.translation.x,  // Forward: no change
        y: -ros_isometry.translation.y, // Left → Right: Y-axis flip
        z: ros_isometry.translation.z,  // Up: no change
    };

    // Convert rotation: extract euler angles, convert to degrees, apply sign flips
    let (roll, pitch, yaw) = ros_isometry.rotation.euler_angles();
    let (carla_roll, carla_pitch, carla_yaw) =
        ros_to_carla_rotation(roll as f64, pitch as f64, yaw as f64);

    let carla_rotation = carla::geom::Rotation {
        roll: carla_roll as f32,
        pitch: carla_pitch as f32,
        yaw: carla_yaw as f32,
    };

    carla::geom::Transform {
        location: carla_location,
        rotation: carla_rotation,
    }
}

/// Convert linear velocity from CARLA (m/s, left-handed) to ROS (m/s, right-handed)
///
/// CARLA velocities are already in m/s, so we only need to flip the Y-axis
///
/// # Arguments
/// * `carla_velocity` - Linear velocity in CARLA coordinate system (m/s)
///
/// # Returns
/// Linear velocity in ROS coordinate system (m/s)
pub fn carla_to_ros_velocity(carla_velocity: &Vector3<f64>) -> Vector3<f64> {
    Vector3::new(
        carla_velocity.x,  // Forward: no change
        -carla_velocity.y, // Right → Left: Y-axis flip
        carla_velocity.z,  // Up: no change
    )
}

/// Convert angular velocity from CARLA (rad/s, left-handed) to ROS (rad/s, right-handed)
///
/// # Arguments
/// * `carla_angular_velocity` - Angular velocity in CARLA coordinate system (rad/s)
///
/// # Returns
/// Angular velocity in ROS coordinate system (rad/s)
pub fn carla_to_ros_angular_velocity(carla_angular_velocity: &Vector3<f64>) -> Vector3<f64> {
    Vector3::new(
        -carla_angular_velocity.x, // Roll: sign flip for right-handed system
        carla_angular_velocity.y,  // Pitch: no change
        -carla_angular_velocity.z, // Yaw: sign flip for right-handed system
    )
}

/// Convert ROS Pose to CARLA Isometry (for spawning)
///
/// Converts a ROS geometry_msgs Pose to a CARLA nalgebra::Isometry3<f32>
/// for spawning vehicles and sensors.
///
/// # Arguments
/// * `pose` - ROS Pose message
///
/// # Returns
/// CARLA transform as Isometry3<f32>
pub fn ros_pose_to_carla_isometry(pose: &geometry_msgs::msg::Pose) -> nalgebra::Isometry3<f32> {
    // Convert position (meters → centimeters, Y-axis flip)
    let ros_position = Vector3::new(pose.position.x, pose.position.y, pose.position.z);
    let carla_position = ros_to_carla_position(&ros_position);

    // Convert ROS quaternion to nalgebra quaternion (f64)
    let q_f64 = nalgebra::Quaternion::new(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
    );

    // Convert to Euler, apply coordinate system transform, back to quaternion (f32)
    let (roll, pitch, yaw) = quaternion_to_euler(&q_f64);
    let carla_quat = euler_to_quaternion(
        -roll, // Roll sign flip for left-handed
        pitch, -yaw, // Yaw sign flip for left-handed
    );

    // Create nalgebra Isometry3<f32>
    let translation = nalgebra::Translation3::new(
        carla_position.x as f32,
        carla_position.y as f32,
        carla_position.z as f32,
    );

    let rotation = nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
        carla_quat.w as f32,
        carla_quat.i as f32,
        carla_quat.j as f32,
        carla_quat.k as f32,
    ));

    nalgebra::Isometry3::from_parts(translation, rotation)
}

/// Normalize angle to range [-π, π]
///
/// NOTE: Utility function kept for potential future use in angle computations
/// and quaternion/euler conversions.
#[allow(dead_code)]
pub fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    } else if a < -PI {
        a += 2.0 * PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2};

    #[test]
    fn test_ros_to_carla_position() {
        // Test forward, left, up: 1m forward, 2m left, 3m up → 1m forward, 2m right, 3m up
        let ros_pos = Vector3::new(1.0, 2.0, 3.0);
        let carla_pos = ros_to_carla_position(&ros_pos);
        assert_eq!(carla_pos.x, 1.0);
        assert_eq!(carla_pos.y, -2.0); // Y-axis flip: left → right
        assert_eq!(carla_pos.z, 3.0);
    }

    #[test]
    fn test_carla_to_ros_position() {
        // Test forward, right, up: 1m forward, 2m right, 3m up → 1m forward, 2m left, 3m up
        let carla_pos = Vector3::new(1.0, -2.0, 3.0);
        let ros_pos = carla_to_ros_position(&carla_pos);
        assert_eq!(ros_pos.x, 1.0);
        assert_eq!(ros_pos.y, 2.0); // Y-axis flip back: right → left
        assert_eq!(ros_pos.z, 3.0);
    }

    #[test]
    fn test_position_round_trip() {
        let original = Vector3::new(1.5, -3.2, 0.7);
        let carla = ros_to_carla_position(&original);
        let back = carla_to_ros_position(&carla);
        assert!((original - back).norm() < 1e-10);
    }

    #[test]
    fn test_ros_to_carla_rotation_90deg_yaw() {
        let (roll, pitch, yaw) = ros_to_carla_rotation(0.0, 0.0, FRAC_PI_2);
        assert!((roll - 0.0).abs() < 1e-10);
        assert!((pitch - 0.0).abs() < 1e-10);
        assert!((yaw - (-90.0)).abs() < 1e-6); // 90° CCW → -90° in CARLA
    }

    #[test]
    fn test_carla_to_ros_rotation_90deg_yaw() {
        let (roll, pitch, yaw) = carla_to_ros_rotation(0.0, 0.0, -90.0);
        assert!((roll - 0.0).abs() < 1e-10);
        assert!((pitch - 0.0).abs() < 1e-10);
        assert!((yaw - FRAC_PI_2).abs() < 1e-6);
    }

    #[test]
    fn test_rotation_round_trip() {
        let (roll, pitch, yaw) = (0.1, 0.2, 0.3);
        let (cr, cp, cy) = ros_to_carla_rotation(roll, pitch, yaw);
        let (r2, p2, y2) = carla_to_ros_rotation(cr, cp, cy);
        assert!((roll - r2).abs() < 1e-10);
        assert!((pitch - p2).abs() < 1e-10);
        assert!((yaw - y2).abs() < 1e-10);
    }

    #[test]
    fn test_euler_to_quaternion_identity() {
        let q = euler_to_quaternion(0.0, 0.0, 0.0);
        assert!((q.w - 1.0).abs() < 1e-10);
        assert!(q.i.abs() < 1e-10);
        assert!(q.j.abs() < 1e-10);
        assert!(q.k.abs() < 1e-10);
    }

    #[test]
    fn test_euler_to_quaternion_90deg_yaw() {
        let q = euler_to_quaternion(0.0, 0.0, FRAC_PI_2);
        // 90° yaw rotation quaternion: [0, 0, sin(45°), cos(45°)]
        assert!(q.i.abs() < 1e-10);
        assert!(q.j.abs() < 1e-10);
        assert!((q.k - FRAC_1_SQRT_2).abs() < 1e-6);
        assert!((q.w - FRAC_1_SQRT_2).abs() < 1e-6);
    }

    #[test]
    fn test_quaternion_to_euler_identity() {
        let q = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        let (roll, pitch, yaw) = quaternion_to_euler(&q);
        assert!((roll - 0.0).abs() < 1e-10);
        assert!((pitch - 0.0).abs() < 1e-10);
        assert!((yaw - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_quaternion_to_euler_90deg_yaw() {
        let q = Quaternion::new(FRAC_1_SQRT_2, 0.0, 0.0, FRAC_1_SQRT_2);
        let (roll, pitch, yaw) = quaternion_to_euler(&q);
        assert!(roll.abs() < 1e-10);
        assert!(pitch.abs() < 1e-10);
        assert!((yaw - FRAC_PI_2).abs() < 1e-6);
    }

    #[test]
    fn test_euler_quaternion_round_trip() {
        let (roll, pitch, yaw) = (0.1, 0.2, 0.3);
        let q = euler_to_quaternion(roll, pitch, yaw);
        let (r2, p2, y2) = quaternion_to_euler(&q);
        assert!((roll - r2).abs() < 1e-10);
        assert!((pitch - p2).abs() < 1e-10);
        assert!((yaw - y2).abs() < 1e-10);
    }

    #[test]
    fn test_euler_quaternion_round_trip_various_angles() {
        let test_cases = vec![
            (0.0, 0.0, 0.0),
            (FRAC_PI_2, 0.0, 0.0),
            (0.0, FRAC_PI_2, 0.0),
            (0.0, 0.0, FRAC_PI_2),
            (0.5, 0.3, 1.2),
            (-0.5, -0.3, -1.2),
        ];

        for (roll, pitch, yaw) in test_cases {
            let q = euler_to_quaternion(roll, pitch, yaw);
            let (r2, p2, y2) = quaternion_to_euler(&q);
            assert!(
                (roll - r2).abs() < 1e-9,
                "Roll mismatch for ({}, {}, {})",
                roll,
                pitch,
                yaw
            );
            assert!(
                (pitch - p2).abs() < 1e-9,
                "Pitch mismatch for ({}, {}, {})",
                roll,
                pitch,
                yaw
            );
            assert!(
                (yaw - y2).abs() < 1e-9,
                "Yaw mismatch for ({}, {}, {})",
                roll,
                pitch,
                yaw
            );
        }
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-10);
        assert!((normalize_angle(PI) - PI).abs() < 1e-10);
        assert!((normalize_angle(-PI) - (-PI)).abs() < 1e-10);
        assert!((normalize_angle(2.0 * PI) - 0.0).abs() < 1e-10);
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-10);
        assert!((normalize_angle(-2.0 * PI) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_ros_quaternion_to_carla_euler() {
        // 90° yaw rotation in ROS
        let q = euler_to_quaternion(0.0, 0.0, FRAC_PI_2);
        let (roll, pitch, yaw) = ros_quaternion_to_carla_euler(&q);
        assert!((roll - 0.0).abs() < 1e-6);
        assert!((pitch - 0.0).abs() < 1e-6);
        assert!((yaw - (-90.0)).abs() < 1e-3); // Should be -90° in CARLA
    }

    #[test]
    fn test_carla_euler_to_ros_quaternion() {
        // -90° yaw in CARLA → 90° yaw in ROS
        let q = carla_euler_to_ros_quaternion(0.0, 0.0, -90.0);
        let (roll, pitch, yaw) = quaternion_to_euler(&q);
        assert!((roll - 0.0).abs() < 1e-6);
        assert!((pitch - 0.0).abs() < 1e-6);
        assert!((yaw - FRAC_PI_2).abs() < 1e-6);
    }
}
