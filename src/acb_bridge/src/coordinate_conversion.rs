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
use nalgebra::{Matrix3, Quaternion, Rotation3, Vector3};
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

// === base_link: the rear axle, not the actor origin ===
//
// Autoware's `base_link` is the centre of the rear axle, and `vehicle_info.param.yaml` is
// authored that way (rear_overhang, wheel_base, front_overhang all measured from it). A
// CARLA actor's origin is not there: on vehicle.tesla.model3 the rear axle is 1.386 m
// behind it. Taking the actor transform as `base_link` therefore moved Autoware's idea of
// the whole body 1.386 m forward -- it believed the car reached 3.93 m ahead of base_link
// where the real bumper is 2.42 m ahead. See carla-scenario-bridge
// docs/roadmap/014-feature-completeness.md, "Pose reference point".
//
// The offset is measured per vehicle from CARLA's rear wheel positions rather than
// hard-coded, because `vehicle_config.yaml` offers several blueprints. Everything below is
// in CARLA's own frame (x forward, y right, z up, metres), so the Y-flip to ROS stays in
// the one place it already lives.

/// CARLA's rotation matrix for a `Rotation` given in degrees.
///
/// Matches LibCarla's `Rotation::RotateVector` (carla/geom/Rotation.h, checked against the
/// 0.9.16 header in a test), which is Rz(yaw) * Ry(-pitch) * Rx(-roll) in standard
/// right-handed algebra applied to CARLA's left-handed numbers -- the forward
/// vector comes out as (cos p cos y, cos p sin y, sin p), as `GetForwardVector` gives.
/// Not `Rotation::to_na()` from carla-rust, which uses +pitch and +roll; the two agree at
/// zero pitch and roll, which is where a car spends its time, but this one is exact.
pub fn carla_rotation_matrix(roll_deg: f64, pitch_deg: f64, yaw_deg: f64) -> Matrix3<f64> {
    let rz = Rotation3::from_axis_angle(&Vector3::z_axis(), yaw_deg.to_radians());
    let ry = Rotation3::from_axis_angle(&Vector3::y_axis(), -pitch_deg.to_radians());
    let rx = Rotation3::from_axis_angle(&Vector3::x_axis(), -roll_deg.to_radians());
    (rz * ry * rx).into_inner()
}

/// World location of a point given in the actor's own frame (CARLA axes, metres).
///
/// `base_link = actor_origin + R_actor * base_link_in_actor`. Used to publish the rear
/// axle, not the actor origin, as Autoware's `base_link`. `rotation_deg` is CARLA's
/// (roll, pitch, yaw) in degrees. A zero offset returns the actor origin unchanged.
pub fn base_link_world_location(
    actor_location: &Vector3<f64>,
    rotation_deg: (f64, f64, f64),
    base_link_in_actor: &Vector3<f64>,
) -> Vector3<f64> {
    let (roll, pitch, yaw) = rotation_deg;
    actor_location + carla_rotation_matrix(roll, pitch, yaw) * base_link_in_actor
}

/// The rear-axle centre in the actor's own frame, from the rear wheels' world positions.
///
/// CARLA reports `WheelPhysicsControl::position` in **world centimetres** at the actor's
/// current pose (the same reading scripts/extract_vehicle_params.py makes). The midpoint
/// of the rear pair is brought into the actor frame by the inverse of the actor rotation:
/// `R_actor^T * (rear_mid_world - actor_origin_world)`. Returns `None` when there are no
/// wheels to average. The z component is returned as measured; the caller decides
/// whether to keep it.
#[cfg_attr(carla_0100, allow(dead_code))] // no wheel positions on CARLA 0.10
pub fn base_link_in_actor_from_wheels(
    rear_wheels_world_cm: &[Vector3<f64>],
    actor_location: &Vector3<f64>,
    rotation_deg: (f64, f64, f64),
) -> Option<Vector3<f64>> {
    if rear_wheels_world_cm.is_empty() {
        return None;
    }
    let sum: Vector3<f64> = rear_wheels_world_cm.iter().sum();
    let rear_mid_world = sum / (rear_wheels_world_cm.len() as f64) / 100.0;
    let (roll, pitch, yaw) = rotation_deg;
    let r = carla_rotation_matrix(roll, pitch, yaw);
    Some(r.transpose() * (rear_mid_world - actor_location))
}

/// Velocity of a point `offset` away from the actor origin on the same rigid body.
///
/// `v_point = v_origin + omega x offset`, all three in the actor's body frame and in
/// **CARLA's axes**, with `omega` in **rad/s** (CARLA reports deg/s -- convert first). Used
/// to move the ego twist from the actor origin to `base_link`, the rear axle, so the Y-flip
/// to ROS stays in `carla_to_ros_velocity`.
///
/// The ordinary right-handed cross product is the right one here even though CARLA's frame
/// is left-handed: `carla_to_ros_angular_velocity` treats angular velocity as the
/// pseudovector it is (a reflection negates it on top of the axis flip), and under that
/// convention `M(a x b) = (-M a) x (M b)` makes the formula come out the same in both
/// frames. Checked in `test_offset_velocity_agrees_with_ros_frame`. A sanity case: positive
/// CARLA yaw rate turns +x towards +y (to the right), so a point ahead of the rear axle
/// moves right, i.e. +y -- which `omega_z * offset_x` in the y slot gives.
///
/// Longitudinal speed is unchanged by a pure yaw rate and an offset along x; only the
/// lateral component moves, by `yaw_rate * offset_x`. See carla-scenario-bridge
/// docs/roadmap/014 "Pose reference point".
pub fn velocity_at_offset(
    velocity: &Vector3<f64>,
    angular_velocity_rad: &Vector3<f64>,
    offset: &Vector3<f64>,
) -> Vector3<f64> {
    velocity + angular_velocity_rad.cross(offset)
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- base_link offset (rear axle) ---

    const MODEL3_REAR_AXLE: f64 = -1.386;

    fn assert_vec_close(a: &Vector3<f64>, b: &Vector3<f64>, tol: f64) {
        assert!((a - b).norm() < tol, "expected {b:?}, got {a:?}");
    }

    // --- twist at base_link ---

    #[test]
    fn test_pure_yaw_about_rear_axle_is_still_at_base_link() {
        // Turning in place about the rear axle at 0.3 rad/s (CARLA sign: to the right).
        // The rear axle does not move; the actor origin, 1.386 m ahead of it, sweeps
        // right at 0.3 * 1.386 m/s.
        let w = 0.3;
        let omega = Vector3::new(0.0, 0.0, w);
        let rear_axle = Vector3::new(MODEL3_REAR_AXLE, 0.0, 0.0);
        // Origin velocity = rear-axle velocity (zero) + omega x (origin - rear_axle).
        let v_origin = velocity_at_offset(&Vector3::zeros(), &omega, &(-rear_axle));
        assert_vec_close(&v_origin, &Vector3::new(0.0, w * 1.386, 0.0), 1e-12);
        // And back: what the bridge does, from the origin CARLA reports to base_link.
        let v_base = velocity_at_offset(&v_origin, &omega, &rear_axle);
        assert_vec_close(&v_base, &Vector3::zeros(), 1e-12);
        // In ROS: origin moves right (-y) while yaw rate is clockwise (-z).
        let v_origin_ros = carla_to_ros_velocity(&v_origin);
        let omega_ros = carla_to_ros_angular_velocity(&omega);
        assert!(v_origin_ros.y < 0.0 && omega_ros.z < 0.0);
    }

    #[test]
    fn test_offset_velocity_keeps_longitudinal_speed() {
        let v = velocity_at_offset(
            &Vector3::new(8.0, 0.2, 0.0),
            &Vector3::new(0.0, 0.0, 0.1),
            &Vector3::new(MODEL3_REAR_AXLE, 0.0, 0.0),
        );
        assert!((v.x - 8.0).abs() < 1e-12);
        assert!((v.y - (0.2 - 0.1 * 1.386)).abs() < 1e-12);
    }

    #[test]
    fn test_offset_velocity_matches_carla_rotation() {
        // Independent of the cross-product argument: advance CARLA's own rotation by a
        // small yaw and watch where an actor-frame point goes, in the body frame.
        let (yaw0, dyaw_deg, dt) = (37.0_f64, 0.01_f64, 0.001_f64);
        let r = Vector3::new(MODEL3_REAR_AXLE, 0.4, 0.3);
        let r0 = carla_rotation_matrix(0.0, 0.0, yaw0);
        let r1 = carla_rotation_matrix(0.0, 0.0, yaw0 + dyaw_deg);
        let v_body_numeric = r0.transpose() * ((r1 - r0) * r / dt);
        let omega = Vector3::new(0.0, 0.0, (dyaw_deg / dt).to_radians());
        let v_body = velocity_at_offset(&Vector3::zeros(), &omega, &r);
        assert_vec_close(&v_body, &v_body_numeric, 1e-4);
    }

    #[test]
    fn test_offset_velocity_agrees_with_ros_frame() {
        // Doing it in CARLA axes and flipping must equal flipping and doing it in ROS axes.
        let v = Vector3::new(3.0, -0.7, 0.2);
        let w = Vector3::new(0.05, -0.2, 0.4);
        let r = Vector3::new(MODEL3_REAR_AXLE, 0.1, -0.3);
        let carla_then_flip = carla_to_ros_velocity(&velocity_at_offset(&v, &w, &r));
        let flip_then_ros = carla_to_ros_velocity(&v)
            + carla_to_ros_angular_velocity(&w).cross(&carla_to_ros_position(&r));
        assert_vec_close(&carla_then_flip, &flip_then_ros, 1e-12);
    }

    #[test]
    fn test_carla_rotation_forward_vector_matches_carla() {
        // GetForwardVector: (cos p cos y, cos p sin y, sin p)
        let (p, y) = (10.0_f64, 30.0_f64);
        let f = carla_rotation_matrix(0.0, p, y) * Vector3::x();
        let (pr, yr) = (p.to_radians(), y.to_radians());
        assert_vec_close(
            &f,
            &Vector3::new(pr.cos() * yr.cos(), pr.cos() * yr.sin(), pr.sin()),
            1e-12,
        );
    }

    #[test]
    fn test_carla_rotation_matches_libcarla_rotate_vector() {
        // The matrix spelled out in LibCarla 0.9.16 carla/geom/Rotation.h RotateVector.
        let (r, p, y) = (7.0_f64, -12.0_f64, 143.0_f64);
        let (cr, sr) = (r.to_radians().cos(), r.to_radians().sin());
        let (cp, sp) = (p.to_radians().cos(), p.to_radians().sin());
        let (cy, sy) = (y.to_radians().cos(), y.to_radians().sin());
        let libcarla = Matrix3::new(
            cp * cy,
            cy * sp * sr - sy * cr,
            -cy * sp * cr - sy * sr,
            cp * sy,
            sy * sp * sr + cy * cr,
            -sy * sp * cr + cy * sr,
            sp,
            -cp * sr,
            cp * cr,
        );
        assert!((carla_rotation_matrix(r, p, y) - libcarla).norm() < 1e-12);
    }

    #[test]
    fn test_base_link_world_yaw_0() {
        let origin = Vector3::new(100.0, -50.0, 0.5);
        let offset = Vector3::new(MODEL3_REAR_AXLE, 0.0, 0.0);
        let bl = base_link_world_location(&origin, (0.0, 0.0, 0.0), &offset);
        assert_vec_close(&bl, &Vector3::new(100.0 - 1.386, -50.0, 0.5), 1e-9);
    }

    #[test]
    fn test_base_link_world_yaw_90() {
        // CARLA yaw +90 deg faces +y (right, in the left-handed frame): behind is -y.
        let origin = Vector3::new(100.0, -50.0, 0.5);
        let offset = Vector3::new(MODEL3_REAR_AXLE, 0.0, 0.0);
        let bl = base_link_world_location(&origin, (0.0, 0.0, 90.0), &offset);
        assert_vec_close(&bl, &Vector3::new(100.0, -50.0 - 1.386, 0.5), 1e-9);
    }

    #[test]
    fn test_base_link_world_yaw_180() {
        let origin = Vector3::new(100.0, -50.0, 0.5);
        let offset = Vector3::new(MODEL3_REAR_AXLE, 0.0, 0.0);
        let bl = base_link_world_location(&origin, (0.0, 0.0, 180.0), &offset);
        assert_vec_close(&bl, &Vector3::new(100.0 + 1.386, -50.0, 0.5), 1e-9);
    }

    #[test]
    fn test_base_link_world_yaw_minus_90() {
        let origin = Vector3::new(100.0, -50.0, 0.5);
        let offset = Vector3::new(MODEL3_REAR_AXLE, 0.0, 0.0);
        let bl = base_link_world_location(&origin, (0.0, 0.0, -90.0), &offset);
        assert_vec_close(&bl, &Vector3::new(100.0, -50.0 + 1.386, 0.5), 1e-9);
    }

    #[test]
    fn test_base_link_world_zero_offset_is_identity() {
        let origin = Vector3::new(12.3, 45.6, 7.8);
        for rot in [(0.0, 0.0, 0.0), (3.0, -5.0, 137.0), (0.0, 0.0, -90.0)] {
            let bl = base_link_world_location(&origin, rot, &Vector3::zeros());
            assert_vec_close(&bl, &origin, 1e-12);
        }
    }

    #[test]
    fn test_base_link_in_actor_from_wheels_round_trips() {
        // Place model3's rear wheels (track 1.6 m, 0.3 m below the origin) at an arbitrary
        // pose, report them in world centimetres as CARLA does, and recover the offset.
        let origin = Vector3::new(190.77, -130.10, 0.3);
        for rot in [
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 90.0),
            (0.0, 0.0, 180.0),
            (0.0, 0.0, -90.0),
            (1.5, -2.0, 175.81),
        ] {
            let wheels_world_cm: Vec<Vector3<f64>> = [
                Vector3::new(MODEL3_REAR_AXLE, -0.8, -0.3),
                Vector3::new(MODEL3_REAR_AXLE, 0.8, -0.3),
            ]
            .iter()
            .map(|w| base_link_world_location(&origin, rot, w) * 100.0)
            .collect();
            let offset = base_link_in_actor_from_wheels(&wheels_world_cm, &origin, rot).unwrap();
            assert_vec_close(&offset, &Vector3::new(MODEL3_REAR_AXLE, 0.0, -0.3), 1e-9);
            // ...and the offset puts base_link back on the axle midpoint.
            let bl = base_link_world_location(&origin, rot, &offset);
            let mid = (wheels_world_cm[0] + wheels_world_cm[1]) / 200.0;
            assert_vec_close(&bl, &mid, 1e-9);
        }
    }

    #[test]
    fn test_base_link_in_actor_from_no_wheels() {
        assert!(base_link_in_actor_from_wheels(&[], &Vector3::zeros(), (0.0, 0.0, 0.0)).is_none());
    }
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
