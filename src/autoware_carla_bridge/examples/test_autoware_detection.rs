use autoware_carla_bridge::{
    autoware_detection::AutowareDetector,
    coordinate_conversion::{
        carla_to_ros_position, euler_to_quaternion, quaternion_to_euler, ros_to_carla_position,
    },
    tf_bridge::TFBuffer,
    urdf_parser::parse_urdf_sensors,
};
use nalgebra::Vector3;
use rclrs::{CreateBasicExecutor, SpinOptions};
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    pretty_env_logger::init();

    log::info!("Starting Autoware detection test...");

    // Initialize ROS 2
    let ctx = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?;
    let mut executor = ctx.create_basic_executor();
    let node = executor.create_node("test_autoware_detection")?;

    log::info!("Creating AutowareDetector...");

    // Create AutowareDetector with 60s detection timeout and 10s health timeout
    let detector = AutowareDetector::new(
        node.clone(),
        Some(Duration::from_secs(60)),
        Some(Duration::from_secs(10)),
    )?;

    log::info!("Waiting for Autoware detection (timeout: 60s)...");
    log::info!("Spinning executor to process ROS messages...");

    // Spin the executor while waiting for detection
    let start = Instant::now();
    let timeout = Duration::from_secs(60);

    while start.elapsed() < timeout && !detector.is_alive() {
        // Spin executor to process callbacks
        let _ = executor.spin(SpinOptions::default().timeout(Duration::from_millis(100)));
    }

    if !detector.is_alive() {
        return Err("Autoware detection timeout after 60s".into());
    }

    log::info!("Autoware detected!");

    // Get the URDF
    if let Some(urdf) = detector.get_urdf() {
        log::info!("URDF received ({} bytes)", urdf.len());

        // Parse URDF for sensors
        log::info!("Parsing URDF for sensors...");
        match parse_urdf_sensors(&urdf) {
            Ok(sensors) => {
                log::info!("Found {} sensors in URDF:", sensors.len());
                for sensor in &sensors {
                    log::info!("  - {} (type: {:?})", sensor.link_name, sensor.sensor_type);
                    log::info!(
                        "    Parent: {}, Position: [{:.3}, {:.3}, {:.3}]",
                        sensor.parent_frame,
                        sensor.position.x,
                        sensor.position.y,
                        sensor.position.z
                    );
                }
            }
            Err(e) => {
                log::error!("Failed to parse URDF: {:?}", e);
            }
        }
    } else {
        log::warn!("No URDF available yet");
    }

    // Check additional components
    log::info!("Checking robot_state_publisher...");
    match detector.check_robot_state_publisher() {
        Ok(found) => {
            if found {
                log::info!("  ✓ robot_state_publisher node found");
            } else {
                log::warn!("  ✗ robot_state_publisher node not found");
            }
        }
        Err(e) => {
            log::error!("  Error checking robot_state_publisher: {:?}", e);
        }
    }

    log::info!("Checking /tf_static topic...");
    match detector.check_tf_static_topic() {
        Ok(found) => {
            if found {
                log::info!("  ✓ /tf_static topic found");
            } else {
                log::warn!("  ✗ /tf_static topic not found");
            }
        }
        Err(e) => {
            log::error!("  Error checking /tf_static topic: {:?}", e);
        }
    }

    // Get diagnostics
    let diag = detector.get_diagnostics();
    log::info!("Diagnostics:");
    log::info!("  State: {:?}", diag.state);
    log::info!("  URDF available: {}", diag.urdf_available);
    if let Some(size) = diag.urdf_size {
        log::info!("  URDF size: {} bytes", size);
    }
    if let Some(elapsed) = diag.time_since_last_update {
        log::info!("  Time since last update: {:?}", elapsed);
    }

    // Test TF2 Buffer
    log::info!("\n=== Testing TF2 Buffer ===");
    match TFBuffer::new(node.clone()) {
        Ok(tf_buffer) => {
            log::info!("TF Buffer created successfully");

            // Give it a moment to receive transforms
            std::thread::sleep(Duration::from_secs(2));

            log::info!("TF Buffer has {} transforms", tf_buffer.len());

            if !tf_buffer.is_empty() {
                log::info!("Available frames:");
                for frame in tf_buffer.get_all_frames() {
                    log::info!("  - {}", frame);
                }

                // Try to get a specific transform if available
                let frames = tf_buffer.get_all_frames();
                if frames.len() >= 2 {
                    let target = &frames[0];
                    let source = &frames[1];
                    log::info!(
                        "Attempting to lookup transform from '{}' to '{}'...",
                        source,
                        target
                    );
                    match tf_buffer.lookup_transform(target, source) {
                        Ok(transform) => {
                            log::info!("  ✓ Transform found!");
                            log::info!(
                                "    Translation: [{:.3}, {:.3}, {:.3}]",
                                transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z
                            );
                        }
                        Err(e) => {
                            log::warn!("  Could not lookup transform: {:?}", e);
                        }
                    }
                }
            } else {
                log::warn!("No transforms received yet");
            }
        }
        Err(e) => {
            log::error!("Failed to create TF Buffer: {:?}", e);
        }
    }

    // Test Coordinate Conversion
    log::info!("\n=== Testing Coordinate Conversion ===");

    // Test position conversion
    log::info!("Position conversion:");
    let ros_pos = Vector3::new(1.0, 2.0, 0.5);
    log::info!(
        "  ROS position (m, right-handed):  [{:.2}, {:.2}, {:.2}]",
        ros_pos.x,
        ros_pos.y,
        ros_pos.z
    );
    let carla_pos = ros_to_carla_position(&ros_pos);
    log::info!(
        "  CARLA position (cm, left-handed): [{:.2}, {:.2}, {:.2}]",
        carla_pos.x,
        carla_pos.y,
        carla_pos.z
    );
    let ros_pos_back = carla_to_ros_position(&carla_pos);
    log::info!(
        "  Round-trip back to ROS:          [{:.2}, {:.2}, {:.2}]",
        ros_pos_back.x,
        ros_pos_back.y,
        ros_pos_back.z
    );

    // Test rotation conversion
    log::info!("\nRotation conversion:");
    let (roll, pitch, yaw) = (0.0, 0.0, std::f64::consts::FRAC_PI_2); // 90° yaw
    log::info!(
        "  ROS Euler (rad):  roll={:.3}, pitch={:.3}, yaw={:.3}",
        roll,
        pitch,
        yaw
    );

    let q = euler_to_quaternion(roll, pitch, yaw);
    log::info!(
        "  Quaternion:       x={:.3}, y={:.3}, z={:.3}, w={:.3}",
        q.i,
        q.j,
        q.k,
        q.w
    );

    let (r2, p2, y2) = quaternion_to_euler(&q);
    log::info!(
        "  Back to Euler:    roll={:.3}, pitch={:.3}, yaw={:.3}",
        r2,
        p2,
        y2
    );

    log::info!("\n=== Test Complete ===");

    Ok(())
}
