use autoware_carla_bridge::{
    autoware_detection::AutowareDetector, urdf_parser::parse_urdf_sensors,
};
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

    log::info!("Test complete!");

    Ok(())
}
