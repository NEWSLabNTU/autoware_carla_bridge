mod autoware;
mod autoware_detection;
mod bridge;
pub mod bridge_config;
mod carla_vehicle;
mod clock;
mod coordinate_conversion;
mod error;
mod sensor_config;
mod tf_bridge;
mod types;
mod urdf_parser;
mod utils;
mod vehicle_control;

use bridge::{actor_bridge::BridgeType, sensor_bridge::SensorBridge};

use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::Duration,
};

use carla::client::Client;
use carla_vehicle::CarlaVehicle;
use clock::SimulatorClock;
use error::Result;
use rclrs::CreateBasicExecutor;

/// Create sensor bridges for all sensors spawned by CarlaVehicle
///
/// This iterates over the sensor types (derived from VehicleConfig blueprints)
/// and creates a SensorBridge for each sensor that was spawned in CARLA.
/// The bridges handle CARLA sensor callbacks and publish data to ROS topics.
fn create_sensor_bridges(
    node: rclrs::Node,
    carla_vehicle: &CarlaVehicle,
    autoware: &autoware::Autoware,
) -> Result<Vec<SensorBridge>> {
    let sensor_types = carla_vehicle.get_sensor_types();
    let sensors = carla_vehicle.get_sensors();
    let mut bridges = Vec::new();

    for (link_name, sensor_type) in sensor_types {
        // Get the sensor actor from the HashMap
        let sensor = match sensors.get(link_name) {
            Some(s) => s.clone(),
            None => {
                tracing::warn!(
                    "Sensor '{}' not found in spawned sensors, skipping",
                    link_name
                );
                continue;
            }
        };

        // Map sensor_config::SensorType to bridge::sensor_bridge::SensorType
        let bridge_sensor_type = match sensor_type {
            sensor_config::SensorType::Camera => bridge::sensor_bridge::SensorType::CameraRgb,
            sensor_config::SensorType::Lidar => bridge::sensor_bridge::SensorType::LidarRayCast,
            sensor_config::SensorType::Imu => bridge::sensor_bridge::SensorType::Imu,
            sensor_config::SensorType::Gnss => bridge::sensor_bridge::SensorType::Gnss,
            sensor_config::SensorType::Radar => {
                tracing::warn!("Radar sensor '{}' not yet supported, skipping", link_name);
                continue;
            }
        };

        // Create bridge type from sensor type
        let bridge_type = BridgeType::Sensor(bridge_sensor_type, link_name.clone());

        // Create sensor bridge
        match SensorBridge::new(node.clone(), sensor, bridge_type, autoware) {
            Ok(bridge) => {
                tracing::info!(
                    "Created sensor bridge for '{}' (type: {:?})",
                    link_name,
                    sensor_type
                );
                bridges.push(bridge);
            }
            Err(e) => {
                tracing::error!("Failed to create sensor bridge for '{}': {}", link_name, e);
                // Continue with other sensors rather than failing completely
            }
        }
    }

    Ok(bridges)
}

/// Bridge configuration loaded from ROS parameters
///
/// Parameters are declared with defaults and can be overridden via:
/// - Launch file: `<param name="carla_address" value="192.168.1.1"/>`
/// - Command line: `--ros-args -p carla_address:=192.168.1.1`
struct BridgeParams {
    pub carla_address: String,
    pub carla_port: u16,
    pub map_name: Option<String>,
    pub vehicle_blueprint: String,
    /// Kept for CLI backward compatibility but ignored (bridge uses infinite retry)
    #[allow(dead_code)]
    pub autoware_timeout: u64,
    pub vehicle_config: String,
    pub bridge_config: String,
}

impl BridgeParams {
    /// Declare and read all bridge parameters from a ROS node
    fn from_node(node: &rclrs::Node) -> Result<Self> {
        use error::BridgeError;

        // Declare parameters with defaults
        let carla_address = node
            .declare_parameter("carla_address")
            .default("127.0.0.1".into())
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let carla_port = node
            .declare_parameter("carla_port")
            .default(2000i64)
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let map_name = node
            .declare_parameter::<Arc<str>>("map_name")
            .optional()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let vehicle_blueprint = node
            .declare_parameter("vehicle_blueprint")
            .default("vehicle.tesla.model3".into())
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let autoware_timeout = node
            .declare_parameter("autoware_timeout")
            .default(60i64)
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let vehicle_config = node
            .declare_parameter("vehicle_config")
            .default("".into())
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let bridge_config = node
            .declare_parameter("bridge_config")
            .default("config/bridge.yaml".into())
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        // Get parameter values
        let carla_address_val: Arc<str> = carla_address.get();
        let carla_port_val: i64 = carla_port.get();
        let map_name_val: Option<Arc<str>> = map_name.get();
        let vehicle_blueprint_val: Arc<str> = vehicle_blueprint.get();
        let autoware_timeout_val: i64 = autoware_timeout.get();
        let vehicle_config_val: Arc<str> = vehicle_config.get();
        let bridge_config_val: Arc<str> = bridge_config.get();

        // Validate required parameters
        if vehicle_config_val.is_empty() {
            return Err(BridgeError::ConfigError(
                "vehicle_config parameter is required but not set".into(),
            ));
        }

        Ok(Self {
            carla_address: carla_address_val.to_string(),
            carla_port: carla_port_val as u16,
            map_name: map_name_val.map(|s| s.to_string()),
            vehicle_blueprint: vehicle_blueprint_val.to_string(),
            autoware_timeout: autoware_timeout_val as u64,
            vehicle_config: vehicle_config_val.to_string(),
            bridge_config: bridge_config_val.to_string(),
        })
    }
}

fn main() -> Result<()> {
    // Install color-eyre for better error reporting
    color_eyre::install().expect("Failed to install color-eyre");

    // Initialize tracing subscriber with env filter
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    // Flag for graceful shutdown when Ctrl-C is pressed
    let running = Arc::new(AtomicBool::new(true));
    {
        let running = running.clone();
        ctrlc::set_handler(move || {
            tracing::info!("Ctrl-C received, shutting down...");
            running.store(false, Ordering::SeqCst);
        })
        .expect("Failed to set Ctrl-C handler");
    }

    // === Step 1: Initialize ROS 2 ===
    // Initialize ROS first so it can handle --ros-args from launch system
    tracing::info!("Initializing ROS 2...");
    let ctx = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?;
    let mut executor = ctx.create_basic_executor();
    let node = executor.create_node("autoware_carla_bridge")?;
    tracing::info!("ROS 2 node created: autoware_carla_bridge");

    // === Step 2: Read parameters from ROS node ===
    let params = BridgeParams::from_node(&node)?;
    tracing::info!("=== Autoware-CARLA Bridge (Autoware-centric) ===");
    tracing::info!("Vehicle blueprint: {}", params.vehicle_blueprint);

    // === Step 3: Connect to CARLA ===
    tracing::info!(
        "Connecting to CARLA at {}:{}...",
        params.carla_address,
        params.carla_port
    );

    // Retry connecting to CARLA in an infinite loop
    let client = loop {
        match Client::try_connect(&params.carla_address, params.carla_port, None) {
            Ok(mut client) => {
                client.set_timeout(Duration::from_secs(30));
                // Try to access world to verify connection is working
                match client.try_world() {
                    Ok(_) => {
                        tracing::info!("Connected to CARLA successfully");
                        break client;
                    }
                    Err(e) => {
                        tracing::warn!("CARLA not ready: {e}, retrying in 5 seconds...");
                        std::thread::sleep(Duration::from_secs(5));
                    }
                }
            }
            Err(e) => {
                tracing::warn!("Failed to connect to CARLA: {e}, retrying in 5 seconds...");
                std::thread::sleep(Duration::from_secs(5));
            }
        }

        // Check for Ctrl-C during retry
        if !running.load(Ordering::SeqCst) {
            tracing::info!("Shutdown requested while connecting to CARLA");
            return Ok(());
        }
    };

    // Load map if specified, otherwise use current map
    let mut world = if let Some(ref map_name) = params.map_name {
        tracing::info!("Loading CARLA map: {}", map_name);
        utils::load_world_smart(&client, map_name)
    } else {
        tracing::info!("Using current CARLA map");
        client.world()
    };

    // Create clock publisher
    let simulator_clock = SimulatorClock::new(node.clone())?;

    // === Step 3: Load bridge configuration ===
    let bridge_config = bridge_config::BridgeConfig::from_file(&params.bridge_config)?;
    let initial_pose = bridge_config.to_isometry();

    // === Step 4: Create Autoware coordinator and wait for Autoware ===
    tracing::info!("Creating Autoware coordinator...");
    let mut autoware = autoware::Autoware::new(
        node.clone(),
        bridge_config.publish_direct_localization,
    )?;

    tracing::info!("Waiting for Autoware to start...");
    tracing::info!("(Start Autoware planning simulator with sample_sensor_kit)");
    tracing::info!("Listening for /robot_description topic...");

    // Wait for Autoware indefinitely (user controls timeout via Ctrl-C)
    // Note: autoware_timeout CLI parameter is ignored for robustness
    let start_time = std::time::Instant::now();
    let mut last_log_time = std::time::Instant::now();
    let mut attempt_count = 0u32;

    loop {
        // Spin executor to process ROS callbacks (e.g., /robot_description subscription)
        executor.spin(rclrs::SpinOptions::spin_once().timeout(Duration::from_millis(100)));

        if autoware.is_alive() {
            tracing::info!(
                "Autoware detected after {} attempts ({:.1}s)",
                attempt_count,
                start_time.elapsed().as_secs_f32()
            );
            break;
        }

        attempt_count += 1;

        // Log progress every 5 seconds
        if last_log_time.elapsed() >= Duration::from_secs(5) {
            tracing::info!(
                "Still waiting for Autoware... (attempt {}, {:.0}s elapsed)",
                attempt_count,
                start_time.elapsed().as_secs_f32()
            );
            tracing::info!("  Expecting /robot_description topic with URDF data");
            last_log_time = std::time::Instant::now();
        }

        if !running.load(Ordering::SeqCst) {
            tracing::info!("Shutdown requested while waiting for Autoware");
            return Ok(());
        }
    }

    tracing::info!("Autoware detected!");

    // === Step 4.5: Wait for TF transforms to be fully received ===
    // The /tf_static topic may arrive in multiple messages from different publishers
    // We need to ensure all transforms from the sensor_kit are available before spawning
    tracing::info!("Waiting for TF transforms to be received...");
    let tf_start_time = std::time::Instant::now();
    let tf_timeout = Duration::from_secs(10);
    let min_transforms = 4; // At minimum: sensor_kit_base_link, top_base_link, top, and at least one sensor

    loop {
        // Spin executor to process TF callbacks
        executor.spin(rclrs::SpinOptions::spin_once().timeout(Duration::from_millis(100)));

        let tf_count = autoware.get_tf_buffer().get_all_frames().len();
        if tf_count >= min_transforms {
            tracing::info!("TF transforms ready: {} frames available", tf_count);
            // Log available frames for debugging
            let frames = autoware.get_tf_buffer().get_all_frames();
            tracing::debug!("Available TF frames: {:?}", frames);
            break;
        }

        if tf_start_time.elapsed() >= tf_timeout {
            tracing::warn!(
                "TF timeout after {:?}: only {} transforms received (need at least {})",
                tf_timeout,
                tf_count,
                min_transforms
            );
            tracing::warn!(
                "Available frames: {:?}",
                autoware.get_tf_buffer().get_all_frames()
            );
            // Continue anyway - the sensor spawning will fail with more specific error
            break;
        }

        if !running.load(Ordering::SeqCst) {
            tracing::info!("Shutdown requested while waiting for TF");
            return Ok(());
        }

        // Log progress every 2 seconds
        if tf_start_time.elapsed().as_secs() % 2 == 0 {
            tracing::debug!(
                "Waiting for TF... {} transforms (need {})",
                tf_count,
                min_transforms
            );
        }
    }

    // === Step 5: Load vehicle configuration (single source of truth for sensors) ===
    tracing::info!(
        "Loading vehicle configuration from: {}",
        params.vehicle_config
    );
    let vehicle_config = sensor_config::VehicleConfig::from_file(&params.vehicle_config)?;

    tracing::info!(
        "Vehicle config loaded: {} sensors to spawn",
        vehicle_config.sensors.len()
    );
    for (link_name, sensor_def) in &vehicle_config.sensors {
        tracing::info!(
            "  - {} (blueprint: {}, type: {:?})",
            link_name,
            sensor_def.blueprint,
            sensor_def.sensor_type()
        );
    }

    // === Step 6: Spawn vehicle and sensors in CARLA ===
    tracing::info!("Spawning vehicle and sensors in CARLA...");
    let carla_vehicle = CarlaVehicle::new(
        &mut world,
        &initial_pose,
        &vehicle_config,
        autoware.get_tf_buffer(),
    )?;

    // Wrap CarlaVehicle in Arc<Mutex<>> for shared access
    let carla_vehicle = Arc::new(Mutex::new(carla_vehicle));

    // Set vehicle reference in Autoware for ground truth publishing and localization init
    autoware.set_vehicle(carla_vehicle.clone());

    // Get raw vehicle for control callbacks (must be done after wrapping)
    let vehicle_guard = carla_vehicle.lock().unwrap();
    let vehicle = vehicle_guard.get_vehicle();
    let vehicle_shared = Arc::new(Mutex::new(Some(vehicle.clone())));
    drop(vehicle_guard);

    tracing::info!("Vehicle and sensors spawned successfully!");

    // === Step 7: Register sensors with Autoware for topic mapping ===
    tracing::info!("Registering sensors with Autoware...");
    for (link_name, sensor_type) in carla_vehicle.lock().unwrap().get_sensor_types() {
        // Map sensor_config::SensorType to bridge::sensor_bridge::SensorType
        let bridge_sensor_type = match sensor_type {
            sensor_config::SensorType::Camera => bridge::sensor_bridge::SensorType::CameraRgb,
            sensor_config::SensorType::Lidar => bridge::sensor_bridge::SensorType::LidarRayCast,
            sensor_config::SensorType::Imu => bridge::sensor_bridge::SensorType::Imu,
            sensor_config::SensorType::Gnss => bridge::sensor_bridge::SensorType::Gnss,
            sensor_config::SensorType::Radar => {
                tracing::debug!(
                    "Radar sensor '{}' not yet supported for topic mapping",
                    link_name
                );
                continue;
            }
        };

        autoware.add_sensors(bridge_sensor_type, link_name.clone());
        tracing::info!(
            "  Registered sensor '{}' (type: {:?})",
            link_name,
            sensor_type
        );
    }

    // === Step 8: Create sensor bridges ===
    tracing::info!("Creating sensor bridges...");
    let _sensor_bridges =
        create_sensor_bridges(node.clone(), &carla_vehicle.lock().unwrap(), &autoware)?;
    tracing::info!("Created {} sensor bridges", _sensor_bridges.len());

    // === Step 9: Create vehicle control bridge ===
    tracing::info!("Creating vehicle control bridge...");
    let vehicle_control =
        vehicle_control::VehicleControlBridge::new(node.clone(), vehicle_shared.clone())?;
    tracing::info!("Vehicle control bridge created");

    tracing::info!("=== Bridge running ===");

    // Main loop timing: 20Hz (50ms per iteration)
    // Used for rate limiting in async mode
    const LOOP_RATE_HZ: u64 = 20;
    let loop_duration = Duration::from_millis(1000 / LOOP_RATE_HZ);

    // === Main Loop ===
    // Works in both sync and async modes:
    // - Sync mode: wait_for_tick blocks until CARLA ticks
    // - Async mode: wait_for_tick times out, rate limiting prevents busy loop
    while running.load(Ordering::SeqCst) {
        let loop_start = std::time::Instant::now();

        // Check Autoware health
        autoware.health_check();
        if !autoware.is_alive() {
            tracing::warn!("Autoware connection lost! Cleaning up...");
            carla_vehicle.lock().unwrap().cleanup()?;
            tracing::info!("Cleanup complete. Waiting for Autoware to restart...");

            // Wait for Autoware to come back
            loop {
                // Spin executor to process ROS callbacks
                executor.spin(rclrs::SpinOptions::spin_once().timeout(Duration::from_millis(100)));

                if autoware.is_alive() {
                    tracing::info!("Autoware reconnected!");
                    break;
                }
                if !running.load(Ordering::SeqCst) {
                    tracing::info!("Shutdown requested during Autoware wait");
                    return Ok(());
                }
            }

            // TODO: Re-spawn vehicle and sensors
            tracing::warn!("Autoware reconnected, but vehicle respawn not yet implemented");
            tracing::warn!("Please restart the bridge");
            break;
        }

        // Wait for next CARLA tick (sync mode) or timeout (async mode)
        // Short timeout allows responsive shutdown and works for async mode
        let _ = world.wait_for_tick_or_timeout(loop_duration);

        // Get current simulation time
        let snapshot = world.snapshot();
        let timestamp = snapshot.timestamp();
        let sec = timestamp.elapsed_seconds;

        // Publish clock
        simulator_clock.publish_clock(Some(sec))?;

        // Spin executor to process ROS callbacks (subscriptions)
        executor.spin(rclrs::SpinOptions::spin_once().timeout(Duration::from_millis(10)));

        // === Main tick: ground truth publishing + localization init ===
        // All pose/init logic is now encapsulated in autoware.tick()
        if let Err(e) = autoware.tick(sec as f64) {
            tracing::warn!("Autoware tick failed: {}", e);
        }

        // Publish vehicle status (velocity, steering, control mode)
        vehicle_control.publish_status(sec)?;

        // Rate limiting: sleep until next scheduled iteration
        // This prevents timing drift from accumulating
        let next_iteration = loop_start + loop_duration;
        let now = std::time::Instant::now();
        if next_iteration > now {
            std::thread::sleep(next_iteration - now);
        }
    }

    tracing::info!("Cleaning up...");
    carla_vehicle.lock().unwrap().cleanup()?;
    tracing::info!("Bridge shutdown complete");

    Ok(())
}
