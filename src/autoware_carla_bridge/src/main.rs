mod autoware;
mod autoware_detection;
mod bridge;
mod carla_vehicle;
mod clock;
mod coordinate_conversion;
mod error;
mod sensor_config;
mod tf_bridge;
mod types;
mod urdf_parser;
mod utils;

use bridge::{
    actor_bridge::BridgeType,
    sensor_bridge::{SensorBridge, SensorType},
};

use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};

use carla::client::{ActorBase, Client};
use carla_vehicle::CarlaVehicle;
use clap::Parser;
use clock::SimulatorClock;
use error::{BridgeError, Result};
use rclrs::CreateBasicExecutor;

/// Create sensor bridges for all sensors spawned by CarlaVehicle
///
/// This iterates over the sensor configurations and creates a SensorBridge
/// for each sensor that was spawned in CARLA. The bridges handle CARLA
/// sensor callbacks and publish data to ROS topics.
fn create_sensor_bridges(
    node: rclrs::Node,
    carla_vehicle: &CarlaVehicle,
    autoware: &autoware::Autoware,
) -> Result<Vec<SensorBridge>> {
    let sensor_configs = carla_vehicle.get_sensor_configs();
    let sensors = carla_vehicle.get_sensors();
    let mut bridges = Vec::new();

    for config in sensor_configs {
        // Get the sensor actor from the HashMap
        let sensor = match sensors.get(&config.link_name) {
            Some(s) => s.clone(),
            None => {
                tracing::warn!(
                    "Sensor '{}' not found in spawned sensors, skipping",
                    config.link_name
                );
                continue;
            }
        };

        // Create bridge type from sensor config
        let bridge_type = BridgeType::Sensor(config.sensor_type, config.link_name.clone());

        // Create sensor bridge
        match SensorBridge::new(node.clone(), sensor, bridge_type, autoware) {
            Ok(bridge) => {
                tracing::info!(
                    "Created sensor bridge for '{}' (type: {:?})",
                    config.link_name,
                    config.sensor_type
                );
                bridges.push(bridge);
            }
            Err(e) => {
                tracing::error!(
                    "Failed to create sensor bridge for '{}': {}",
                    config.link_name,
                    e
                );
                // Continue with other sensors rather than failing completely
            }
        }
    }

    Ok(bridges)
}

/// ROS 2 CARLA bridge for Autoware (Autoware-centric architecture)
///
/// This bridge waits for Autoware to start, then spawns a vehicle in CARLA
/// based on the initial pose set in RViz. It forwards vehicle state from
/// CARLA to Autoware for localization, and applies control commands from
/// Autoware to the CARLA vehicle.
#[derive(Debug, Clone, Parser)]
#[clap(version, about)]
struct Opts {
    /// CARLA simulator address
    #[clap(long, default_value = "127.0.0.1")]
    pub carla_address: String,

    /// CARLA simulator port
    #[clap(long, default_value = "2000")]
    pub carla_port: u16,

    /// CARLA map to load (e.g., "Town01", "Town10HD"). If not specified, uses current map.
    #[clap(long)]
    pub map_name: Option<String>,

    /// Vehicle blueprint to spawn (CARLA blueprint ID)
    #[clap(long, default_value = "vehicle.tesla.model3")]
    pub vehicle_blueprint: String,

    /// Timeout in seconds to wait for Autoware detection (0 = wait forever)
    #[clap(long, default_value_t = 60)]
    pub autoware_timeout: u64,

    /// Timeout in seconds to wait for initial pose from RViz (0 = wait forever)
    #[clap(long, default_value_t = 60)]
    pub pose_timeout: u64,

    /// Path to CARLA sensor configuration file (YAML)
    #[clap(long, default_value = "config/carla_sensors.yaml")]
    pub sensor_config: String,
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

    let opts = Opts::parse();

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

    tracing::info!("=== Autoware-CARLA Bridge (Autoware-centric) ===");
    tracing::info!("Vehicle blueprint: {}", opts.vehicle_blueprint);

    // === Step 1: Initialize ROS 2 ===
    tracing::info!("Initializing ROS 2...");
    let ctx = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?;
    let mut executor = ctx.create_basic_executor();
    let node = executor.create_node("autoware_carla_bridge")?;
    tracing::info!("ROS 2 node created: autoware_carla_bridge");

    // === Step 2: Connect to CARLA ===
    tracing::info!(
        "Connecting to CARLA at {}:{}...",
        opts.carla_address,
        opts.carla_port
    );
    let client = Client::connect(&opts.carla_address, opts.carla_port, None);

    // Load map if specified, otherwise use current map
    let mut world = if let Some(ref map_name) = opts.map_name {
        tracing::info!("Loading CARLA map: {}", map_name);
        utils::load_world_smart(&client, map_name)
    } else {
        tracing::info!("Using current CARLA map");
        client.world()
    };

    tracing::info!("Connected to CARLA successfully");

    // Create clock publisher
    let simulator_clock = SimulatorClock::new(node.clone())?;

    // === Step 3: Create Autoware coordinator and wait for Autoware ===
    tracing::info!("Creating Autoware coordinator...");
    let mut autoware = autoware::Autoware::new(node.clone())?;

    tracing::info!("Waiting for Autoware to start...");
    tracing::info!("(Start Autoware planning simulator with sample_sensor_kit)");

    let autoware_timeout = if opts.autoware_timeout == 0 {
        None
    } else {
        Some(Duration::from_secs(opts.autoware_timeout))
    };

    // Wait for Autoware with timeout
    let start_time = std::time::Instant::now();
    loop {
        // Spin executor to process ROS callbacks (e.g., /robot_description subscription)
        executor.spin(rclrs::SpinOptions::spin_once().timeout(Duration::from_millis(100)));

        if autoware.is_alive() {
            break;
        }

        if !running.load(Ordering::SeqCst) {
            tracing::info!("Shutdown requested while waiting for Autoware");
            return Ok(());
        }

        if let Some(timeout) = autoware_timeout {
            if start_time.elapsed() >= timeout {
                return Err(BridgeError::AutowareIssue(
                    "Timeout waiting for Autoware to start".to_string(),
                ));
            }
        }
    }

    tracing::info!("Autoware detected!");

    // === Step 4: Parse URDF to get sensor configurations ===
    tracing::info!("Parsing URDF from Autoware...");
    autoware.parse_sensors()?;

    let sensor_configs = autoware.sensor_configs();
    tracing::info!("Found {} sensors in URDF:", sensor_configs.len());
    for config in sensor_configs {
        tracing::info!(
            "  - {} (type: {:?}, parent: {})",
            config.link_name,
            config.sensor_type,
            config.parent_frame
        );
    }

    // === Step 5: Wait for initial pose from RViz ===
    tracing::info!("Waiting for initial pose from RViz...");
    tracing::info!("(Use '2D Pose Estimate' tool in RViz to set vehicle position)");

    let pose_timeout = if opts.pose_timeout == 0 {
        None
    } else {
        Some(Duration::from_secs(opts.pose_timeout))
    };

    autoware.wait_for_initial_pose(pose_timeout, &running, &mut executor)?;

    // Get initial pose from Autoware
    let initial_pose = autoware.take_initial_pose()?;

    // === Step 6: Load CARLA sensor configuration ===
    let carla_config = match sensor_config::CarlaConfig::from_file(&opts.sensor_config) {
        Ok(config) => config,
        Err(e) => {
            tracing::warn!(
                "Failed to load sensor config from '{}': {}",
                opts.sensor_config,
                e
            );
            tracing::warn!("Using default sensor parameters");
            sensor_config::CarlaConfig::default()
        }
    };

    // === Step 7: Spawn vehicle and sensors in CARLA ===
    tracing::info!("Spawning vehicle and sensors in CARLA...");
    let mut carla_vehicle = CarlaVehicle::new(
        &mut world,
        &opts.vehicle_blueprint,
        &initial_pose,
        sensor_configs,
        autoware.get_tf_buffer(),
        &carla_config,
    )?;

    let vehicle = carla_vehicle.get_vehicle();

    tracing::info!("Vehicle and sensors spawned successfully!");

    // === Step 8: Create sensor bridges ===
    tracing::info!("Creating sensor bridges...");
    let _sensor_bridges = create_sensor_bridges(node.clone(), &carla_vehicle, &autoware)?;
    tracing::info!("Created {} sensor bridges", _sensor_bridges.len());

    tracing::info!("=== Bridge running ===");

    // Track consecutive timeouts to detect CARLA connection issues
    let mut consecutive_timeouts = 0;
    const MAX_CONSECUTIVE_TIMEOUTS: u32 = 5;

    // === Main Loop ===
    while running.load(Ordering::SeqCst) {
        // Check Autoware health
        autoware.health_check();
        if !autoware.is_alive() {
            tracing::warn!("Autoware connection lost! Cleaning up...");
            carla_vehicle.cleanup()?;
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

        // Wait for next CARLA tick with short timeout to allow responsive shutdown
        match world.wait_for_tick_or_timeout(Duration::from_secs(1)) {
            Some(_) => {
                consecutive_timeouts = 0;
            }
            None => {
                // Timeout - check if we should continue waiting
                consecutive_timeouts += 1;

                if consecutive_timeouts >= MAX_CONSECUTIVE_TIMEOUTS {
                    tracing::error!(
                        "Reached {} consecutive timeouts. CARLA may be unresponsive.",
                        MAX_CONSECUTIVE_TIMEOUTS
                    );
                    break;
                }
                continue;
            }
        }

        // Get current simulation time
        let snapshot = world.snapshot();
        let timestamp = snapshot.timestamp();
        let sec = timestamp.elapsed_seconds;

        // Publish clock
        simulator_clock.publish_clock(Some(sec))?;

        // Get vehicle transform and velocity from CARLA
        let transform = vehicle.transform();
        let velocity = vehicle.velocity();
        let angular_velocity = vehicle.angular_velocity();

        // Convert CARLA geom types to nalgebra for coordinate conversion
        let na_transform = transform.to_na();
        let na_velocity = velocity.to_na();
        let na_angular_velocity = angular_velocity.to_na();

        // Convert CARLA coordinates to ROS coordinates
        let position = coordinate_conversion::carla_to_ros_position(&nalgebra::Vector3::new(
            na_transform.translation.x as f64,
            na_transform.translation.y as f64,
            na_transform.translation.z as f64,
        ));

        // Convert CARLA rotation (quaternion) to ROS quaternion
        // 1. Extract quaternion from CARLA transform (UnitQuaternion<f32>)
        // 2. Convert to f64 for coordinate_conversion functions
        // 3. Convert to Euler angles to apply coordinate system transform
        // 4. Apply coordinate flip (roll and yaw signs)
        // 5. Convert back to quaternion
        let carla_quat = nalgebra::Quaternion::new(
            na_transform.rotation.w as f64,
            na_transform.rotation.i as f64,
            na_transform.rotation.j as f64,
            na_transform.rotation.k as f64,
        );
        let (roll, pitch, yaw) = coordinate_conversion::quaternion_to_euler(&carla_quat);
        let orientation = coordinate_conversion::euler_to_quaternion(
            -roll, // Roll sign flip for ROS right-handed system
            pitch, -yaw, // Yaw sign flip for ROS right-handed system
        );

        let linear_vel = coordinate_conversion::carla_to_ros_velocity(&nalgebra::Vector3::new(
            na_velocity.x as f64,
            na_velocity.y as f64,
            na_velocity.z as f64,
        ));

        let angular_vel =
            coordinate_conversion::carla_to_ros_angular_velocity(&nalgebra::Vector3::new(
                na_angular_velocity.x as f64,
                na_angular_velocity.y as f64,
                na_angular_velocity.z as f64,
            ));

        // Create ROS timestamp
        let ros_timestamp = builtin_interfaces::msg::Time {
            sec: sec as i32,
            nanosec: ((sec - sec.floor()) * 1e9) as u32,
        };

        // Publish localization
        autoware.publish_localization(
            &ros_timestamp,
            &[position.x, position.y, position.z],
            &[orientation.w, orientation.i, orientation.j, orientation.k],
            &[linear_vel.x, linear_vel.y, linear_vel.z],
            &[angular_vel.x, angular_vel.y, angular_vel.z],
        )?;

        // TODO: Apply control commands from Autoware to CARLA vehicle
        // let actuation_cmd = autoware.get_actuation_cmd();
        // Apply throttle/brake/steering to vehicle
    }

    tracing::info!("Cleaning up...");
    carla_vehicle.cleanup()?;
    tracing::info!("Bridge shutdown complete");

    Ok(())
}
