mod autoware;
mod bridge;
mod clock;
mod error;
mod types;
mod utils;

use std::{
    collections::HashMap,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};

use bridge::actor_bridge::{ActorBridge, BridgeType};
use carla::{
    client::{ActorBase, Client, Vehicle},
    rpc::ActorId,
};
use clap::Parser;
use clock::SimulatorClock;
use error::{BridgeError, Result};
use rclrs::CreateBasicExecutor;

/// ROS 2 Carla bridge for Autoware
#[derive(Debug, Clone, Parser)]
#[clap(version, about)]
struct Opts {
    /// Carla simulator address.
    #[clap(long, default_value = "127.0.0.1")]
    pub carla_address: String,

    /// Carla simulator port.
    #[clap(long, default_value = "2000")]
    pub carla_port: u16,

    /// Vehicle role_name to bridge (e.g., "ego_vehicle")
    #[clap(long, conflicts_with = "vehicle_id")]
    pub vehicle_name: Option<String>,

    /// Vehicle actor ID to bridge
    #[clap(long, conflicts_with = "vehicle_name")]
    pub vehicle_id: Option<u32>,

    /// Timeout in seconds to wait for vehicle (0 = wait forever)
    #[clap(long, default_value_t = 30)]
    pub vehicle_wait_timeout: u64,
}

/// Structure to hold ego vehicle and its sensors
struct EgoBridges {
    vehicle: Box<dyn ActorBridge>,
    sensors: HashMap<ActorId, Box<dyn ActorBridge>>,
}

/// Find the target vehicle by name or ID
fn find_target_vehicle(client: &Client, opts: &Opts, timeout_secs: u64) -> Result<Vehicle> {
    let start_time = std::time::Instant::now();
    let timeout = if timeout_secs == 0 {
        Duration::MAX
    } else {
        Duration::from_secs(timeout_secs)
    };

    loop {
        let world = client.world();
        let actors = world.actors();

        for actor in actors.iter() {
            if let carla::client::ActorKind::Vehicle(vehicle) = actor.into_kinds() {
                // Check if this is our target vehicle
                let matches = if let Some(ref name) = opts.vehicle_name {
                    // Match by role_name
                    vehicle
                        .attributes()
                        .iter()
                        .find(|attr| attr.id() == "role_name")
                        .map(|attr| attr.value_string() == *name)
                        .unwrap_or(false)
                } else if let Some(id) = opts.vehicle_id {
                    // Match by actor ID
                    vehicle.id() == id
                } else {
                    false
                };

                if matches {
                    log::info!("Found target vehicle: ID={}", vehicle.id());
                    return Ok(vehicle);
                }
            }
        }

        // Check timeout
        if start_time.elapsed() >= timeout {
            return Err(BridgeError::CarlaIssue(
                "Timeout waiting for target vehicle to appear",
            ));
        }

        // Log progress every 5 seconds
        if start_time.elapsed().as_secs().is_multiple_of(5) && start_time.elapsed().as_secs() > 0 {
            log::info!(
                "Still waiting for target vehicle... ({} seconds elapsed)",
                start_time.elapsed().as_secs()
            );
        }

        // Wait a bit before polling again
        std::thread::sleep(Duration::from_secs(1));
    }
}

fn main() -> Result<()> {
    pretty_env_logger::init();

    let opts = Opts::parse();

    // Validate that either vehicle_name or vehicle_id is provided
    if opts.vehicle_name.is_none() && opts.vehicle_id.is_none() {
        return Err(BridgeError::CarlaIssue(
            "Either --vehicle-name or --vehicle-id must be specified",
        ));
    }

    // Flag for graceful shutdown when Ctrl-C is pressed
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })
    .expect("Failed to set Ctrl-C handler");

    log::info!("Running Carla Autoware ROS 2 bridge (1-to-1 mode)...");
    if let Some(ref name) = opts.vehicle_name {
        log::info!("Target vehicle: role_name = '{}'", name);
    } else if let Some(id) = opts.vehicle_id {
        log::info!("Target vehicle: actor ID = {}", id);
    }

    // Initialize ROS 2 context and executor
    let ctx = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?;
    let executor = ctx.create_basic_executor();
    let node = executor.create_node("autoware_carla_bridge")?;

    // Connect to CARLA (passive mode - do NOT configure synchronous mode)
    let client = Client::connect(&opts.carla_address, opts.carla_port, None);
    let world = client.world();

    // Find target ego vehicle
    log::info!("Searching for target vehicle...");
    let ego_vehicle = find_target_vehicle(&client, &opts, opts.vehicle_wait_timeout)?;
    let ego_vehicle_id = ego_vehicle.id();

    // Create single Autoware instance (root namespace)
    let mut autoware = autoware::Autoware::new();

    // Create clock publisher
    let simulator_clock =
        SimulatorClock::new(node.clone()).expect("Unable to create simulator clock!");

    // Discover sensors attached to ego vehicle and register them with Autoware
    log::info!("Discovering sensors attached to ego vehicle...");
    let mut sensor_info = Vec::new();

    for actor in world.actors().iter() {
        let actor_id = actor.id(); // Save ID before consuming actor
        if let carla::client::ActorKind::Sensor(sensor) = actor.into_kinds() {
            // Check if this sensor belongs to our ego vehicle
            if let Some(parent) = sensor.parent() {
                if parent.id() == ego_vehicle_id {
                    // This sensor belongs to our ego vehicle
                    match bridge::actor_bridge::get_bridge_type(carla::client::ActorKind::Sensor(
                        sensor.clone(),
                    )) {
                        Ok(BridgeType::Sensor(sensor_type, sensor_name)) => {
                            log::info!("Found sensor: {} (type: {:?})", sensor_name, sensor_type);
                            // Add sensor to autoware to register topics
                            autoware.add_sensors(sensor_type, sensor_name.clone());
                            sensor_info.push((actor_id, sensor.clone(), sensor_type, sensor_name));
                        }
                        Ok(_) => {
                            log::debug!("Ignoring non-sensor actor");
                        }
                        Err(BridgeError::OwnerlessSensor { sensor_id }) => {
                            log::debug!(
                                "Sensor {} is ownerless (should not happen for our ego vehicle)",
                                sensor_id
                            );
                        }
                        Err(err) => {
                            log::error!("Unexpected error processing sensor: {err:?}");
                            return Err(err);
                        }
                    }
                }
            }
        }
    }

    // Create vehicle bridge
    log::info!(
        "Creating bridge for ego vehicle (ID: {})...",
        ego_vehicle_id
    );
    let vehicle_bridge = bridge::actor_bridge::create_bridge(
        node.clone(),
        carla::client::ActorKind::Vehicle(ego_vehicle.clone()),
        BridgeType::Vehicle,
        &autoware,
    )?;

    // Create sensor bridges (now that Autoware has all sensor topics registered)
    let mut ego_bridges = EgoBridges {
        vehicle: vehicle_bridge,
        sensors: HashMap::new(),
    };

    for (sensor_id, sensor, sensor_type, sensor_name) in sensor_info {
        let sensor_bridge = bridge::actor_bridge::create_bridge(
            node.clone(),
            carla::client::ActorKind::Sensor(sensor),
            BridgeType::Sensor(sensor_type, sensor_name),
            &autoware,
        )?;
        ego_bridges.sensors.insert(sensor_id, sensor_bridge);
    }

    log::info!(
        "Bridge initialization complete. Ego vehicle has {} sensors.",
        ego_bridges.sensors.len()
    );

    // Track consecutive timeouts to detect CARLA connection issues
    let mut consecutive_timeouts = 0;
    const MAX_CONSECUTIVE_TIMEOUTS: u32 = 5;

    // === Main loop ===
    // Keep running until Ctrl-C is pressed
    while running.load(Ordering::SeqCst) {
        // Wait for next CARLA tick
        match world.wait_for_tick() {
            Ok(_) => {
                // Successfully waited for tick, reset timeout counter
                consecutive_timeouts = 0;
            }
            Err(e) => {
                // Check if this is a timeout error
                log::warn!("Failed to wait for CARLA tick: {e:?}");
                consecutive_timeouts += 1;

                if consecutive_timeouts >= MAX_CONSECUTIVE_TIMEOUTS {
                    log::error!(
                        "Reached {} consecutive timeouts. CARLA may be unresponsive. Stopping bridge.",
                        MAX_CONSECUTIVE_TIMEOUTS
                    );
                    break;
                }
                continue;
            }
        }

        // Get current simulation time
        let sec = world.snapshot().timestamp().elapsed_seconds;

        // Publish clock
        simulator_clock
            .publish_clock(Some(sec))
            .expect("Unable to publish clock");

        // Step ego vehicle bridge
        ego_bridges.vehicle.step(sec)?;

        // Sensors use callbacks, no explicit step needed
    }

    log::info!("Bridge shutting down gracefully");
    Ok(())
}
