mod autoware;
mod bridge;
mod clock;
mod error;
mod types;
mod utils;

use std::{
    collections::{HashMap, HashSet},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::Duration,
};

use bridge::actor_bridge::{ActorBridge, BridgeType};
use carla::{
    client::{ActorBase, Client},
    rpc::ActorId,
};
use clap::Parser;
use clock::SimulatorClock;
use error::{BridgeError, Result};
use rclrs::CreateBasicExecutor;

// The default interval between ticks
const DEFAULT_CARLA_TICK_INTERVAL_MS: &str = "50";

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

    /// Carla Tick interval (ms)
    #[clap(long, default_value = DEFAULT_CARLA_TICK_INTERVAL_MS)]
    pub tick: u64,

    /// The multiplier to slow down simulated time
    /// For example, if slowdown == 2, 1 simulated second = 2 real seconds
    /// Suggest to set higher if the machine is not powerful enough
    #[clap(long, default_value = "1")]
    pub slowdown: u16,
}

fn main() -> Result<()> {
    pretty_env_logger::init();

    let Opts {
        carla_address,
        carla_port,
        tick,
        slowdown,
    } = Opts::parse();

    // Flag for graceful shutdown when Ctrl-C is pressed
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })
    .expect("Failed to set Ctrl-C handler");

    log::info!("Running Carla Autoware ROS 2 bridge...");

    // Initialize ROS 2 context and executor
    let ctx = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?;
    let executor = ctx.create_basic_executor();
    let node = executor.create_node("autoware_carla_bridge")?;

    // Carla
    let client = Client::connect(&carla_address, carla_port, None);
    let mut world = client.world();
    // Carla settings (synchronous)
    let mut carla_settings = world.settings();
    carla_settings.synchronous_mode = true; // Need to tick by ourselves
    carla_settings.fixed_delta_seconds = Some(tick as f64 * 0.001); // Interval between ticks
    world.apply_settings(&carla_settings, Duration::from_millis(1000));

    // Create bridge list
    let mut bridge_list: HashMap<ActorId, Box<dyn ActorBridge>> = HashMap::new();

    // Create clock publisher
    // Node is Arc so we can clone it
    let simulator_clock =
        SimulatorClock::new(node.clone()).expect("Unable to create simulator clock!");

    // Create thread for ticking
    let client_for_tick = Client::connect(&carla_address, carla_port, None);
    thread::spawn(move || loop {
        let mut world = client_for_tick.world();
        world.tick();
        let sec = world.snapshot().timestamp().elapsed_seconds;
        simulator_clock
            .publish_clock(Some(sec))
            .expect("Unable to publish clock");
        thread::sleep(Duration::from_millis(tick * slowdown as u64));
    });

    let mut autoware_list: HashMap<String, autoware::Autoware> = HashMap::new();

    // Track consecutive timeouts to detect CARLA connection issues
    let mut consecutive_timeouts = 0;
    const MAX_CONSECUTIVE_TIMEOUTS: u32 = 5;

    // === Main loop ===
    // Keep running until Ctrl-C is pressed
    while running.load(Ordering::SeqCst) {
        {
            let mut actor_list: HashMap<ActorId, _> = world
                .actors()
                .iter()
                .map(|actor| (actor.id(), actor))
                .collect();
            let prev_actor_ids: HashSet<u32> = bridge_list.keys().cloned().collect();
            let cur_actor_ids: HashSet<u32> = actor_list.keys().cloned().collect();
            let added_ids = &cur_actor_ids - &prev_actor_ids;
            let deleted_ids = &prev_actor_ids - &cur_actor_ids;

            for id in added_ids {
                let actor = actor_list.remove(&id).expect("ID should be in the list!");
                let bridge = match bridge::actor_bridge::get_bridge_type(actor.clone()) {
                    Ok(BridgeType::Vehicle(vehicle_name)) => {
                        let aw = autoware_list
                            .entry(vehicle_name.clone())
                            .or_insert_with(|| autoware::Autoware::new(vehicle_name.clone()));
                        bridge::actor_bridge::create_bridge(
                            node.clone(), // Node is Arc, cheap to clone
                            actor,
                            BridgeType::Vehicle(vehicle_name),
                            aw,
                        )?
                    }
                    Ok(BridgeType::Sensor(vehicle_name, sensor_type, sensor_name)) => {
                        let aw = autoware_list
                            .entry(vehicle_name.clone())
                            .or_insert_with(|| autoware::Autoware::new(vehicle_name.clone()));
                        aw.add_sensors(sensor_type, sensor_name.clone());
                        bridge::actor_bridge::create_bridge(
                            node.clone(), // Node is Arc, cheap to clone
                            actor,
                            BridgeType::Sensor(vehicle_name, sensor_type, sensor_name),
                            aw,
                        )?
                    }
                    Ok(_) => {
                        log::debug!("Ignore type which are not vehicle and sensor.");
                        continue;
                    }
                    Err(BridgeError::OwnerlessSensor { sensor_id }) => {
                        log::debug!(
                            "Ignore the sensor with ID {sensor_id} is not attached to any vehicle."
                        );
                        continue;
                    }
                    Err(BridgeError::Npc { npc_role_name }) => {
                        log::debug!("Ignore NPC vehicle {npc_role_name}.");
                        continue;
                    }
                    Err(err) => {
                        log::error!("Unexpected error: {err:?}");
                        return Err(err);
                    }
                };
                bridge_list.insert(id, bridge);
                log::info!("Actor {id} created");
            }

            let mut is_actor_removed = false;
            for id in deleted_ids {
                bridge_list.remove(&id).expect("ID should be in the list!");
                log::info!("Actor {id} deleted");
                is_actor_removed = true;
            }
            // If there is actors removed, reget all the actor's list. This can avoid getting non-existed vehicles.
            if is_actor_removed {
                continue;
            }
        }

        let sec = world.snapshot().timestamp().elapsed_seconds;
        bridge_list
            .values_mut()
            .try_for_each(|bridge| bridge.step(sec))?;

        // Wait for next CARLA tick, checking for timeout errors
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
            }
        }
    }

    log::info!("Bridge shutting down gracefully");
    Ok(())
}
