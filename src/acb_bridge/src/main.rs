mod autoware;
mod autoware_detection;
mod bridge;
mod carla_vehicle;
mod clock;
mod coordinate_conversion;
mod error;
mod ground_truth_objects;
mod longitudinal_map;
mod sensor_config;
mod sensor_release;
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

use carla::client::{ActorBase, Client};
use carla_vehicle::CarlaVehicle;
use clock::SimulatorClock;
use error::Result;
use rclrs::CreateBasicExecutor;

/// Map a configured sensor type to the publisher that handles it.
///
/// `None` means "nothing publishes this", with the reason logged once per sensor. Both
/// the registration pass and the bridge-creation pass go through here so they cannot
/// disagree about what is supported.
fn bridge_sensor_type(
    link_name: &str,
    sensor_type: sensor_config::SensorType,
) -> Option<bridge::sensor_bridge::SensorType> {
    use sensor_config::SensorType as ConfigType;

    match sensor_type {
        ConfigType::Camera => Some(bridge::sensor_bridge::SensorType::CameraRgb),
        ConfigType::Lidar => Some(bridge::sensor_bridge::SensorType::LidarRayCast),
        ConfigType::SemanticLidar => Some(bridge::sensor_bridge::SensorType::LidarRayCastSemantic),
        ConfigType::Imu => Some(bridge::sensor_bridge::SensorType::Imu),
        ConfigType::Gnss => Some(bridge::sensor_bridge::SensorType::Gnss),
        ConfigType::Radar => {
            tracing::warn!(
                "Radar sensor '{link_name}' is not supported yet; it will publish nothing"
            );
            None
        }
        ConfigType::Unsupported => {
            tracing::warn!(
                "Sensor '{link_name}' has a blueprint this bridge does not recognise; it will \
                 publish nothing. Check its `blueprint` in vehicle_config.yaml"
            );
            None
        }
    }
}

/// Create sensor bridges for all sensors spawned by CarlaVehicle
///
/// This iterates over the sensor types (derived from VehicleConfig blueprints)
/// and creates a SensorBridge for each sensor that was spawned in CARLA.
/// The bridges handle CARLA sensor callbacks and publish data to ROS topics.
fn create_sensor_bridges(
    node: rclrs::Node,
    carla_vehicle: &CarlaVehicle,
    autoware: &autoware::Autoware,
    clock_offset: utils::SimClockOffset,
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
        let Some(mapped_type) = bridge_sensor_type(link_name, *sensor_type) else {
            continue;
        };

        // Create bridge type from sensor type
        let bridge_type = BridgeType::Sensor(mapped_type, link_name.clone());

        // Create sensor bridge
        match SensorBridge::new(node.clone(), sensor, bridge_type, autoware, clock_offset.clone()) {
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
    pub vehicle_name: String,
    pub vehicle_config: String,
    /// Publish pose directly to /localization/kinematic_state (bypasses Autoware localization)
    /// Set to true for testing without Autoware localization pipeline
    pub publish_direct_localization: bool,
    /// Publish `/clock` from CARLA simulation time.
    ///
    /// Must be `false` in the scenario ego's ROS domain, where SSv2's `traffic_simulator`
    /// already owns `/clock` — two publishers make every localization node log
    /// "Detected jump back in time. Clearing TF buffer". Stays `true` in background-AV
    /// domains, which have no SSv2 and would otherwise have no simulation clock at all.
    pub publish_clock: bool,
    /// Scenario runner's release-notice PUB socket, or empty to disable the channel.
    ///
    /// The runner despawns the vehicle we attached sensors to, destroying them while we
    /// are still listening. It tells us first over this channel so we can stop them; see
    /// `sensor_release` and docs/issues/015.
    pub release_notify_endpoint: String,
    /// Where to send the acknowledgement (the runner's PULL socket).
    pub release_ack_endpoint: String,
    /// Report the measured front-wheel angle on `/vehicle/status/steering_status`.
    ///
    /// **Defaults to false**, which echoes the command. Measured is the honest answer and
    /// was the default until an A/B measurement showed it destabilises Autoware's lateral
    /// controller here: the command maps to the wheel *limit* while the vehicle turns at
    /// the Ackermann *mean*, so honest feedback sits ~18% below the command and MPC winds
    /// up against it. Turn this on once the command mapping is calibrated. See
    /// docs/issues/009 and 006.
    pub report_measured_steering: bool,
    pub steering_multiplier: f64,
    pub publish_ground_truth_objects: bool,
    pub ground_truth_range_m: f64,
    pub honor_emergency_cmd: bool,
    pub accel_map_path: String,
    pub brake_map_path: String,
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

        let vehicle_name = node
            .declare_parameter("vehicle_name")
            .default("hero".into())
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let vehicle_config = node
            .declare_parameter("vehicle_config")
            .default("".into())
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let publish_direct_localization = node
            .declare_parameter("publish_direct_localization")
            .default(false)
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let publish_clock = node
            .declare_parameter("publish_clock")
            .default(true)
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let release_notify_endpoint = node
            .declare_parameter("release_notify_endpoint")
            .default("tcp://127.0.0.1:5556".into())
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let release_ack_endpoint = node
            .declare_parameter("release_ack_endpoint")
            .default("tcp://127.0.0.1:5557".into())
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let report_measured_steering = node
            .declare_parameter("report_measured_steering")
            .default(false)
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let steering_multiplier = node
            .declare_parameter("steering_multiplier")
            .default(1.0)
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let publish_ground_truth_objects = node
            .declare_parameter("publish_ground_truth_objects")
            .default(false)
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        // Off by default: measured on this stack, the flag is true in 29% of samples while
        // driving while Autoware commands acceleration, so acting on it brakes against a
        // normal control command. See docs/issues/021.
        let honor_emergency_cmd = node
            .declare_parameter("honor_emergency_cmd")
            .default(false)
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let ground_truth_range_m = node
            .declare_parameter("ground_truth_range_m")
            .default(100.0)
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        // Measured pedal response, in the shape Autoware's raw_vehicle_cmd_converter uses.
        // Empty means fall back to the single-constant conversion this replaced.
        let accel_map_path = node
            .declare_parameter("accel_map_path")
            .default(Arc::from(""))
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        let brake_map_path = node
            .declare_parameter("brake_map_path")
            .default(Arc::from(""))
            .mandatory()
            .map_err(|e| BridgeError::Rclrs(e.into()))?;

        // Get parameter values
        let carla_address_val: Arc<str> = carla_address.get();
        let carla_port_val: i64 = carla_port.get();
        let vehicle_name_val: Arc<str> = vehicle_name.get();
        let vehicle_config_val: Arc<str> = vehicle_config.get();
        let publish_direct_localization_val: bool = publish_direct_localization.get();
        let publish_clock_val: bool = publish_clock.get();
        let release_notify_val: Arc<str> = release_notify_endpoint.get();
        let release_ack_val: Arc<str> = release_ack_endpoint.get();
        let report_measured_steering_val: bool = report_measured_steering.get();
        let steering_multiplier_val: f64 = steering_multiplier.get();
        let publish_ground_truth_objects_val: bool = publish_ground_truth_objects.get();
        let ground_truth_range_m_val: f64 = ground_truth_range_m.get();
        let honor_emergency_cmd_val: bool = honor_emergency_cmd.get();
        let accel_map_path_val: Arc<str> = accel_map_path.get();
        let brake_map_path_val: Arc<str> = brake_map_path.get();

        // Validate required parameters
        if vehicle_config_val.is_empty() {
            return Err(BridgeError::ConfigError(
                "vehicle_config parameter is required but not set".into(),
            ));
        }

        Ok(Self {
            carla_address: carla_address_val.to_string(),
            carla_port: carla_port_val as u16,
            vehicle_name: vehicle_name_val.to_string(),
            vehicle_config: vehicle_config_val.to_string(),
            publish_direct_localization: publish_direct_localization_val,
            publish_clock: publish_clock_val,
            release_notify_endpoint: release_notify_val.to_string(),
            release_ack_endpoint: release_ack_val.to_string(),
            report_measured_steering: report_measured_steering_val,
            steering_multiplier: steering_multiplier_val,
            publish_ground_truth_objects: publish_ground_truth_objects_val,
            ground_truth_range_m: ground_truth_range_m_val,
            honor_emergency_cmd: honor_emergency_cmd_val,
            accel_map_path: accel_map_path_val.to_string(),
            brake_map_path: brake_map_path_val.to_string(),
        })
    }
}

/// Connect to CARLA with infinite retry loop.
///
/// Returns a connected Client with timeout configured. Checks the `running` flag
/// between attempts to allow graceful shutdown via Ctrl-C.
fn connect_to_carla(params: &BridgeParams, running: &AtomicBool) -> Option<Client> {
    tracing::info!(
        "Connecting to CARLA at {}:{}...",
        params.carla_address,
        params.carla_port
    );

    loop {
        match Client::connect(&params.carla_address, params.carla_port, None) {
            Ok(mut client) => {
                if let Err(e) = client.set_timeout(Duration::from_secs(30)) {
                    tracing::warn!("Failed to set timeout: {e}, retrying in 5 seconds...");
                    std::thread::sleep(Duration::from_secs(5));
                    continue;
                }
                match client.world() {
                    Ok(_) => {
                        tracing::info!("Connected to CARLA successfully");
                        return Some(client);
                    }
                    Err(e) => {
                        tracing::warn!("CARLA not ready: {e}, retrying in 5 seconds...");
                    }
                }
            }
            Err(e) => {
                tracing::warn!("Failed to connect to CARLA: {e}, retrying in 5 seconds...");
            }
        }

        if !running.load(Ordering::SeqCst) {
            tracing::info!("Shutdown requested while connecting to CARLA");
            return None;
        }

        std::thread::sleep(Duration::from_secs(5));
    }
}

/// What a failed tick wait means.
///
/// A timeout is **not** evidence that CARLA is gone. Whoever owns the tick -- SSv2 through
/// `carla_scenario_bridge` -- pauses between frames routinely: Autoware initialisation alone
/// runs to `initialize_duration`, 120 s by default, during which no frame is sent at all.
/// Treating those quiet periods as disconnection tears down a perfectly healthy bridge.
///
/// Only a genuine transport failure counts. See `docs/roadmap/011-robustness.md` (gap 8).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TickOutcome {
    /// No tick arrived in time. Normal while the simulation is paused.
    Idle,
    /// The connection to CARLA looks gone.
    ConnectionLost,
}

/// Why the main loop stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SessionExit {
    /// Ctrl-C. Tear down and exit.
    Shutdown,
    /// CARLA is gone. Reconnect, then rebuild everything.
    CarlaLost,
    /// Autoware restarted. CARLA is fine; rebuild sensors and bridges against the vehicle
    /// that exists now, without dropping the CARLA connection.
    AutowareRestarted,
    /// The vehicle was destroyed (a scenario runner despawned it between runs). CARLA and
    /// Autoware are fine; destroy the orphaned sensors and wait for the next spawn.
    VehicleLost,
}

/// Classify a CARLA error from a tick wait.
fn classify_tick_error(error: &carla::CarlaError) -> TickOutcome {
    use carla::{CarlaError, ConnectionError};

    match error {
        // The wait expired with no tick. Says nothing about the connection.
        CarlaError::Connection(ConnectionError::Timeout { .. }) => TickOutcome::Idle,
        // Anything else on this path -- an explicit disconnect, an FFI failure, a
        // resource error -- means the world handle is no longer usable.
        _ => TickOutcome::ConnectionLost,
    }
}

/// Ask the *server* whether an actor still exists, over RPC.
///
/// Unlike a frame snapshot this works while the simulation is paused, and unlike
/// `World::actor` / `Actor::is_alive` it cannot answer from the client's own registry:
/// `World::actors()` is a fresh `get_actors` call. Returns `true` on a transport error,
/// so a hiccup is never mistaken for a despawn -- the tick path owns connection loss.
fn actor_still_exists(world: &carla::client::World, id: carla::rpc::ActorId) -> bool {
    match world.actors() {
        Ok(actors) => actors.iter().any(|actor| actor.id() == id),
        Err(e) => {
            tracing::debug!("Actor existence check failed: {e}");
            true
        }
    }
}

/// Wait for one CARLA frame.
///
/// `Ok(None)` is the wait expiring with no frame -- normal while the simulation is
/// paused. `Ok(Some(snapshot))` is a frame that genuinely arrived.
///
/// **Read the world's state from the returned snapshot, never from `World::snapshot()`.**
/// `World::snapshot()` answers from the client's cached episode state, so once frames
/// stop arriving it keeps returning the last one it saw, forever, with every actor in it
/// still "present". This function used to throw the return value away
/// (`let _ = world.wait_for_tick_or_timeout(timeout)?;`) and then call `World::snapshot()`
/// -- so a paused simulation was indistinguishable from a running one, and a vehicle
/// destroyed after the last frame stayed alive in the bridge's eyes indefinitely. See
/// `docs/issues/014-*`.
///
/// `Err` does not necessarily mean CARLA is gone; classify it with [`classify_tick_error`].
fn carla_tick(
    world: &carla::client::World,
    timeout: Duration,
) -> std::result::Result<Option<carla::client::WorldSnapshot>, carla::CarlaError> {
    world.wait_for_tick_or_timeout(timeout)
}

/// Poll CARLA actors until a vehicle with the given role_name is found.
///
/// Returns `Some(vehicle)` when found, or `None` if Ctrl-C is received.
/// Logs progress every 5 seconds.
/// How a wait for the scenario's vehicle ended.
enum WaitOutcome {
    /// Vehicle found, with the world (episode) it lives in.
    Found(carla::client::World, carla::client::Vehicle),
    /// Ctrl-C.
    Shutdown,
    /// Nothing found for a whole reconnect interval. The server may have been
    /// restarted underneath this client (0.9.16 crashes on its sensor-stream
    /// teardown race routinely); a stale client can keep answering from the dead
    /// session without erroring, so the caller should reconnect and retry.
    Stale,
}

/// Reconnect after this much fruitless waiting. Between scenarios a long empty wait
/// is normal, so this fires repeatedly there — reconnecting with nothing attached
/// is free, and it is the only way to notice a silently replaced server.
const WAIT_RECONNECT_INTERVAL: Duration = Duration::from_secs(60);

fn wait_for_vehicle(client: &Client, vehicle_name: &str, running: &AtomicBool) -> WaitOutcome {
    use carla::client::ActorBase;
    tracing::info!("Waiting for vehicle (role_name={vehicle_name}) in CARLA...");
    let start = std::time::Instant::now();
    let mut last_log = std::time::Instant::now();
    loop {
        if !running.load(Ordering::SeqCst) {
            return WaitOutcome::Shutdown;
        }
        if start.elapsed() > WAIT_RECONNECT_INTERVAL {
            tracing::info!(
                "No vehicle after {}s; refreshing the CARLA connection in case the \
                 server was replaced",
                WAIT_RECONNECT_INTERVAL.as_secs()
            );
            return WaitOutcome::Stale;
        }
        // Re-fetch the world every poll: a scenario runner may load a different map
        // between spawns, which starts a NEW episode. A world handle from before the
        // reload keeps answering from the dead episode, so the new vehicle is never
        // seen and the bridge waits forever with the stack starving behind it.
        let world = match client.world() {
            Ok(w) => w,
            Err(e) => {
                tracing::warn!("Failed to get world while waiting for vehicle: {e}");
                std::thread::sleep(Duration::from_secs(1));
                continue;
            }
        };
        match world.actors() {
            Ok(actors) => match actors.filter("vehicle.*") {
                Ok(vehicles) => {
                    for actor in vehicles.iter() {
                        if let Ok(attrs) = actor.attributes() {
                            let matches = attrs
                                .iter()
                                .any(|a| a.id() == "role_name" && a.value_string() == vehicle_name);
                            if matches {
                                tracing::info!(
                                    "Found vehicle: ID={} type={} role_name={}",
                                    actor.id(),
                                    actor.type_id(),
                                    vehicle_name,
                                );
                                if let carla::client::ActorKind::Vehicle(v) = actor.into_kinds() {
                                    return WaitOutcome::Found(world, v);
                                }
                            }
                        }
                    }
                }
                Err(e) => tracing::warn!("Failed to filter actors: {e}"),
            },
            Err(e) => tracing::warn!("Failed to get actors: {e}"),
        }
        if last_log.elapsed() >= Duration::from_secs(5) {
            tracing::info!(
                "Still waiting for vehicle (role_name={vehicle_name})... ({:.0}s elapsed)",
                start.elapsed().as_secs_f32()
            );
            last_log = std::time::Instant::now();
        }
        std::thread::sleep(Duration::from_secs(1));
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
    let node = executor.create_node("acb_bridge")?;
    tracing::info!("ROS 2 node created: acb_bridge");

    // === Step 2: Read parameters from ROS node ===
    let params = BridgeParams::from_node(&node)?;
    tracing::info!("=== Autoware-CARLA Bridge (Autoware-centric) ===");
    if params.publish_direct_localization {
        // Publishing ground truth to /localization/kinematic_state and /tf puts this bridge
        // in direct competition with Autoware's EKF, which owns both. Useful for bringing a
        // stack up without localization; wrong for anything measuring localization.
        tracing::warn!(
            "publish_direct_localization is ENABLED: this bridge will publish \
             /localization/kinematic_state and /tf from CARLA ground truth, competing with \
             Autoware's own localization. Use it only when Autoware's localization is \
             disabled, and never when measuring localization accuracy."
        );
    } else {
        tracing::info!("publish_direct_localization: false (Autoware owns localization)");
    }
    if params.publish_clock {
        tracing::info!("publish_clock: true (this bridge owns /clock in its ROS domain)");
    } else {
        tracing::info!("publish_clock: false (something else owns /clock, e.g. SSv2)");
    }

    // Create clock publisher (persists across reconnections)
    let simulator_clock = SimulatorClock::new(node.clone())?;

    // CARLA reports server uptime, not scenario time. Reset per CARLA connection: a
    // restarted server rewinds elapsed_seconds.
    let mut clock_epoch = utils::ClockEpoch::new();

    // === Step 3: Create Autoware coordinator and wait for Autoware ===
    tracing::info!("Creating Autoware coordinator...");
    let mut autoware = autoware::Autoware::new(node.clone(), params.publish_direct_localization)?;

    tracing::info!("Waiting for Autoware to start...");
    tracing::info!("(Start Autoware planning simulator with sample_sensor_kit)");
    tracing::info!("Listening for /robot_description topic...");

    // Wait for Autoware indefinitely (user controls timeout via Ctrl-C)
    let start_time = std::time::Instant::now();
    let mut last_log_time = std::time::Instant::now();
    let mut attempt_count = 0u32;

    loop {
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

    // === Step 4: Wait for TF transforms to be fully received ===
    tracing::info!("Waiting for TF transforms to be received...");
    let tf_start_time = std::time::Instant::now();
    let tf_timeout = Duration::from_secs(10);
    let min_transforms = 4;

    loop {
        executor.spin(rclrs::SpinOptions::spin_once().timeout(Duration::from_millis(100)));

        let tf_count = autoware.get_tf_buffer().get_all_frames().len();
        if tf_count >= min_transforms {
            tracing::info!("TF transforms ready: {} frames available", tf_count);
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
            break;
        }

        if !running.load(Ordering::SeqCst) {
            tracing::info!("Shutdown requested while waiting for TF");
            return Ok(());
        }

        if tf_start_time.elapsed().as_secs().is_multiple_of(2) {
            tracing::debug!(
                "Waiting for TF... {} transforms (need {})",
                tf_count,
                min_transforms
            );
        }
    }

    // === Step 5: Load vehicle configuration (sensor definitions) ===
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

    // ========================================================================
    // CARLA connection loop: connect → spawn → run → reconnect on disconnect
    // ========================================================================
    // False when only Autoware restarted: CARLA is still healthy, so its connection and
    // simulation clock must be preserved.
    let mut reconnect_carla = true;
    // Whether the coming reconnect is because CARLA went away. A reconnect made purely to
    // drop stale sensor streams (see SessionExit::VehicleLost) talks to the same server,
    // whose uptime never paused -- rebasing the clock there would rewind `/clock` under a
    // background AV that owns it, which is the "Detected jump back in time" failure.
    let mut carla_may_have_restarted = true;
    let mut client: Option<Client> = None;

    loop {
        // === Connect to CARLA ===
        if reconnect_carla || client.is_none() {
            client = match connect_to_carla(&params, &running) {
                Some(c) => Some(c),
                None => return Ok(()), // Ctrl-C during connection
            };

            // A reconnected (possibly restarted) server may have rewound its uptime.
            if carla_may_have_restarted {
                clock_epoch.reset();
            }
        }
        let client = client.as_ref().expect("client connected above");

        // === Wait for vehicle then attach sensors ===
        tracing::info!(
            "Waiting for vehicle '{}' (spawned by scenario script)...",
            params.vehicle_name
        );
        // The world handle comes back WITH the vehicle: wait_for_vehicle re-fetches it
        // every poll, so a map reload between spawns (new episode) is picked up and the
        // sensors attach into the episode the vehicle actually lives in.
        let (mut world, hero_vehicle) =
            match wait_for_vehicle(client, &params.vehicle_name, &running) {
                WaitOutcome::Found(w, v) => (w, v),
                WaitOutcome::Shutdown => return Ok(()),
                WaitOutcome::Stale => {
                    reconnect_carla = true;
                    continue;
                }
            };

        tracing::info!("Attaching sensors to hero vehicle...");
        let carla_vehicle = match CarlaVehicle::new(
            &mut world,
            hero_vehicle,
            &vehicle_config,
            autoware.get_tf_buffer(),
        ) {
            Ok(v) => v,
            Err(e) => {
                tracing::error!("Failed to attach sensors to vehicle: {e}, reconnecting in 5s...");
                std::thread::sleep(Duration::from_secs(5));
                continue;
            }
        };

        let carla_vehicle = Arc::new(Mutex::new(carla_vehicle));
        autoware.set_vehicle(carla_vehicle.clone());

        let vehicle_guard = carla_vehicle.lock().unwrap();
        let vehicle = vehicle_guard.get_vehicle().clone();
        let vehicle_shared = Arc::new(Mutex::new(Some(vehicle.clone())));
        let vehicle_id = vehicle.id();
        drop(vehicle_guard);

        tracing::info!("Sensors attached to hero vehicle successfully!");

        // === Register sensors with Autoware for topic mapping ===
        tracing::info!("Registering sensors with Autoware...");
        for (link_name, sensor_type) in carla_vehicle.lock().unwrap().get_sensor_types() {
            let Some(mapped_type) = bridge_sensor_type(link_name, *sensor_type) else {
                continue;
            };

            autoware.add_sensors(mapped_type, link_name.clone());
            tracing::info!(
                "  Registered sensor '{}' (type: {:?})",
                link_name,
                sensor_type
            );
        }

        // === Create sensor bridges ===
        //
        // The sensor clock is shared with every bridge and re-anchored each frame by the
        // main loop, so sensor stamps carry the simulation time a measurement was taken
        // rather than whichever `/clock` tick happened to be current when the CARLA
        // callback ran. See utils::SimClockOffset and docs/issues/016.
        let clock_offset = utils::SimClockOffset::new();

        // Optional: report CARLA's actors as perception output instead of deriving them
        // from the LiDAR. Off unless asked for, and only correct with Autoware's own
        // perception disabled -- see ground_truth_objects.
        let ground_truth_objects = if params.publish_ground_truth_objects {
            match ground_truth_objects::GroundTruthObjectPublisher::new(
                node.clone(),
                params.vehicle_name.to_string(),
                params.ground_truth_range_m,
            ) {
                Ok(p) => Some(p),
                Err(e) => {
                    tracing::error!("Ground-truth object publisher failed to start: {e}");
                    None
                }
            }
        } else {
            None
        };

        tracing::info!("Creating sensor bridges...");
        let _sensor_bridges =
            create_sensor_bridges(
                node.clone(),
                &carla_vehicle.lock().unwrap(),
                &autoware,
                clock_offset.clone(),
            )?;
        tracing::info!("Created {} sensor bridges", _sensor_bridges.len());

        // === Listen for release requests from the scenario runner ===
        // It owns the vehicle's lifetime and destroys these sensors with it. Being told
        // first is the difference between a clean teardown and this client retrying four
        // dead streams at 2.6 MB/s of server log; see docs/issues/015.
        let _release_listener = if params.release_notify_endpoint.is_empty() {
            tracing::info!("Sensor release channel disabled (no endpoint configured)");
            None
        } else {
            sensor_release::SensorReleaseListener::start(
                &params.release_notify_endpoint,
                &params.release_ack_endpoint,
                carla_vehicle.clone(),
                vehicle_id,
            )
        };

        // === Create vehicle control bridge ===
        tracing::info!("Creating vehicle control bridge...");
        // Measured pedal maps, when both are configured and readable. A missing or broken
        // map is not fatal: the bridge falls back to the constant conversion and says so,
        // because refusing to drive at all would be a worse failure than driving coarsely.
        // "none" as well as empty: a ROS launch argument cannot carry an empty value
        // (`accel_map_path:=` is rejected as malformed), so there has to be a word for off.
        let disabled = |p: &str| p.is_empty() || p.eq_ignore_ascii_case("none");
        let longitudinal = if disabled(&params.accel_map_path) || disabled(&params.brake_map_path) {
            tracing::info!(
                "Longitudinal pedal maps not configured; using the single-constant conversion. \
                 Set accel_map_path and brake_map_path for measured response."
            );
            None
        } else {
            match longitudinal_map::LongitudinalCalibration::load(
                std::path::Path::new(&params.accel_map_path),
                std::path::Path::new(&params.brake_map_path),
            ) {
                Ok(cal) => {
                    tracing::info!(
                        "Longitudinal pedal maps loaded from {} and {}",
                        params.accel_map_path,
                        params.brake_map_path
                    );
                    Some(cal)
                }
                Err(e) => {
                    tracing::error!(
                        "Could not load the longitudinal pedal maps ({e}); falling back to the \
                         single-constant conversion, which is wrong by a factor of two at full \
                         throttle. Fix the maps or clear the parameters to silence this."
                    );
                    None
                }
            }
        };

        let vehicle_control = vehicle_control::VehicleControlBridge::new(
            node.clone(),
            vehicle_shared.clone(),
            params.report_measured_steering,
            params.steering_multiplier as f32,
            longitudinal,
            params.honor_emergency_cmd,
        )?;
        tracing::info!("Vehicle control bridge created");

        tracing::info!("=== Bridge running ===");

        // Reset heartbeat: the spawn retry loop may have taken a long time (e.g., if a
        // previous session left a vehicle actor blocking the spawn point). Without this
        // reset, the health check would immediately fail on the first tick.
        autoware.reset_heartbeat();

        // Main loop timing: 20 Hz, so a 50 ms tick timeout.
        //
        // This value is deliberately not configurable. It was worth a parameter back when a
        // timeout meant "CARLA is gone", because then it set how long a paused simulation
        // could pause before being declared dead -- and 150 ms was far too short, since SSv2
        // routinely pauses for Autoware's initialize_duration of 120 s. Now that a timeout
        // classifies as TickOutcome::Idle and only transport errors count toward the
        // reconnect counter, the timeout no longer decides anything: it only sets how
        // promptly a new frame is noticed, and how often the idle log ticks over.
        //
        // 50 ms is chosen against the frames it waits for. CARLA runs at a 100 ms step in
        // this workspace, so polling twice per step notices a frame without busy-waiting,
        // and the residual latency it can add is half a poll -- 25 ms against a 100 ms
        // frame. A knob here would let someone tune a number that no longer affects
        // robustness, which is how a parameter becomes a trap.
        const LOOP_RATE_HZ: u64 = 20;
        let loop_duration = Duration::from_millis(1000 / LOOP_RATE_HZ);

        // Consecutive *transport* failures. A tick timeout is not one of these -- see
        // TickOutcome.
        let mut consecutive_failures: u32 = 0;
        const MAX_CONSECUTIVE_FAILURES: u32 = 3;

        // Consecutive ticks with no frame. Only used to rate-limit the log; a paused
        // simulation is a normal state that can last minutes.
        let mut idle_ticks: u64 = 0;
        /// Roughly one loop period each, so this is ~2s of silence before the first note.
        const IDLE_TICKS_BEFORE_FIRST_LOG: u64 = 40;
        /// After that, roughly every 30s at a 50ms loop.
        const IDLE_TICK_LOG_INTERVAL: u64 = 600;
        /// How often to ask the server whether the vehicle still exists while no frames
        /// are arriving. ~0.5s at a 50ms loop.
        ///
        /// This interval is the width of the error storm in docs/issues/015: from the
        /// moment the scenario runner destroys our sensors until we notice and drop the
        /// connection, the server logs "Invalid session" at ~2.6 MB/s. Profiling a run
        /// showed exactly that shape -- a burst at each teardown, zero in between -- so
        /// the interval is the whole cost. At 2s it was ~10 MB and ~400k lines per run;
        /// 0.5s quarters it. One `get_actors` RPC per half second while paused is
        /// cheaper than the log it prevents.
        const IDLE_TICKS_BETWEEN_ACTOR_CHECKS: u64 = 10;

        // === Main Loop ===
        let exit_reason = loop {
            if !running.load(Ordering::SeqCst) {
                break SessionExit::Shutdown;
            }

            let loop_start = std::time::Instant::now();

            // Check Autoware health
            autoware.health_check();
            if !autoware.is_alive() {
                tracing::warn!("Autoware connection lost! Cleaning up...");
                if let Err(e) = carla_vehicle.lock().unwrap().cleanup() {
                    tracing::warn!("Cleanup failed: {e}");
                }
                tracing::info!("Cleanup complete. Waiting for Autoware to restart...");

                loop {
                    executor
                        .spin(rclrs::SpinOptions::spin_once().timeout(Duration::from_millis(100)));

                    if autoware.is_alive() {
                        tracing::info!("Autoware reconnected!");
                        break;
                    }
                    if !running.load(Ordering::SeqCst) {
                        tracing::info!("Shutdown requested during Autoware wait");
                        return Ok(());
                    }
                }

                // Rebuild against whatever vehicle exists now. Sensors were destroyed by the
                // cleanup above, and the scenario owns the vehicle, so it may be the same
                // actor or a fresh one -- either way the bridges must be built again from
                // the current world rather than reused.
                tracing::info!("Rebuilding sensors and bridges for the reconnected Autoware");
                break SessionExit::AutowareRestarted;
            }

            // Wait for the next CARLA frame. A frame that did not arrive is `Ok(None)`,
            // not an error -- see carla_tick.
            let idle = |idle_ticks: &mut u64| {
                // The simulation is paused, not broken. Log at a decreasing rate so a
                // 120s Autoware startup does not bury everything else.
                *idle_ticks += 1;
                if *idle_ticks == IDLE_TICKS_BEFORE_FIRST_LOG
                    || (*idle_ticks > IDLE_TICKS_BEFORE_FIRST_LOG
                        && idle_ticks.is_multiple_of(IDLE_TICK_LOG_INTERVAL))
                {
                    tracing::info!(
                        "No CARLA frame for {:.0}s; the simulation is paused (waiting for \
                         the scenario to advance frames)",
                        *idle_ticks as f64 * loop_duration.as_secs_f64()
                    );
                }
            };

            // Re-anchor the sensor clock every frame. `/clock` restarts with each
            // scenario, so an offset learned once would be wrong for every run after the
            // first. See utils::SimClockOffset and docs/issues/016.
            if let Ok(snap) = world.snapshot() {
                clock_offset.observe(&node, snap.timestamp().elapsed_seconds);
            }

            let snapshot = match carla_tick(&world, loop_duration) {
                Ok(Some(snapshot)) => {
                    consecutive_failures = 0;
                    idle_ticks = 0;
                    snapshot
                }
                Ok(None) => {
                    idle(&mut idle_ticks);

                    // A paused simulation sends no frames, but the server still answers
                    // RPCs -- so "no frame" is not the same as "no information". Ask it
                    // directly, every few seconds, whether the vehicle is still there.
                    //
                    // Without this the despawn is not noticed until the *next* run's
                    // first frame, which between scenario runs can be minutes away: the
                    // bridge sits holding sensors for an actor that is long gone, and its
                    // status topics stay silent for reasons no log line explains.
                    //
                    // This is a promptness fix, not a fix for the orphaned-stream error
                    // storm in docs/issues/015. That was the hypothesis; it was measured
                    // and it was wrong -- with the ego despawned and both clients idle,
                    // the server emits nothing at all (0 B/s). The storm grows *during*
                    // runs, not in the gap between them.
                    if idle_ticks.is_multiple_of(IDLE_TICKS_BETWEEN_ACTOR_CHECKS)
                        && !actor_still_exists(&world, vehicle_id)
                    {
                        tracing::info!(
                            "Vehicle '{}' (actor {vehicle_id}) is gone while the simulation \
                             is paused; releasing sensors now rather than waiting for a \
                             frame that may be minutes away",
                            params.vehicle_name
                        );
                        break SessionExit::VehicleLost;
                    }
                    continue;
                }
                Err(e) => match classify_tick_error(&e) {
                    TickOutcome::Idle => {
                        idle(&mut idle_ticks);
                        continue;
                    }
                    TickOutcome::ConnectionLost => {
                        consecutive_failures += 1;
                        if consecutive_failures >= MAX_CONSECUTIVE_FAILURES {
                            tracing::error!(
                                "CARLA disconnected ({consecutive_failures} consecutive \
                                 transport failures, last error: {e}), will reconnect..."
                            );
                            break SessionExit::CarlaLost;
                        }
                        tracing::warn!(
                            "CARLA transport error \
                             ({consecutive_failures}/{MAX_CONSECUTIVE_FAILURES}): {e}, retrying..."
                        );
                        std::thread::sleep(Duration::from_secs(1));
                        continue;
                    }
                },
            };

            // The scenario runner owns the vehicle and despawns it when a scenario ends.
            // Sensors die with the actor, so continuing to run against a destroyed
            // vehicle publishes nothing and permanently wedges the stack (every scenario
            // rerun used to require a full ego-stack restart because of this). Checked
            // only after a successful tick: during a pause the client's episode view is
            // stale, and a transport error already has its own exit path above.
            // Ask the *server* whether the actor still exists, rather than the handle.
            // libcarla's Actor::IsAlive() is a client-side flag that only flips when this
            // client destroys the actor -- and this bridge never destroys the vehicle, the
            // scenario runner does, from its own client. So `is_alive()` answered true
            // forever: the session never ended, the sensor publishers stayed up with no
            // CARLA data behind them, and the ego's Autoware ran on nothing. Its
            // diagnostics showed the shape of it -- localization scan-matching, perception
            // pointcloud rate and planning trajectory rate all ERROR, so
            // /system/operation_mode/availability said autonomous=False and the state
            // machine sat in PLANNING until the scenario timed out.
            // The snapshot the tick returned is the server's actor list for the frame that
            // just arrived. `World::snapshot()` is NOT a substitute -- it answers from the
            // client's cached episode state, so a paused simulation keeps handing back the
            // last frame with every actor still in it. Neither is `World::GetActor` nor
            // `Actor::IsAlive`: both can answer from the client's own registry and keep
            // reporting an actor this client remembers but the server destroyed.
            if !snapshot.contains(vehicle_id) {
                tracing::info!(
                    "Vehicle '{}' (actor {vehicle_id}) is gone from the world; releasing \
                     sensors and waiting for the next spawn",
                    params.vehicle_name
                );
                break SessionExit::VehicleLost;
            }

            let sec = snapshot.timestamp().elapsed_seconds;

            // Publish clock, but only if we own it in this domain. `sec` is CARLA server
            // uptime, so it is rebased onto scenario time before publishing.
            if params.publish_clock {
                let sim_sec = clock_epoch.to_sim_time(sec);
                if let Err(e) = simulator_clock.publish_clock(Some(sim_sec)) {
                    tracing::warn!("Failed to publish clock: {e}");
                }
            }

            // Spin executor to process ROS callbacks (subscriptions). This is also what
            // feeds /clock into the node's ROS clock, so it must happen before anything
            // reads a timestamp below.
            executor.spin(rclrs::SpinOptions::spin_once().timeout(Duration::from_millis(10)));

            // Every published stamp comes from the node's ROS clock, never from CARLA's
            // own timestamps -- see utils::ros_time_now.
            let stamp_sec = utils::ros_time_now_secs(&node);

            // Main tick: ground truth publishing
            if let Err(e) = autoware.tick(stamp_sec) {
                tracing::warn!("Autoware tick failed: {}", e);
            }

            if let Some(ref gt) = ground_truth_objects {
                if let Err(e) = gt.publish(&world) {
                    tracing::warn!("Failed to publish ground-truth objects: {e}");
                }
            }

            // Publish vehicle status (velocity, steering, control mode)
            if let Err(e) = vehicle_control.publish_status(stamp_sec) {
                tracing::warn!("Failed to publish vehicle status: {e}");
            }

            // Rate limiting
            let next_iteration = loop_start + loop_duration;
            let now = std::time::Instant::now();
            if next_iteration > now {
                std::thread::sleep(next_iteration - now);
            }
        };

        // Clean up old vehicle before reconnecting or exiting
        tracing::info!("Cleaning up CARLA actors...");
        // Best-effort cleanup: CARLA may already be gone
        if let Err(e) = carla_vehicle.lock().unwrap().cleanup() {
            tracing::warn!("Cleanup failed (CARLA may be gone): {e}");
        }

        // Clear vehicle reference so stale pointers aren't used
        vehicle_shared.lock().unwrap().take();

        // And drop the coordinator's copy. `Autoware` outlives this loop, so its
        // `Arc<Mutex<CarlaVehicle>>` -- which holds a `Vehicle`, hence the episode, hence
        // the *client* -- would otherwise keep the old connection alive across a
        // reconnect, streaming sessions and all. See docs/issues/015.
        autoware.clear_vehicle();

        match exit_reason {
            SessionExit::Shutdown => {
                tracing::info!("Bridge shutdown complete");
                return Ok(());
            }
            SessionExit::CarlaLost => {
                tracing::info!("Attempting to reconnect to CARLA...");
                reconnect_carla = true;
                carla_may_have_restarted = true;
            }
            SessionExit::AutowareRestarted => {
                // CARLA is healthy, so the connection and the simulation clock are kept.
                // Resetting the clock epoch here would rewind /clock under a simulation
                // that never paused.
                tracing::info!("Rebuilding for the restarted Autoware; CARLA connection kept");
                reconnect_carla = false;
            }
            SessionExit::VehicleLost => {
                // Reconnect, even though CARLA is perfectly healthy.
                //
                // The scenario runner destroys the ego *and its children*, so this
                // bridge's sensors are destroyed underneath it while it is still
                // listening to their streams. There is no way to unsubscribe afterwards:
                // `Sensor::stop()` on a destroyed actor fails ("close: Bad file
                // descriptor"), and the listening client then retries the dead stream
                // forever. Measured against a plain two-client reproducer, the server
                // answers those retries at ~48,000 "Invalid session: no stream available
                // with id N" per second -- 2.6 MB/s of log -- until the server
                // segfaults. Killing the listening client drops the rate to zero
                // instantly; nothing else does.
                //
                // Dropping the connection is the only lever this side owns. It was
                // already the accidental cure: wait_for_vehicle reconnects after
                // WAIT_RECONNECT_INTERVAL, so a long idle gap self-healed after 60 s
                // while back-to-back scenario runs never got that far. Do it at once
                // instead. Reconnecting with nothing attached costs a few hundred ms.
                //
                // The real fix is ordering -- the listener must stop() before the owner
                // destroys -- and that needs the two processes to agree. See
                // docs/issues/015.
                tracing::info!(
                    "Waiting for the next vehicle spawn; reconnecting to CARLA to drop \
                     the sensor streams the scenario runner destroyed underneath us"
                );
                reconnect_carla = true;
                carla_may_have_restarted = false;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use carla::{CarlaError, ConnectionError};

    fn timeout_error() -> CarlaError {
        CarlaError::Connection(ConnectionError::Timeout {
            operation: "wait_for_tick".to_string(),
            duration: Duration::from_millis(50),
            source: None,
        })
    }

    fn disconnected_error() -> CarlaError {
        CarlaError::Connection(ConnectionError::Disconnected {
            reason: "socket closed".to_string(),
            source: None,
        })
    }

    /// Regression guard for gap 8. Whoever owns the tick pauses between frames routinely --
    /// Autoware startup alone can run to 120s with no frame sent. Treating that as
    /// disconnection tore down a healthy bridge.
    #[test]
    fn a_tick_timeout_is_not_a_disconnect() {
        assert_eq!(classify_tick_error(&timeout_error()), TickOutcome::Idle);
    }

    #[test]
    fn a_lost_connection_is_a_disconnect() {
        assert_eq!(
            classify_tick_error(&disconnected_error()),
            TickOutcome::ConnectionLost
        );
    }

    /// Anything that is not an explicit timeout leaves the world handle unusable, so it must
    /// count toward reconnection rather than being mistaken for an idle simulation.
    #[test]
    fn other_errors_count_as_connection_loss() {
        let internal = CarlaError::Internal(carla::InternalError::FfiError {
            message: "snapshot failed".to_string(),
            source: None,
        });
        assert_eq!(classify_tick_error(&internal), TickOutcome::ConnectionLost);
    }
}
