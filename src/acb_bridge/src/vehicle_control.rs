/// Vehicle control integration for Autoware-CARLA bridge
///
/// This module owns the whole vehicle interface: every `/control/command/*` topic the
/// bridge honours and every `/vehicle/status/*` topic it reports on. Nothing else in the
/// crate may publish to those topics -- two publishers on one topic inside one process is
/// the `/clock` regression all over again (see `docs/roadmap/011-robustness.md`).
///
/// This module handles:
/// - Subscribing to Autoware control, gear, turn-indicator and hazard-light commands
/// - Applying them to the CARLA vehicle, including its light state
/// - Publishing vehicle status back to Autoware
use crate::error::Result;
use carla::{
    client::{ActorBase, Vehicle},
    rpc::{VehicleControl, VehicleLightState, VehicleWheelLocation},
};
use rclrs::IntoPrimitiveOptions;
use std::sync::{Arc, Mutex};

/// Fall back maximum steering tire angle in radians (~70 degrees).
///
/// Only used when CARLA will not answer `physics_control` -- the real value comes from
/// the wheels of the vehicle that actually spawned. See `docs/issues/006-*`.
const FALLBACK_MAX_STEER_ANGLE: f32 = 1.22;

/// Maximum expected acceleration magnitude for throttle/brake mapping (m/s²)
const MAX_ACCEL: f32 = 3.0;

/// How CARLA turns a normalized steer command into a physical tire angle.
///
/// Read from the spawned vehicle's own wheels, so it follows the blueprint rather than
/// assuming the Tesla. See `docs/issues/006-*`.
#[derive(Debug, Clone, Copy)]
struct SteerGeometry {
    /// The steered wheels' physical limit, radians.
    max_steer_angle: f32,
    /// track / wheelbase -- the Ackermann differential term. Zero reduces the model to a
    /// plain bicycle, which is the behaviour this replaced.
    track_over_wheelbase: f32,
}

impl Default for SteerGeometry {
    fn default() -> Self {
        Self {
            max_steer_angle: FALLBACK_MAX_STEER_ANGLE,
            track_over_wheelbase: 0.0,
        }
    }
}

/// The bicycle-model tire angle CARLA actually delivers for a normalized steer command.
///
/// CARLA drives the *inner* wheel to `cmd * max_steer_angle` and places the outer wheel by
/// Ackermann geometry, `cot(outer) = cot(inner) + track / wheelbase`. The angle the vehicle
/// turns at is the mean of the two, which is well below the wheel limit: for the Tesla at
/// full lock the wheels sit at 70 and 47.4 degrees, a 58.7 degree mean.
///
/// Verified against CARLA 0.9.16 by `scripts/probe_carla_conventions.py`, which reads the
/// physical wheel angles back over the whole command range. This model reproduces them to
/// better than 0.05 degrees.
fn effective_tire_angle(steer_cmd: f32, geometry: &SteerGeometry) -> f32 {
    let inner = steer_cmd.abs().clamp(0.0, 1.0) * geometry.max_steer_angle;
    if inner <= f32::EPSILON {
        return 0.0;
    }
    let outer = (inner.tan().recip() + geometry.track_over_wheelbase)
        .recip()
        .atan();
    (0.5 * (inner + outer)).copysign(steer_cmd)
}

/// Reject a steering trim that would disable or invert steering.
///
/// A zero or negative multiplier is always a configuration mistake rather than a tuning
/// choice: zero means the vehicle cannot steer at all, and negative means it steers the
/// wrong way, both of which present as a control problem a long way from the config file.
fn sane_steering_multiplier(value: f32) -> f32 {
    if value.is_finite() && value > 0.0 {
        return value;
    }
    tracing::warn!(
        "steering_multiplier {value} is not a positive number; using 1.0. Zero would stop \
         the vehicle steering and a negative value would invert it."
    );
    1.0
}

/// The normalized steer command that delivers `tire_angle`, in CARLA's sign convention.
///
/// Dividing by the wheel limit -- what this replaced -- asks for the angle the *inner*
/// wheel would reach and therefore under-delivers by 7-13% across Autoware's 0.70 rad
/// planning range. `effective_tire_angle` is monotonic in the command, so invert it by
/// bisection; 30 iterations resolve it far below the resolution of the physics.
fn steer_command_for(tire_angle: f32, geometry: &SteerGeometry) -> f32 {
    let target = tire_angle.abs();
    if target <= f32::EPSILON {
        return 0.0;
    }
    if target >= effective_tire_angle(1.0, geometry) {
        return 1.0_f32.copysign(tire_angle);
    }
    let (mut lo, mut hi) = (0.0_f32, 1.0_f32);
    for _ in 0..30 {
        let mid = 0.5 * (lo + hi);
        if effective_tire_angle(mid, geometry) < target {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    (0.5 * (lo + hi)).copysign(tire_angle)
}

/// What the bridge last applied to CARLA, and therefore what it reports to Autoware.
///
/// Autoware compares commanded against reported state (`vehicle_cmd_gate` will not
/// consider a shift complete until the report agrees), so these have to be the values
/// actually pushed to the actor rather than the values requested.
#[derive(Debug, Clone, Copy)]
struct AppliedState {
    /// `autoware_vehicle_msgs/GearReport` constant.
    gear: u8,
    /// `autoware_vehicle_msgs/TurnIndicatorsReport` constant.
    turn_indicators: u8,
    /// `autoware_vehicle_msgs/HazardLightsReport` constant.
    hazard_lights: u8,
    /// True while the last control command asked for braking, for the brake lights.
    braking: bool,
    /// Whether a gear command has ever been received. Until one has, reverse is inferred
    /// from the sign of the commanded velocity, so a stack with no `vehicle_cmd_gate`
    /// still reverses.
    gear_commanded: bool,
    /// Whether Autoware has declared an emergency on `/control/command/emergency_cmd`.
    ///
    /// `vehicle_cmd_gate` raises this when the stack decides the vehicle must stop now --
    /// an MRM, an AEB trigger, a failed validator. It is a separate channel from the
    /// control command precisely so that it still means something when the control command
    /// cannot be trusted, so it overrides rather than blends with it.
    emergency: bool,
    /// The light state last pushed to CARLA, so an unchanged one costs no RPC.
    ///
    /// Control commands arrive at 20 Hz and the lights almost never change between them;
    /// `set_light_state` on every one would add 20 round trips a second per vehicle to a
    /// server that several stacks already share.
    last_lights: Option<VehicleLightState>,
}

impl Default for AppliedState {
    fn default() -> Self {
        Self {
            // A vehicle that boots in drive is Autoware's own convention for simulation.
            gear: autoware_vehicle_msgs::msg::GearReport::DRIVE,
            turn_indicators: autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE,
            hazard_lights: autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE,
            braking: false,
            gear_commanded: false,
            // Not in an emergency until Autoware says so. Defaulting the other way would
            // hold the vehicle whenever the topic is absent, which is most bench setups.
            emergency: false,
            last_lights: None,
        }
    }
}

impl AppliedState {
    fn is_reverse(&self) -> bool {
        use autoware_vehicle_msgs::msg::GearReport;
        self.gear == GearReport::REVERSE || self.gear == GearReport::REVERSE_2
    }

    fn is_park(&self) -> bool {
        self.gear == autoware_vehicle_msgs::msg::GearReport::PARK
    }

    fn is_neutral(&self) -> bool {
        self.gear == autoware_vehicle_msgs::msg::GearReport::NEUTRAL
    }

    /// The CARLA light state this applied state implies.
    fn light_state(&self) -> VehicleLightState {
        use autoware_vehicle_msgs::msg::{HazardLightsReport, TurnIndicatorsReport};

        let mut lights = VehicleLightState::NONE;

        // Hazards win over the indicators while they are on, as on a real vehicle.
        if self.hazard_lights == HazardLightsReport::ENABLE {
            lights |= VehicleLightState::LEFT_BLINKER | VehicleLightState::RIGHT_BLINKER;
        } else {
            match self.turn_indicators {
                TurnIndicatorsReport::ENABLE_LEFT => lights |= VehicleLightState::LEFT_BLINKER,
                TurnIndicatorsReport::ENABLE_RIGHT => lights |= VehicleLightState::RIGHT_BLINKER,
                _ => {}
            }
        }

        // CARLA drives neither of these for an externally controlled vehicle.
        if self.braking {
            lights |= VehicleLightState::BRAKE;
        }
        if self.is_reverse() {
            lights |= VehicleLightState::REVERSE;
        }

        lights
    }
}

/// Vehicle control manager
///
/// Handles bidirectional control between Autoware and CARLA.
///
/// Subscribes to:
/// - `/control/command/control_cmd` (autoware_control_msgs/Control)
/// - `/control/command/gear_cmd` (autoware_vehicle_msgs/GearCommand)
/// - `/control/command/turn_indicators_cmd` (autoware_vehicle_msgs/TurnIndicatorsCommand)
/// - `/control/command/hazard_lights_cmd` (autoware_vehicle_msgs/HazardLightsCommand)
/// - `/control/command/emergency_cmd` (tier4_vehicle_msgs/VehicleEmergencyStamped)
///
/// Publishes:
/// - `/vehicle/status/velocity_status` (VelocityReport)
/// - `/vehicle/status/steering_status` (SteeringReport)
/// - `/vehicle/status/control_mode` (ControlModeReport)
/// - `/vehicle/status/gear_status` (GearReport)
/// - `/vehicle/status/turn_indicators_status` (TurnIndicatorsReport)
/// - `/vehicle/status/hazard_lights_status` (HazardLightsReport)
/// - `/vehicle/status/actuation_status` (tier4_vehicle_msgs/ActuationStatusStamped)
pub struct VehicleControlBridge {
    // Publishers
    velocity_pub: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::VelocityReport>>,
    steering_pub: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::SteeringReport>>,
    control_mode_pub: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>>,
    gear_pub: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::GearReport>>,
    turn_indicators_pub: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>>,
    hazard_lights_pub: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>>,
    actuation_pub: Arc<rclrs::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>>,

    // Subscribers stored to keep them alive
    _control_sub: Arc<rclrs::Subscription<autoware_control_msgs::msg::Control>>,
    _gear_sub: Arc<rclrs::Subscription<autoware_vehicle_msgs::msg::GearCommand>>,
    _emergency_sub: Arc<rclrs::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>>,
    _turn_indicators_sub:
        Arc<rclrs::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>>,
    _hazard_lights_sub: Arc<rclrs::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>>,

    // CARLA vehicle reference (shared with main loop)
    vehicle: Arc<Mutex<Option<Vehicle>>>,

    /// What was last applied, shared with the command callbacks.
    state: Arc<Mutex<AppliedState>>,

    /// How the spawned vehicle converts a steer command into a tire angle.
    steer_geometry: SteerGeometry,

    /// Report the *measured* front-wheel angle rather than echoing the command.
    ///
    /// True is the honest answer and the default (issue 009). It is a knob because it
    /// changes what Autoware's lateral controller sees: echoing the command hands MPC a
    /// perfect, instantaneous actuator, while the measured angle is the real one -- which
    /// lags, and which under-delivers by ~18% at the top of the planning range because the
    /// command maps to the wheel *limit* while the vehicle turns at the Ackermann *mean*
    /// (issue 006). Set false to get the old behaviour when bisecting a lateral-control
    /// problem.
    report_measured_steering: bool,
}

impl VehicleControlBridge {
    /// Create a new vehicle control bridge
    ///
    /// # Arguments
    /// * `node` - ROS node for creating publishers/subscribers
    /// * `vehicle` - Arc<Mutex<Option<Vehicle>>> shared with main loop
    pub fn new(
        node: rclrs::Node,
        vehicle: Arc<Mutex<Option<Vehicle>>>,
        report_measured_steering: bool,
        steering_multiplier: f32,
        longitudinal: Option<crate::longitudinal_map::LongitudinalCalibration>,
        honor_emergency_cmd: bool,
        control_trace: Option<Arc<crate::control_trace::ControlTrace>>,
    ) -> Result<Self> {
        let steer_geometry = Self::read_steer_geometry(&vehicle);

        // Create publishers
        let velocity_pub =
            Arc::new(node.create_publisher("/vehicle/status/velocity_status".reliable())?);

        let steering_pub =
            Arc::new(node.create_publisher("/vehicle/status/steering_status".reliable())?);

        let control_mode_pub =
            Arc::new(node.create_publisher("/vehicle/status/control_mode".reliable())?);

        let gear_pub = Arc::new(node.create_publisher("/vehicle/status/gear_status".reliable())?);

        let turn_indicators_pub =
            Arc::new(node.create_publisher("/vehicle/status/turn_indicators_status".reliable())?);

        let hazard_lights_pub =
            Arc::new(node.create_publisher("/vehicle/status/hazard_lights_status".reliable())?);

        let actuation_pub =
            Arc::new(node.create_publisher("/vehicle/status/actuation_status".reliable())?);

        let state = Arc::new(Mutex::new(AppliedState::default()));

        // Create control command subscriber (Autoware 1.5.0 uses Control message)
        let vehicle_for_control = vehicle.clone();
        let state_for_control = state.clone();
        // Trim on the steering command, applied to the normalized value sent to CARLA.
        // 1.0 sends exactly what the Ackermann inverse asks for, which is right for the
        // blueprints measured so far (docs/issues/006). It exists because that inverse is
        // derived from physics_control, and a blueprint whose tyres or steering curve
        // differ from its geometry can still under- or over-steer against the model.
        // Captured by the callback rather than stored: nothing else needs it.
        let trim = sane_steering_multiplier(steering_multiplier);
        let calibration = longitudinal.clone();
        let trace = control_trace.clone();
        let control_sub = Arc::new(node.create_subscription(
            "/control/command/control_cmd".reliable(),
            move |msg: autoware_control_msgs::msg::Control| {
                // Taken before anything else in the callback, so the row covers the whole of
                // the bridge's share of the path rather than a convenient part of it.
                let received = std::time::Instant::now();
                if let Err(e) = Self::apply_control_command(
                    &vehicle_for_control,
                    &state_for_control,
                    steer_geometry,
                    trim,
                    calibration.as_ref(),
                    trace.as_deref(),
                    received,
                    &msg,
                ) {
                    tracing::error!("Failed to apply control command: {}", e);
                }
            },
        )?);

        // Gear: Autoware shifts through this topic, not through the sign of the commanded
        // velocity. Pull-out, pull-over and every parking manoeuvre depend on it.
        let vehicle_for_gear = vehicle.clone();
        let state_for_gear = state.clone();
        let gear_sub = Arc::new(node.create_subscription(
            "/control/command/gear_cmd".reliable(),
            move |msg: autoware_vehicle_msgs::msg::GearCommand| {
                {
                    let mut state = state_for_gear.lock().unwrap();
                    if msg.command != autoware_vehicle_msgs::msg::GearCommand::NONE {
                        state.gear = msg.command;
                        state.gear_commanded = true;
                    }
                }
                Self::apply_lights(&vehicle_for_gear, &state_for_gear);
            },
        )?);

        let vehicle_for_turn = vehicle.clone();
        let state_for_turn = state.clone();
        // Autoware raised an emergency and, until now, nothing in the vehicle was listening:
        // `/control/command/emergency_cmd` had one publisher and zero subscribers on a
        // running stack. Hazard lights come with it, which is what a real vehicle does and
        // what makes the state visible in the simulation.
        let vehicle_for_emergency = vehicle.clone();
        let state_for_emergency = state.clone();
        if !honor_emergency_cmd {
            tracing::info!(
                "Not acting on /control/command/emergency_cmd: on this stack the flag does \
                 not carry an actionable emergency (measured true in 29% of samples while \
                 driving, with Autoware commanding +0.43 m/s^2 acceleration at the same \
                 time). Set honor_emergency_cmd:=true where it does. See docs/issues/021."
            );
        }
        let emergency_sub = Arc::new(node.create_subscription(
            "/control/command/emergency_cmd".reliable(),
            move |msg: tier4_vehicle_msgs::msg::VehicleEmergencyStamped| {
                if !honor_emergency_cmd {
                    return;
                }
                let changed = {
                    let mut state = state_for_emergency.lock().unwrap();
                    let changed = state.emergency != msg.emergency;
                    state.emergency = msg.emergency;
                    if changed {
                        state.hazard_lights = if msg.emergency {
                            autoware_vehicle_msgs::msg::HazardLightsReport::ENABLE
                        } else {
                            autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE
                        };
                    }
                    changed
                };
                if changed {
                    if msg.emergency {
                        tracing::warn!(
                            "Autoware declared an emergency; braking fully until it clears"
                        );
                    } else {
                        tracing::info!("Emergency cleared; returning to commanded control");
                    }
                    Self::apply_lights(&vehicle_for_emergency, &state_for_emergency);
                }
            },
        )?);

        let turn_indicators_sub = Arc::new(node.create_subscription(
            "/control/command/turn_indicators_cmd".reliable(),
            move |msg: autoware_vehicle_msgs::msg::TurnIndicatorsCommand| {
                {
                    let mut state = state_for_turn.lock().unwrap();
                    // NO_COMMAND (0) means "leave it alone", per the message definition.
                    if msg.command != autoware_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND
                    {
                        state.turn_indicators = msg.command;
                    }
                }
                Self::apply_lights(&vehicle_for_turn, &state_for_turn);
            },
        )?);

        let vehicle_for_hazard = vehicle.clone();
        let state_for_hazard = state.clone();
        let hazard_lights_sub = Arc::new(node.create_subscription(
            "/control/command/hazard_lights_cmd".reliable(),
            move |msg: autoware_vehicle_msgs::msg::HazardLightsCommand| {
                {
                    let mut state = state_for_hazard.lock().unwrap();
                    if msg.command != autoware_vehicle_msgs::msg::HazardLightsCommand::NO_COMMAND {
                        state.hazard_lights = msg.command;
                    }
                }
                Self::apply_lights(&vehicle_for_hazard, &state_for_hazard);
            },
        )?);

        tracing::info!("Vehicle control bridge created");
        tracing::info!(
            "  Steering: limit {:.3} rad, track/wheelbase {:.4}, {:.3} rad at full lock",
            steer_geometry.max_steer_angle,
            steer_geometry.track_over_wheelbase,
            effective_tire_angle(1.0, &steer_geometry),
        );
        tracing::info!(
            "  Steering status: {}",
            if report_measured_steering {
                "measured wheel angle"
            } else {
                "commanded angle (actuator hidden from the controller)"
            }
        );
        tracing::info!("  Subscribed to: /control/command/control_cmd");
        tracing::info!("  Subscribed to: /control/command/gear_cmd");
        tracing::info!("  Subscribed to: /control/command/turn_indicators_cmd");
        tracing::info!("  Subscribed to: /control/command/hazard_lights_cmd");
        tracing::info!("  Publishing: /vehicle/status/velocity_status");
        tracing::info!("  Publishing: /vehicle/status/steering_status");
        tracing::info!("  Publishing: /vehicle/status/control_mode");
        tracing::info!("  Publishing: /vehicle/status/gear_status");
        tracing::info!("  Publishing: /vehicle/status/turn_indicators_status");
        tracing::info!("  Publishing: /vehicle/status/hazard_lights_status");
        tracing::info!("  Publishing: /vehicle/status/actuation_status");

        Ok(Self {
            velocity_pub,
            steering_pub,
            control_mode_pub,
            gear_pub,
            turn_indicators_pub,
            hazard_lights_pub,
            actuation_pub,
            _control_sub: control_sub,
            _gear_sub: gear_sub,
            _emergency_sub: emergency_sub,
            _turn_indicators_sub: turn_indicators_sub,
            _hazard_lights_sub: hazard_lights_sub,
            vehicle,
            state,
            steer_geometry,
            report_measured_steering,
        })
    }

    /// Read the spawned vehicle's steering geometry from CARLA.
    ///
    /// Two things come out of `physics_control`: the steered wheels' limit, and the
    /// track/wheelbase ratio that sets how far the outer wheel trails the inner one.
    /// `vehicle_config.yaml` offers several blueprints and each has its own, so neither
    /// can be a constant. See `docs/issues/006-*`.
    ///
    /// Degrades to a plain bicycle model -- today's behaviour before this -- rather than to
    /// no steering, so a CARLA hiccup costs accuracy and not control.
    fn read_steer_geometry(vehicle: &Arc<Mutex<Option<Vehicle>>>) -> SteerGeometry {
        let guard = vehicle.lock().unwrap();
        let Some(vehicle) = guard.as_ref() else {
            tracing::warn!(
                "No vehicle when reading physics control; using the fall back steering \
                 limit of {FALLBACK_MAX_STEER_ANGLE:.3} rad with no Ackermann term"
            );
            return SteerGeometry::default();
        };

        let physics = match vehicle.physics_control() {
            Ok(physics) => physics,
            Err(e) => {
                tracing::warn!(
                    "Failed to read CARLA physics control ({e}); using the fall back \
                     steering limit of {FALLBACK_MAX_STEER_ANGLE:.3} rad with no \
                     Ackermann term"
                );
                return SteerGeometry::default();
            }
        };

        // CARLA reports max_steer_angle per wheel, in degrees. Only the steered wheels
        // carry a non-zero value, which is also how the axles are told apart here.
        let (steered, fixed): (Vec<_>, Vec<_>) = physics
            .wheels
            .iter()
            .partition(|w| w.max_steer_angle > 0.0);

        let max_degrees = steered
            .iter()
            .map(|w| w.max_steer_angle)
            .fold(0.0_f32, f32::max);
        if max_degrees <= 0.0 {
            tracing::warn!(
                "CARLA reported no steerable wheel; using the fall back steering limit of \
                 {FALLBACK_MAX_STEER_ANGLE:.3} rad with no Ackermann term"
            );
            return SteerGeometry::default();
        }

        // Track and wheelbase from the wheel offsets, which carla-rust exposes as
        // `offset` from the vehicle origin. Only their ratio is used, so whatever unit
        // CARLA reports cancels and no conversion is needed.
        let track_over_wheelbase = match (steered.as_slice(), fixed.as_slice()) {
            ([a, b], [c, d]) => {
                let track: f32 = (a.position.x - b.position.x).hypot(a.position.y - b.position.y);
                let front = (
                    0.5 * (a.position.x + b.position.x),
                    0.5 * (a.position.y + b.position.y),
                );
                let rear = (
                    0.5 * (c.position.x + d.position.x),
                    0.5 * (c.position.y + d.position.y),
                );
                let wheelbase: f32 = (front.0 - rear.0).hypot(front.1 - rear.1);
                if wheelbase > 0.0 && track > 0.0 {
                    track / wheelbase
                } else {
                    tracing::warn!(
                        "Degenerate wheel geometry (track {track:.1}, wheelbase \
                         {wheelbase:.1}); steering without an Ackermann term"
                    );
                    0.0
                }
            }
            _ => {
                tracing::warn!(
                    "Expected two steered and two fixed wheels, found {} and {}; steering \
                     without an Ackermann term",
                    steered.len(),
                    fixed.len()
                );
                0.0
            }
        };

        let geometry = SteerGeometry {
            max_steer_angle: max_degrees.to_radians(),
            track_over_wheelbase,
        };
        tracing::info!(
            "Steering geometry from CARLA physics: limit {:.1} deg, track/wheelbase \
             {:.4}, so {:.1} deg at full lock",
            max_degrees,
            track_over_wheelbase,
            effective_tire_angle(1.0, &geometry).to_degrees(),
        );
        geometry
    }

    /// Push the light state implied by `state` to CARLA.
    ///
    /// Best effort: a light that fails to set is cosmetic, and the caller is a ROS
    /// callback with nowhere to return an error to.
    fn apply_lights(vehicle: &Arc<Mutex<Option<Vehicle>>>, state: &Arc<Mutex<AppliedState>>) {
        // Nothing to send if the state has not moved. Callbacks run at command rate.
        let lights = {
            let mut state = state.lock().unwrap();
            let desired = state.light_state();
            if state.last_lights == Some(desired) {
                return;
            }
            state.last_lights = Some(desired);
            desired
        };

        // Locks are taken state-then-vehicle everywhere; never hold the vehicle lock
        // while reaching back for the state one.
        let failed = {
            let guard = vehicle.lock().unwrap();
            match guard.as_ref() {
                Some(vehicle) => vehicle.set_light_state(&lights).err(),
                None => None,
            }
        };

        if let Some(e) = failed {
            tracing::debug!("Failed to set vehicle light state: {e}");
            // Let the next change retry rather than believing a failed write.
            state.lock().unwrap().last_lights = None;
        }
    }

    /// Apply control command from Autoware to CARLA vehicle
    ///
    /// Converts Autoware Control (physical units) to CARLA VehicleControl (0-1 normalized):
    /// - steering_tire_angle (rad) → steer (-1 to 1) via the vehicle's own max steer angle
    /// - acceleration (m/s²) → throttle (0-1) or brake (0-1)
    ///
    /// The gear comes from `/control/command/gear_cmd` (see `AppliedState`), not from the
    /// sign of the commanded velocity.
    fn apply_control_command(
        vehicle: &Arc<Mutex<Option<Vehicle>>>,
        state: &Arc<Mutex<AppliedState>>,
        geometry: SteerGeometry,
        steering_multiplier: f32,
        longitudinal: Option<&crate::longitudinal_map::LongitudinalCalibration>,
        trace: Option<&crate::control_trace::ControlTrace>,
        received: std::time::Instant,
        cmd: &autoware_control_msgs::msg::Control,
    ) -> Result<()> {
        let accel = cmd.longitudinal.acceleration;

        let applied = {
            let mut state = state.lock().unwrap();

            // Until a gear command has been seen, fall back to the old heuristic so a
            // stack without vehicle_cmd_gate still reverses.
            if !state.gear_commanded {
                state.gear = if cmd.longitudinal.velocity < -0.01 {
                    autoware_vehicle_msgs::msg::GearReport::REVERSE
                } else {
                    autoware_vehicle_msgs::msg::GearReport::DRIVE
                };
            }
            state.braking = accel < -0.01;
            *state
        };

        let vehicle_guard = vehicle.lock().unwrap();
        if let Some(ref v) = *vehicle_guard {
            let mut control = VehicleControl {
                throttle: 0.0,
                steer: 0.0,
                brake: 0.0,
                hand_brake: false,
                reverse: applied.is_reverse(),
                manual_gear_shift: false,
                gear: 0,
            };

            // Steering: tire angle (rad) to normalized (-1 to 1), through CARLA's own
            // Ackermann geometry rather than a straight division by the wheel limit, which
            // asks for the inner wheel's angle and so under-delivers. See docs/issues/006-*.
            // Negate: Autoware positive = left turn (ROS), CARLA positive = right turn.
            control.steer = (-steer_command_for(cmd.lateral.steering_tire_angle, &geometry)
                * steering_multiplier)
                .clamp(-1.0, 1.0);

            // Longitudinal: acceleration (m/s²) → throttle or brake (0 to 1), through the
            // measured pedal maps when they are available. The fallback divides by a single
            // constant, which measurement shows to be wrong by a factor of two at full
            // throttle and up to four under braking -- see longitudinal_map.
            // The pedal maps are indexed by speed, so read it from the actor already
            // locked here rather than taking the lock again.
            let speed = v
                .velocity()
                .map(|vel| (vel.x as f64).hypot(vel.y as f64))
                .unwrap_or(0.0);
            // An emergency overrides the control command entirely, so it is decided before
            // any of the pedal-map work below. Autoware publishes it on its own topic so
            // that it still holds when the control command does not; blending the two would
            // defeat the point.
            if applied.emergency {
                control.throttle = 0.0;
                control.brake = 1.0;
                // Once stopped, hold with the handbrake -- CARLA's automatic transmission
                // idle-creeps at zero throttle, and an emergency stop that creeps is not one.
                control.hand_brake = speed < 0.1;
                v.apply_control(&control);
                return Ok(());
            }

            let pedals = match longitudinal {
                Some(cal) => cal.command_for(accel as f64, speed),
                None => crate::longitudinal_map::PedalCommand {
                    throttle: (accel / MAX_ACCEL).clamp(0.0, 1.0),
                    brake: (-accel / MAX_ACCEL).clamp(0.0, 1.0),
                },
            };

            // Apply what the maps chose, rather than picking the pedal from the sign of the
            // request. Those are not the same decision: CARLA's drag decelerates the car
            // harder than a mild braking request asks for -- 1.5 m/s^2 at 3 m/s against a
            // requested 0.9 -- so *holding* a gentle deceleration takes a little throttle,
            // not none. Gating on `accel < 0` zeroed that throttle and left the car
            // coasting, and measurement found it: of 39 samples where deceleration was
            // requested while moving, 35 had neither pedal applied, and the car decelerated
            // about 60% harder than asked. Braking tracked its request with a gain of 0.04.
            //
            // The two fields are mutually exclusive by construction, so this applies at
            // most one of them.
            //
            // Neutral has no drive torque, as in a real gearbox. Braking is never
            // suppressed: a deceleration command must reach the brakes whatever gear is
            // selected.
            if !applied.is_neutral() {
                control.throttle = pedals.throttle;
            }
            control.brake = pedals.brake;

            // Commanded standstill: hold with the handbrake. CARLA's automatic
            // transmission idle-creeps at zero throttle/brake, so a stopped vehicle
            // otherwise crawls to ~1 m/s under near-zero hold commands, and the next
            // firm brake command then reads as a -10 m/s² spike -- which scenario
            // tooling validating against vehicle performance bounds treats as fatal.
            //
            // PARK holds unconditionally, which is what the gear means.
            if applied.is_park() || (cmd.longitudinal.velocity.abs() <= 0.01 && accel <= 0.01) {
                control.hand_brake = true;
                if applied.is_park() {
                    control.throttle = 0.0;
                }
            }

            // The two stages the bridge owns: turning the command into a CARLA control, and
            // the RPC that delivers it. Recorded rather than inferred -- see control_trace.
            let converted = std::time::Instant::now();
            v.apply_control(&control)?;
            if let Some(t) = trace {
                let stamp = cmd.stamp.sec as f64 + cmd.stamp.nanosec as f64 * 1e-9;
                t.record(stamp, received, converted, std::time::Instant::now());
            }

            tracing::debug!(
                "Applied control: steer={:.3}, throttle={:.3}, brake={:.3}, accel={:.2} m/s², \
                 gear={}",
                control.steer,
                control.throttle,
                control.brake,
                accel,
                applied.gear,
            );
        }
        drop(vehicle_guard);

        // Brake and reverse lights follow the control that was just applied. Done after
        // the vehicle lock is released, and only when the state actually changed.
        Self::apply_lights(vehicle, state);

        Ok(())
    }

    /// The measured front-wheel steering angle in ROS convention (radians, positive left).
    ///
    /// CARLA reports the physical wheel angle in degrees. Reporting the *commanded* value
    /// instead -- which this used to do, via `Vehicle::control()` -- hands Autoware's MPC
    /// back the command it just issued and removes the actuator from its loop entirely.
    /// See `docs/issues/009-*`.
    fn measured_steering_angle(&self, vehicle: &Vehicle, commanded_steer: f32) -> f32 {
        // `VehicleWheelLocation` is an autocxx-generated POD without `Copy`, so the two
        // wheels are read one at a time rather than through an array.
        let front = vehicle
            .wheel_steer_angle(VehicleWheelLocation::FL_Wheel)
            .and_then(|fl| {
                vehicle
                    .wheel_steer_angle(VehicleWheelLocation::FR_Wheel)
                    .map(|fr| (fl, fr))
            });

        match front {
            Ok((fl, fr)) => {
                let mean_degrees = 0.5 * (fl + fr);
                // Negate: CARLA positive = right turn, Autoware positive = left turn.
                -mean_degrees.to_radians()
            }
            _ => {
                // Rate-limited by being a debug line: a CARLA build without the API would
                // otherwise log at 20 Hz for the whole run.
                tracing::debug!("Wheel steer angle unavailable; reporting the commanded angle");
                -effective_tire_angle(commanded_steer, &self.steer_geometry)
            }
        }
    }

    /// Publish vehicle status to Autoware
    ///
    /// Should be called in the main loop at regular intervals (e.g., 20 Hz)
    ///
    /// # Arguments
    /// * `timestamp` - Current simulation timestamp
    pub fn publish_status(&self, timestamp: f64) -> Result<()> {
        let vehicle_guard = self.vehicle.lock().unwrap();
        if let Some(ref vehicle) = *vehicle_guard {
            let ros_timestamp = builtin_interfaces::msg::Time {
                sec: timestamp.floor() as i32,
                nanosec: ((timestamp - timestamp.floor()) * 1e9) as u32,
            };

            // Get vehicle state from CARLA
            let transform = vehicle.transform()?;
            let velocity_vec = vehicle.velocity()?;
            let angular_velocity_vec = vehicle.angular_velocity()?;
            let control = vehicle.control()?;
            let state = *self.state.lock().unwrap();

            // VelocityReport is a base_link message. CARLA reports velocity in world
            // coordinates, so it has to be rotated into the vehicle's own frame before
            // publishing -- otherwise "lateral" is a world-Y component and "longitudinal"
            // is an unsigned magnitude that stays positive while reversing. See
            // `docs/issues/003-*`.
            let body_velocity = transform.rotation.inverse_rotate_vector(&velocity_vec);
            let longitudinal_velocity = body_velocity.x;
            // Negate: CARLA body Y = right (left-handed), ROS lateral = left (right-handed)
            let lateral_velocity = -body_velocity.y;
            // Negate: CARLA Z angular = clockwise, ROS yaw rate = counter-clockwise.
            // Convert: CARLA reports angular velocity in DEGREES per second (measured,
            // see docs/issues/008), while VelocityReport.heading_rate is rad/s.
            let heading_rate = -angular_velocity_vec.z.to_radians();

            // Publish VelocityReport
            let velocity_report = autoware_vehicle_msgs::msg::VelocityReport {
                header: std_msgs::msg::Header {
                    stamp: ros_timestamp.clone(),
                    frame_id: "base_link".to_string(),
                },
                longitudinal_velocity,
                lateral_velocity,
                heading_rate,
            };

            self.velocity_pub.publish(&velocity_report)?;

            // Publish SteeringReport from the measured wheel angle
            let steering_report = autoware_vehicle_msgs::msg::SteeringReport {
                stamp: ros_timestamp.clone(),
                steering_tire_angle: if self.report_measured_steering {
                    self.measured_steering_angle(vehicle, control.steer)
                } else {
                    // Negate: CARLA positive = right turn, Autoware positive = left turn.
                    -effective_tire_angle(control.steer, &self.steer_geometry)
                },
            };

            self.steering_pub.publish(&steering_report)?;

            // Publish ControlModeReport (always AUTONOMOUS in simulation)
            let control_mode = autoware_vehicle_msgs::msg::ControlModeReport {
                stamp: ros_timestamp.clone(),
                mode: autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS,
            };

            self.control_mode_pub.publish(&control_mode)?;

            // Publish GearReport for the gear actually applied
            let gear_report = autoware_vehicle_msgs::msg::GearReport {
                stamp: ros_timestamp.clone(),
                report: state.gear,
            };

            self.gear_pub.publish(&gear_report)?;

            // Publish the light reports, so Autoware sees its own requests acknowledged
            self.turn_indicators_pub.publish(
                &autoware_vehicle_msgs::msg::TurnIndicatorsReport {
                    stamp: ros_timestamp.clone(),
                    report: state.turn_indicators,
                },
            )?;

            self.hazard_lights_pub
                .publish(&autoware_vehicle_msgs::msg::HazardLightsReport {
                    stamp: ros_timestamp.clone(),
                    report: state.hazard_lights,
                })?;

            // Publish ActuationStatusStamped in CARLA's own normalization, which is what
            // the message is defined in.
            self.actuation_pub
                .publish(&tier4_vehicle_msgs::msg::ActuationStatusStamped {
                    header: std_msgs::msg::Header {
                        stamp: ros_timestamp,
                        frame_id: "base_link".to_string(),
                    },
                    status: tier4_vehicle_msgs::msg::ActuationStatus {
                        accel_status: control.throttle as f64,
                        brake_status: control.brake as f64,
                        steer_status: control.steer as f64,
                    },
                })?;

            tracing::trace!(
                "Published vehicle status: vel={:.2} m/s, steer={:.3} rad, gear={}",
                longitudinal_velocity,
                steering_report.steering_tire_angle,
                state.gear,
            );
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use autoware_vehicle_msgs::msg::{GearReport, HazardLightsReport, TurnIndicatorsReport};

    #[test]
    fn a_fresh_bridge_reports_drive_and_no_lights() {
        let state = AppliedState::default();
        assert_eq!(state.gear, GearReport::DRIVE);
        assert!(!state.is_reverse());
        assert_eq!(state.light_state(), VehicleLightState::NONE);
    }

    /// Regression guard for issue 004: reverse must come from the gear, and the reverse
    /// lamp with it.
    #[test]
    fn reverse_gear_drives_the_reverse_lamp() {
        let state = AppliedState {
            gear: GearReport::REVERSE,
            ..Default::default()
        };
        assert!(state.is_reverse());
        assert!(state.light_state().contains(VehicleLightState::REVERSE));
    }

    /// Regression guard for issue 005.
    #[test]
    fn indicators_map_to_the_matching_blinker() {
        let left = AppliedState {
            turn_indicators: TurnIndicatorsReport::ENABLE_LEFT,
            ..Default::default()
        };
        assert!(left.light_state().contains(VehicleLightState::LEFT_BLINKER));
        assert!(!left
            .light_state()
            .contains(VehicleLightState::RIGHT_BLINKER));

        let right = AppliedState {
            turn_indicators: TurnIndicatorsReport::ENABLE_RIGHT,
            ..Default::default()
        };
        assert!(right
            .light_state()
            .contains(VehicleLightState::RIGHT_BLINKER));
        assert!(!right
            .light_state()
            .contains(VehicleLightState::LEFT_BLINKER));
    }

    #[test]
    fn hazards_light_both_blinkers_and_outrank_an_indicator() {
        let state = AppliedState {
            turn_indicators: TurnIndicatorsReport::ENABLE_LEFT,
            hazard_lights: HazardLightsReport::ENABLE,
            ..Default::default()
        };
        let lights = state.light_state();
        assert!(lights.contains(VehicleLightState::LEFT_BLINKER));
        assert!(lights.contains(VehicleLightState::RIGHT_BLINKER));
    }

    #[test]
    fn braking_lights_the_brake_lamp() {
        let state = AppliedState {
            braking: true,
            ..Default::default()
        };
        assert!(state.light_state().contains(VehicleLightState::BRAKE));
    }

    /// The Tesla Model 3 as CARLA 0.9.16 reports it.
    fn tesla() -> SteerGeometry {
        SteerGeometry {
            max_steer_angle: 70.0_f32.to_radians(),
            track_over_wheelbase: 0.5548,
        }
    }

    /// Wheel angles measured off a live server by `scripts/probe_steer_curve.py`, at rest.
    ///
    /// The model has to reproduce the *mean* of the two front wheels, which is the angle
    /// the vehicle actually turns at -- not the wheel limit the command used to be scaled
    /// against.
    #[test]
    fn effective_angle_matches_measured_wheels() {
        // (steer command, measured mean of FL and FR, degrees)
        let measured = [
            (0.10, 6.78),
            (0.20, 13.18),
            (0.25, 16.26),
            (0.30, 19.28),
            (0.40, 25.16),
            (0.50, 30.88),
            (0.60, 36.49),
            (0.70, 42.04),
            (0.80, 47.56),
            (0.90, 53.11),
            (1.00, 58.71),
        ];
        for (cmd, expected_deg) in measured {
            let got = effective_tire_angle(cmd, &tesla()).to_degrees();
            assert!(
                (got - expected_deg).abs() < 0.15,
                "cmd {cmd}: model {got:.2} deg, measured {expected_deg:.2} deg"
            );
        }
    }

    #[test]
    fn steer_command_inverts_the_model() {
        for cmd in [0.05_f32, 0.2, 0.5, 0.8, 1.0] {
            let angle = effective_tire_angle(cmd, &tesla());
            let back = steer_command_for(angle, &tesla());
            assert!((back - cmd).abs() < 1e-3, "cmd {cmd} round tripped to {back}");
        }
    }

    /// The bug this fixes: dividing by the wheel limit under-delivers, and by how much.
    #[test]
    fn dividing_by_the_wheel_limit_under_delivers() {
        let g = tesla();
        // The top of Autoware's 0.70 rad planning range.
        let requested = 0.70_f32;
        let old_cmd = requested / g.max_steer_angle;
        let old_delivered = effective_tire_angle(old_cmd, &g);
        assert!(
            old_delivered < requested * 0.92,
            "expected the old mapping to under-deliver, got {old_delivered:.4} for \
             {requested:.4}"
        );
        // The fix delivers what was asked for.
        let delivered = effective_tire_angle(steer_command_for(requested, &g), &g);
        assert!((delivered - requested).abs() < 1e-3);
    }

    #[test]
    fn steering_is_signed_and_saturates() {
        let g = tesla();
        assert_eq!(steer_command_for(0.0, &g), 0.0);
        assert!(steer_command_for(-0.3, &g) < 0.0);
        assert_eq!(steer_command_for(2.0, &g), 1.0);
        assert_eq!(steer_command_for(-2.0, &g), -1.0);
    }

    /// Without geometry the model must reduce to exactly what it replaced.
    #[test]
    fn zero_ackermann_term_is_the_old_linear_mapping() {
        let g = SteerGeometry {
            max_steer_angle: 1.22,
            track_over_wheelbase: 0.0,
        };
        assert!((steer_command_for(0.61, &g) - 0.5).abs() < 1e-4);
    }

    #[test]
    fn a_unit_multiplier_leaves_the_command_alone() {
        assert_eq!(sane_steering_multiplier(1.0), 1.0);
        assert_eq!(sane_steering_multiplier(0.85), 0.85);
        assert_eq!(sane_steering_multiplier(1.3), 1.3);
    }

    /// Zero cannot steer and a negative value steers backwards; both are config mistakes
    /// that would present as a control fault far from the config file.
    #[test]
    fn a_useless_multiplier_falls_back_to_one() {
        assert_eq!(sane_steering_multiplier(0.0), 1.0);
        assert_eq!(sane_steering_multiplier(-1.0), 1.0);
        assert_eq!(sane_steering_multiplier(f32::NAN), 1.0);
        assert_eq!(sane_steering_multiplier(f32::INFINITY), 1.0);
    }

    /// The trim scales the command and the clamp still holds at the rails.
    #[test]
    fn the_multiplier_scales_and_still_saturates() {
        let g = tesla();
        let base = steer_command_for(0.30, &g);
        assert!((base * 0.5).abs() < base.abs());
        assert_eq!((base * 100.0_f32).clamp(-1.0, 1.0), 1.0);
        assert_eq!((-base * 100.0_f32).clamp(-1.0, 1.0), -1.0);
    }
}
