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
    rpc::{VehicleControl, VehicleLightState},
};
use rclrs::IntoPrimitiveOptions;
use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

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
#[derive(Debug, Clone)]
struct SteerGeometry {
    /// The steered wheels' physical limit, radians.
    max_steer_angle: f32,
    /// track / wheelbase -- the Ackermann differential term. Zero reduces the model to a
    /// plain bicycle, which is the behaviour this replaced.
    track_over_wheelbase: f32,
    /// `VehiclePhysicsControl::steering_curve` as (speed km/h, factor) points, sorted by
    /// speed. CARLA multiplies the achieved wheel angle by this curve at the vehicle's
    /// speed, so the two terms above are exact only at standstill. See `steering_curve_factor`.
    steering_curve: Vec<(f32, f32)>,
}

/// The steering curve that changes nothing: a factor of 1.0 at every speed.
const FLAT_STEERING_CURVE: [(f32, f32); 1] = [(0.0, 1.0)];

impl Default for SteerGeometry {
    fn default() -> Self {
        Self {
            max_steer_angle: FALLBACK_MAX_STEER_ANGLE,
            track_over_wheelbase: 0.0,
            steering_curve: FLAT_STEERING_CURVE.to_vec(),
        }
    }
}

/// CARLA's speed-dependent steering reduction at `speed_kmh`.
///
/// Measured on 0.9.16 (roadmap 014, "Steering"; raw data in
/// `docs/measurements/steering_tesla_model3_0916.csv` of the superproject): the command
/// itself is linear, but the angle the wheels reach is multiplied by `steering_curve`
/// interpolated at the vehicle's speed in km/h. The Tesla's (0,1.0) (20,0.9) (60,0.8)
/// (120,0.7) matched every point to +-0.003, so a command model that ignores it
/// under-steers by 10% at 20 km/h and ~16% at 40.
///
/// Linear between points and held at the end values outside them, as UE4 evaluates the
/// curve. The points are sorted where they are read; an unsorted slice is sorted here too,
/// rather than trusted, because a mis-ordered curve would interpolate nonsense silently.
/// An empty curve is no reduction at all.
fn steering_curve_factor(curve: &[(f32, f32)], speed_kmh: f32) -> f32 {
    let sorted;
    let curve = if curve.is_sorted_by(|a, b| a.0 <= b.0) {
        curve
    } else {
        let mut owned = curve.to_vec();
        owned.sort_by(|a, b| a.0.total_cmp(&b.0));
        sorted = owned;
        &sorted
    };
    let (Some(first), Some(last)) = (curve.first(), curve.last()) else {
        return 1.0;
    };
    if speed_kmh <= first.0 {
        return first.1;
    }
    if speed_kmh >= last.0 {
        return last.1;
    }
    for pair in curve.windows(2) {
        let ((x0, y0), (x1, y1)) = (pair[0], pair[1]);
        if speed_kmh <= x1 {
            let span = x1 - x0;
            if span <= f32::EPSILON {
                return y1;
            }
            return y0 + (y1 - y0) * (speed_kmh - x0) / span;
        }
    }
    last.1
}

/// The steering-curve factor the command should be divided by, or 1.0 when compensation
/// is switched off (`compensate_steering_curve`).
///
/// CARLA applies the curve against the *magnitude* of the forward speed, so reversing is
/// reduced just as driving forward is.
fn steer_speed_factor(geometry: &SteerGeometry, compensate: bool, longitudinal_mps: f32) -> f32 {
    if !compensate {
        return 1.0;
    }
    steering_curve_factor(&geometry.steering_curve, longitudinal_mps.abs() * 3.6)
}

/// Divide a normalized steer command by the steering-curve factor, clamped to CARLA's
/// range. Returns the command and whether the clamp had to engage.
///
/// Dividing pre-distorts the command so that, after CARLA multiplies by the same factor,
/// the wheels land where the Ackermann inverse aimed them. Near full lock at speed that
/// asks for more than 1.0; the rail is the most CARLA will give, and the shortfall is
/// real rather than a model error. A factor that is not a positive number leaves the
/// command alone instead of sending an infinity or a sign flip to the actuator.
fn compensate_steer(steer: f32, factor: f32) -> (f32, bool) {
    let raw = if factor.is_finite() && factor > 0.0 {
        steer / factor
    } else {
        steer
    };
    let clamped = raw.clamp(-1.0, 1.0);
    (clamped, clamped != raw)
}

/// Optional steering actuator dynamics applied to Autoware's commanded tire angle before it
/// reaches CARLA: a first-order lag, then a rate limit (roadmap 014, "Steering").
///
/// CARLA's wheels slew at ~150 deg/s (measured), far faster than a production steering
/// actuator, so Autoware's commands land almost instantly. These two knobs let a run model a
/// slower actuator; both default off. Starting values from tier4/scenario_simulator_v2#1849
/// are 20 deg/s and tau = 0.2 s.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SteerDynamics {
    /// Maximum tire-angle rate, rad/s; 0 (or less) disables the limit.
    pub rate_limit: f32,
    /// First-order lag time constant, s; 0 (or less) disables the lag.
    pub time_constant: f32,
}

impl SteerDynamics {
    pub fn is_off(&self) -> bool {
        self.rate_limit <= 0.0 && self.time_constant <= 0.0
    }
}

/// The longest gap integrated in one step. Commands arrive at ~30 Hz; a longer gap is a
/// paused simulation or a new run, and integrating it whole would only jump to the target.
const MAX_STEER_DYNAMICS_DT: f64 = 0.5;

/// One step of the actuator: from the angle it last produced (`previous`: angle, sim time)
/// towards `target`, at simulation time `now`. Off, or with no previous step, it passes the
/// target through.
fn steer_dynamics_step(
    dynamics: &SteerDynamics,
    previous: Option<(f32, f64)>,
    target: f32,
    now: f64,
) -> f32 {
    let Some((angle, at)) = previous else {
        return target;
    };
    if dynamics.is_off() {
        return target;
    }
    let dt = (now - at).clamp(0.0, MAX_STEER_DYNAMICS_DT) as f32;
    let mut next = if dynamics.time_constant > 0.0 {
        angle + (target - angle) * (1.0 - (-dt / dynamics.time_constant).exp())
    } else {
        target
    };
    if dynamics.rate_limit > 0.0 {
        let max_step = dynamics.rate_limit * dt;
        next = angle + (next - angle).clamp(-max_step, max_step);
    }
    next
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
///
/// CARLA 0.10's Chaos vehicles respond to the *square* of the command: steady-state
/// curvature on vehicle.lincoln.mkz at 3 m/s was 0.0033, 0.0135 and 0.0305 1/m for
/// commands 0.1, 0.2 and 0.3, a 1 : 4.1 : 9.2 ratio (carla-scenario-bridge roadmap 019;
/// TIER IV's square-root fix for the same engine). There the inner wheel follows
/// `cmd^2 * max_steer_angle`, and `steer_command_for` inverts it to a square root.
fn effective_tire_angle(steer_cmd: f32, geometry: &SteerGeometry) -> f32 {
    let magnitude = steer_cmd.abs().clamp(0.0, 1.0);
    #[cfg(carla_0100)]
    let magnitude = magnitude * magnitude;
    let inner = magnitude * geometry.max_steer_angle;
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
/// Speed below which the vehicle counts as not moving, in m/s. CARLA reports small
/// non-zero velocities for a stationary car, so this is not zero.
const STALL_SPEED_MPS: f64 = 0.1;

/// Throttle below which no meaningful drive torque was asked for. Without this a vehicle
/// legitimately held at a stop line, where the pedal maps ask for a whisper of throttle
/// against drag, would read as stuck.
const STALL_MIN_THROTTLE: f32 = 0.05;

/// How long the vehicle must be told to move, and not move, before that is worth saying.
/// Long enough to cover a standing start on a heavy vehicle rather than report every
/// pull-away as a fault.
const STALL_AFTER: Duration = Duration::from_secs(5);

/// Spacing between repeats while a stall persists, so a stuck run costs one line every
/// fifteen seconds instead of one per control command at 20 Hz.
const STALL_WARN_INTERVAL: Duration = Duration::from_secs(15);

/// Speed below which a commanded standstill is held with the handbrake.
///
/// Autoware commands zero velocity *before* the car has stopped: its stop sequence reaches
/// `velocity = 0` with the car still rolling at up to ~1 m/s, and keeps braking with
/// `stopped_acc`. Locking CARLA's handbrake at that moment stopped the car within two ticks
/// -- measured -6.6 and -8.7 m/s^2 at 20 Hz, from 0.8 m/s -- and AWF's UC-AEB-001-0001
/// fails any deceleration beyond 6 m/s^2 (carla-scenario-bridge roadmap 020). Below 0.1 m/s the lock takes off
/// at most 2 m/s^2 in one tick, which the bridge's median filter discards; above it the
/// service brake finishes the stop. Same threshold as the emergency branch.
const HANDBRAKE_ENGAGE_SPEED_MPS: f64 = 0.1;

/// Whether to hold the vehicle with the handbrake this tick: always in PARK; otherwise on
/// a commanded standstill (velocity and acceleration both ~0 or braking) once the car has
/// actually come to rest (`HANDBRAKE_ENGAGE_SPEED_MPS`). Holding stops CARLA's automatic
/// transmission from idle-creeping a stopped car.
fn hold_with_handbrake(is_park: bool, cmd_velocity: f64, cmd_accel: f64, speed: f64) -> bool {
    is_park
        || (cmd_velocity.abs() <= 0.01 && cmd_accel <= 0.01 && speed < HANDBRAKE_ENGAGE_SPEED_MPS)
}

/// What `StallWatch` decided to say, if anything.
#[derive(Debug, PartialEq)]
enum StallEvent {
    /// The vehicle has been commanded to move and has not moved for this long.
    Stuck { seconds: f64 },
    /// It started moving again, after this long stuck.
    Recovered { seconds: f64 },
}

/// Notices that the vehicle has been told to move and is not moving.
///
/// This does not intervene and is not a controller. Its whole job is to say out loud that
/// a run has entered a state nothing else reports: `vehicle_cmd_gate` goes on issuing
/// positive acceleration, this bridge goes on applying real throttle, and the vehicle
/// stays at a standstill. A traced run spent its last 140 seconds doing exactly that at
/// 22.7% throttle, with no diagnostic from any node -- see docs/issues/016.
///
/// Reporting it from here is deliberate: this is the one place that knows both what was
/// asked for and what the actor did about it.
#[derive(Debug, Default)]
struct StallWatch {
    /// When the vehicle first met every condition for being stuck, while it still does.
    since: Option<Instant>,
    /// When the last warning went out, for rate limiting.
    warned_at: Option<Instant>,
    /// Whether a stall was reported and has not yet cleared, so recovery is logged once
    /// and only when there was something to recover from.
    reported: bool,
}

impl StallWatch {
    /// Fold in one applied command. `stuck_now` is the caller's judgement of whether this
    /// command asked for motion that did not happen; `moving` is whether the vehicle is
    /// actually going anywhere.
    ///
    /// Both are needed, because the stuck condition can end two ways and only one of them
    /// is good news. The vehicle can start moving, or Autoware can stop asking it to --
    /// and reporting the second as recovery is a lie the log then tells for the rest of
    /// the run. Observed for real: during a held vehicle the planner's target dipped below
    /// the threshold for a single command and the bridge announced "moving again" about a
    /// car that had not moved at all.
    fn observe(&mut self, now: Instant, stuck_now: bool, moving: bool) -> Option<StallEvent> {
        if !stuck_now {
            let was_reported = self.reported;
            let since = self.since;
            *self = Self::default();
            if was_reported && moving {
                if let Some(since) = since {
                    return Some(StallEvent::Recovered {
                        seconds: now.duration_since(since).as_secs_f64(),
                    });
                }
            }
            // Cleared because nothing is asking for motion any more. That is not recovery
            // and saying so would be worse than saying nothing.
            return None;
        }

        let since = *self.since.get_or_insert(now);
        let held = now.duration_since(since);
        if held < STALL_AFTER {
            return None;
        }
        let due = match self.warned_at {
            None => true,
            Some(last) => now.duration_since(last) >= STALL_WARN_INTERVAL,
        };
        if !due {
            return None;
        }
        self.warned_at = Some(now);
        self.reported = true;
        Some(StallEvent::Stuck {
            seconds: held.as_secs_f64(),
        })
    }
}

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
    /// The tire angle the steering dynamics last produced and the simulation time of that
    /// step (`SteerDynamics`); `None` until the first control command.
    steer_actuator: Option<(f32, f64)>,
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
            steer_actuator: None,
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

/// Answer to a `/control/control_mode_request`, exactly as SSv2's stock ego simulation gives
/// it (`concealer/src/autoware_universe.cpp`, the `control_mode_request_server`):
/// AUTONOMOUS succeeds, MANUAL and everything else fail.
///
/// MANUAL fails in the stock simulation because it only arrives on a remote override, which
/// scenario_simulator_v2 does not support; this bridge does not support one either. The
/// reported mode on `/vehicle/status/control_mode` stays AUTONOMOUS regardless -- a request
/// that succeeds asks for the mode the vehicle is already in, and one that fails changes
/// nothing.
pub fn control_mode_request_accepted(mode: u8) -> bool {
    use autoware_vehicle_msgs::srv::control_mode_command::ControlModeCommand_Request as Req;
    mode == Req::AUTONOMOUS
}

fn control_mode_name(mode: u8) -> &'static str {
    use autoware_vehicle_msgs::srv::control_mode_command::ControlModeCommand_Request as Req;
    match mode {
        Req::NO_COMMAND => "NO_COMMAND",
        Req::AUTONOMOUS => "AUTONOMOUS",
        Req::AUTONOMOUS_STEER_ONLY => "AUTONOMOUS_STEER_ONLY",
        Req::AUTONOMOUS_VELOCITY_ONLY => "AUTONOMOUS_VELOCITY_ONLY",
        Req::MANUAL => "MANUAL",
        _ => "UNKNOWN",
    }
}

/// Server for `/control/control_mode_request` (autoware_vehicle_msgs/srv/ControlModeCommand).
///
/// The stock ego simulation serves this and nothing in a CARLA stack did, so Autoware's
/// operation-mode transition asked a service that never answered (roadmap 014, gap 12).
///
/// It lives for the node's whole lifetime rather than one vehicle session: the service is
/// part of the vehicle interface Autoware sees, not of any particular CARLA actor, and a
/// per-session server would vanish between scenario runs. Requests are only answered while
/// the executor spins, which is the main loop's job.
pub struct ControlModeService {
    _service: rclrs::Service<autoware_vehicle_msgs::srv::ControlModeCommand>,
}

impl ControlModeService {
    pub const SERVICE_NAME: &'static str = "/control/control_mode_request";

    pub fn new(node: &rclrs::Node) -> Result<Self> {
        use autoware_vehicle_msgs::srv::control_mode_command::{
            ControlModeCommand_Request, ControlModeCommand_Response,
        };
        let service = node.create_service::<autoware_vehicle_msgs::srv::ControlModeCommand, _>(
            Self::SERVICE_NAME,
            |request: ControlModeCommand_Request| {
                let success = control_mode_request_accepted(request.mode);
                tracing::info!(
                    "{} {} ({}) -> success={success}",
                    ControlModeService::SERVICE_NAME,
                    control_mode_name(request.mode),
                    request.mode,
                );
                ControlModeCommand_Response { success }
            },
        )?;
        tracing::info!("  Serving: {}", Self::SERVICE_NAME);
        Ok(Self { _service: service })
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

    /// Divide the steer command by CARLA's steering curve at the current speed
    /// (`compensate_steering_curve`, roadmap 014). Kept here as well as in the command
    /// callback so the echoed steering report can undo the same factor.
    compensate_steering_curve: bool,

    /// `base_link` (the rear-axle centre) in the actor frame, CARLA axes, metres -- the
    /// same offset `CarlaVehicle::base_link_in_actor` measured at spawn. The VelocityReport
    /// is the rear axle's, not the actor origin's (roadmap 014, "Pose reference point").
    base_link_in_actor: nalgebra::Vector3<f64>,
}

impl VehicleControlBridge {
    /// Create a new vehicle control bridge
    ///
    /// # Arguments
    /// * `node` - ROS node for creating publishers/subscribers
    /// * `vehicle` - Arc<Mutex<Option<Vehicle>>> shared with main loop
    // One argument per launch parameter it honours; a config struct would only move the list.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        node: rclrs::Node,
        vehicle: Arc<Mutex<Option<Vehicle>>>,
        report_measured_steering: bool,
        steering_multiplier: f32,
        compensate_steering_curve: bool,
        steer_dynamics: SteerDynamics,
        longitudinal: Option<crate::longitudinal_map::LongitudinalCalibration>,
        honor_emergency_cmd: bool,
        control_trace: Option<Arc<crate::control_trace::ControlTrace>>,
        base_link_in_actor: nalgebra::Vector3<f64>,
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
        let stall = Arc::new(Mutex::new(StallWatch::default()));

        // Create control command subscriber (Autoware 1.5.0 uses Control message)
        let vehicle_for_control = vehicle.clone();
        let state_for_control = state.clone();
        let stall_for_control = stall.clone();
        // Trim on the steering command, applied to the normalized value sent to CARLA.
        // 1.0 sends exactly what the Ackermann inverse asks for, which is right for the
        // blueprints measured so far (docs/issues/006). It exists because that inverse is
        // derived from physics_control, and a blueprint whose tyres or steering curve
        // differ from its geometry can still under- or over-steer against the model.
        // Captured by the callback rather than stored: nothing else needs it.
        let trim = sane_steering_multiplier(steering_multiplier);
        let geometry_for_control = steer_geometry.clone();
        let calibration = longitudinal.clone();
        let trace = control_trace.clone();
        // The node's own ROS clock, read inside the callback: /clock is simulation time, and
        // the command's stamp is too, so the two are comparable.
        let node_for_clock = node.clone();
        let control_sub = Arc::new(node.create_subscription(
            "/control/command/control_cmd".reliable(),
            move |msg: autoware_control_msgs::msg::Control| {
                // Taken before anything else in the callback, so the row covers the whole of
                // the bridge's share of the path rather than a convenient part of it.
                let received = std::time::Instant::now();
                if let Err(e) = Self::apply_control_command(
                    &vehicle_for_control,
                    &state_for_control,
                    &geometry_for_control,
                    trim,
                    compensate_steering_curve,
                    &steer_dynamics,
                    calibration.as_ref(),
                    trace.as_deref(),
                    &stall_for_control,
                    received,
                    crate::utils::ros_time_now_secs(&node_for_clock),
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
            "  Steering curve: {}",
            if compensate_steering_curve {
                format!(
                    "compensated, {:?} (km/h, factor)",
                    steer_geometry.steering_curve
                )
            } else {
                "not compensated (compensate_steering_curve:=false); the ego under-steers \
                 at speed by CARLA's steering_curve"
                    .to_string()
            }
        );
        tracing::info!(
            "  Steering dynamics: {}",
            if steer_dynamics.is_off() {
                "off (commands applied as received)".to_string()
            } else {
                format!(
                    "rate limit {}, lag {}",
                    if steer_dynamics.rate_limit > 0.0 {
                        format!("{:.1} deg/s", steer_dynamics.rate_limit.to_degrees())
                    } else {
                        "off".to_string()
                    },
                    if steer_dynamics.time_constant > 0.0 {
                        format!("tau {:.3} s", steer_dynamics.time_constant)
                    } else {
                        "off".to_string()
                    }
                )
            }
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
            compensate_steering_curve,
            base_link_in_actor,
        })
    }

    /// Read the spawned vehicle's steering geometry from CARLA.
    ///
    /// Three things come out of `physics_control`: the steered wheels' limit, the
    /// track/wheelbase ratio that sets how far the outer wheel trails the inner one, and
    /// the steering curve that scales both down with speed (roadmap 014).
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
        let (steered, fixed): (Vec<_>, Vec<_>) =
            physics.wheels.iter().partition(|w| w.max_steer_angle > 0.0);

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

        // CARLA 0.10 exposes no wheel positions (carla-scenario-bridge roadmap 019), so the
        // Ackermann term cannot be measured there; steer as a plain bicycle model.
        #[cfg(carla_0100)]
        let track_over_wheelbase = {
            let _ = (&steered, &fixed);
            tracing::warn!(
                "CARLA 0.10 exposes no wheel positions; steering without an Ackermann term"
            );
            0.0_f32
        };
        // Track and wheelbase from the wheel offsets, which carla-rust exposes as
        // `offset` from the vehicle origin. Only their ratio is used, so whatever unit
        // CARLA reports cancels and no conversion is needed.
        #[cfg(not(carla_0100))]
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

        // The speed-dependent reduction CARLA applies on top of the geometry (roadmap 014).
        // Non-finite points are dropped rather than let into the interpolation.
        let mut steering_curve: Vec<(f32, f32)> = physics
            .steering_curve
            .iter()
            .map(|p| (p.x, p.y))
            .filter(|(x, y)| x.is_finite() && y.is_finite())
            .collect();
        steering_curve.sort_by(|a, b| a.0.total_cmp(&b.0));
        if steering_curve.is_empty() {
            tracing::info!("CARLA reported no steering_curve; steering without speed compensation");
            steering_curve = FLAT_STEERING_CURVE.to_vec();
        }

        let geometry = SteerGeometry {
            max_steer_angle: max_degrees.to_radians(),
            track_over_wheelbase,
            steering_curve,
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
        geometry: &SteerGeometry,
        steering_multiplier: f32,
        compensate_steering_curve: bool,
        steer_dynamics: &SteerDynamics,
        longitudinal: Option<&crate::longitudinal_map::LongitudinalCalibration>,
        trace: Option<&crate::control_trace::ControlTrace>,
        stall: &Mutex<StallWatch>,
        received: std::time::Instant,
        now_sim_s: f64,
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
            let tire_angle = steer_dynamics_step(
                steer_dynamics,
                state.steer_actuator,
                cmd.lateral.steering_tire_angle,
                now_sim_s,
            );
            state.steer_actuator = Some((tire_angle, now_sim_s));
            (*state, tire_angle)
        };
        let (applied, tire_angle) = applied;

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
            //
            // Then divide by CARLA's steering curve at the current speed: the inverse above
            // is exact only at standstill, and CARLA scales the achieved angle down with
            // speed (0.9 at 20 km/h, 0.8 at 60 on the Tesla -- roadmap 014, "Steering").
            // Indexed by the same body-frame longitudinal speed the VelocityReport carries.
            // A failed read costs the compensation for one command, not the command.
            let longitudinal_mps = v
                .transform()
                .and_then(|t| {
                    v.velocity()
                        .map(|vel| t.rotation.inverse_rotate_vector(&vel).x)
                })
                .unwrap_or(0.0);
            let factor = steer_speed_factor(geometry, compensate_steering_curve, longitudinal_mps);
            let (steer, clamped) = compensate_steer(
                -steer_command_for(tire_angle, geometry) * steering_multiplier,
                factor,
            );
            if clamped {
                tracing::debug!(
                    "Steer command saturated: {:.3} rad at {:.1} km/h needs more than full \
                     lock after the steering-curve factor {:.3}",
                    cmd.lateral.steering_tire_angle,
                    longitudinal_mps.abs() * 3.6,
                    factor
                );
            }
            control.steer = steer;

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
            if hold_with_handbrake(
                applied.is_park(),
                cmd.longitudinal.velocity as f64,
                accel as f64,
                speed,
            ) {
                control.hand_brake = true;
                if applied.is_park() {
                    control.throttle = 0.0;
                }
            }

            // The two stages the bridge owns: turning the command into a CARLA control, and
            // the RPC that delivers it. Recorded rather than inferred -- see control_trace.
            let converted = std::time::Instant::now();
            v.apply_control(&control)?;

            // Did that command actually do anything? Everything needed to answer is in
            // scope here and nowhere else: what Autoware asked for, what pedal it became,
            // and how fast the actor is going. Nothing intervenes -- a stuck vehicle is
            // not this bridge's to free -- but a run that spends two minutes applying
            // throttle to a stationary car should not do it silently.
            let stuck_now = !applied.is_park()
                && !applied.is_neutral()
                && !control.hand_brake
                && control.throttle >= STALL_MIN_THROTTLE
                && control.brake <= 0.01
                && (cmd.longitudinal.velocity.abs() as f64) > STALL_SPEED_MPS
                && speed < STALL_SPEED_MPS;
            let moving = speed >= STALL_SPEED_MPS;
            if let Some(event) = stall.lock().unwrap().observe(converted, stuck_now, moving) {
                match event {
                    StallEvent::Stuck { seconds } => tracing::warn!(
                        "Commanded to move but stationary for {seconds:.0}s: throttle \
                         {:.3}, no brake, gear {}, asked for {:.2} m/s, measured \
                         {:.2} m/s. The command path is doing its job, so the vehicle is \
                         being held by the simulation -- wedged geometry is what this was \
                         when it was traced (docs/issues/016).",
                        control.throttle,
                        applied.gear,
                        cmd.longitudinal.velocity,
                        speed
                    ),
                    StallEvent::Recovered { seconds } => {
                        tracing::info!("Moving again after {seconds:.0}s commanded-but-stationary")
                    }
                }
            }
            if let Some(t) = trace {
                let stamp = cmd.stamp.sec as f64 + cmd.stamp.nanosec as f64 * 1e-9;
                // Both in simulation time, so the difference is the command's real staleness
                // when the bridge finally acted on it -- queueing included.
                let age_ms = (now_sim_s - stamp) * 1e3;
                t.record(
                    stamp,
                    received,
                    converted,
                    std::time::Instant::now(),
                    age_ms,
                );
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
        // CARLA 0.10.0 raises `std::exception` from every `get_wheel_steer_angle` call
        // (carla-scenario-bridge roadmap 019), so skip the two failing RPCs per frame and
        // report the command through the steering model, as the error path below would.
        #[cfg(carla_0100)]
        {
            let _ = vehicle;
            -effective_tire_angle(commanded_steer, &self.steer_geometry)
        }
        #[cfg(not(carla_0100))]
        {
            // `VehicleWheelLocation` is an autocxx-generated POD without `Copy`, so the two
            // wheels are read one at a time rather than through an array.
            let front = vehicle
                .wheel_steer_angle(carla::rpc::VehicleWheelLocation::FL_Wheel)
                .and_then(|fl| {
                    vehicle
                        .wheel_steer_angle(carla::rpc::VehicleWheelLocation::FR_Wheel)
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
    }

    /// Publish vehicle status to Autoware
    ///
    /// Report the vehicle at rest, once, as the last word on a vehicle that has gone.
    ///
    /// Called when the scenario runner has despawned the ego. Without it the last report
    /// Autoware holds is the ego's final speed -- scenarios end on a position condition,
    /// usually at 3 m/s -- and a long-lived stack carries that into the next scenario: its
    /// first /clock tick resumes an EKF still moving at that speed while the new ego sits
    /// still, which the pose-instability detector reports as an ERROR before
    /// localization is re-initialized. Stamped with the last frame time the bridge saw for
    /// the vehicle, so it lands ahead of the next scenario's first tick.
    pub fn publish_standstill(&self, stamp: &builtin_interfaces::msg::Time) -> Result<()> {
        self.velocity_pub
            .publish(&autoware_vehicle_msgs::msg::VelocityReport {
                header: std_msgs::msg::Header {
                    stamp: stamp.clone(),
                    frame_id: "base_link".to_string(),
                },
                longitudinal_velocity: 0.0,
                lateral_velocity: 0.0,
                heading_rate: 0.0,
            })?;
        Ok(())
    }

    /// Should be called in the main loop at regular intervals (e.g., 20 Hz)
    ///
    /// # Arguments
    /// * `stamp` - the frame's CARLA time plus the episode epoch (`SimClock::stamp`)
    pub fn publish_status(&self, stamp: &builtin_interfaces::msg::Time) -> Result<()> {
        let vehicle_guard = self.vehicle.lock().unwrap();
        if let Some(ref vehicle) = *vehicle_guard {
            let ros_timestamp = stamp.clone();

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
            // CARLA reports angular velocity in DEGREES per second (measured, see
            // docs/issues/008); the cross product below and heading_rate want rad/s.
            let body_angular = transform
                .rotation
                .inverse_rotate_vector(&angular_velocity_vec);
            let body_angular = nalgebra::Vector3::new(
                (body_angular.x as f64).to_radians(),
                (body_angular.y as f64).to_radians(),
                (body_angular.z as f64).to_radians(),
            );
            // CARLA measures at the actor origin; base_link is the rear axle, 1.386 m behind
            // it on a model3. Longitudinal speed is the same at both, but lateral velocity
            // differs by yaw_rate x offset -- published uncorrected, the rear axle appeared
            // to slide sideways in every turn. Same correction as the odometry twist in
            // `AutowareCoordinator::tick`; see `coordinate_conversion::velocity_at_offset`
            // and roadmap 014.
            let base_link_velocity = crate::coordinate_conversion::velocity_at_offset(
                &nalgebra::Vector3::new(
                    body_velocity.x as f64,
                    body_velocity.y as f64,
                    body_velocity.z as f64,
                ),
                &body_angular,
                &self.base_link_in_actor,
            );
            let longitudinal_velocity = base_link_velocity.x as f32;
            // Negate: CARLA body Y = right (left-handed), ROS lateral = left (right-handed)
            let lateral_velocity = -base_link_velocity.y as f32;
            // Negate: CARLA Z angular = clockwise, ROS yaw rate = counter-clockwise. The
            // same at every point of a rigid body, so no offset term. Body-frame z rather
            // than world z, which differ only on a slope.
            let heading_rate = -body_angular.z as f32;

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
                    // The command is multiplied back by the factor it was divided by -- which is
                    // where CARLA applies the curve, before Ackermann -- so the echo stays what
                    // Autoware asked for rather than the pre-distorted command.
                    -effective_tire_angle(
                        control.steer
                            * steer_speed_factor(
                                &self.steer_geometry,
                                self.compensate_steering_curve,
                                longitudinal_velocity,
                            ),
                        &self.steer_geometry,
                    )
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

    #[test]
    fn handbrake_waits_for_the_car_to_stop() {
        // Autoware reaches velocity 0 while the car still rolls: service brake only.
        assert!(!hold_with_handbrake(false, 0.0, -3.4, 0.8));
        assert!(!hold_with_handbrake(false, 0.0, -0.4, 0.1));
        // At rest under a standstill command: hold.
        assert!(hold_with_handbrake(false, 0.0, -0.4, 0.05));
        assert!(hold_with_handbrake(false, 0.0, 0.0, 0.0));
        // Moving or accelerating commands never hold; PARK always does.
        assert!(!hold_with_handbrake(false, 1.0, -0.4, 0.0));
        assert!(!hold_with_handbrake(false, 0.0, 0.5, 0.0));
        assert!(hold_with_handbrake(true, 3.0, 1.0, 5.0));
    }

    fn dyn_(rate_deg_s: f32, tau: f32) -> SteerDynamics {
        SteerDynamics {
            rate_limit: rate_deg_s.to_radians(),
            time_constant: tau,
        }
    }

    #[test]
    fn steer_dynamics_off_or_first_command_passes_through() {
        assert_eq!(
            steer_dynamics_step(&SteerDynamics::default(), Some((0.0, 1.0)), 0.3, 1.05),
            0.3
        );
        assert_eq!(steer_dynamics_step(&dyn_(20.0, 0.2), None, 0.3, 1.0), 0.3);
    }

    #[test]
    fn steer_rate_limit_caps_the_step() {
        // 20 deg/s for 0.05 s = 1 deg.
        let next = steer_dynamics_step(&dyn_(20.0, 0.0), Some((0.0, 10.0)), 0.5, 10.05);
        assert!((next - 1f32.to_radians()).abs() < 1e-6, "{next}");
        let back = steer_dynamics_step(&dyn_(20.0, 0.0), Some((0.0, 10.0)), -0.5, 10.05);
        assert!((back + 1f32.to_radians()).abs() < 1e-6, "{back}");
        // A small change inside the limit lands exactly.
        assert_eq!(
            steer_dynamics_step(&dyn_(20.0, 0.0), Some((0.0, 10.0)), 0.01, 10.05),
            0.01
        );
    }

    #[test]
    fn steer_lag_reaches_63_percent_after_one_time_constant() {
        let d = dyn_(0.0, 0.2);
        let (mut angle, mut t) = (0.0f32, 0.0f64);
        for _ in 0..4 {
            angle = steer_dynamics_step(&d, Some((angle, t)), 0.1, t + 0.05);
            t += 0.05;
        }
        let expected = 0.1 * (1.0 - (-1.0f32).exp());
        assert!((angle - expected).abs() < 1e-5, "{angle} vs {expected}");
    }

    #[test]
    fn steer_dynamics_bounds_a_long_gap_and_ignores_time_going_back() {
        let d = dyn_(20.0, 0.0);
        let after_pause = steer_dynamics_step(&d, Some((0.0, 0.0)), 1.0, 600.0);
        assert!((after_pause - (20f32.to_radians() * MAX_STEER_DYNAMICS_DT as f32)).abs() < 1e-6);
        assert_eq!(steer_dynamics_step(&d, Some((0.2, 5.0)), 1.0, 4.0), 0.2);
    }
    use autoware_vehicle_msgs::msg::{GearReport, HazardLightsReport, TurnIndicatorsReport};

    /// Matches stock `autoware_universe.cpp`: only AUTONOMOUS succeeds.
    #[test]
    fn control_mode_request_matches_the_stock_ego_simulation() {
        use autoware_vehicle_msgs::srv::control_mode_command::ControlModeCommand_Request as Req;
        assert!(control_mode_request_accepted(Req::AUTONOMOUS));
        assert!(!control_mode_request_accepted(Req::MANUAL));
        assert!(!control_mode_request_accepted(Req::NO_COMMAND));
        assert!(!control_mode_request_accepted(Req::AUTONOMOUS_STEER_ONLY));
        assert!(!control_mode_request_accepted(
            Req::AUTONOMOUS_VELOCITY_ONLY
        ));
        assert!(!control_mode_request_accepted(200));
        assert_eq!(control_mode_name(Req::MANUAL), "MANUAL");
        assert_eq!(control_mode_name(200), "UNKNOWN");
    }

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
            steering_curve: TESLA_STEERING_CURVE.to_vec(),
        }
    }

    /// `vehicle.tesla.model3`'s steering_curve on 0.9.16, (km/h, factor). Roadmap 014.
    const TESLA_STEERING_CURVE: [(f32, f32); 4] =
        [(0.0, 1.0), (20.0, 0.9), (60.0, 0.8), (120.0, 0.7)];

    /// Interpolation against the measured curve, including between points (10, 40) and
    /// beyond the last (200), where CARLA holds the end value.
    #[test]
    fn steering_curve_factor_matches_the_measured_curve() {
        let cases = [
            (0.0, 1.0),
            (10.0, 0.95),
            (20.0, 0.9),
            (40.0, 0.85),
            (60.0, 0.8),
            (120.0, 0.7),
            (200.0, 0.7),
        ];
        for (kmh, expected) in cases {
            let got = steering_curve_factor(&TESLA_STEERING_CURVE, kmh);
            assert!(
                (got - expected).abs() < 1e-5,
                "{kmh} km/h: factor {got}, expected {expected}"
            );
        }
        // Below the first point holds the first value.
        assert_eq!(steering_curve_factor(&TESLA_STEERING_CURVE, -5.0), 1.0);
    }

    #[test]
    fn steering_curve_factor_sorts_and_tolerates_degenerate_curves() {
        let shuffled = [(60.0, 0.8), (0.0, 1.0), (120.0, 0.7), (20.0, 0.9)];
        assert!((steering_curve_factor(&shuffled, 40.0) - 0.85).abs() < 1e-5);
        assert_eq!(steering_curve_factor(&[], 40.0), 1.0);
        assert_eq!(steering_curve_factor(&FLAT_STEERING_CURVE, 80.0), 1.0);
    }

    /// Reversing is reduced like driving forward: the curve is indexed by |speed|.
    #[test]
    fn speed_factor_uses_the_speed_magnitude_in_kmh() {
        let g = tesla();
        // 40 km/h = 11.11 m/s.
        let fwd = steer_speed_factor(&g, true, 40.0 / 3.6);
        let rev = steer_speed_factor(&g, true, -40.0 / 3.6);
        assert!((fwd - 0.85).abs() < 1e-4);
        assert_eq!(fwd, rev);
    }

    /// The command is pre-distorted by 1/factor, so CARLA's own multiplication lands the
    /// wheels on the requested angle; past full lock it saturates and says so.
    #[test]
    fn compensation_divides_and_clamps() {
        let g = tesla();
        let requested = 0.30_f32;
        let base = steer_command_for(requested, &g);
        let factor = steer_speed_factor(&g, true, 40.0 / 3.6);
        let (cmd, clamped) = compensate_steer(base, factor);
        assert!(!clamped);
        assert!((cmd - base / 0.85).abs() < 1e-4);
        // What CARLA delivers: the curve scales the steer input, and Ackermann is applied
        // to the result -- so dividing the command by the factor is exact, not approximate.
        let delivered = effective_tire_angle(cmd * factor, &g);
        assert!(
            (delivered - requested).abs() < 1e-3,
            "delivered {delivered:.4} for {requested:.4}"
        );

        assert_eq!(compensate_steer(0.9, 0.8), (1.0, true));
        assert_eq!(compensate_steer(-0.9, 0.8), (-1.0, true));
        // A factor that is not a positive number leaves the command alone.
        assert_eq!(compensate_steer(0.4, 0.0), (0.4, false));
        assert_eq!(compensate_steer(0.4, f32::NAN), (0.4, false));
    }

    /// Switched off, the factor is 1.0 at every speed and the command is untouched.
    #[test]
    fn disabled_compensation_is_the_identity() {
        let g = tesla();
        for kmh in [0.0_f32, 20.0, 40.0, 120.0] {
            let factor = steer_speed_factor(&g, false, kmh / 3.6);
            assert_eq!(factor, 1.0);
            for steer in [-1.0_f32, -0.3, 0.0, 0.45, 1.0] {
                assert_eq!(compensate_steer(steer, factor), (steer, false));
            }
        }
    }

    /// At standstill compensation is a no-op, so the at-rest measurements still hold.
    #[test]
    fn compensation_is_a_noop_at_standstill() {
        assert_eq!(steer_speed_factor(&tesla(), true, 0.0), 1.0);
    }

    /// Wheel angles measured off a live server by `scripts/probe_steer_curve.py`, at rest.
    ///
    /// The model has to reproduce the *mean* of the two front wheels, which is the angle
    /// the vehicle actually turns at -- not the wheel limit the command used to be scaled
    /// against.
    #[test]
    #[cfg(not(carla_0100))] // 0.9.16 PhysX steering is linear in the command
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
            assert!(
                (back - cmd).abs() < 1e-3,
                "cmd {cmd} round tripped to {back}"
            );
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
    #[cfg(not(carla_0100))] // 0.9.16 PhysX steering is linear in the command
    fn zero_ackermann_term_is_the_old_linear_mapping() {
        let g = SteerGeometry {
            max_steer_angle: 1.22,
            track_over_wheelbase: 0.0,
            ..Default::default()
        };
        assert!((steer_command_for(0.61, &g) - 0.5).abs() < 1e-4);
    }

    /// CARLA 0.10 (Chaos): the delivered angle is quadratic in the command, so a quarter
    /// of the limit needs half the command, and the inverse round-trips.
    #[test]
    #[cfg(carla_0100)]
    fn chaos_steering_is_quadratic_in_the_command() {
        let g = SteerGeometry {
            max_steer_angle: 1.22,
            track_over_wheelbase: 0.0,
            ..Default::default()
        };
        assert!((effective_tire_angle(0.5, &g) - 0.305).abs() < 1e-4);
        assert!((steer_command_for(0.305, &g) - 0.5).abs() < 1e-4);
        assert!((steer_command_for(-0.305, &g) + 0.5).abs() < 1e-4);
        for cmd in [0.05_f32, 0.2, 0.7, 1.0] {
            let back = steer_command_for(effective_tire_angle(cmd, &g), &g);
            assert!((back - cmd).abs() < 1e-3, "{cmd} -> {back}");
        }
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

#[cfg(test)]
mod stall_watch_tests {
    use super::*;

    /// A fixed origin plus offsets, so the tests do not wait out real seconds.
    fn at(origin: Instant, secs: u64) -> Instant {
        origin + Duration::from_secs(secs)
    }

    #[test]
    fn a_moving_vehicle_says_nothing() {
        let t0 = Instant::now();
        let mut w = StallWatch::default();
        for s in 0..60 {
            assert_eq!(w.observe(at(t0, s), false, true), None);
        }
    }

    #[test]
    fn a_standing_start_is_not_a_stall() {
        let t0 = Instant::now();
        let mut w = StallWatch::default();
        // Stuck, but for less than STALL_AFTER: every vehicle is briefly here when it
        // pulls away, and reporting that would make the warning worthless.
        for s in 0..STALL_AFTER.as_secs() {
            assert_eq!(w.observe(at(t0, s), true, false), None);
        }
        assert_eq!(
            w.observe(at(t0, 1), false, true),
            None,
            "recovered before reporting"
        );
    }

    #[test]
    fn a_real_stall_reports_once_then_repeats_on_the_interval() {
        let t0 = Instant::now();
        let mut w = StallWatch::default();
        assert_eq!(w.observe(t0, true, false), None);
        let first = w.observe(at(t0, STALL_AFTER.as_secs()), true, false);
        assert!(matches!(first, Some(StallEvent::Stuck { .. })), "{first:?}");

        // Silent until the interval has passed, however many commands arrive.
        for s in STALL_AFTER.as_secs() + 1..STALL_AFTER.as_secs() + STALL_WARN_INTERVAL.as_secs() {
            assert_eq!(w.observe(at(t0, s), true, false), None, "at {s}s");
        }
        let again = w.observe(
            at(t0, STALL_AFTER.as_secs() + STALL_WARN_INTERVAL.as_secs()),
            true,
            false,
        );
        assert!(matches!(again, Some(StallEvent::Stuck { .. })), "{again:?}");
    }

    #[test]
    fn recovery_is_reported_only_after_a_stall_was() {
        let t0 = Instant::now();
        let mut w = StallWatch::default();
        w.observe(t0, true, false);
        w.observe(at(t0, STALL_AFTER.as_secs()), true, false);
        let recovered = w.observe(at(t0, STALL_AFTER.as_secs() + 3), false, true);
        match recovered {
            Some(StallEvent::Recovered { seconds }) => {
                assert!(
                    (seconds - (STALL_AFTER.as_secs_f64() + 3.0)).abs() < 0.5,
                    "{seconds}"
                )
            }
            other => panic!("expected recovery, got {other:?}"),
        }
        // And the watcher is clean again: the next quiet command says nothing.
        assert_eq!(
            w.observe(at(t0, STALL_AFTER.as_secs() + 4), false, true),
            None
        );
    }

    #[test]
    fn the_command_giving_up_is_not_recovery() {
        let t0 = Instant::now();
        let mut w = StallWatch::default();
        w.observe(t0, true, false);
        assert!(matches!(
            w.observe(at(t0, STALL_AFTER.as_secs()), true, false),
            Some(StallEvent::Stuck { .. })
        ));
        // The stuck condition ends because nothing asks for motion any more, while the
        // vehicle is still exactly where it was. Announcing recovery here would be a lie
        // -- it is what the bridge did when this was first injected for real.
        assert_eq!(
            w.observe(at(t0, STALL_AFTER.as_secs() + 1), false, false),
            None
        );
    }

    #[test]
    fn the_clock_restarts_when_the_vehicle_moves_between_stalls() {
        let t0 = Instant::now();
        let mut w = StallWatch::default();
        for s in 0..STALL_AFTER.as_secs() {
            w.observe(at(t0, s), true, false);
        }
        // One moving command resets the run, so the next stall has to earn its own
        // STALL_AFTER rather than inheriting the previous one's.
        assert_eq!(w.observe(at(t0, STALL_AFTER.as_secs()), false, true), None);
        assert_eq!(
            w.observe(at(t0, STALL_AFTER.as_secs() + 1), true, false),
            None
        );
    }
}
