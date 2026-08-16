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
    _turn_indicators_sub:
        Arc<rclrs::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>>,
    _hazard_lights_sub: Arc<rclrs::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>>,

    // CARLA vehicle reference (shared with main loop)
    vehicle: Arc<Mutex<Option<Vehicle>>>,

    /// What was last applied, shared with the command callbacks.
    state: Arc<Mutex<AppliedState>>,

    /// Physical steering limit of the spawned vehicle, radians.
    max_steer_angle: f32,
}

impl VehicleControlBridge {
    /// Create a new vehicle control bridge
    ///
    /// # Arguments
    /// * `node` - ROS node for creating publishers/subscribers
    /// * `vehicle` - Arc<Mutex<Option<Vehicle>>> shared with main loop
    pub fn new(node: rclrs::Node, vehicle: Arc<Mutex<Option<Vehicle>>>) -> Result<Self> {
        let max_steer_angle = Self::read_max_steer_angle(&vehicle);

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
        let control_sub = Arc::new(node.create_subscription(
            "/control/command/control_cmd".reliable(),
            move |msg: autoware_control_msgs::msg::Control| {
                if let Err(e) = Self::apply_control_command(
                    &vehicle_for_control,
                    &state_for_control,
                    max_steer_angle,
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
        tracing::info!("  Max steering tire angle: {:.3} rad", max_steer_angle);
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
            _turn_indicators_sub: turn_indicators_sub,
            _hazard_lights_sub: hazard_lights_sub,
            vehicle,
            state,
            max_steer_angle,
        })
    }

    /// Read the spawned vehicle's physical steering limit from CARLA.
    ///
    /// `vehicle_config.yaml` offers several blueprints and each has its own limit;
    /// hardcoding the Tesla's 70 degrees scales every steering command by a constant
    /// factor on anything else. See `docs/issues/006-*`.
    fn read_max_steer_angle(vehicle: &Arc<Mutex<Option<Vehicle>>>) -> f32 {
        let guard = vehicle.lock().unwrap();
        let Some(vehicle) = guard.as_ref() else {
            tracing::warn!(
                "No vehicle when reading physics control; using the fall back max steer \
                 angle of {FALLBACK_MAX_STEER_ANGLE:.3} rad"
            );
            return FALLBACK_MAX_STEER_ANGLE;
        };

        match vehicle.physics_control() {
            Ok(physics) => {
                // CARLA reports max_steer_angle per wheel, in degrees. Only the steered
                // wheels carry a non-zero value, so the maximum over all of them is the
                // vehicle's steering limit without having to know the axle layout.
                let max_degrees = physics
                    .wheels
                    .iter()
                    .map(|w| w.max_steer_angle)
                    .fold(0.0_f32, f32::max);

                if max_degrees > 0.0 {
                    let radians = max_degrees.to_radians();
                    tracing::info!(
                        "Max steering tire angle from CARLA physics: {:.1} deg ({:.3} rad)",
                        max_degrees,
                        radians
                    );
                    radians
                } else {
                    tracing::warn!(
                        "CARLA reported no steerable wheel; using the fall back max steer \
                         angle of {FALLBACK_MAX_STEER_ANGLE:.3} rad"
                    );
                    FALLBACK_MAX_STEER_ANGLE
                }
            }
            Err(e) => {
                tracing::warn!(
                    "Failed to read CARLA physics control ({e}); using the fall back max \
                     steer angle of {FALLBACK_MAX_STEER_ANGLE:.3} rad"
                );
                FALLBACK_MAX_STEER_ANGLE
            }
        }
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
        max_steer_angle: f32,
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

            // Steering: convert tire angle (rad) to normalized (-1 to 1)
            // Negate: Autoware positive = left turn (ROS), CARLA positive = right turn (left-handed)
            control.steer = (-cmd.lateral.steering_tire_angle / max_steer_angle).clamp(-1.0, 1.0);

            // Longitudinal: acceleration (m/s²) → throttle or brake (0 to 1).
            if accel > 0.01 {
                // Neutral has no drive torque, as in a real gearbox. Braking is never
                // suppressed: a deceleration command must reach the brakes whatever gear
                // is selected.
                if !applied.is_neutral() {
                    control.throttle = (accel / MAX_ACCEL).clamp(0.0, 1.0);
                }
                control.brake = 0.0;
            } else if accel < -0.01 {
                control.throttle = 0.0;
                control.brake = (-accel / MAX_ACCEL).clamp(0.0, 1.0);
            }

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

            v.apply_control(&control)?;

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
                -commanded_steer * self.max_steer_angle
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
                steering_tire_angle: self.measured_steering_angle(vehicle, control.steer),
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
}
