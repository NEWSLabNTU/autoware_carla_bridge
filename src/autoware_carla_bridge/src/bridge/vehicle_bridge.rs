use std::sync::{atomic::Ordering, Arc};

use arc_swap::ArcSwap;
use atomic_float::AtomicF32;
use carla::{
    client::{ActorBase, Vehicle},
    rpc::{VehicleControl, VehicleWheelLocation},
};
use interp::{interp, InterpMode};

use super::actor_bridge::{ActorBridge, BridgeType};
use crate::{
    autoware::Autoware,
    error::{BridgeError, Result},
    utils,
};

pub struct VehicleBridge {
    vehicle_name: String,
    actor: Vehicle,

    // Publishers
    publisher_actuation: Arc<rclrs::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>>,
    publisher_velocity: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::VelocityReport>>,
    publisher_steer: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::SteeringReport>>,
    publisher_gear: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::GearReport>>,
    publisher_control: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>>,
    publisher_turnindicator:
        Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>>,
    publisher_hazardlight: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>>,

    // Subscriptions (kept alive)
    _subscription_actuation_cmd:
        Arc<rclrs::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>>,
    _subscription_gear_cmd: Arc<rclrs::Subscription<autoware_vehicle_msgs::msg::GearCommand>>,
    _subscription_gate_mode: Arc<rclrs::Subscription<tier4_control_msgs::msg::GateMode>>,
    _subscription_turnindicator_cmd:
        Arc<rclrs::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>>,
    _subscription_hazardlight_cmd:
        Arc<rclrs::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>>,

    // Shared state
    velocity: Arc<AtomicF32>,
    current_actuation_cmd: Arc<ArcSwap<tier4_vehicle_msgs::msg::ActuationCommandStamped>>,
    current_gear: Arc<ArcSwap<u8>>,
    current_gate_mode: Arc<ArcSwap<tier4_control_msgs::msg::GateMode>>,

    // Control state
    tau: f32,
    prev_timestamp: Option<f64>,
    prev_steer_output: f32,
}

impl VehicleBridge {
    pub fn get_bridge_type(actor: Vehicle) -> Result<BridgeType> {
        let mut vehicle_name = actor
            .attributes()
            .iter()
            .find(|attr| attr.id() == "role_name")
            .ok_or(BridgeError::CarlaIssue(
                "Unable to find role_name in the vehicle",
            ))?
            .value_string();

        // Remove "autoware_" in role name
        if !vehicle_name.starts_with("autoware_") {
            return Err(BridgeError::Npc {
                npc_role_name: vehicle_name,
            });
        }
        vehicle_name = vehicle_name.replace("autoware_", "");

        log::info!("Detect a vehicle {vehicle_name}");

        Ok(BridgeType::Vehicle(vehicle_name))
    }

    pub fn new(
        node: rclrs::Node,
        actor: Vehicle,
        bridge_type: BridgeType,
        autoware: &Autoware,
    ) -> Result<VehicleBridge> {
        let vehicle_name = match bridge_type {
            BridgeType::Vehicle(v) => v,
            _ => panic!("Should never happen!"),
        };

        // Create publishers
        let publisher_actuation = Arc::new(
            node.create_publisher::<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
                &autoware.topic_actuation_status,
            )?,
        );
        let publisher_velocity = Arc::new(
            node.create_publisher::<autoware_vehicle_msgs::msg::VelocityReport>(
                &autoware.topic_velocity_status,
            )?,
        );
        let publisher_steer = Arc::new(
            node.create_publisher::<autoware_vehicle_msgs::msg::SteeringReport>(
                &autoware.topic_steering_status,
            )?,
        );
        let publisher_gear = Arc::new(
            node.create_publisher::<autoware_vehicle_msgs::msg::GearReport>(
                &autoware.topic_gear_status,
            )?,
        );
        let publisher_control = Arc::new(
            node.create_publisher::<autoware_vehicle_msgs::msg::ControlModeReport>(
                &autoware.topic_control_mode,
            )?,
        );
        let publisher_turnindicator = Arc::new(
            node.create_publisher::<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
                &autoware.topic_turn_indicators_status,
            )?,
        );
        let publisher_hazardlight = Arc::new(
            node.create_publisher::<autoware_vehicle_msgs::msg::HazardLightsReport>(
                &autoware.topic_hazard_lights_status,
            )?,
        );

        let velocity = Arc::new(AtomicF32::new(0.0));

        // Initialize shared state for actuation commands
        let current_actuation_cmd = Arc::new(ArcSwap::from_pointee(
            tier4_vehicle_msgs::msg::ActuationCommandStamped {
                header: std_msgs::msg::Header {
                    stamp: builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
                    frame_id: "".to_string(),
                },
                actuation: tier4_vehicle_msgs::msg::ActuationCommand {
                    accel_cmd: 0.0,
                    brake_cmd: 0.0,
                    steer_cmd: 0.0,
                },
            },
        ));

        // Create subscription for actuation commands
        let cloned_cmd = current_actuation_cmd.clone();
        let subscription_actuation_cmd = Arc::new(node.create_subscription(
            &autoware.topic_actuation_cmd,
            move |cmd: tier4_vehicle_msgs::msg::ActuationCommandStamped| {
                cloned_cmd.store(Arc::new(cmd));
            },
        )?);

        // Initialize shared state for gear commands
        let current_gear = Arc::new(ArcSwap::from_pointee(
            autoware_vehicle_msgs::msg::GearReport::NONE,
        ));
        let cloned_gear = current_gear.clone();
        let subscription_gear_cmd = Arc::new(node.create_subscription(
            &autoware.topic_gear_cmd,
            move |cmd: autoware_vehicle_msgs::msg::GearCommand| {
                cloned_gear.store(Arc::new(cmd.command));
            },
        )?);

        // Initialize shared state for gate mode
        let current_gate_mode =
            Arc::new(ArcSwap::from_pointee(tier4_control_msgs::msg::GateMode {
                data: tier4_control_msgs::msg::GateMode::AUTO,
            }));
        let cloned_gate_mode = current_gate_mode.clone();
        let subscription_gate_mode = Arc::new(node.create_subscription(
            &autoware.topic_current_gate_mode,
            move |mode: tier4_control_msgs::msg::GateMode| {
                cloned_gate_mode.store(Arc::new(mode));
            },
        )?);

        // Subscription for turn indicators (not implemented yet)
        let subscription_turnindicator_cmd = Arc::new(node.create_subscription(
            &autoware.topic_turn_indicators_cmd,
            move |_cmd: autoware_vehicle_msgs::msg::TurnIndicatorsCommand| {
                // TODO: Not supported yet
            },
        )?);

        // Subscription for hazard lights (not implemented yet)
        let subscription_hazardlight_cmd = Arc::new(node.create_subscription(
            &autoware.topic_hazard_lights_cmd,
            move |_cmd: autoware_vehicle_msgs::msg::HazardLightsCommand| {
                // TODO: Not supported yet
            },
        )?);

        Ok(VehicleBridge {
            vehicle_name,
            actor,
            publisher_actuation,
            publisher_velocity,
            publisher_steer,
            publisher_gear,
            publisher_control,
            publisher_turnindicator,
            publisher_hazardlight,
            _subscription_actuation_cmd: subscription_actuation_cmd,
            _subscription_gear_cmd: subscription_gear_cmd,
            _subscription_gate_mode: subscription_gate_mode,
            _subscription_turnindicator_cmd: subscription_turnindicator_cmd,
            _subscription_hazardlight_cmd: subscription_hazardlight_cmd,
            velocity,
            current_actuation_cmd,
            current_gear,
            current_gate_mode,
            tau: 0.2,
            prev_timestamp: None,
            prev_steer_output: 0.0,
        })
    }

    fn pub_current_actuation(&mut self, timestamp: f64) -> Result<()> {
        let control = self.actor.control();
        let mut header = utils::create_ros_header(Some(timestamp));
        header.frame_id = String::from("base_link");

        let actuation_msg = tier4_vehicle_msgs::msg::ActuationStatusStamped {
            header,
            status: tier4_vehicle_msgs::msg::ActuationStatus {
                accel_status: control.throttle as f64,
                brake_status: control.brake as f64,
                steer_status: -control.steer as f64,
            },
        };

        log::debug!(
            "Carla => Autoware: accel_status={:.3}, brake_status={:.3}, steer_status={:.3}",
            control.throttle,
            control.brake,
            -control.steer
        );

        self.publisher_actuation.publish(&actuation_msg)?;
        Ok(())
    }

    fn pub_current_velocity(&mut self, timestamp: f64) -> Result<()> {
        let velocity = self.actor.velocity();
        let mut header = utils::create_ros_header(Some(timestamp));
        header.frame_id = String::from("base_link");

        let velocity_msg = autoware_vehicle_msgs::msg::VelocityReport {
            header,
            // Since the velocity report from Carla is always positive, we need to check reverse.
            longitudinal_velocity: if self.actor.control().reverse {
                -velocity.norm()
            } else {
                velocity.norm()
            },
            lateral_velocity: 0.0,
            heading_rate: -self
                .actor
                .wheel_steer_angle(VehicleWheelLocation::FL_Wheel)
                .to_radians(),
        };

        log::debug!(
            "Carla => Autoware: current velocity: {}",
            velocity_msg.longitudinal_velocity
        );

        self.publisher_velocity.publish(&velocity_msg)?;
        self.velocity
            .store(velocity_msg.longitudinal_velocity, Ordering::Relaxed);

        Ok(())
    }

    fn pub_current_steer(&mut self, timestamp: f64) -> Result<()> {
        let steer_msg = autoware_vehicle_msgs::msg::SteeringReport {
            stamp: builtin_interfaces::msg::Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1_000_000_000_f64) as u32,
            },
            steering_tire_angle: -self
                .actor
                .wheel_steer_angle(VehicleWheelLocation::FL_Wheel)
                .to_radians(),
        };

        self.publisher_steer.publish(&steer_msg)?;
        Ok(())
    }

    fn pub_current_gear(&mut self, timestamp: f64) -> Result<()> {
        let gear_msg = autoware_vehicle_msgs::msg::GearReport {
            stamp: builtin_interfaces::msg::Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1_000_000_000_f64) as u32,
            },
            report: **self.current_gear.load(),
        };

        self.publisher_gear.publish(&gear_msg)?;
        Ok(())
    }

    fn pub_current_control(&mut self, timestamp: f64) -> Result<()> {
        let mode = if self.current_gate_mode.load().data == tier4_control_msgs::msg::GateMode::AUTO
        {
            autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS
        } else {
            autoware_vehicle_msgs::msg::ControlModeReport::MANUAL
        };

        let control_msg = autoware_vehicle_msgs::msg::ControlModeReport {
            stamp: builtin_interfaces::msg::Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1_000_000_000_f64) as u32,
            },
            mode,
        };

        self.publisher_control.publish(&control_msg)?;
        Ok(())
    }

    fn pub_current_indicator(&mut self, timestamp: f64) -> Result<()> {
        // TODO: Not support yet
        let turnindicator_msg = autoware_vehicle_msgs::msg::TurnIndicatorsReport {
            stamp: builtin_interfaces::msg::Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1_000_000_000_f64) as u32,
            },
            report: autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE,
        };

        self.publisher_turnindicator.publish(&turnindicator_msg)?;
        Ok(())
    }

    fn pub_hazard_light(&mut self, timestamp: f64) -> Result<()> {
        // TODO: Not support yet
        let hazardlight_msg = autoware_vehicle_msgs::msg::HazardLightsReport {
            stamp: builtin_interfaces::msg::Time {
                sec: timestamp.floor() as i32,
                nanosec: (timestamp.fract() * 1_000_000_000_f64) as u32,
            },
            report: autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE,
        };

        self.publisher_hazardlight.publish(&hazardlight_msg)?;
        Ok(())
    }

    fn first_order_steering(&mut self, steer_input: f32, now_ts: f64) -> f32 {
        let mut out = self.prev_steer_output;
        if let Some(prev) = self.prev_timestamp {
            let dt = (now_ts - prev).max(0.0) as f32;
            if dt > 0.0 {
                out = self.prev_steer_output
                    + (steer_input - self.prev_steer_output) * (dt / (self.tau + dt));
            }
        } else {
            out = steer_input;
        }
        self.prev_steer_output = out;
        self.prev_timestamp = Some(now_ts);
        out
    }

    fn update_carla_control(&mut self, timestamp: f64) {
        let tier4_vehicle_msgs::msg::ActuationCommandStamped {
            actuation:
                tier4_vehicle_msgs::msg::ActuationCommand {
                    mut accel_cmd,
                    mut brake_cmd,
                    mut steer_cmd,
                },
            ..
        } = **self.current_actuation_cmd.load();

        log::debug!(
            "Autoware => Bridge: accel_cmd={accel_cmd:.3}, brake_cmd={brake_cmd:.3}, steer_cmd={steer_cmd:.3}",
        );

        // Default states
        let mut reverse = false;
        let mut hand_brake = false;

        match **self.current_gear.load() {
            autoware_vehicle_msgs::msg::GearReport::DRIVE => { /* Do nothing */ }
            autoware_vehicle_msgs::msg::GearReport::REVERSE => {
                /* Set reverse to true for reverse gear */
                reverse = true;
            }
            autoware_vehicle_msgs::msg::GearReport::PARK => {
                /* Force the vehicle to stop */
                accel_cmd = 0.0;
                brake_cmd = 0.0;
                steer_cmd = 0.0;
                hand_brake = true;
            }
            _ => { /* Do nothing */ }
        };

        // Convert steer_cmd based on steering curve and speed of the vehicle
        let steering_curve = self.actor.physics_control().steering_curve;
        let v_x: Vec<f32> = steering_curve.iter().map(|v| v[0]).collect();
        let v_y: Vec<f32> = steering_curve.iter().map(|v| v[1]).collect();

        let current_speed = self.actor.velocity().x.abs();
        let max_steer_ratio = interp(&v_x, &v_y, current_speed, &InterpMode::default());

        let steer_norm = self.first_order_steering((-steer_cmd) as f32, timestamp);
        let steer = steer_norm * max_steer_ratio;

        self.actor.apply_control(&VehicleControl {
            throttle: accel_cmd as f32,
            steer,
            brake: brake_cmd as f32,
            hand_brake,
            reverse,
            manual_gear_shift: false,
            gear: 0,
        });

        log::debug!(
            "Bridge => Carla: throttle={:.3}, steer={:.3}, brake={:.3}, hand_brake={}, reverse={}",
            accel_cmd as f32,
            steer,
            brake_cmd as f32,
            hand_brake,
            reverse,
        );
    }

    pub fn vehicle_name(&self) -> &str {
        &self.vehicle_name
    }
}

impl ActorBridge for VehicleBridge {
    fn step(&mut self, timestamp: f64) -> Result<()> {
        self.pub_current_actuation(timestamp)?;
        self.pub_current_velocity(timestamp)?;
        self.pub_current_steer(timestamp)?;
        self.pub_current_gear(timestamp)?;
        self.pub_current_control(timestamp)?;
        self.pub_current_indicator(timestamp)?;
        self.pub_hazard_light(timestamp)?;
        self.update_carla_control(timestamp);
        Ok(())
    }
}

impl Drop for VehicleBridge {
    fn drop(&mut self) {
        log::info!("Remove vehicle name {}", self.vehicle_name());
    }
}
