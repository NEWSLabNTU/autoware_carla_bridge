/// Vehicle control integration for Autoware-CARLA bridge
///
/// This module handles:
/// - Subscribing to Autoware control commands
/// - Applying control to CARLA vehicle
/// - Publishing vehicle status to Autoware
use crate::error::Result;
use carla::{
    client::{ActorBase, Vehicle},
    rpc::VehicleControl,
};
use rclrs::IntoPrimitiveOptions;
use std::sync::{Arc, Mutex};

/// Maximum steering tire angle in radians (~70 degrees)
const MAX_STEER_ANGLE: f32 = 1.22;

/// Maximum expected acceleration magnitude for throttle/brake mapping (m/s²)
const MAX_ACCEL: f32 = 3.0;

/// Vehicle control manager
///
/// Handles bidirectional control between Autoware and CARLA:
/// - Subscribes to `/control/command/control_cmd` (autoware_control_msgs/Control)
/// - Publishes `/vehicle/status/velocity_status` (VelocityReport)
/// - Publishes `/vehicle/status/steering_status` (SteeringReport)
/// - Publishes `/vehicle/status/control_mode` (ControlModeReport)
/// - Publishes `/vehicle/status/gear_status` (GearReport)
pub struct VehicleControlBridge {
    // Publishers
    velocity_pub: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::VelocityReport>>,
    steering_pub: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::SteeringReport>>,
    control_mode_pub: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>>,
    gear_pub: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::GearReport>>,

    // Subscriber stored to keep it alive
    _control_sub: Arc<rclrs::Subscription<autoware_control_msgs::msg::Control>>,

    // CARLA vehicle reference (shared with main loop)
    vehicle: Arc<Mutex<Option<Vehicle>>>,
}

impl VehicleControlBridge {
    /// Create a new vehicle control bridge
    ///
    /// # Arguments
    /// * `node` - ROS node for creating publishers/subscribers
    /// * `vehicle` - Arc<Mutex<Option<Vehicle>>> shared with main loop
    pub fn new(node: rclrs::Node, vehicle: Arc<Mutex<Option<Vehicle>>>) -> Result<Self> {
        // Create publishers
        let velocity_pub =
            Arc::new(node.create_publisher("/vehicle/status/velocity_status".reliable())?);

        let steering_pub =
            Arc::new(node.create_publisher("/vehicle/status/steering_status".reliable())?);

        let control_mode_pub =
            Arc::new(node.create_publisher("/vehicle/status/control_mode".reliable())?);

        let gear_pub = Arc::new(node.create_publisher("/vehicle/status/gear_status".reliable())?);

        // Create control command subscriber (Autoware 1.5.0 uses Control message)
        let vehicle_for_callback = vehicle.clone();
        let control_sub = Arc::new(node.create_subscription(
            "/control/command/control_cmd".reliable(),
            move |msg: autoware_control_msgs::msg::Control| {
                if let Err(e) = Self::apply_control_command(&vehicle_for_callback, &msg) {
                    tracing::error!("Failed to apply control command: {}", e);
                }
            },
        )?);

        tracing::info!("Vehicle control bridge created");
        tracing::info!("  Subscribed to: /control/command/control_cmd");
        tracing::info!("  Publishing: /vehicle/status/velocity_status");
        tracing::info!("  Publishing: /vehicle/status/steering_status");
        tracing::info!("  Publishing: /vehicle/status/control_mode");
        tracing::info!("  Publishing: /vehicle/status/gear_status");

        Ok(Self {
            velocity_pub,
            steering_pub,
            control_mode_pub,
            gear_pub,
            _control_sub: control_sub,
            vehicle,
        })
    }

    /// Apply control command from Autoware to CARLA vehicle
    ///
    /// Converts Autoware Control (physical units) to CARLA VehicleControl (0-1 normalized):
    /// - steering_tire_angle (rad) → steer (-1 to 1) via max_steer_angle
    /// - acceleration (m/s²) → throttle (0-1) or brake (0-1)
    fn apply_control_command(
        vehicle: &Arc<Mutex<Option<Vehicle>>>,
        cmd: &autoware_control_msgs::msg::Control,
    ) -> Result<()> {
        let vehicle_guard = vehicle.lock().unwrap();
        if let Some(ref v) = *vehicle_guard {
            let mut control = VehicleControl {
                throttle: 0.0,
                steer: 0.0,
                brake: 0.0,
                hand_brake: false,
                reverse: false,
                manual_gear_shift: false,
                gear: 0,
            };

            // Steering: convert tire angle (rad) to normalized (-1 to 1)
            // Negate: Autoware positive = left turn (ROS), CARLA positive = right turn (left-handed)
            control.steer =
                (-cmd.lateral.steering_tire_angle / MAX_STEER_ANGLE).clamp(-1.0, 1.0);

            // Longitudinal: acceleration (m/s²) → throttle or brake (0 to 1)
            let accel = cmd.longitudinal.acceleration;
            if accel > 0.01 {
                control.throttle = (accel / MAX_ACCEL).clamp(0.0, 1.0);
                control.brake = 0.0;
            } else if accel < -0.01 {
                control.throttle = 0.0;
                control.brake = (-accel / MAX_ACCEL).clamp(0.0, 1.0);
            }

            // Reverse if target velocity is negative
            if cmd.longitudinal.velocity < -0.01 {
                control.reverse = true;
            }

            v.apply_control(&control)?;

            tracing::debug!(
                "Applied control: steer={:.3}, throttle={:.3}, brake={:.3}, accel={:.2} m/s²",
                control.steer,
                control.throttle,
                control.brake,
                accel,
            );
        }

        Ok(())
    }

    /// Publish vehicle status to Autoware
    ///
    /// Should be called in the main loop at regular intervals (e.g., 10 Hz)
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
            let velocity_vec = vehicle.velocity()?;
            let angular_velocity_vec = vehicle.angular_velocity()?;
            let control = vehicle.control()?;

            // Calculate velocities
            // Longitudinal = speed magnitude (always positive for forward)
            let longitudinal_velocity =
                (velocity_vec.x.powi(2) + velocity_vec.y.powi(2) + velocity_vec.z.powi(2)).sqrt();
            // Negate: CARLA Y = right (left-handed), ROS lateral = left (right-handed)
            let lateral_velocity = -velocity_vec.y;
            // Negate: CARLA Z angular = clockwise, ROS yaw rate = counter-clockwise
            let heading_rate = -angular_velocity_vec.z;

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

            // Publish SteeringReport
            // Convert CARLA steering (-1 to 1) to tire angle (radians)
            // Negate: CARLA positive = right turn, Autoware positive = left turn
            let steering_report = autoware_vehicle_msgs::msg::SteeringReport {
                stamp: ros_timestamp.clone(),
                steering_tire_angle: -control.steer * MAX_STEER_ANGLE,
            };

            self.steering_pub.publish(&steering_report)?;

            // Publish ControlModeReport (always AUTONOMOUS in simulation)
            let control_mode = autoware_vehicle_msgs::msg::ControlModeReport {
                stamp: ros_timestamp.clone(),
                mode: 1, // AUTONOMOUS = 1
            };

            self.control_mode_pub.publish(&control_mode)?;

            // Publish GearReport (always DRIVE for forward motion)
            let gear_report = autoware_vehicle_msgs::msg::GearReport {
                stamp: ros_timestamp,
                report: autoware_vehicle_msgs::msg::GearReport::DRIVE,
            };

            self.gear_pub.publish(&gear_report)?;

            tracing::trace!(
                "Published vehicle status: vel={:.2} m/s, steer={:.3} rad",
                longitudinal_velocity,
                steering_report.steering_tire_angle
            );
        }

        Ok(())
    }
}
