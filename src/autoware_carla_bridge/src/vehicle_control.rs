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

/// Vehicle control manager
///
/// Handles bidirectional control between Autoware and CARLA:
/// - Subscribes to `/control/command/actuation_cmd` (ActuationCommandStamped)
/// - Publishes `/vehicle/status/velocity_status` (VelocityReport)
/// - Publishes `/vehicle/status/steering_status` (SteeringReport)
/// - Publishes `/vehicle/status/control_mode` (ControlModeReport)
pub struct VehicleControlBridge {
    // Publishers
    velocity_pub: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::VelocityReport>>,
    steering_pub: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::SteeringReport>>,
    control_mode_pub: Arc<rclrs::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>>,

    // Subscriber stored to keep it alive
    _control_sub: Arc<rclrs::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>>,

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

        // Create control command subscriber
        let vehicle_for_callback = vehicle.clone();
        let control_sub = Arc::new(node.create_subscription(
            "/control/command/actuation_cmd".reliable(),
            move |msg: tier4_vehicle_msgs::msg::ActuationCommandStamped| {
                if let Err(e) = Self::apply_control_command(&vehicle_for_callback, &msg) {
                    tracing::error!("Failed to apply control command: {}", e);
                }
            },
        )?);

        tracing::info!("Vehicle control bridge created");
        tracing::info!("  Subscribed to: /control/command/actuation_cmd");
        tracing::info!("  Publishing: /vehicle/status/velocity_status");
        tracing::info!("  Publishing: /vehicle/status/steering_status");
        tracing::info!("  Publishing: /vehicle/status/control_mode");

        Ok(Self {
            velocity_pub,
            steering_pub,
            control_mode_pub,
            _control_sub: control_sub,
            vehicle,
        })
    }

    /// Apply control command from Autoware to CARLA vehicle
    fn apply_control_command(
        vehicle: &Arc<Mutex<Option<Vehicle>>>,
        cmd: &tier4_vehicle_msgs::msg::ActuationCommandStamped,
    ) -> Result<()> {
        let vehicle_guard = vehicle.lock().unwrap();
        if let Some(ref v) = *vehicle_guard {
            // Convert Autoware actuation command to CARLA VehicleControl
            // ActuationCommandStamped provides direct accel/brake/steer commands

            let mut control = VehicleControl {
                throttle: 0.0,
                steer: 0.0,
                brake: 0.0,
                hand_brake: false,
                reverse: false,
                manual_gear_shift: false,
                gear: 0,
            };

            // Steering: steer_cmd is already normalized (-1.0 to 1.0)
            control.steer = (cmd.actuation.steer_cmd as f32).clamp(-1.0, 1.0);

            // Acceleration/Brake: accel_cmd and brake_cmd are normalized (0.0 to 1.0)
            // If accel_cmd is positive, use throttle. If brake_cmd is positive, use brake.
            let accel = cmd.actuation.accel_cmd as f32;
            let brake = cmd.actuation.brake_cmd as f32;

            if accel > 0.01 {
                // Acceleration command
                control.throttle = accel.clamp(0.0, 1.0);
                control.brake = 0.0;
            } else if brake > 0.01 {
                // Brake command
                control.throttle = 0.0;
                control.brake = brake.clamp(0.0, 1.0);
            } else {
                // No command - coast
                control.throttle = 0.0;
                control.brake = 0.0;
            }

            // Apply control to vehicle
            v.apply_control(&control);

            tracing::debug!(
                "Applied control: steer={:.3}, throttle={:.3}, brake={:.3}",
                control.steer,
                control.throttle,
                control.brake,
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
            let velocity_vec = vehicle.velocity();
            let angular_velocity_vec = vehicle.angular_velocity();
            let control = vehicle.control();

            // Calculate velocities
            let longitudinal_velocity =
                (velocity_vec.x.powi(2) + velocity_vec.y.powi(2) + velocity_vec.z.powi(2)).sqrt();
            let lateral_velocity = velocity_vec.y; // Assuming Y is lateral
            let heading_rate = angular_velocity_vec.z; // Yaw rate

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
            const STEERING_ANGLE_MAX: f32 = 1.22; // radians (~70 degrees)
            let steering_report = autoware_vehicle_msgs::msg::SteeringReport {
                stamp: ros_timestamp.clone(),
                steering_tire_angle: control.steer * STEERING_ANGLE_MAX,
            };

            self.steering_pub.publish(&steering_report)?;

            // Publish ControlModeReport (always AUTONOMOUS in simulation)
            let control_mode = autoware_vehicle_msgs::msg::ControlModeReport {
                stamp: ros_timestamp,
                mode: 1, // AUTONOMOUS = 1
            };

            self.control_mode_pub.publish(&control_mode)?;

            tracing::trace!(
                "Published vehicle status: vel={:.2} m/s, steer={:.3} rad",
                longitudinal_velocity,
                steering_report.steering_tire_angle
            );
        }

        Ok(())
    }
}
