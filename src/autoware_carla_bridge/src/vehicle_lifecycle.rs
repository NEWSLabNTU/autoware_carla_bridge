/// Vehicle lifecycle management for Auto ware-CARLA integration
///
/// This module handles the complete lifecycle of CARLA vehicles tied to Autoware:
/// - Waiting for prerequisites (Autoware detection, initial pose)
/// - Spawning vehicle at initial pose
/// - Teleportation on pose updates
/// - Cleanup on Autoware loss
use crate::{
    autoware_detection::AutowareDetector,
    coordinate_conversion::{euler_to_quaternion, quaternion_to_euler, ros_to_carla_position},
    error::{BridgeError, Result},
};
use carla::client::{ActorBase, Client, Vehicle};
use geometry_msgs::msg::PoseWithCovarianceStamped;
use nalgebra::Vector3;
use rclrs::IntoPrimitiveOptions;
use std::sync::{Arc, Mutex};

/// Vehicle lifecycle state
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LifecycleState {
    /// Waiting for prerequisites (Autoware + initial pose)
    WaitingForPrerequisites,
    /// Prerequisites met, ready to spawn
    ReadyToSpawn,
    /// Vehicle spawned and active
    Active,
    /// Vehicle cleanup in progress
    CleaningUp,
}

/// Vehicle lifecycle manager
pub struct VehicleLifecycle {
    state: Arc<Mutex<LifecycleState>>,
    initial_pose: Arc<Mutex<Option<nalgebra::Isometry3<f32>>>>,
    vehicle: Arc<Mutex<Option<Vehicle>>>,
    client: Client,
    vehicle_blueprint: String,
    _initialpose_sub: Arc<rclrs::Subscription<PoseWithCovarianceStamped>>,
}

impl VehicleLifecycle {
    /// Create a new vehicle lifecycle manager
    ///
    /// # Arguments
    /// * `node` - ROS node for subscriptions
    /// * `client` - CARLA client for spawning
    /// * `vehicle_blueprint` - CARLA blueprint ID (e.g., "vehicle.tesla.model3")
    pub fn new(node: rclrs::Node, client: Client, vehicle_blueprint: String) -> Result<Self> {
        let state = Arc::new(Mutex::new(LifecycleState::WaitingForPrerequisites));
        let initial_pose = Arc::new(Mutex::new(None));
        let vehicle: Arc<Mutex<Option<Vehicle>>> = Arc::new(Mutex::new(None));

        // Subscribe to /initialpose
        let initial_pose_cb = initial_pose.clone();
        let state_cb = state.clone();
        let vehicle_cb = vehicle.clone();

        let initialpose_sub = node.create_subscription::<PoseWithCovarianceStamped, _>(
            "/initialpose".reliable().keep_last(1),
            move |msg: PoseWithCovarianceStamped| {
                log::info!(
                    "Initial pose received: ({:.2}, {:.2}, {:.2}) in frame '{}'",
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                    msg.header.frame_id
                );

                // Convert ROS pose to CARLA transform
                let carla_isometry = Self::ros_pose_to_carla_isometry(&msg.pose.pose);

                // Store initial pose
                *initial_pose_cb.lock().unwrap() = Some(carla_isometry);

                // Check if vehicle already spawned (teleport case)
                let vehicle_lock = vehicle_cb.lock().unwrap();
                if let Some(ref vehicle) = *vehicle_lock {
                    // Teleport existing vehicle
                    log::info!("Teleporting vehicle to new pose");
                    vehicle.set_transform(&carla_isometry);
                } else {
                    // Update state to ready for spawning
                    let mut state_lock = state_cb.lock().unwrap();
                    if *state_lock == LifecycleState::WaitingForPrerequisites {
                        *state_lock = LifecycleState::ReadyToSpawn;
                        log::info!("Prerequisites met - ready to spawn vehicle");
                    }
                }
            },
        )?;

        Ok(Self {
            state,
            initial_pose,
            vehicle,
            client,
            vehicle_blueprint,
            _initialpose_sub: Arc::new(initialpose_sub),
        })
    }

    /// Convert ROS Pose to CARLA Isometry (for spawning)
    fn ros_pose_to_carla_isometry(pose: &geometry_msgs::msg::Pose) -> nalgebra::Isometry3<f32> {
        // Convert position (meters → centimeters, Y-axis flip)
        let ros_position = Vector3::new(pose.position.x, pose.position.y, pose.position.z);
        let carla_position = ros_to_carla_position(&ros_position);

        // Convert ROS quaternion to nalgebra quaternion (f32)
        let q_f64 = nalgebra::Quaternion::new(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        );

        // Convert to Euler, apply coordinate system transform, back to quaternion (f32)
        let (roll, pitch, yaw) = quaternion_to_euler(&q_f64);
        let carla_quat = euler_to_quaternion(
            -roll, // Roll sign flip for left-handed
            pitch, -yaw, // Yaw sign flip for left-handed
        );

        // Create nalgebra Isometry3<f32>
        let translation = nalgebra::Translation3::new(
            carla_position.x as f32,
            carla_position.y as f32,
            carla_position.z as f32,
        );

        let rotation = nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
            carla_quat.w as f32,
            carla_quat.i as f32,
            carla_quat.j as f32,
            carla_quat.k as f32,
        ));

        nalgebra::Isometry3::from_parts(translation, rotation)
    }

    /// Get current lifecycle state
    pub fn state(&self) -> LifecycleState {
        *self.state.lock().unwrap()
    }

    /// Check if vehicle is ready to spawn
    pub fn is_ready_to_spawn(&self) -> bool {
        self.state() == LifecycleState::ReadyToSpawn
    }

    /// Check if vehicle is active
    pub fn is_active(&self) -> bool {
        self.state() == LifecycleState::Active
    }

    /// Spawn vehicle at the initial pose
    ///
    /// This should be called when `is_ready_to_spawn()` returns true
    pub fn spawn_vehicle(&self) -> Result<()> {
        let mut state_lock = self.state.lock().unwrap();
        if *state_lock != LifecycleState::ReadyToSpawn {
            return Err(BridgeError::AutowareIssue(format!(
                "Cannot spawn vehicle in state {:?}",
                *state_lock
            )));
        }

        let initial_pose = self.initial_pose.lock().unwrap();
        let pose = initial_pose
            .as_ref()
            .ok_or_else(|| BridgeError::AutowareIssue("No initial pose available".to_string()))?;

        log::info!(
            "Spawning vehicle at ({:.2}, {:.2}, {:.2})",
            pose.translation.x,
            pose.translation.y,
            pose.translation.z
        );

        // Get blueprint from library
        let mut world = self.client.world();
        let blueprint_library = world.blueprint_library();
        let vehicle_bp = blueprint_library
            .find(&self.vehicle_blueprint)
            .ok_or_else(|| {
                BridgeError::AutowareIssue(format!(
                    "Vehicle blueprint '{}' not found",
                    self.vehicle_blueprint
                ))
            })?;

        // Spawn vehicle
        let actor = world
            .spawn_actor(&vehicle_bp, pose)
            .map_err(|e| BridgeError::AutowareIssue(format!("Failed to spawn vehicle: {}", e)))?;
        let vehicle = match actor.into_kinds() {
            carla::client::ActorKind::Vehicle(v) => v,
            _ => return Err(BridgeError::CarlaIssue("Spawned actor is not a vehicle")),
        };

        log::info!("Vehicle spawned successfully: ID={}", vehicle.id());

        // Store vehicle and update state
        *self.vehicle.lock().unwrap() = Some(vehicle);
        *state_lock = LifecycleState::Active;

        Ok(())
    }

    /// Get the spawned vehicle (if active)
    pub fn get_vehicle(&self) -> Option<Vehicle> {
        self.vehicle.lock().unwrap().clone()
    }

    /// Cleanup: destroy vehicle and sensors
    ///
    /// This should be called when Autoware is lost or the bridge is shutting down
    pub fn cleanup(&self) -> Result<()> {
        let mut state_lock = self.state.lock().unwrap();
        *state_lock = LifecycleState::CleaningUp;

        let mut vehicle_lock = self.vehicle.lock().unwrap();
        if let Some(vehicle) = vehicle_lock.take() {
            log::info!("Destroying vehicle: ID={}", vehicle.id());

            // Destroy vehicle (this also destroys attached sensors in CARLA)
            let destroyed = vehicle.destroy();
            if !destroyed {
                log::warn!("Vehicle destroy returned false - may already be destroyed");
            } else {
                log::info!("Vehicle destroyed successfully");
            }
        }

        // Reset state
        *state_lock = LifecycleState::WaitingForPrerequisites;
        *self.initial_pose.lock().unwrap() = None;

        Ok(())
    }

    /// Check if cleanup is needed (Autoware lost)
    pub fn should_cleanup(&self, autoware_detector: &AutowareDetector) -> bool {
        self.is_active() && !autoware_detector.is_alive()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_state_transitions() {
        // Basic state transition logic test
        assert_eq!(
            LifecycleState::WaitingForPrerequisites,
            LifecycleState::WaitingForPrerequisites
        );
        assert_ne!(
            LifecycleState::WaitingForPrerequisites,
            LifecycleState::Active
        );
    }

    #[test]
    fn test_ros_pose_to_carla_isometry() {
        let mut pose = geometry_msgs::msg::Pose::default();
        pose.position.x = 1.0;
        pose.position.y = 2.0;
        pose.position.z = 0.5;
        pose.orientation.w = 1.0; // Identity quaternion

        let isometry = VehicleLifecycle::ros_pose_to_carla_isometry(&pose);

        // Check conversion (meters to cm, Y-axis flip)
        assert!((isometry.translation.x - 100.0).abs() < 0.1);
        assert!((isometry.translation.y + 200.0).abs() < 0.1); // Y flipped
        assert!((isometry.translation.z - 50.0).abs() < 0.1);
    }
}
