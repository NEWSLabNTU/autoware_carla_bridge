/// CARLA vehicle management for Autoware-CARLA integration
///
/// This module handles spawning and cleanup of CARLA vehicles for Autoware.
/// The vehicle and sensors are spawned immediately in the constructor.
///
/// ## Design Philosophy
/// - **Config as single source of truth**: `VehicleConfig` defines which sensors to spawn
/// - **No name-based inference**: Blueprints come from config, not URDF link name patterns
/// - **TF for positions only**: URDF/TF is only used to get sensor transform relative to base_link
use crate::{
    error::{BridgeError, Result},
    sensor_config::{SensorType, VehicleConfig},
    tf_bridge::TFBuffer,
};
use carla::{
    client::{ActorBase, Sensor, Vehicle, World},
    rpc::AttachmentType,
};
use std::collections::HashMap;

/// CARLA vehicle manager
///
/// Manages CARLA vehicle and sensor actors. The vehicle and all sensors
/// are spawned immediately in the constructor at the specified initial pose.
pub struct CarlaVehicle {
    vehicle: Vehicle,
    sensors: HashMap<String, Sensor>,
    /// Sensor types keyed by link name (derived from VehicleConfig blueprints)
    sensor_types: HashMap<String, SensorType>,
}

impl CarlaVehicle {
    /// Create a new CARLA vehicle and spawn it with sensors
    ///
    /// This immediately spawns the vehicle and all configured sensors in CARLA.
    /// The `vehicle_config` is the **single source of truth** for which sensors to spawn.
    ///
    /// # Arguments
    /// * `world` - Mutable CARLA world reference
    /// * `initial_pose` - Spawn location and orientation in ROS coordinates
    /// * `vehicle_config` - Vehicle and sensor configuration (single source of truth)
    /// * `tf_buffer` - TF buffer for sensor position lookups
    ///
    /// # Returns
    /// A CarlaVehicle instance with spawned vehicle and sensors
    pub fn new(
        world: &mut World,
        initial_pose: &nalgebra::Isometry3<f32>,
        vehicle_config: &VehicleConfig,
        tf_buffer: &TFBuffer,
    ) -> Result<Self> {
        tracing::info!(
            "Initial pose from Autoware (ROS coords): x={:.2}, y={:.2}, z={:.2}",
            initial_pose.translation.x,
            initial_pose.translation.y,
            initial_pose.translation.z
        );

        // Spawn vehicle using blueprint from config
        let vehicle = Self::spawn_vehicle(world, &vehicle_config.vehicle.blueprint, initial_pose)?;

        // Log actual spawned position in CARLA
        let spawned_transform = vehicle.transform()?;
        tracing::info!(
            "Vehicle spawned in CARLA (CARLA coords): x={:.1}, y={:.1}, z={:.1}",
            spawned_transform.location.x,
            spawned_transform.location.y,
            spawned_transform.location.z
        );
        tracing::info!("Vehicle spawned successfully: ID={}", vehicle.id());

        // Spawn sensors from vehicle_config (single source of truth)
        tracing::info!(
            "Spawning {} sensors from vehicle_config...",
            vehicle_config.sensors.len()
        );

        // Debug: Show available TF frames
        let available_frames = tf_buffer.get_all_frames();
        tracing::info!("Available TF frames ({} total):", available_frames.len());
        for frame in &available_frames {
            tracing::debug!("  - {}", frame);
        }

        let (sensors, sensor_types) =
            Self::spawn_sensors(world, &vehicle, vehicle_config, tf_buffer)?;

        tracing::info!("All sensors spawned successfully");

        Ok(Self {
            vehicle,
            sensors,
            sensor_types,
        })
    }

    /// Spawn vehicle at the specified pose (private)
    fn spawn_vehicle(
        world: &mut World,
        vehicle_blueprint: &str,
        initial_pose: &nalgebra::Isometry3<f32>,
    ) -> Result<Vehicle> {
        // Get blueprint from library
        let blueprint_library = world.blueprint_library()?;
        let vehicle_bp = blueprint_library.find(vehicle_blueprint)?.ok_or_else(|| {
            BridgeError::AutowareIssue(format!(
                "Vehicle blueprint '{}' not found",
                vehicle_blueprint
            ))
        })?;

        // Convert ROS pose to CARLA transform using centralized helper
        let carla_transform =
            crate::coordinate_conversion::ros_isometry_to_carla_transform(initial_pose);

        tracing::info!(
            "Coordinate conversion: ROS({:.2}, {:.2}, {:.2}) → CARLA({:.1}, {:.1}, {:.1})",
            initial_pose.translation.x,
            initial_pose.translation.y,
            initial_pose.translation.z,
            carla_transform.location.x,
            carla_transform.location.y,
            carla_transform.location.z
        );

        // Log the exact transform we're about to pass to CARLA
        tracing::info!(
            "Calling spawn_actor with transform: loc=({:.1}, {:.1}, {:.1}), rot=(r:{:.1}, p:{:.1}, y:{:.1})",
            carla_transform.location.x,
            carla_transform.location.y,
            carla_transform.location.z,
            carla_transform.rotation.roll,
            carla_transform.rotation.pitch,
            carla_transform.rotation.yaw
        );

        // Spawn vehicle at the requested transform
        let actor = world
            .spawn_actor(&vehicle_bp, &carla_transform)
            .map_err(|e| BridgeError::AutowareIssue(format!("Failed to spawn vehicle: {}", e)))?;

        // Wait for CARLA to process the spawn before querying actor state
        // Works for both sync mode (waits for tick) and async mode (times out after 100ms)
        tracing::debug!("Waiting for CARLA to process spawn...");
        let _ = world.wait_for_tick_or_timeout(std::time::Duration::from_millis(100))?;

        let vehicle = match actor.into_kinds() {
            carla::client::ActorKind::Vehicle(v) => v,
            _ => return Err(BridgeError::CarlaIssue("Spawned actor is not a vehicle")),
        };

        Ok(vehicle)
    }

    /// Spawn sensors and attach to vehicle (private)
    ///
    /// Iterates over `vehicle_config.sensors` to spawn each sensor. The blueprint
    /// comes directly from the config (single source of truth), while the transform
    /// is looked up from TF.
    fn spawn_sensors(
        world: &mut World,
        vehicle: &Vehicle,
        vehicle_config: &VehicleConfig,
        tf_buffer: &TFBuffer,
    ) -> Result<(HashMap<String, Sensor>, HashMap<String, SensorType>)> {
        let blueprint_library = world.blueprint_library()?;
        let mut spawned_sensors = HashMap::new();
        let mut sensor_types = HashMap::new();

        for (link_name, sensor_def) in &vehicle_config.sensors {
            // Get blueprint directly from config (no name-based inference!)
            let mut sensor_bp =
                blueprint_library
                    .find(&sensor_def.blueprint)?
                    .ok_or_else(|| {
                        BridgeError::AutowareIssue(format!(
                            "Sensor blueprint '{}' not found for '{}'",
                            sensor_def.blueprint, link_name
                        ))
                    })?;

            // Apply parameters from config
            sensor_def.apply_to_blueprint(&mut sensor_bp)?;

            // Try to get transform from TF buffer (base_link → sensor)
            tracing::info!("Looking up TF transform: base_link → '{}'", link_name);
            let na_transform = match tf_buffer.lookup_transform("base_link", link_name) {
                Ok(tf) => {
                    // Use TF transform
                    let trans = &tf.transform.translation;
                    let rot = &tf.transform.rotation;

                    tracing::info!(
                        "✓ Found TF for '{}': pos=({:.3}, {:.3}, {:.3}) parent='{}'",
                        link_name,
                        trans.x,
                        trans.y,
                        trans.z,
                        tf.header.frame_id
                    );

                    nalgebra::Isometry3::from_parts(
                        nalgebra::Translation3::new(trans.x as f32, trans.y as f32, trans.z as f32),
                        nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                            rot.w as f32,
                            rot.x as f32,
                            rot.y as f32,
                            rot.z as f32,
                        )),
                    )
                }
                Err(e) => {
                    // TF lookup failed - this is required for config-driven spawning
                    tracing::error!(
                        "✗ TF lookup failed for '{}': {} - Sensor link must exist in TF tree!",
                        link_name,
                        e
                    );
                    return Err(BridgeError::AutowareIssue(format!(
                        "Sensor '{}' not found in TF tree. Ensure the link exists in sensor_kit URDF.",
                        link_name
                    )));
                }
            };

            // Convert ROS sensor transform to CARLA transform using centralized helper
            let carla_transform =
                crate::coordinate_conversion::ros_isometry_to_carla_transform(&na_transform);

            tracing::info!(
                "Sensor '{}' transform: ROS({:.3}, {:.3}, {:.3}) → CARLA({:.1}, {:.1}, {:.1})",
                link_name,
                na_transform.translation.x,
                na_transform.translation.y,
                na_transform.translation.z,
                carla_transform.location.x,
                carla_transform.location.y,
                carla_transform.location.z
            );

            // Spawn sensor attached to vehicle
            let sensor_actor = world
                .spawn_actor_opt(
                    &sensor_bp,
                    &carla_transform,
                    Some(vehicle),
                    AttachmentType::Rigid,
                )
                .map_err(|e| {
                    BridgeError::AutowareIssue(format!(
                        "Failed to spawn sensor '{}': {}",
                        link_name, e
                    ))
                })?;

            let sensor = match sensor_actor.into_kinds() {
                carla::client::ActorKind::Sensor(s) => s,
                _ => return Err(BridgeError::CarlaIssue("Spawned actor is not a sensor")),
            };

            // Derive sensor type from blueprint (for sensor bridge creation)
            let sensor_type = sensor_def.sensor_type();

            tracing::info!(
                "Spawned sensor '{}' (blueprint: {}, type: {:?}, ID: {})",
                link_name,
                sensor_def.blueprint,
                sensor_type,
                sensor.id()
            );

            spawned_sensors.insert(link_name.clone(), sensor);
            sensor_types.insert(link_name.clone(), sensor_type);
        }

        Ok((spawned_sensors, sensor_types))
    }

    /// Get reference to the spawned vehicle
    pub fn get_vehicle(&self) -> &Vehicle {
        &self.vehicle
    }

    /// Get reference to all spawned sensors
    pub fn get_sensors(&self) -> &HashMap<String, Sensor> {
        &self.sensors
    }

    /// Get sensor types (keyed by link name)
    ///
    /// This returns the sensor types derived from VehicleConfig blueprints,
    /// allowing main.rs to create sensor bridges with the correct parameters.
    pub fn get_sensor_types(&self) -> &HashMap<String, SensorType> {
        &self.sensor_types
    }

    /// Cleanup: destroy vehicle and sensors
    ///
    /// This should be called when the bridge is shutting down
    pub fn cleanup(&mut self) -> Result<()> {
        // Destroy all sensors first
        for (name, sensor) in self.sensors.drain() {
            tracing::info!("Destroying sensor '{}' (ID: {})", name, sensor.id());
            match sensor.destroy() {
                Ok(true) => {}
                Ok(false) => tracing::warn!(
                    "Sensor '{}' destroy returned false - may already be destroyed",
                    name
                ),
                Err(e) => tracing::warn!("Sensor '{}' destroy failed: {e}", name),
            }
        }

        // Destroy vehicle
        tracing::info!("Destroying vehicle: ID={}", self.vehicle.id());
        match self.vehicle.destroy() {
            Ok(true) => tracing::info!("Vehicle destroyed successfully"),
            Ok(false) => {
                tracing::warn!("Vehicle destroy returned false - may already be destroyed")
            }
            Err(e) => tracing::warn!("Vehicle destroy failed: {e}"),
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    // Tests removed since they tested lifecycle state management
    // which is no longer part of CarlaVehicle
}
