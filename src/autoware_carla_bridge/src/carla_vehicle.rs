/// CARLA vehicle management for Autoware-CARLA integration
///
/// This module handles spawning and cleanup of CARLA vehicles for Autoware.
/// The vehicle and sensors are spawned immediately in the constructor.
use crate::{
    error::{BridgeError, Result},
    tf_bridge::TFBuffer,
    urdf_parser::SensorConfig,
};
use carla::{
    client::{ActorBase, Sensor, Vehicle, World},
    geom::Transform,
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
}

impl CarlaVehicle {
    /// Create a new CARLA vehicle and spawn it with sensors
    ///
    /// This immediately spawns the vehicle and all configured sensors in CARLA.
    ///
    /// # Arguments
    /// * `world` - Mutable CARLA world reference
    /// * `vehicle_blueprint` - CARLA blueprint ID (e.g., "vehicle.tesla.model3")
    /// * `initial_pose` - Spawn location and orientation in CARLA coordinates
    /// * `sensor_configs` - Sensor configurations from URDF parsing
    /// * `tf_buffer` - TF buffer for sensor transforms
    ///
    /// # Returns
    /// A CarlaVehicle instance with spawned vehicle and sensors
    pub fn new(
        world: &mut World,
        vehicle_blueprint: &str,
        initial_pose: &nalgebra::Isometry3<f32>,
        sensor_configs: &[SensorConfig],
        _tf_buffer: &TFBuffer,
    ) -> Result<Self> {
        tracing::info!(
            "Spawning vehicle at ({:.2}, {:.2}, {:.2})",
            initial_pose.translation.x,
            initial_pose.translation.y,
            initial_pose.translation.z
        );

        // Spawn vehicle
        let vehicle = Self::spawn_vehicle(world, vehicle_blueprint, initial_pose)?;

        tracing::info!("Vehicle spawned successfully: ID={}", vehicle.id());

        // Spawn sensors
        tracing::info!("Spawning {} sensors...", sensor_configs.len());
        let sensors = Self::spawn_sensors(world, &vehicle, sensor_configs)?;

        tracing::info!("All sensors spawned successfully");

        Ok(Self { vehicle, sensors })
    }

    /// Spawn vehicle at the specified pose (private)
    fn spawn_vehicle(
        world: &mut World,
        vehicle_blueprint: &str,
        initial_pose: &nalgebra::Isometry3<f32>,
    ) -> Result<Vehicle> {
        // Get blueprint from library
        let blueprint_library = world.blueprint_library();
        let vehicle_bp = blueprint_library.find(vehicle_blueprint).ok_or_else(|| {
            BridgeError::AutowareIssue(format!(
                "Vehicle blueprint '{}' not found",
                vehicle_blueprint
            ))
        })?;

        // Convert nalgebra Isometry3 to CARLA Transform
        let carla_transform = Transform::from_na(initial_pose);

        // Spawn vehicle
        let actor = world
            .spawn_actor(&vehicle_bp, &carla_transform)
            .map_err(|e| BridgeError::AutowareIssue(format!("Failed to spawn vehicle: {}", e)))?;

        let vehicle = match actor.into_kinds() {
            carla::client::ActorKind::Vehicle(v) => v,
            _ => return Err(BridgeError::CarlaIssue("Spawned actor is not a vehicle")),
        };

        Ok(vehicle)
    }

    /// Spawn sensors and attach to vehicle (private)
    fn spawn_sensors(
        world: &mut World,
        vehicle: &Vehicle,
        sensor_configs: &[SensorConfig],
    ) -> Result<HashMap<String, Sensor>> {
        let blueprint_library = world.blueprint_library();
        let mut spawned_sensors = HashMap::new();

        for config in sensor_configs {
            // Map SensorType to CARLA blueprint
            let blueprint_id = match config.sensor_type {
                crate::bridge::sensor_bridge::SensorType::CameraRgb => "sensor.camera.rgb",
                crate::bridge::sensor_bridge::SensorType::LidarRayCast => "sensor.lidar.ray_cast",
                crate::bridge::sensor_bridge::SensorType::LidarRayCastSemantic => {
                    "sensor.lidar.ray_cast_semantic"
                }
                crate::bridge::sensor_bridge::SensorType::Imu => "sensor.other.imu",
                crate::bridge::sensor_bridge::SensorType::Gnss => "sensor.other.gnss",
                _ => {
                    tracing::warn!("Skipping unsupported sensor type: {:?}", config.sensor_type);
                    continue;
                }
            };

            // Get blueprint
            let sensor_bp = blueprint_library.find(blueprint_id).ok_or_else(|| {
                BridgeError::AutowareIssue(format!("Sensor blueprint '{}' not found", blueprint_id))
            })?;

            // Create transform from URDF/TF (nalgebra)
            let na_transform = nalgebra::Isometry3::from_parts(
                nalgebra::Translation3::new(
                    config.position.x as f32,
                    config.position.y as f32,
                    config.position.z as f32,
                ),
                nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                    config.orientation.w as f32,
                    config.orientation.i as f32,
                    config.orientation.j as f32,
                    config.orientation.k as f32,
                )),
            );

            // Convert to CARLA Transform
            let carla_transform = Transform::from_na(&na_transform);

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
                        config.link_name, e
                    ))
                })?;

            let sensor = match sensor_actor.into_kinds() {
                carla::client::ActorKind::Sensor(s) => s,
                _ => return Err(BridgeError::CarlaIssue("Spawned actor is not a sensor")),
            };

            tracing::info!(
                "Spawned sensor '{}' (type: {:?}, ID: {})",
                config.link_name,
                config.sensor_type,
                sensor.id()
            );

            spawned_sensors.insert(config.link_name.clone(), sensor);
        }

        Ok(spawned_sensors)
    }

    /// Get reference to the spawned vehicle
    pub fn get_vehicle(&self) -> &Vehicle {
        &self.vehicle
    }

    /// Get reference to all spawned sensors
    pub fn get_sensors(&self) -> &HashMap<String, Sensor> {
        &self.sensors
    }

    /// Cleanup: destroy vehicle and sensors
    ///
    /// This should be called when the bridge is shutting down
    pub fn cleanup(&mut self) -> Result<()> {
        // Destroy all sensors first
        for (name, sensor) in self.sensors.drain() {
            tracing::info!("Destroying sensor '{}' (ID: {})", name, sensor.id());
            let destroyed = sensor.destroy();
            if !destroyed {
                tracing::warn!(
                    "Sensor '{}' destroy returned false - may already be destroyed",
                    name
                );
            }
        }

        // Destroy vehicle
        tracing::info!("Destroying vehicle: ID={}", self.vehicle.id());
        let destroyed = self.vehicle.destroy();
        if !destroyed {
            tracing::warn!("Vehicle destroy returned false - may already be destroyed");
        } else {
            tracing::info!("Vehicle destroyed successfully");
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Tests removed since they tested lifecycle state management
    // which is no longer part of CarlaVehicle
}
