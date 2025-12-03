/// CARLA vehicle management for Autoware-CARLA integration
///
/// This module handles spawning and cleanup of CARLA vehicles for Autoware.
/// The vehicle and sensors are spawned immediately in the constructor.
use crate::{
    error::{BridgeError, Result},
    sensor_config::CarlaConfig,
    tf_bridge::TFBuffer,
    urdf_parser::SensorConfig,
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
    sensor_configs: Vec<SensorConfig>,
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
    /// * `carla_config` - CARLA sensor configuration (CARLA-specific parameters)
    ///
    /// # Returns
    /// A CarlaVehicle instance with spawned vehicle and sensors
    pub fn new(
        world: &mut World,
        vehicle_blueprint: &str,
        initial_pose: &nalgebra::Isometry3<f32>,
        sensor_configs: &[SensorConfig],
        tf_buffer: &TFBuffer,
        carla_config: &CarlaConfig,
    ) -> Result<Self> {
        tracing::info!(
            "Initial pose from Autoware (ROS coords): x={:.2}, y={:.2}, z={:.2}",
            initial_pose.translation.x,
            initial_pose.translation.y,
            initial_pose.translation.z
        );

        // Spawn vehicle
        let vehicle = Self::spawn_vehicle(world, vehicle_blueprint, initial_pose)?;

        // Log actual spawned position in CARLA
        let spawned_transform = vehicle.transform();
        tracing::info!(
            "Vehicle spawned in CARLA (CARLA coords): x={:.1}, y={:.1}, z={:.1}",
            spawned_transform.location.x,
            spawned_transform.location.y,
            spawned_transform.location.z
        );
        tracing::info!("Vehicle spawned successfully: ID={}", vehicle.id());

        // Spawn sensors
        tracing::info!("Spawning {} sensors...", sensor_configs.len());

        // Debug: Show available TF frames
        let available_frames = tf_buffer.get_all_frames();
        tracing::info!("Available TF frames ({} total):", available_frames.len());
        for frame in &available_frames {
            tracing::debug!("  - {}", frame);
        }

        let sensors =
            Self::spawn_sensors(world, &vehicle, sensor_configs, tf_buffer, carla_config)?;

        tracing::info!("All sensors spawned successfully");

        Ok(Self {
            vehicle,
            sensors,
            sensor_configs: sensor_configs.to_vec(),
        })
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
        let _ = world.wait_for_tick_or_timeout(std::time::Duration::from_millis(100));

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
        tf_buffer: &TFBuffer,
        carla_config: &CarlaConfig,
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
            let mut sensor_bp = blueprint_library.find(blueprint_id).ok_or_else(|| {
                BridgeError::AutowareIssue(format!("Sensor blueprint '{}' not found", blueprint_id))
            })?;

            // Apply CARLA-specific parameters from config
            let config_sensor_type = match config.sensor_type {
                crate::bridge::sensor_bridge::SensorType::CameraRgb => {
                    crate::sensor_config::SensorType::Camera
                }
                crate::bridge::sensor_bridge::SensorType::LidarRayCast
                | crate::bridge::sensor_bridge::SensorType::LidarRayCastSemantic => {
                    crate::sensor_config::SensorType::Lidar
                }
                crate::bridge::sensor_bridge::SensorType::Imu => {
                    crate::sensor_config::SensorType::Imu
                }
                crate::bridge::sensor_bridge::SensorType::Gnss => {
                    crate::sensor_config::SensorType::Gnss
                }
                _ => {
                    tracing::warn!(
                        "Unknown sensor type for config mapping: {:?}",
                        config.sensor_type
                    );
                    crate::sensor_config::SensorType::Camera // Default fallback
                }
            };

            let sensor_params =
                carla_config.get_sensor_params(&config.link_name, config_sensor_type);
            sensor_params.apply_to_blueprint(&mut sensor_bp)?;

            // Try to get transform from TF buffer (base_link → sensor)
            tracing::info!(
                "Looking up TF transform: base_link → '{}'",
                config.link_name
            );
            let na_transform = match tf_buffer.lookup_transform("base_link", &config.link_name) {
                Ok(tf) => {
                    // Use TF transform
                    let trans = &tf.transform.translation;
                    let rot = &tf.transform.rotation;

                    tracing::info!(
                        "✓ Found TF for '{}': pos=({:.3}, {:.3}, {:.3}) parent='{}'",
                        config.link_name,
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
                    // Fall back to URDF data
                    tracing::error!(
                        "✗ TF lookup failed for '{}': {} - Using URDF data: pos=({:.3}, {:.3}, {:.3})",
                        config.link_name,
                        e,
                        config.position.x,
                        config.position.y,
                        config.position.z
                    );

                    // If both TF and URDF have zero position, this will fail in CARLA
                    if config.position.x.abs() < 0.001
                        && config.position.y.abs() < 0.001
                        && config.position.z.abs() < 0.001
                    {
                        tracing::error!(
                            "URDF position is also (0,0,0) for '{}' - CARLA will reject this!",
                            config.link_name
                        );
                    }

                    nalgebra::Isometry3::from_parts(
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
                    )
                }
            };

            // Convert ROS sensor transform to CARLA transform using centralized helper
            let carla_transform =
                crate::coordinate_conversion::ros_isometry_to_carla_transform(&na_transform);

            tracing::info!(
                "Sensor '{}' transform: ROS({:.3}, {:.3}, {:.3}) → CARLA({:.1}, {:.1}, {:.1})",
                config.link_name,
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

    /// Get reference to sensor configurations
    ///
    /// This returns the sensor configurations used to spawn the sensors,
    /// allowing main.rs to create sensor bridges with the correct parameters.
    pub fn get_sensor_configs(&self) -> &[SensorConfig] {
        &self.sensor_configs
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
    // Tests removed since they tested lifecycle state management
    // which is no longer part of CarlaVehicle
}
