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
    coordinate_conversion,
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
/// Manages an existing CARLA hero vehicle and its spawned sensors.
/// The vehicle must already exist in CARLA (spawned externally, e.g. by demo_scenario.py).
pub struct CarlaVehicle {
    vehicle: Vehicle,
    sensors: HashMap<String, Sensor>,
    /// Sensor types keyed by link name (derived from VehicleConfig blueprints)
    sensor_types: HashMap<String, SensorType>,
    /// Where Autoware's `base_link` -- the rear-axle centre -- sits in the actor's own
    /// frame, in CARLA axes (x forward, y right, z up) and metres. About (-1.386, 0, 0)
    /// on vehicle.tesla.model3. Measured once, at adoption; see `measure_base_link`.
    base_link_in_actor: nalgebra::Vector3<f64>,
    /// Each sensor's attach transform (sensor frame -> actor frame, CARLA axes).
    sensor_mounts: HashMap<String, nalgebra::Isometry3<f32>>,
    /// The vehicle's bounding box in the actor frame: centre and half extents.
    body: Option<(nalgebra::Vector3<f32>, nalgebra::Vector3<f32>)>,
}

/// How far beyond the bounding box a lidar return still counts as the vehicle itself.
const SELF_BOX_MARGIN_M: f32 = 0.1;

/// A rear axle further than this from the actor origin is a bad reading, not a vehicle.
/// Half the length of a long bus; a car's is under 2 m.
// Only the wheel-based measurement uses it, and CARLA 0.10 has no wheel positions.
#[cfg_attr(carla_0100, allow(dead_code))]
const MAX_PLAUSIBLE_BASE_LINK_OFFSET_M: f64 = 6.0;

/// What `spawn_sensors` returns, keyed by sensor link name: the spawned actors, their
/// types, and each one's mount in the actor frame.
type SpawnedSensors = (
    HashMap<String, Sensor>,
    HashMap<String, SensorType>,
    HashMap<String, nalgebra::Isometry3<f32>>,
);

impl CarlaVehicle {
    /// Create a CarlaVehicle wrapper around an existing hero vehicle and spawn sensors on it
    ///
    /// The vehicle must already be present in CARLA (spawned externally). This constructor
    /// only attaches sensors to the provided vehicle.
    ///
    /// # Arguments
    /// * `world` - Mutable CARLA world reference
    /// * `vehicle` - Existing CARLA vehicle actor (role_name="hero")
    /// * `vehicle_config` - Vehicle and sensor configuration (single source of truth)
    /// * `tf_buffer` - TF buffer for sensor position lookups
    /// * `base_link_offset_x` - Override for the rear axle's position along the actor's
    ///   x axis (metres); `None` measures it from CARLA's wheel positions
    ///
    /// # Returns
    /// A CarlaVehicle instance managing the given vehicle and its spawned sensors
    pub fn new(
        world: &mut World,
        vehicle: Vehicle,
        vehicle_config: &VehicleConfig,
        tf_buffer: &TFBuffer,
        base_link_offset_x: Option<f64>,
    ) -> Result<Self> {
        // Log vehicle position in CARLA
        let spawned_transform = vehicle.transform()?;
        tracing::info!(
            "Hero vehicle found: ID={} at CARLA({:.1}, {:.1}, {:.1})",
            vehicle.id(),
            spawned_transform.location.x,
            spawned_transform.location.y,
            spawned_transform.location.z
        );

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

        // Before the sensors: they are declared relative to base_link, so where base_link
        // is decides where they are attached.
        let base_link_in_actor = Self::measure_base_link(&vehicle, base_link_offset_x);

        let (sensors, sensor_types, sensor_mounts) = Self::spawn_sensors(
            world,
            &vehicle,
            vehicle_config,
            tf_buffer,
            &base_link_in_actor,
        )?;

        tracing::info!("All sensors spawned successfully");

        let body = match vehicle.bounding_box() {
            Ok(bb) => Some((
                nalgebra::Vector3::new(
                    bb.transform.location.x,
                    bb.transform.location.y,
                    bb.transform.location.z,
                ),
                nalgebra::Vector3::new(bb.extent.x, bb.extent.y, bb.extent.z),
            )),
            Err(e) => {
                tracing::warn!("No bounding box for the vehicle ({e}); lidar self-returns kept");
                None
            }
        };

        Ok(Self {
            vehicle,
            sensors,
            sensor_types,
            base_link_in_actor,
            sensor_mounts,
            body,
        })
    }

    /// Locate Autoware's `base_link` (the rear-axle centre) in the actor's own frame.
    ///
    /// A CARLA actor's origin is not the rear axle -- on vehicle.tesla.model3 the axle is
    /// 1.386 m behind it -- while `vehicle_info.param.yaml` measures every overhang from
    /// the axle. Publishing the actor origin as `base_link` therefore put Autoware's idea of
    /// the body 1.386 m ahead of the real one. See carla-scenario-bridge
    /// docs/roadmap/014-feature-completeness.md, "Pose reference point".
    ///
    /// Measured from the rear wheels rather than hard-coded, because `vehicle_config.yaml`
    /// offers several blueprints. The rear pair is the two wheels that do not steer -- the
    /// same split scripts/extract_vehicle_params.py and `read_steer_geometry` use, and for a
    /// four-wheeled CARLA vehicle the same wheels as indices 2 and 3 (FL, FR, RL, RR).
    ///
    /// z is dropped: base_link stays at the actor origin's height. Only the horizontal
    /// placement is what Autoware's footprint gets wrong.
    ///
    /// Degrades to zero -- base_link at the actor origin, the behaviour before this --
    /// rather than failing the attach: a wrong reference point costs accuracy, not the run.
    fn measure_base_link(vehicle: &Vehicle, override_x: Option<f64>) -> nalgebra::Vector3<f64> {
        if let Some(x) = override_x {
            tracing::info!(
                "base_link at ({x:.3}, 0.000, 0.000) m in the actor frame (CARLA axes), from \
                 the base_link_offset_x parameter rather than the wheels"
            );
            return nalgebra::Vector3::new(x, 0.0, 0.0);
        }

        let fallback = |why: String| {
            tracing::warn!(
                "Cannot locate the rear axle ({why}); publishing base_link at the actor \
                 origin. Autoware will place the vehicle body ~1.4 m ahead of where it is \
                 (on a model3) and sensors will sit that far back. Set base_link_offset_x \
                 to override."
            );
            nalgebra::Vector3::zeros()
        };

        // CARLA 0.10's physics control carries no wheel world position: the Python API's
        // `offset`, `location` and `old_location` all read zero on 0.10.0 (measured
        // 2026-10-11, carla-scenario-bridge roadmap 019), and carla-rust has no `position`.
        // There the rear axle has to come from `base_link_offset_x`.
        #[cfg(carla_0100)]
        {
            let _ = vehicle;
            fallback(
                "CARLA 0.10 exposes no wheel positions; set base_link_offset_x for this \
                 blueprint"
                    .to_string(),
            )
        }

        #[cfg(not(carla_0100))]
        {
            let transform = match vehicle.transform() {
                Ok(t) => t,
                Err(e) => return fallback(format!("no actor transform: {e}")),
            };
            let physics = match vehicle.physics_control() {
                Ok(p) => p,
                Err(e) => return fallback(format!("physics_control failed: {e}")),
            };

            // `position` is world centimetres at the actor's current pose.
            let rear: Vec<nalgebra::Vector3<f64>> = physics
                .wheels
                .iter()
                .filter(|w| w.max_steer_angle <= 0.0)
                .map(|w| {
                    nalgebra::Vector3::new(
                        w.position.x as f64,
                        w.position.y as f64,
                        w.position.z as f64,
                    )
                })
                .collect();
            if rear.len() != 2 {
                return fallback(format!(
                    "expected 2 fixed (rear) wheels, found {} of {}",
                    rear.len(),
                    physics.wheels.len()
                ));
            }

            let location = nalgebra::Vector3::new(
                transform.location.x as f64,
                transform.location.y as f64,
                transform.location.z as f64,
            );
            let rotation = (
                transform.rotation.roll as f64,
                transform.rotation.pitch as f64,
                transform.rotation.yaw as f64,
            );
            let Some(measured) =
                coordinate_conversion::base_link_in_actor_from_wheels(&rear, &location, rotation)
            else {
                return fallback("no rear wheels".to_string());
            };

            let horizontal = measured.x.hypot(measured.y);
            if !horizontal.is_finite() || horizontal > MAX_PLAUSIBLE_BASE_LINK_OFFSET_M {
                // Wheels read before the vehicle's first physics step can report the world
                // origin, which lands here as an offset of hundreds of metres.
                return fallback(format!(
                    "measured ({:.3}, {:.3}) m from the actor origin, which is not a car",
                    measured.x, measured.y
                ));
            }

            let offset = nalgebra::Vector3::new(measured.x, measured.y, 0.0);
            tracing::info!(
                "base_link (rear-axle centre) at ({:.3}, {:.3}, 0.000) m in the actor frame \
                 (CARLA axes, x forward, y right); wheels put it {:.3} m below the origin, \
                 ignored",
                offset.x,
                offset.y,
                -measured.z
            );
            offset
        }
    }

    /// Where `base_link` (the rear-axle centre) sits in the actor's own frame, in CARLA
    /// axes and metres. Zero when it could not be measured.
    pub fn base_link_in_actor(&self) -> &nalgebra::Vector3<f64> {
        &self.base_link_in_actor
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
        base_link_in_actor: &nalgebra::Vector3<f64>,
    ) -> Result<SpawnedSensors> {
        // TF gives base_link -> sensor; CARLA attaches relative to the actor origin. The
        // two differ by the rear-axle offset, so attach at T_actor<-base_link *
        // T_base_link<-sensor. A pure translation, carried into ROS axes (Y-flip) because
        // that is the frame the TF transform is in until the conversion below.
        let base_link_ros =
            coordinate_conversion::carla_to_ros_position(base_link_in_actor).cast::<f32>();
        let actor_from_base_link = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::from(base_link_ros),
            nalgebra::UnitQuaternion::identity(),
        );

        let blueprint_library = world.blueprint_library()?;
        let mut spawned_sensors = HashMap::new();
        let mut sensor_types = HashMap::new();
        let mut mounts = HashMap::new();

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

            // base_link -> sensor, re-expressed from the actor origin (still ROS axes).
            let na_transform = actor_from_base_link * na_transform;

            // Convert ROS sensor transform to CARLA transform using centralized helper
            let carla_transform =
                crate::coordinate_conversion::ros_isometry_to_carla_transform(&na_transform);

            tracing::info!(
                "Sensor '{}' transform from actor origin: ROS({:.3}, {:.3}, {:.3}) → \
                 CARLA({:.3}, {:.3}, {:.3})",
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

            // In sync mode a spawned actor is not queryable until the next tick lands.
            //
            // SAFETY: deliberately ignored. Whoever owns the tick may be paused -- SSv2
            // pauses between frames for the whole of Autoware startup -- so a failure here
            // means "no frame yet", not "spawn failed". The sensor is used through
            // callbacks that CARLA only fires once it is live, so waiting is an
            // optimisation rather than a correctness requirement.
            if let Err(e) = world.wait_for_tick() {
                tracing::debug!("No tick while finalising sensor '{link_name}': {e}");
            }

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
            mounts.insert(link_name.clone(), carla_transform.to_na());
        }

        Ok((spawned_sensors, sensor_types, mounts))
    }

    /// The vehicle's body as seen from sensor `link_name`, for dropping lidar returns that
    /// hit the vehicle itself. `None` when the mount or the bounding box is unknown.
    pub fn self_box(&self, link_name: &str) -> Option<crate::bridge::sensor_bridge::SelfBox> {
        let mount = self.sensor_mounts.get(link_name)?;
        let (center, half) = self.body?;
        Some(crate::bridge::sensor_bridge::SelfBox {
            actor_from_sensor: *mount,
            center,
            half: half.add_scalar(SELF_BOX_MARGIN_M),
        })
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

    /// Stop listening to every sensor, without destroying anything.
    ///
    /// Called when the scenario runner is about to despawn the vehicle these are attached
    /// to. It destroys them with the vehicle, and a sensor destroyed while still listening
    /// leaves this client retrying a dead stream forever -- see `sensor_release` and
    /// `docs/issues/015`. Stopping first costs nothing and leaves nothing behind.
    ///
    /// Best effort: a sensor that is already gone has nothing useful to report.
    pub fn stop_sensors(&mut self) {
        let mut stopped = 0usize;
        for (name, sensor) in &self.sensors {
            match sensor.stop() {
                Ok(()) => stopped += 1,
                Err(e) => tracing::debug!("Sensor '{name}' stop failed (already gone?): {e}"),
            }
        }
        if stopped > 0 {
            tracing::info!("Stopped {stopped} sensor stream(s) before the vehicle is despawned");
        }
    }

    /// Cleanup: destroy spawned sensors only.
    ///
    /// The vehicle is owned by the scenario script, not the bridge, so it is not
    /// destroyed here. Only the sensors spawned by the bridge are cleaned up.
    pub fn cleanup(&mut self) -> Result<()> {
        for (name, sensor) in self.sensors.drain() {
            tracing::info!("Destroying sensor '{}' (ID: {})", name, sensor.id());

            // Close the stream before destroying the actor. Destroying a sensor that is
            // still listening leaves the server delivering into a session whose subscriber
            // is going away, which is the `Invalid session: no stream available with id N`
            // that precedes CARLA 0.9.16's teardown segfault. Every CARLA example stops a
            // sensor before destroying it, for this reason. See docs/issues/015-*.
            //
            // Best effort: a sensor that was never listening, or whose actor is already
            // gone, reports an error here and there is nothing useful to do about it.
            if let Err(e) = sensor.stop() {
                tracing::debug!("Sensor '{}' stop failed (already gone?): {e}", name);
            }

            match sensor.destroy() {
                Ok(true) => {}
                Ok(false) => tracing::warn!(
                    "Sensor '{}' destroy returned false - may already be destroyed",
                    name
                ),
                Err(e) => tracing::warn!("Sensor '{}' destroy failed: {e}", name),
            }
        }
        tracing::info!("Sensors cleaned up (vehicle owned by scenario script, not destroyed)");
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    // Tests removed since they tested lifecycle state management
    // which is no longer part of CarlaVehicle
}
