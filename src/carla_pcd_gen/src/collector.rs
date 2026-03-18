use carla::{
    client::{ActorBase, Sensor, World},
    geom::{Location, Rotation, Transform},
    sensor::{data::LidarMeasurement, SensorDataBase},
};
use crossbeam::channel;
use eyre::{Result, WrapErr};
use nalgebra::Point3;
use std::thread;
use tracing::info;

use crate::voxel_grid::VoxelGrid;

pub struct CollectorConfig {
    pub num_lidars: usize,
    pub ticks_per_position: u32,
    pub voxel_size: f32,
    pub max_samples: u32,
    pub lidar_range: f32,
    pub lidar_channels: u32,
}

pub fn collect(
    world: &mut World,
    spawn_points: &[Transform],
    config: &CollectorConfig,
) -> Result<VoxelGrid> {
    // Channel for streaming points from sensor callbacks to aggregator
    let (tx, rx) = channel::bounded::<(f32, f32, f32, f32)>(65536);

    // Spawn aggregator thread that builds the voxel grid
    let voxel_size = config.voxel_size;
    let max_samples = config.max_samples;
    let aggregator = thread::spawn(move || {
        let mut grid = VoxelGrid::new(voxel_size, max_samples);
        while let Ok((x, y, z, i)) = rx.recv() {
            grid.insert(x, y, z, i);
        }
        grid
    });

    // Configure LiDAR blueprint
    let bp_lib = world.blueprint_library()?;
    let mut bp = bp_lib
        .find("sensor.lidar.ray_cast")?
        .ok_or_else(|| eyre::eyre!("sensor.lidar.ray_cast blueprint not found"))?;

    let range_str = config.lidar_range.to_string();
    let channels_str = config.lidar_channels.to_string();
    // High point density for map generation
    let pps = (config.lidar_channels * 10000).to_string();

    for (attr, val) in [
        ("range", range_str.as_str()),
        ("channels", channels_str.as_str()),
        ("points_per_second", pps.as_str()),
        ("rotation_frequency", "20"),
        ("upper_fov", "15"),
        ("lower_fov", "-25"),
    ] {
        eyre::ensure!(
            bp.set_attribute(attr, val),
            "failed to set LiDAR attribute {attr}={val}"
        );
    }

    // Spawn N LiDAR sensors at a high initial position (off-map)
    let initial_tf = Transform {
        location: Location {
            x: 0.0,
            y: 0.0,
            z: 500.0,
        },
        rotation: Rotation {
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
        },
    };

    let mut sensors: Vec<Sensor> = Vec::with_capacity(config.num_lidars);
    for i in 0..config.num_lidars {
        let actor = world
            .spawn_actor(&bp, &initial_tf)
            .wrap_err_with(|| format!("failed to spawn LiDAR sensor {i}"))?;
        let sensor = Sensor::try_from(actor)
            .map_err(|_| eyre::eyre!("failed to convert actor to Sensor for LiDAR {i}"))?;

        let tx = tx.clone();
        sensor.listen(move |data| {
            let sensor_tf = data.sensor_transform().to_na();
            if let Ok(measure) = LidarMeasurement::try_from(data) {
                for det in measure.as_slice() {
                    let local_pt = Point3::new(det.point.x, det.point.y, det.point.z);
                    let world_pt = sensor_tf * local_pt;
                    // Points are stored in CARLA's native coordinate system (no Y-flip).
                    // The bridge also publishes live LiDAR scans in CARLA coordinates
                    // (see sensor_bridge.rs publish_lidar), so NDT matching is consistent.
                    let _ = tx.try_send((world_pt.x, world_pt.y, world_pt.z, det.intensity));
                }
            }
        })?;

        sensors.push(sensor);
    }
    info!("Spawned {} LiDAR sensors", sensors.len());

    // Drop the original sender so aggregator can finish when all sensor senders are dropped
    drop(tx);

    // Process spawn points in batches of N sensors
    let num_sensors = sensors.len();
    let total_positions = spawn_points.len();
    let total_batches = (total_positions + num_sensors - 1) / num_sensors;

    for (batch_idx, chunk) in spawn_points.chunks(num_sensors).enumerate() {
        // Teleport each sensor to its spawn point (with Z offset for roof height)
        for (sensor, sp) in sensors.iter().zip(chunk.iter()) {
            let tf = Transform {
                location: Location {
                    x: sp.location.x,
                    y: sp.location.y,
                    z: sp.location.z + 2.5,
                },
                rotation: sp.rotation.clone(),
            };
            sensor.set_transform(&tf)?;
        }

        // Tick the simulation to capture LiDAR data
        for _ in 0..config.ticks_per_position {
            world.tick()?;
        }

        if (batch_idx + 1) % 10 == 0 || batch_idx + 1 == total_batches {
            info!(
                "Batch {}/{} ({} spawn points processed)",
                batch_idx + 1,
                total_batches,
                ((batch_idx + 1) * num_sensors).min(total_positions),
            );
        }
    }

    // Stop listeners and destroy sensors
    for sensor in &sensors {
        sensor.stop()?;
    }
    // Drop sensors to release channel senders (each listen callback holds a tx clone)
    for sensor in sensors {
        sensor.destroy()?;
    }

    // Wait for aggregator to finish
    let grid = aggregator
        .join()
        .map_err(|_| eyre::eyre!("aggregator thread panicked"))?;

    Ok(grid)
}
