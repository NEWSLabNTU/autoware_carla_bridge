mod collector;
mod pcd_writer;
mod voxel_grid;

use clap::Parser;
use eyre::Result;
use std::{path::PathBuf, time::Duration};
use tracing::info;

#[derive(Parser)]
#[command(
    name = "carla_pcd_gen",
    about = "Generate Autoware-compatible point cloud maps from CARLA"
)]
struct Cli {
    /// CARLA server host
    #[arg(long, default_value = "localhost")]
    host: String,

    /// CARLA server port
    #[arg(long, default_value_t = 2000)]
    port: u16,

    /// Map directory containing Autoware map files
    #[arg(long)]
    map_dir: PathBuf,

    /// Number of parallel LiDAR sensors
    #[arg(long, default_value_t = 8)]
    lidars: usize,

    /// Simulation ticks per spawn position
    #[arg(long, default_value_t = 2)]
    ticks_per_position: u32,

    /// Number of spawn points to use (0 = all)
    #[arg(long, default_value_t = 0)]
    spawn_points: usize,

    /// Voxel grid resolution in meters
    #[arg(long, default_value_t = 0.2)]
    voxel_size: f32,

    /// Maximum samples per voxel before saturation
    #[arg(long, default_value_t = 5)]
    max_samples: u32,

    /// LiDAR range in meters
    #[arg(long, default_value_t = 85.0)]
    lidar_range: f32,

    /// Number of LiDAR channels
    #[arg(long, default_value_t = 64)]
    lidar_channels: u32,
}

fn main() -> Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .init();

    let cli = Cli::parse();

    info!("Connecting to CARLA at {}:{}", cli.host, cli.port);
    let mut client = carla::client::Client::connect(&cli.host, cli.port, None)?;
    client.set_timeout(Duration::from_secs(30))?;

    let mut world = client.world()?;

    // Extract map name (e.g. "Town01" from "Carla/Maps/Town01")
    let map = world.map()?;
    let map_name_full = map.name();
    let map_name = map_name_full
        .split('/')
        .last()
        .unwrap_or(&map_name_full)
        .to_string();
    info!("Map: {map_name}");

    // Save original settings and switch to synchronous mode
    let original_settings = world.settings()?;
    let mut settings = original_settings.clone();
    settings.synchronous_mode = true;
    settings.fixed_delta_seconds = Some(0.05);
    settings.no_rendering_mode = true;
    world.apply_settings(&settings, Duration::from_secs(2))?;

    // Get spawn points
    let all_spawn_points = world.map()?.recommended_spawn_points()?;
    let all_slice = all_spawn_points.as_slice();
    let spawn_points: Vec<_> = if cli.spawn_points == 0 || cli.spawn_points >= all_slice.len() {
        all_slice.to_vec()
    } else {
        all_slice[..cli.spawn_points].to_vec()
    };
    info!("Using {} spawn points", spawn_points.len());

    // Collect point cloud
    let config = collector::CollectorConfig {
        num_lidars: cli.lidars,
        ticks_per_position: cli.ticks_per_position,
        voxel_size: cli.voxel_size,
        max_samples: cli.max_samples,
        lidar_range: cli.lidar_range,
        lidar_channels: cli.lidar_channels,
    };

    let grid = collector::collect(&mut world, &spawn_points, &config)?;
    info!("Collected {} voxels", grid.len());

    // Restore original settings
    world.apply_settings(&original_settings, Duration::from_secs(2))?;

    let map_dir = &cli.map_dir;
    std::fs::create_dir_all(map_dir)?;

    // Write PCD file
    let pcd_path = map_dir.join("pointcloud_map.pcd");
    pcd_writer::write_pcd(&grid, &pcd_path)?;

    // Summary
    let pcd_size = std::fs::metadata(&pcd_path)?.len();
    info!(
        "pointcloud_map.pcd: {} voxels, {} bytes",
        grid.len(),
        pcd_size
    );

    Ok(())
}
