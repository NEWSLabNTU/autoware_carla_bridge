use eyre::{Result, WrapErr};
use pcd_rs::{DataKind, PcdSerialize, WriterInit};
use std::path::Path;

use crate::voxel_grid::VoxelGrid;

#[derive(PcdSerialize)]
struct PcdPoint {
    x: f32,
    y: f32,
    z: f32,
}

pub fn write_pcd(grid: &VoxelGrid, path: &Path) -> Result<()> {
    let num_points = grid.len();
    let mut writer = WriterInit {
        width: num_points as u64,
        height: 1,
        viewpoint: Default::default(),
        data_kind: DataKind::Binary,
        schema: None,
    }
    .create(path)
    .map_err(|e| eyre::eyre!("{e}"))
    .wrap_err("failed to create PCD writer")?;

    for (x, y, z) in grid.iter() {
        writer
            .push(&PcdPoint { x, y, z })
            .map_err(|e| eyre::eyre!("{e}"))?;
    }
    writer.finish().map_err(|e| eyre::eyre!("{e}"))?;

    Ok(())
}
