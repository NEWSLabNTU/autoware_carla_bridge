use std::collections::HashMap;

struct VoxelCell {
    sum_x: f64,
    sum_y: f64,
    sum_z: f64,
    sum_i: f64,
    count: u32,
}

pub struct VoxelGrid {
    cells: HashMap<(i32, i32, i32), VoxelCell>,
    resolution: f32,
    max_samples: u32,
}

impl VoxelGrid {
    pub fn new(resolution: f32, max_samples: u32) -> Self {
        Self {
            cells: HashMap::new(),
            resolution,
            max_samples,
        }
    }

    /// Insert a point. Returns false if the voxel is already saturated.
    pub fn insert(&mut self, x: f32, y: f32, z: f32, intensity: f32) -> bool {
        let key = (
            (x / self.resolution).floor() as i32,
            (y / self.resolution).floor() as i32,
            (z / self.resolution).floor() as i32,
        );
        let cell = self.cells.entry(key).or_insert(VoxelCell {
            sum_x: 0.0,
            sum_y: 0.0,
            sum_z: 0.0,
            sum_i: 0.0,
            count: 0,
        });
        if cell.count >= self.max_samples {
            return false;
        }
        cell.sum_x += x as f64;
        cell.sum_y += y as f64;
        cell.sum_z += z as f64;
        cell.sum_i += intensity as f64;
        cell.count += 1;
        true
    }

    pub fn len(&self) -> usize {
        self.cells.len()
    }

    /// Iterate averaged (x, y, z) per voxel.
    pub fn iter(&self) -> impl Iterator<Item = (f32, f32, f32)> + '_ {
        self.cells.values().map(|cell| {
            let n = cell.count as f64;
            (
                (cell.sum_x / n) as f32,
                (cell.sum_y / n) as f32,
                (cell.sum_z / n) as f32,
            )
        })
    }
}
