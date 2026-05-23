pub const CHUNK_SIZE: usize = 64;
pub const BOUNDED_QUEUE_CAPACITY: usize = 64;

pub struct VoxelPoint {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub count: u32,
}

pub struct VoxelGrid {
    pub roots: std::collections::HashMap<(i64, i64, i64), Box<[Option<Box<[Option<Box<[u32]>>]>>]>>,
    pub origin_meters: [f64; 3],
    pub dimensions_meters: [f64; 3],
    pub voxel_size: f64,
    pub inverse_voxel_size: f64,
    top_stride: i64,
    middle_stride: i64,
    leaf_stride: i64,
    level1_children_per_axis: i64,
    level2_children_per_axis: i64,
    level1_len: usize,
    level2_len: usize,
    leaf_len: usize,
}

impl VoxelGrid {
    pub fn new(origin_meters: [f64; 3], dimensions_meters: [f64; 3], voxel_size: f64) -> Self {
        assert!(voxel_size > 0.0);
        let inverse_voxel_size = 1.0 / voxel_size;
        let grid_x = (dimensions_meters[0] * inverse_voxel_size).ceil() as u64;
        let grid_y = (dimensions_meters[1] * inverse_voxel_size).ceil() as u64;
        let grid_z = (dimensions_meters[2] * inverse_voxel_size).ceil() as u64;
        let max_grid = grid_x.max(grid_y).max(grid_z);
        let exponent = if max_grid <= 1 {
            0
        } else {
            64 - (max_grid - 1).leading_zeros()
        };
        let leaf_stride = 1_i64 << ((exponent + 3) / 4);
        let middle_stride = 1_i64 << ((exponent * 2 + 3) / 4);
        let top_stride = 1_i64 << ((exponent * 3 + 3) / 4);
        let level1_children_per_axis = top_stride / middle_stride;
        let level2_children_per_axis = middle_stride / leaf_stride;
        let level1_len = (level1_children_per_axis
            * level1_children_per_axis
            * level1_children_per_axis) as usize;
        let level2_len = (level2_children_per_axis
            * level2_children_per_axis
            * level2_children_per_axis) as usize;
        let leaf_len = (leaf_stride * leaf_stride * leaf_stride) as usize;
        Self {
            roots: std::collections::HashMap::new(),
            origin_meters,
            dimensions_meters,
            voxel_size,
            inverse_voxel_size,
            top_stride,
            middle_stride,
            leaf_stride,
            level1_children_per_axis,
            level2_children_per_axis,
            level1_len,
            level2_len,
            leaf_len,
        }
    }

    pub fn insert(&mut self, x: f64, y: f64, z: f64) {
        let voxel_x = ((x - self.origin_meters[0]) * self.inverse_voxel_size).floor() as i64;
        let voxel_y = ((y - self.origin_meters[1]) * self.inverse_voxel_size).floor() as i64;
        let voxel_z = ((z - self.origin_meters[2]) * self.inverse_voxel_size).floor() as i64;
        let root_x = Self::div_floor(voxel_x, self.top_stride);
        let root_y = Self::div_floor(voxel_y, self.top_stride);
        let root_z = Self::div_floor(voxel_z, self.top_stride);
        let level1_x =
            Self::div_floor(voxel_x, self.middle_stride).rem_euclid(self.level1_children_per_axis);
        let level1_y =
            Self::div_floor(voxel_y, self.middle_stride).rem_euclid(self.level1_children_per_axis);
        let level1_z =
            Self::div_floor(voxel_z, self.middle_stride).rem_euclid(self.level1_children_per_axis);
        let level2_x =
            Self::div_floor(voxel_x, self.leaf_stride).rem_euclid(self.level2_children_per_axis);
        let level2_y =
            Self::div_floor(voxel_y, self.leaf_stride).rem_euclid(self.level2_children_per_axis);
        let level2_z =
            Self::div_floor(voxel_z, self.leaf_stride).rem_euclid(self.level2_children_per_axis);
        let local_x = voxel_x.rem_euclid(self.leaf_stride);
        let local_y = voxel_y.rem_euclid(self.leaf_stride);
        let local_z = voxel_z.rem_euclid(self.leaf_stride);
        let level1_index =
            Self::linear_index(level1_x, level1_y, level1_z, self.level1_children_per_axis);
        let level2_index =
            Self::linear_index(level2_x, level2_y, level2_z, self.level2_children_per_axis);
        let voxel_index = Self::linear_index(local_x, local_y, local_z, self.leaf_stride);
        let level1_len = self.level1_len;
        let level2_len = self.level2_len;
        let leaf_len = self.leaf_len;
        let level1_array = self
            .roots
            .entry((root_x, root_y, root_z))
            .or_insert_with(|| Self::new_level1_array(level1_len));
        let level2_array =
            level1_array[level1_index].get_or_insert_with(|| Self::new_level2_array(level2_len));
        let counts =
            level2_array[level2_index].get_or_insert_with(|| Self::new_leaf_array(leaf_len));
        counts[voxel_index] = counts[voxel_index].wrapping_add(1);
    }

    pub fn export(&self) -> Vec<VoxelPoint> {
        let mut points = Vec::new();
        for (&(root_x, root_y, root_z), level1_array) in &self.roots {
            for level1_index in 0..level1_array.len() {
                let Some(level2_array) = &level1_array[level1_index] else {
                    continue;
                };
                let (level1_x, level1_y, level1_z) =
                    Self::decode_index(level1_index, self.level1_children_per_axis);
                for level2_index in 0..level2_array.len() {
                    let Some(counts) = &level2_array[level2_index] else {
                        continue;
                    };
                    let (level2_x, level2_y, level2_z) =
                        Self::decode_index(level2_index, self.level2_children_per_axis);
                    for voxel_index in 0..counts.len() {
                        let count = counts[voxel_index];
                        if count == 0 {
                            continue;
                        }
                        let (local_x, local_y, local_z) =
                            Self::decode_index(voxel_index, self.leaf_stride);
                        let voxel_x = root_x * self.top_stride
                            + level1_x * self.middle_stride
                            + level2_x * self.leaf_stride
                            + local_x;
                        let voxel_y = root_y * self.top_stride
                            + level1_y * self.middle_stride
                            + level2_y * self.leaf_stride
                            + local_y;
                        let voxel_z = root_z * self.top_stride
                            + level1_z * self.middle_stride
                            + level2_z * self.leaf_stride
                            + local_z;
                        points.push(VoxelPoint {
                            x: self.origin_meters[0] + (voxel_x as f64 + 0.5) * self.voxel_size,
                            y: self.origin_meters[1] + (voxel_y as f64 + 0.5) * self.voxel_size,
                            z: self.origin_meters[2] + (voxel_z as f64 + 0.5) * self.voxel_size,
                            count,
                        });
                    }
                }
            }
        }
        points
    }

    fn new_level1_array(len: usize) -> Box<[Option<Box<[Option<Box<[u32]>>]>>]> {
        std::iter::repeat_with(|| None)
            .take(len)
            .collect::<Vec<_>>()
            .into_boxed_slice()
    }

    fn new_level2_array(len: usize) -> Box<[Option<Box<[u32]>>]> {
        std::iter::repeat_with(|| None)
            .take(len)
            .collect::<Vec<_>>()
            .into_boxed_slice()
    }

    fn new_leaf_array(len: usize) -> Box<[u32]> {
        vec![0u32; len].into_boxed_slice()
    }

    fn linear_index(x: i64, y: i64, z: i64, width: i64) -> usize {
        (x + y * width + z * width * width) as usize
    }

    fn decode_index(index: usize, width: i64) -> (i64, i64, i64) {
        let index = index as i64;
        let x = index % width;
        let y = (index / width) % width;
        let z = index / (width * width);
        (x, y, z)
    }

    fn div_floor(a: i64, b: i64) -> i64 {
        let q = a / b;
        let r = a % b;
        if r != 0 && ((r > 0) != (b > 0)) {
            q - 1
        } else {
            q
        }
    }
}
