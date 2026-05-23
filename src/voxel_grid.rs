use anyhow::Result;

pub const CHUNK_SIZE: usize = 64;
pub const BOUNDED_QUEUE_CAPACITY: usize = 64;

pub struct VoxelPoint {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub count: u16,
}

enum Leaf {
    SparseCount(Vec<(u16, u16)>),
    SparsePresence(Vec<u16>),
    DenseCount(Box<[u16]>),
    DensePresence(Box<[u64]>),
}

type Level2Slot = Option<Box<Leaf>>;
type Level1Array = Box<[Level2Slot]>;
type Level1Slot = Option<Level1Array>;
type RootLevel1 = Box<[Level1Slot]>;

pub struct VoxelGrid {
    roots: std::collections::HashMap<(i64, i64, i64), RootLevel1>,
    pub origin_meters: [f64; 3],
    pub dimensions_meters: [f64; 3],
    pub voxel_size: f64,
    pub inverse_voxel_size: f64,
    counts_as_intensity: bool,
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
    pub fn new(
        origin_meters: [f64; 3],
        dimensions_meters: [f64; 3],
        voxel_size: f64,
        counts_as_intensity: bool,
    ) -> Self {
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
            counts_as_intensity,
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
        let counts_as_intensity = self.counts_as_intensity;
        let level1_array = self
            .roots
            .entry((root_x, root_y, root_z))
            .or_insert_with(|| Self::new_level1_array(level1_len));
        let level2_array =
            level1_array[level1_index].get_or_insert_with(|| Self::new_level2_array(level2_len));
        let leaf = level2_array[level2_index]
            .get_or_insert_with(|| Self::new_sparse_leaf(counts_as_intensity));
        Self::leaf_insert(leaf, voxel_index, leaf_len, counts_as_intensity);
    }

    pub fn drain<F>(mut self, mut visit: F) -> Result<()>
    where
        F: FnMut(VoxelPoint) -> Result<()>,
    {
        let origin_meters = self.origin_meters;
        let voxel_size = self.voxel_size;
        let top_stride = self.top_stride;
        let middle_stride = self.middle_stride;
        let leaf_stride = self.leaf_stride;
        let level1_children_per_axis = self.level1_children_per_axis;
        let level2_children_per_axis = self.level2_children_per_axis;
        let counts_as_intensity = self.counts_as_intensity;

        for ((root_x, root_y, root_z), mut level1_array) in self.roots.drain() {
            for level1_index in 0..level1_array.len() {
                let Some(mut level2_array) = std::mem::take(&mut level1_array[level1_index]) else {
                    continue;
                };
                let (level1_x, level1_y, level1_z) =
                    Self::decode_index(level1_index, level1_children_per_axis);
                for level2_index in 0..level2_array.len() {
                    let Some(leaf) = std::mem::take(&mut level2_array[level2_index]) else {
                        continue;
                    };
                    let (level2_x, level2_y, level2_z) =
                        Self::decode_index(level2_index, level2_children_per_axis);
                    let mut emit = |voxel_index: usize, count: u16| -> Result<()> {
                        if counts_as_intensity && count == 0 {
                            return Ok(());
                        }
                        let (local_x, local_y, local_z) =
                            Self::decode_index(voxel_index, leaf_stride);
                        let voxel_x = root_x * top_stride
                            + level1_x * middle_stride
                            + level2_x * leaf_stride
                            + local_x;
                        let voxel_y = root_y * top_stride
                            + level1_y * middle_stride
                            + level2_y * leaf_stride
                            + local_y;
                        let voxel_z = root_z * top_stride
                            + level1_z * middle_stride
                            + level2_z * leaf_stride
                            + local_z;
                        visit(VoxelPoint {
                            x: origin_meters[0] + (voxel_x as f64 + 0.5) * voxel_size,
                            y: origin_meters[1] + (voxel_y as f64 + 0.5) * voxel_size,
                            z: origin_meters[2] + (voxel_z as f64 + 0.5) * voxel_size,
                            count: if counts_as_intensity { count } else { 1 },
                        })
                    };
                    match *leaf {
                        Leaf::SparseCount(entries) => {
                            for &(idx, count) in &entries {
                                emit(idx as usize, count)?;
                            }
                        }
                        Leaf::SparsePresence(indices) => {
                            for &idx in &indices {
                                emit(idx as usize, 1)?;
                            }
                        }
                        Leaf::DenseCount(counts) => {
                            for voxel_index in 0..counts.len() {
                                emit(voxel_index, counts[voxel_index])?;
                            }
                        }
                        Leaf::DensePresence(bits) => {
                            for (word_index, word) in bits.iter().enumerate() {
                                let mut word = *word;
                                while word != 0 {
                                    let bit = word.trailing_zeros() as usize;
                                    let voxel_index = word_index * 64 + bit;
                                    emit(voxel_index, 1)?;
                                    word &= word - 1;
                                }
                            }
                        }
                    }
                }
            }
        }
        Ok(())
    }

    fn leaf_insert(leaf: &mut Leaf, voxel_index: usize, leaf_len: usize, counts_as_intensity: bool) {
        if counts_as_intensity {
            Self::leaf_insert_count(leaf, voxel_index, leaf_len);
        } else {
            Self::leaf_insert_presence(leaf, voxel_index, leaf_len);
        }
    }

    fn leaf_insert_count(leaf: &mut Leaf, voxel_index: usize, leaf_len: usize) {
        match leaf {
            Leaf::DenseCount(counts) => {
                Self::bump_count(&mut counts[voxel_index]);
            }
            Leaf::SparseCount(entries) => {
                let idx = voxel_index as u16;
                for entry in entries.iter_mut() {
                    if entry.0 == idx {
                        Self::bump_count(&mut entry.1);
                        return;
                    }
                }
                if entries.len() + 1 >= Self::dense_promote_at(leaf_len) {
                    let mut dense = Self::sparse_count_to_dense(entries, leaf_len);
                    Self::bump_count(&mut dense[voxel_index]);
                    *leaf = Leaf::DenseCount(dense);
                    return;
                }
                entries.push((idx, 1));
            }
            _ => unreachable!(),
        }
    }

    fn leaf_insert_presence(leaf: &mut Leaf, voxel_index: usize, leaf_len: usize) {
        match leaf {
            Leaf::DensePresence(bits) => {
                if Self::test_bit(bits, voxel_index) {
                    return;
                }
                Self::set_bit(bits, voxel_index);
            }
            Leaf::SparsePresence(indices) => {
                let idx = voxel_index as u16;
                for &existing in indices.iter() {
                    if existing == idx {
                        return;
                    }
                }
                if indices.len() + 1 >= Self::dense_promote_at(leaf_len) {
                    let dense = Self::sparse_presence_to_dense(indices, leaf_len);
                    *leaf = Leaf::DensePresence(dense);
                    if let Leaf::DensePresence(bits) = leaf {
                        Self::set_bit(bits, voxel_index);
                    }
                    return;
                }
                indices.push(idx);
            }
            _ => unreachable!(),
        }
    }

    fn new_sparse_leaf(counts_as_intensity: bool) -> Box<Leaf> {
        if counts_as_intensity {
            Box::new(Leaf::SparseCount(Vec::with_capacity(4)))
        } else {
            Box::new(Leaf::SparsePresence(Vec::with_capacity(4)))
        }
    }

    fn sparse_count_to_dense(entries: &[(u16, u16)], leaf_len: usize) -> Box<[u16]> {
        let mut dense = vec![0u16; leaf_len];
        for &(idx, count) in entries {
            dense[idx as usize] = count;
        }
        dense.into_boxed_slice()
    }

    fn sparse_presence_to_dense(indices: &[u16], leaf_len: usize) -> Box<[u64]> {
        let word_count = leaf_len.div_ceil(64);
        let mut bits = vec![0u64; word_count];
        for &idx in indices {
            Self::set_bit(&mut bits, idx as usize);
        }
        bits.into_boxed_slice()
    }

    fn set_bit(bits: &mut [u64], index: usize) {
        bits[index / 64] |= 1u64 << (index % 64);
    }

    fn test_bit(bits: &[u64], index: usize) -> bool {
        (bits[index / 64] & (1u64 << (index % 64))) != 0
    }

    fn dense_promote_at(leaf_len: usize) -> usize {
        (leaf_len / 4).max(128)
    }

    fn new_level1_array(len: usize) -> RootLevel1 {
        std::iter::repeat_with(|| None)
            .take(len)
            .collect::<Vec<_>>()
            .into_boxed_slice()
    }

    fn new_level2_array(len: usize) -> Level1Array {
        std::iter::repeat_with(|| None)
            .take(len)
            .collect::<Vec<_>>()
            .into_boxed_slice()
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

    fn bump_count(slot: &mut u16) {
        *slot = slot.saturating_add(1);
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
