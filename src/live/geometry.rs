use super::error::LiveError;

const EXACT_INDEX_LIMIT: f64 = 9_007_199_254_740_992.0;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct VoxelKey {
    pub x: i64,
    pub y: i64,
    pub z: i64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct ChunkKey {
    pub x: i64,
    pub y: i64,
    pub z: i64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VoxelBounds {
    pub min: VoxelKey,
    pub max: VoxelKey,
}

impl VoxelBounds {
    pub fn try_new(min: VoxelKey, max: VoxelKey) -> Result<Self, LiveError> {
        if min.x > max.x || min.y > max.y || min.z > max.z {
            return Err(LiveError::InvalidBounds);
        }
        Ok(Self { min, max })
    }

    pub fn contains(&self, key: VoxelKey) -> bool {
        key.x >= self.min.x
            && key.x <= self.max.x
            && key.y >= self.min.y
            && key.y <= self.max.y
            && key.z >= self.min.z
            && key.z <= self.max.z
    }
}

#[derive(Clone, Copy, Debug)]
pub struct VoxelGeometry {
    origin: [f64; 3],
    voxel_size: f64,
    inv_voxel_size: f64,
}

impl VoxelGeometry {
    pub fn new(origin: [f64; 3], voxel_size: f64) -> Result<Self, LiveError> {
        if !origin.iter().copied().all(f64::is_finite) {
            return Err(LiveError::InvalidOrigin);
        }
        if !voxel_size.is_finite() || voxel_size <= 0.0 {
            return Err(LiveError::InvalidVoxelSize);
        }
        Ok(Self {
            origin,
            voxel_size,
            inv_voxel_size: 1.0 / voxel_size,
        })
    }

    pub fn origin(&self) -> [f64; 3] {
        self.origin
    }

    pub fn voxel_size(&self) -> f64 {
        self.voxel_size
    }

    pub fn voxel_key(&self, world: [f64; 3]) -> Result<VoxelKey, LiveError> {
        let grid = self.grid_coord(world)?;
        Ok(VoxelKey {
            x: floor_exact(grid[0])?,
            y: floor_exact(grid[1])?,
            z: floor_exact(grid[2])?,
        })
    }

    pub fn grid_coord(&self, world: [f64; 3]) -> Result<[f64; 3], LiveError> {
        let mut grid = [0.0; 3];
        for axis in 0..3 {
            if !world[axis].is_finite() {
                return Err(LiveError::InvalidCoordinate);
            }
            let scaled = (world[axis] - self.origin[axis]) * self.inv_voxel_size;
            if !scaled.is_finite() || scaled.abs() >= EXACT_INDEX_LIMIT {
                return Err(LiveError::InvalidCoordinate);
            }
            grid[axis] = scaled;
        }
        Ok(grid)
    }

    pub fn voxel_lower(&self, key: VoxelKey) -> [f64; 3] {
        [
            self.origin[0] + key.x as f64 * self.voxel_size,
            self.origin[1] + key.y as f64 * self.voxel_size,
            self.origin[2] + key.z as f64 * self.voxel_size,
        ]
    }

    pub fn voxel_center(&self, key: VoxelKey) -> [f64; 3] {
        let lower = self.voxel_lower(key);
        [
            lower[0] + 0.5 * self.voxel_size,
            lower[1] + 0.5 * self.voxel_size,
            lower[2] + 0.5 * self.voxel_size,
        ]
    }
}

pub(crate) fn floor_exact(scaled: f64) -> Result<i64, LiveError> {
    let floored = scaled.floor();
    if !floored.is_finite() || floored.abs() >= EXACT_INDEX_LIMIT {
        return Err(LiveError::InvalidCoordinate);
    }
    Ok(floored as i64)
}

pub fn chunk_and_local(voxel: VoxelKey, side: i64) -> (ChunkKey, u16) {
    let chunk = ChunkKey {
        x: voxel.x.div_euclid(side),
        y: voxel.y.div_euclid(side),
        z: voxel.z.div_euclid(side),
    };
    let local_x = voxel.x.rem_euclid(side) as u16;
    let local_y = voxel.y.rem_euclid(side) as u16;
    let local_z = voxel.z.rem_euclid(side) as u16;
    let width = side as u16;
    let local = local_x + width * local_y + width * width * local_z;
    (chunk, local)
}

pub fn voxel_from_chunk(chunk: ChunkKey, local: u16, side: i64) -> VoxelKey {
    let width = side as u16;
    let area = width * width;
    let local_z = local / area;
    let remainder = local % area;
    let local_y = remainder / width;
    let local_x = remainder % width;
    VoxelKey {
        x: chunk.x * side + i64::from(local_x),
        y: chunk.y * side + i64::from(local_y),
        z: chunk.z * side + i64::from(local_z),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn geometry(origin: [f64; 3], size: f64) -> VoxelGeometry {
        VoxelGeometry::new(origin, size).unwrap()
    }

    #[test]
    fn floor_semantics_for_negative_and_shifted_origin() {
        let geometry = geometry([8.0, -4.0, 1.0], 0.5);
        assert_eq!(
            geometry.voxel_key([8.0, -4.0, 1.0]).unwrap(),
            VoxelKey { x: 0, y: 0, z: 0 }
        );
        assert_eq!(
            geometry.voxel_key([7.75, -4.0, 1.0]).unwrap(),
            VoxelKey { x: -1, y: 0, z: 0 }
        );
        assert_eq!(
            geometry.voxel_key([9.0, -5.0, 2.0]).unwrap(),
            VoxelKey { x: 2, y: -2, z: 2 }
        );
        let key = geometry.voxel_key([-4.0, 3.0, -8.0]).unwrap();
        let lower = geometry.voxel_lower(key);
        let point = [-4.0, 3.0, -8.0];
        for axis in 0..3 {
            assert!(point[axis] >= lower[axis]);
            assert!(point[axis] < lower[axis] + geometry.voxel_size());
        }
        let center = geometry.voxel_center(VoxelKey { x: 2, y: -3, z: 4 });
        assert_eq!(center[0], 8.0 + 2.5 * 0.5);
        assert_eq!(center[1], -4.0 + -2.5 * 0.5);
        assert_eq!(center[2], 1.0 + 4.5 * 0.5);
    }

    #[test]
    fn boundary_face_belongs_to_the_higher_index() {
        let geometry = geometry([0.0, 0.0, 0.0], 1.0);
        assert_eq!(
            geometry.voxel_key([1.0, 0.0, 0.0]).unwrap(),
            VoxelKey { x: 1, y: 0, z: 0 }
        );
        assert_eq!(
            geometry.voxel_key([-1.0, -2.0, -3.0]).unwrap(),
            VoxelKey {
                x: -1,
                y: -2,
                z: -3
            }
        );
        assert_eq!(
            geometry.voxel_key([-0.0, -0.0, -0.0]).unwrap(),
            VoxelKey { x: 0, y: 0, z: 0 }
        );
    }

    #[test]
    fn rejects_non_finite_and_non_representable_indices() {
        let geometry = geometry([0.0, 0.0, 0.0], 1.0);
        assert_eq!(
            geometry.voxel_key([f64::NAN, 0.0, 0.0]),
            Err(LiveError::InvalidCoordinate)
        );
        assert_eq!(
            geometry.voxel_key([f64::INFINITY, 0.0, 0.0]),
            Err(LiveError::InvalidCoordinate)
        );
        assert_eq!(
            geometry.voxel_key([1.0e20, 0.0, 0.0]),
            Err(LiveError::InvalidCoordinate)
        );
        assert_eq!(
            geometry.voxel_key([-EXACT_INDEX_LIMIT, 0.0, 0.0]),
            Err(LiveError::InvalidCoordinate)
        );
        assert!(
            geometry
                .voxel_key([EXACT_INDEX_LIMIT - 4096.0, 0.0, 1.0])
                .is_ok()
        );
        assert!(VoxelGeometry::new([f64::NAN, 0.0, 0.0], 1.0).is_err());
        assert!(VoxelGeometry::new([0.0, 0.0, 0.0], 0.0).is_err());
        assert!(VoxelGeometry::new([0.0, 0.0, 0.0], -1.0).is_err());
        assert!(VoxelGeometry::new([0.0, 0.0, 0.0], f64::INFINITY).is_err());
    }

    #[test]
    fn chunk_mapping_uses_euclidean_division() {
        let side = 16i64;
        let samples = [-33, -32, -17, -16, -1, 0, 15, 16, 31];
        for x in samples {
            for y in samples {
                for z in [0, -1, 16] {
                    let voxel = VoxelKey { x, y, z };
                    let (chunk, local) = chunk_and_local(voxel, side);
                    assert_eq!(chunk.x, x.div_euclid(side));
                    assert_eq!(chunk.y, y.div_euclid(side));
                    assert_eq!(chunk.z, z.div_euclid(side));
                    let lx = x.rem_euclid(side) as u16;
                    let ly = y.rem_euclid(side) as u16;
                    let lz = z.rem_euclid(side) as u16;
                    assert_eq!(local, lx + 16 * ly + 256 * lz);
                    assert_eq!(voxel_from_chunk(chunk, local, side), voxel);
                }
            }
        }
    }
}
