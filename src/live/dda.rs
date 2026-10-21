use super::error::LiveError;
use super::geometry::{VoxelGeometry, VoxelKey, floor_exact};
use super::stats::BatchStats;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RayCellRole {
    Free,
    Hit,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayCellContext {
    pub key: VoxelKey,
    pub role: RayCellRole,
    pub ray_length_m: f64,
    pub t_enter: f64,
    pub t_exit: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RayWalk {
    pub free_cells: u64,
    pub hit_cells: u64,
}

pub fn cell_upper_bound(start: VoxelKey, end: VoxelKey) -> u128 {
    u128::from(start.x.abs_diff(end.x))
        + u128::from(start.y.abs_diff(end.y))
        + u128::from(start.z.abs_diff(end.z))
        + 1
}

pub fn visit_ray<F>(
    geometry: &VoxelGeometry,
    source: [f64; 3],
    hit: [f64; 3],
    ray_length_m: f64,
    step_limit: u64,
    mut visit: F,
) -> Result<RayWalk, LiveError>
where
    F: FnMut(RayCellContext),
{
    let src = geometry.grid_coord(source)?;
    let dst = geometry.grid_coord(hit)?;
    let hit_key = VoxelKey {
        x: floor_exact(dst[0])?,
        y: floor_exact(dst[1])?,
        z: floor_exact(dst[2])?,
    };
    let mut x = floor_exact(src[0])?;
    let mut y = floor_exact(src[1])?;
    let mut z = floor_exact(src[2])?;
    let delta = [dst[0] - src[0], dst[1] - src[1], dst[2] - src[2]];

    if x == hit_key.x && y == hit_key.y && z == hit_key.z {
        visit(context(x, y, z, RayCellRole::Hit, ray_length_m, 0.0, 1.0));
        return Ok(RayWalk {
            free_cells: 0,
            hit_cells: 1,
        });
    }

    let (step_x, t_delta_x, mut t_max_x) = axis_setup(src[0], x, delta[0]);
    let (step_y, t_delta_y, mut t_max_y) = axis_setup(src[1], y, delta[1]);
    let (step_z, t_delta_z, mut t_max_z) = axis_setup(src[2], z, delta[2]);
    skip_boundary(&mut x, step_x, &mut t_max_x, t_delta_x);
    skip_boundary(&mut y, step_y, &mut t_max_y, t_delta_y);
    skip_boundary(&mut z, step_z, &mut t_max_z, t_delta_z);

    if x == hit_key.x && y == hit_key.y && z == hit_key.z {
        visit(context(x, y, z, RayCellRole::Hit, ray_length_m, 0.0, 1.0));
        return Ok(RayWalk {
            free_cells: 0,
            hit_cells: 1,
        });
    }

    let mut t_enter: f64 = 0.0;
    let mut free_cells = 0u64;
    let mut steps = 0u64;
    loop {
        if steps > step_limit {
            return Err(LiveError::TraversalFailed {
                stats: BatchStats::default(),
            });
        }
        steps += 1;
        if x == hit_key.x && y == hit_key.y && z == hit_key.z {
            visit(context(
                x,
                y,
                z,
                RayCellRole::Hit,
                ray_length_m,
                t_enter.min(1.0),
                1.0,
            ));
            return Ok(RayWalk {
                free_cells,
                hit_cells: 1,
            });
        }
        let exit_t = t_max_x.min(t_max_y).min(t_max_z);
        if exit_t.is_nan() {
            return Err(LiveError::TraversalFailed {
                stats: BatchStats::default(),
            });
        }
        if exit_t >= 1.0 {
            if t_enter < 1.0 {
                visit(context(
                    x,
                    y,
                    z,
                    RayCellRole::Free,
                    ray_length_m,
                    t_enter,
                    1.0,
                ));
                free_cells += 1;
            }
            visit(context(
                hit_key.x,
                hit_key.y,
                hit_key.z,
                RayCellRole::Hit,
                ray_length_m,
                1.0,
                1.0,
            ));
            return Ok(RayWalk {
                free_cells,
                hit_cells: 1,
            });
        }
        if exit_t > t_enter {
            visit(context(
                x,
                y,
                z,
                RayCellRole::Free,
                ray_length_m,
                t_enter,
                exit_t,
            ));
            free_cells += 1;
        }
        let previous = (x, y, z);
        if t_max_x <= exit_t {
            x += step_x;
            t_max_x += t_delta_x;
        }
        if t_max_y <= exit_t {
            y += step_y;
            t_max_y += t_delta_y;
        }
        if t_max_z <= exit_t {
            z += step_z;
            t_max_z += t_delta_z;
        }
        if (x, y, z) == previous {
            return Err(LiveError::TraversalFailed {
                stats: BatchStats::default(),
            });
        }
        t_enter = exit_t;
    }
}

fn context(
    x: i64,
    y: i64,
    z: i64,
    role: RayCellRole,
    ray_length_m: f64,
    t_enter: f64,
    t_exit: f64,
) -> RayCellContext {
    RayCellContext {
        key: VoxelKey { x, y, z },
        role,
        ray_length_m,
        t_enter,
        t_exit,
    }
}

fn axis_setup(src: f64, voxel: i64, delta: f64) -> (i64, f64, f64) {
    if delta > 0.0 {
        let next = voxel as f64 + 1.0;
        (1, 1.0 / delta, (next - src) / delta)
    } else if delta < 0.0 {
        let next = voxel as f64;
        (-1, -1.0 / delta, (next - src) / delta)
    } else {
        (0, f64::INFINITY, f64::INFINITY)
    }
}

fn skip_boundary(voxel: &mut i64, step: i64, t_max: &mut f64, t_delta: f64) {
    if step != 0 && *t_max <= 0.0 {
        *voxel += step;
        *t_max += t_delta;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::live::error::LiveError;

    fn geometry(origin: [f64; 3], size: f64) -> VoxelGeometry {
        VoxelGeometry::new(origin, size).unwrap()
    }

    fn walk(geometry: &VoxelGeometry, source: [f64; 3], hit: [f64; 3]) -> Vec<RayCellContext> {
        let start = geometry.voxel_key(source).unwrap();
        let end = geometry.voxel_key(hit).unwrap();
        let limit = u64::try_from(cell_upper_bound(start, end).saturating_add(8)).unwrap();
        let mut cells = Vec::new();
        visit_ray(geometry, source, hit, 0.0, limit, |cell| cells.push(cell)).unwrap();
        cells
    }

    fn keys(cells: &[RayCellContext]) -> Vec<(VoxelKey, RayCellRole)> {
        cells.iter().map(|cell| (cell.key, cell.role)).collect()
    }

    fn assert_keys(
        geometry: &VoxelGeometry,
        source: [f64; 3],
        hit: [f64; 3],
        expected: &[(i64, i64, i64, RayCellRole)],
    ) {
        let actual = keys(&walk(geometry, source, hit));
        let expected: Vec<_> = expected
            .iter()
            .map(|(x, y, z, role)| {
                (
                    VoxelKey {
                        x: *x,
                        y: *y,
                        z: *z,
                    },
                    *role,
                )
            })
            .collect();
        assert_eq!(actual, expected);
    }

    #[test]
    fn axis_aligned_positive_and_negative() {
        let geometry = geometry([0.0, 0.0, 0.0], 1.0);
        assert_keys(
            &geometry,
            [0.5, 0.5, 0.5],
            [3.5, 0.5, 0.5],
            &[
                (0, 0, 0, RayCellRole::Free),
                (1, 0, 0, RayCellRole::Free),
                (2, 0, 0, RayCellRole::Free),
                (3, 0, 0, RayCellRole::Hit),
            ],
        );
        assert_keys(
            &geometry,
            [3.5, 0.5, 0.5],
            [0.5, 0.5, 0.5],
            &[
                (3, 0, 0, RayCellRole::Free),
                (2, 0, 0, RayCellRole::Free),
                (1, 0, 0, RayCellRole::Free),
                (0, 0, 0, RayCellRole::Hit),
            ],
        );
        assert_keys(
            &geometry,
            [0.5, -1.5, 0.2],
            [0.5, 1.2, 0.2],
            &[
                (0, -2, 0, RayCellRole::Free),
                (0, -1, 0, RayCellRole::Free),
                (0, 0, 0, RayCellRole::Free),
                (0, 1, 0, RayCellRole::Hit),
            ],
        );
    }

    #[test]
    fn crosses_origin_and_negative_voxels() {
        let geometry = geometry([0.0, 0.0, 0.0], 1.0);
        assert_keys(
            &geometry,
            [-1.5, 0.5, 0.5],
            [1.5, 0.5, 0.5],
            &[
                (-2, 0, 0, RayCellRole::Free),
                (-1, 0, 0, RayCellRole::Free),
                (0, 0, 0, RayCellRole::Free),
                (1, 0, 0, RayCellRole::Hit),
            ],
        );
    }

    #[test]
    fn zero_length_and_same_cell_emit_only_hit() {
        let geometry = geometry([0.0, 0.0, 0.0], 1.0);
        assert_keys(
            &geometry,
            [1.2, -0.2, 4.8],
            [1.2, -0.2, 4.8],
            &[(1, -1, 4, RayCellRole::Hit)],
        );
        assert_keys(
            &geometry,
            [0.1, 0.1, 0.1],
            [0.9, 0.2, 0.8],
            &[(0, 0, 0, RayCellRole::Hit)],
        );
    }

    #[test]
    fn boundary_start_end_and_ties() {
        let geometry = geometry([0.0, 0.0, 0.0], 1.0);
        assert_keys(
            &geometry,
            [0.0, 0.5, 0.5],
            [2.0, 0.5, 0.5],
            &[
                (0, 0, 0, RayCellRole::Free),
                (1, 0, 0, RayCellRole::Free),
                (2, 0, 0, RayCellRole::Hit),
            ],
        );
        assert_keys(
            &geometry,
            [2.0, 0.5, 0.5],
            [0.0, 0.5, 0.5],
            &[(1, 0, 0, RayCellRole::Free), (0, 0, 0, RayCellRole::Hit)],
        );
        assert_keys(
            &geometry,
            [0.5, 0.5, 0.5],
            [1.5, 1.5, 0.5],
            &[(0, 0, 0, RayCellRole::Free), (1, 1, 0, RayCellRole::Hit)],
        );
        assert_keys(
            &geometry,
            [0.5, 0.5, 0.5],
            [1.5, 1.5, 1.5],
            &[(0, 0, 0, RayCellRole::Free), (1, 1, 1, RayCellRole::Hit)],
        );
        assert_keys(
            &geometry,
            [1.0, 1.0, 1.0],
            [0.0, 0.0, 0.0],
            &[(0, 0, 0, RayCellRole::Hit)],
        );
        assert_keys(
            &geometry,
            [0.0, 0.0, 0.5],
            [2.0, 0.0, 0.5],
            &[
                (0, 0, 0, RayCellRole::Free),
                (1, 0, 0, RayCellRole::Free),
                (2, 0, 0, RayCellRole::Hit),
            ],
        );
    }

    #[test]
    fn shifted_origin_and_non_power_of_two_size() {
        let geometry = geometry([1.25, -4.0, 8.0], 0.3);
        let source = [1.25, -4.0, 8.0];
        let hit = [1.25 + 0.75, -4.0 - 0.15, 8.0 + 0.45];
        let cells = walk(&geometry, source, hit);
        assert_eq!(cells.last().unwrap().role, RayCellRole::Hit);
        assert!(
            cells
                .iter()
                .rev()
                .skip(1)
                .all(|cell| cell.role == RayCellRole::Free)
        );
        assert_eq!(cells.last().unwrap().key, geometry.voxel_key(hit).unwrap());
        let reference = reference_cells(&geometry, source, hit);
        assert_eq!(keys(&cells), reference);
    }

    #[test]
    fn small_voxels_and_endpoint_on_boundary() {
        let fine = geometry([0.0, 0.0, 0.0], 1.0e-4);
        let cells = walk(&fine, [0.0, 0.0, 0.0], [5.0e-4, 0.0, 0.0]);
        assert_eq!(cells.last().unwrap().key, VoxelKey { x: 5, y: 0, z: 0 });
        assert_eq!(cells.len(), 6);
        let unit = geometry([0.0, 0.0, 0.0], 1.0);
        let cells = walk(&unit, [0.5, 0.5, 0.5], [2.0, 0.5, 0.5]);
        assert_eq!(
            keys(&cells),
            vec![
                (VoxelKey { x: 0, y: 0, z: 0 }, RayCellRole::Free),
                (VoxelKey { x: 1, y: 0, z: 0 }, RayCellRole::Free),
                (VoxelKey { x: 2, y: 0, z: 0 }, RayCellRole::Hit),
            ]
        );
    }

    #[test]
    fn rejects_extreme_coordinates_and_matches_bound() {
        let geometry = geometry([0.0, 0.0, 0.0], 1.0);
        assert_eq!(
            visit_ray(
                &geometry,
                [1.0e20, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                0.0,
                10,
                |_| {}
            ),
            Err(LiveError::InvalidCoordinate)
        );
        let source = [0.5, 0.5, 0.5];
        let hit = [40.5, -10.5, 3.5];
        let cells = walk(&geometry, source, hit);
        let bound = cell_upper_bound(
            geometry.voxel_key(source).unwrap(),
            geometry.voxel_key(hit).unwrap(),
        );
        assert!(cells.len() as u128 <= bound);
        assert_eq!(
            cells
                .iter()
                .filter(|cell| cell.role == RayCellRole::Hit)
                .count(),
            1
        );
        assert!(cells.iter().any(|cell| cell.role == RayCellRole::Free));
        let unique: std::collections::HashSet<_> = cells.iter().map(|cell| cell.key).collect();
        assert_eq!(unique.len(), cells.len());
    }

    #[test]
    fn long_ray_terminates_and_is_deterministic() {
        let geometry = geometry([0.0, 0.0, 0.0], 1.0);
        let cells = walk(&geometry, [0.5, 0.2, -0.4], [250.5, 0.2, -0.4]);
        assert_eq!(cells.len(), 251);
        assert_eq!(cells.last().unwrap().role, RayCellRole::Hit);
        let again = walk(&geometry, [0.5, 0.2, -0.4], [250.5, 0.2, -0.4]);
        assert_eq!(keys(&cells), keys(&again));
        for cell in &cells {
            assert!(cell.t_enter >= 0.0 && cell.t_exit <= 1.0 + 1e-9);
            assert!(cell.t_exit + 1e-12 >= cell.t_enter);
        }
    }

    #[test]
    fn randomized_rays_match_slab_reference() {
        let geometries = [
            geometry([0.0, 0.0, 0.0], 1.0),
            geometry([2.5, -1.25, 0.5], 0.5),
            geometry([-8.0, 3.0, 1.0], 0.3),
            geometry([0.0, 0.0, 0.0], 1.7),
        ];
        for (index, geometry) in geometries.iter().enumerate() {
            let mut lcg = Lcg(0x1234_5678_9abc_def0 + index as u64 * 97);
            for _ in 0..80 {
                let source = random_point(&mut lcg, -4.0, 4.0);
                let hit = random_point(&mut lcg, -4.0, 4.0);
                if geometry.voxel_key(source).is_err() || geometry.voxel_key(hit).is_err() {
                    continue;
                }
                let span = cell_upper_bound(
                    geometry.voxel_key(source).unwrap(),
                    geometry.voxel_key(hit).unwrap(),
                );
                if span > 48 {
                    continue;
                }
                let actual = keys(&walk(geometry, source, hit));
                let expected = reference_cells(geometry, source, hit);
                assert_eq!(actual, expected, "source {source:?} hit {hit:?}");
                assert!(actual.len() as u128 <= span);
                assert_eq!(actual.last().map(|cell| cell.1), Some(RayCellRole::Hit));
                assert_eq!(
                    actual
                        .iter()
                        .filter(|cell| cell.1 == RayCellRole::Hit)
                        .count(),
                    1
                );
            }
        }
    }

    struct Lcg(u64);

    impl Lcg {
        fn next_unit(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            f64::from((self.0 >> 33) as u32) / f64::from(u32::MAX)
        }
    }

    fn random_point(lcg: &mut Lcg, lo: f64, hi: f64) -> [f64; 3] {
        [
            lo + (hi - lo) * lcg.next_unit(),
            lo + (hi - lo) * lcg.next_unit(),
            lo + (hi - lo) * lcg.next_unit(),
        ]
    }

    fn reference_cells(
        geometry: &VoxelGeometry,
        source: [f64; 3],
        hit: [f64; 3],
    ) -> Vec<(VoxelKey, RayCellRole)> {
        let src = geometry.grid_coord(source).unwrap();
        let dst = geometry.grid_coord(hit).unwrap();
        let start = VoxelKey {
            x: floor_exact(src[0]).unwrap(),
            y: floor_exact(src[1]).unwrap(),
            z: floor_exact(src[2]).unwrap(),
        };
        let end = VoxelKey {
            x: floor_exact(dst[0]).unwrap(),
            y: floor_exact(dst[1]).unwrap(),
            z: floor_exact(dst[2]).unwrap(),
        };
        let delta = [dst[0] - src[0], dst[1] - src[1], dst[2] - src[2]];
        let mut free = Vec::new();
        for z in (start.z.min(end.z) - 1)..=(start.z.max(end.z) + 1) {
            for y in (start.y.min(end.y) - 1)..=(start.y.max(end.y) + 1) {
                for x in (start.x.min(end.x) - 1)..=(start.x.max(end.x) + 1) {
                    let key = VoxelKey { x, y, z };
                    if key == end {
                        continue;
                    }
                    if let Some(t_enter) = positive_overlap(src, delta, key) {
                        free.push((t_enter, key));
                    }
                }
            }
        }
        free.sort_by(|left, right| {
            left.0
                .partial_cmp(&right.0)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(left.1.x.cmp(&right.1.x))
                .then(left.1.y.cmp(&right.1.y))
                .then(left.1.z.cmp(&right.1.z))
        });
        let mut cells: Vec<_> = free
            .into_iter()
            .map(|(_, key)| (key, RayCellRole::Free))
            .collect();
        cells.push((end, RayCellRole::Hit));
        cells
    }

    fn positive_overlap(src: [f64; 3], delta: [f64; 3], key: VoxelKey) -> Option<f64> {
        let mut lo: f64 = 0.0;
        let mut hi: f64 = 1.0;
        let mins = [key.x as f64, key.y as f64, key.z as f64];
        for axis in 0..3 {
            let min = mins[axis];
            let max = min + 1.0;
            let (enter, exit) = axis_window(src[axis], delta[axis], min, max)?;
            lo = lo.max(enter);
            hi = hi.min(exit);
            if hi <= lo {
                return None;
            }
        }
        let samples = [
            lo + (hi - lo) * 0.5,
            lo + (hi - lo) * 0.25,
            lo + (hi - lo) * 0.75,
        ];
        let inside = samples.into_iter().any(|t| {
            let point = [
                src[0] + delta[0] * t,
                src[1] + delta[1] * t,
                src[2] + delta[2] * t,
            ];
            floor_exact(point[0]).ok() == Some(key.x)
                && floor_exact(point[1]).ok() == Some(key.y)
                && floor_exact(point[2]).ok() == Some(key.z)
        });
        inside.then_some(lo)
    }

    fn axis_window(origin: f64, delta: f64, min: f64, max: f64) -> Option<(f64, f64)> {
        if delta == 0.0 {
            if origin >= min && origin < max {
                Some((f64::NEG_INFINITY, f64::INFINITY))
            } else {
                None
            }
        } else if delta > 0.0 {
            Some(((min - origin) / delta, (max - origin) / delta))
        } else {
            Some(((max - origin) / delta, (min - origin) / delta))
        }
    }
}
