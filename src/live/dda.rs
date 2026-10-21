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
    pub ray_length_m: Option<f64>,
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

#[derive(Clone, Copy, Debug)]
pub(crate) struct GridRay {
    pub(crate) src: [f64; 3],
    pub(crate) dst: [f64; 3],
    pub(crate) start: VoxelKey,
    pub(crate) end: VoxelKey,
}

impl GridRay {
    pub(crate) fn new(
        geometry: &VoxelGeometry,
        source: [f64; 3],
        hit: [f64; 3],
    ) -> Result<Self, LiveError> {
        let src = geometry.grid_coord(source)?;
        let dst = geometry.grid_coord(hit)?;
        Ok(Self {
            src,
            dst,
            start: key_of(src)?,
            end: key_of(dst)?,
        })
    }

    pub(crate) fn cell_upper_bound(&self) -> u128 {
        cell_upper_bound(self.start, self.end)
    }
}

fn key_of(grid: [f64; 3]) -> Result<VoxelKey, LiveError> {
    Ok(VoxelKey {
        x: floor_exact(grid[0])?,
        y: floor_exact(grid[1])?,
        z: floor_exact(grid[2])?,
    })
}

pub fn visit_ray<F>(
    geometry: &VoxelGeometry,
    source: [f64; 3],
    hit: [f64; 3],
    ray_length_m: Option<f64>,
    step_limit: u64,
    visit: F,
) -> Result<RayWalk, LiveError>
where
    F: FnMut(RayCellContext),
{
    let ray = GridRay::new(geometry, source, hit)?;
    walk(&ray, ray_length_m, step_limit, visit)
}

pub(crate) fn walk<F>(
    ray: &GridRay,
    ray_length_m: Option<f64>,
    step_limit: u64,
    mut visit: F,
) -> Result<RayWalk, LiveError>
where
    F: FnMut(RayCellContext),
{
    let target = [ray.end.x, ray.end.y, ray.end.z];
    let mut cell = [ray.start.x, ray.start.y, ray.start.z];
    let context = |cell: [i64; 3], role, t_enter, t_exit| RayCellContext {
        key: VoxelKey {
            x: cell[0],
            y: cell[1],
            z: cell[2],
        },
        role,
        ray_length_m,
        t_enter,
        t_exit,
    };
    let hit_only = RayWalk {
        free_cells: 0,
        hit_cells: 1,
    };

    if cell == target {
        visit(context(target, RayCellRole::Hit, 0.0, 1.0));
        return Ok(hit_only);
    }

    let delta = [
        ray.dst[0] - ray.src[0],
        ray.dst[1] - ray.src[1],
        ray.dst[2] - ray.src[2],
    ];
    let step = [step_of(delta[0]), step_of(delta[1]), step_of(delta[2])];
    let mut t_next = [0.0f64; 3];
    for axis in 0..3 {
        t_next[axis] = crossing(ray.src[axis], delta[axis], step[axis], cell[axis]);
        if step[axis] != 0 && t_next[axis] <= 0.0 {
            cell[axis] += step[axis];
            t_next[axis] = crossing(ray.src[axis], delta[axis], step[axis], cell[axis]);
        }
    }

    if cell == target {
        visit(context(target, RayCellRole::Hit, 0.0, 1.0));
        return Ok(hit_only);
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
        if cell == target {
            visit(context(target, RayCellRole::Hit, t_enter.min(1.0), 1.0));
            return Ok(RayWalk {
                free_cells,
                hit_cells: 1,
            });
        }
        let exit_t = t_next[0].min(t_next[1]).min(t_next[2]);
        if exit_t.is_nan() {
            return Err(LiveError::TraversalFailed {
                stats: BatchStats::default(),
            });
        }
        if exit_t >= 1.0 {
            if t_enter < 1.0 {
                visit(context(cell, RayCellRole::Free, t_enter, 1.0));
                free_cells += 1;
            }
            visit(context(target, RayCellRole::Hit, 1.0, 1.0));
            return Ok(RayWalk {
                free_cells,
                hit_cells: 1,
            });
        }
        if exit_t > t_enter {
            visit(context(cell, RayCellRole::Free, t_enter, exit_t));
            free_cells += 1;
        }
        let previous = cell;
        for axis in 0..3 {
            if t_next[axis] <= exit_t {
                cell[axis] += step[axis];
                t_next[axis] = crossing(ray.src[axis], delta[axis], step[axis], cell[axis]);
            }
        }
        if cell == previous {
            return Err(LiveError::TraversalFailed {
                stats: BatchStats::default(),
            });
        }
        t_enter = exit_t;
    }
}

#[inline]
fn step_of(delta: f64) -> i64 {
    if delta > 0.0 {
        1
    } else if delta < 0.0 {
        -1
    } else {
        0
    }
}

#[inline]
fn crossing(src: f64, delta: f64, step: i64, voxel: i64) -> f64 {
    match step {
        1 => ((voxel + 1) as f64 - src) / delta,
        -1 => (voxel as f64 - src) / delta,
        _ => f64::INFINITY,
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
        visit_ray(geometry, source, hit, None, limit, |cell| cells.push(cell)).unwrap();
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
                None,
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

    fn key_at(grid: [f64; 3]) -> VoxelKey {
        VoxelKey {
            x: floor_exact(grid[0]).unwrap(),
            y: floor_exact(grid[1]).unwrap(),
            z: floor_exact(grid[2]).unwrap(),
        }
    }

    fn crossing_reference(
        geometry: &VoxelGeometry,
        source: [f64; 3],
        hit: [f64; 3],
    ) -> Vec<(VoxelKey, RayCellRole)> {
        let src = geometry.grid_coord(source).unwrap();
        let dst = geometry.grid_coord(hit).unwrap();
        let start = key_at(src);
        let end = key_at(dst);
        let delta = [dst[0] - src[0], dst[1] - src[1], dst[2] - src[2]];
        let offset = [
            src[0] - start.x as f64,
            src[1] - start.y as f64,
            src[2] - start.z as f64,
        ];
        let mut times = vec![0.0, 1.0];
        for axis in 0..3 {
            if delta[axis] == 0.0 {
                continue;
            }
            let lo = src[axis].min(dst[axis]);
            let hi = src[axis].max(dst[axis]);
            let mut boundary = lo.floor() + 1.0;
            while boundary < hi {
                times.push((boundary - src[axis]) / delta[axis]);
                boundary += 1.0;
            }
        }
        times.sort_by(f64::total_cmp);
        times.dedup();
        let mut cells: Vec<(VoxelKey, RayCellRole)> = Vec::new();
        for pair in times.windows(2) {
            if pair[1] <= pair[0] {
                continue;
            }
            let mid = 0.5 * (pair[0] + pair[1]);
            let key = VoxelKey {
                x: start.x + floor_exact(offset[0] + delta[0] * mid).unwrap(),
                y: start.y + floor_exact(offset[1] + delta[1] * mid).unwrap(),
                z: start.z + floor_exact(offset[2] + delta[2] * mid).unwrap(),
            };
            if key != end && cells.last() != Some(&(key, RayCellRole::Free)) {
                cells.push((key, RayCellRole::Free));
            }
        }
        cells.push((end, RayCellRole::Hit));
        cells
    }

    fn assert_steps_follow_direction(
        geometry: &VoxelGeometry,
        source: [f64; 3],
        hit: [f64; 3],
        cells: &[RayCellContext],
    ) {
        let src = geometry.grid_coord(source).unwrap();
        let dst = geometry.grid_coord(hit).unwrap();
        let sign = |value: f64| {
            if value > 0.0 {
                1
            } else if value < 0.0 {
                -1
            } else {
                0
            }
        };
        let direction = [
            sign(dst[0] - src[0]),
            sign(dst[1] - src[1]),
            sign(dst[2] - src[2]),
        ];
        for pair in cells.windows(2) {
            let step = [
                pair[1].key.x - pair[0].key.x,
                pair[1].key.y - pair[0].key.y,
                pair[1].key.z - pair[0].key.z,
            ];
            assert_ne!(step, [0, 0, 0], "source {source:?} hit {hit:?}");
            for axis in 0..3 {
                assert!(
                    step[axis] == 0 || step[axis] == direction[axis],
                    "source {source:?} hit {hit:?} step {step:?}"
                );
            }
        }
    }

    fn axes_changed(pair: &[RayCellContext]) -> usize {
        usize::from(pair[0].key.x != pair[1].key.x)
            + usize::from(pair[0].key.y != pair[1].key.y)
            + usize::from(pair[0].key.z != pair[1].key.z)
    }

    #[test]
    fn long_two_axis_ties_step_together() {
        let geometry = geometry([0.0, 0.0, 0.0], 1.0);
        let source = [0.5, 0.5, 0.5];
        let hit = [30.5, 10.5, 0.5];
        let cells = walk(&geometry, source, hit);
        assert_eq!(cells.len(), 31);
        assert_eq!(cells.last().unwrap().key, VoxelKey { x: 30, y: 10, z: 0 });
        let diagonal = cells
            .windows(2)
            .filter(|pair| axes_changed(pair) == 2)
            .count();
        assert_eq!(diagonal, 10);
        assert_eq!(keys(&cells), crossing_reference(&geometry, source, hit));
        assert_steps_follow_direction(&geometry, source, hit, &cells);
    }

    #[test]
    fn long_three_axis_ties_from_corner_to_corner() {
        let geometry = geometry([0.0, 0.0, 0.0], 1.0);
        let source = [0.0, 0.0, 0.0];
        let hit = [24.0, 12.0, 8.0];
        let cells = walk(&geometry, source, hit);
        assert_eq!(cells.len(), 25);
        let free = &cells[..24];
        assert!(free.iter().all(|cell| cell.role == RayCellRole::Free));
        for (k, cell) in free.iter().enumerate() {
            let k = k as i64;
            assert_eq!(
                cell.key,
                VoxelKey {
                    x: k,
                    y: k / 2,
                    z: k / 3
                }
            );
            assert!(cell.t_exit > cell.t_enter);
        }
        let triple = free
            .windows(2)
            .filter(|pair| axes_changed(pair) == 3)
            .count();
        let double = free
            .windows(2)
            .filter(|pair| axes_changed(pair) == 2)
            .count();
        assert_eq!(triple, 3);
        assert_eq!(double, 12);
        assert_eq!(cells[24].key, VoxelKey { x: 24, y: 12, z: 8 });
        assert_eq!(cells[24].role, RayCellRole::Hit);
        assert_eq!(keys(&cells), crossing_reference(&geometry, source, hit));
        assert_steps_follow_direction(&geometry, source, hit, &cells);
    }

    #[test]
    fn long_negative_ties_with_shifted_origin() {
        let geometry = geometry([0.0, 0.0, 0.0], 1.0);
        let source = [19.75, 8.0, 6.0];
        let hit = [-13.75, 17.0, -12.0];
        let cells = walk(&geometry, source, hit);
        assert_eq!(keys(&cells), crossing_reference(&geometry, source, hit));
        assert_steps_follow_direction(&geometry, source, hit, &cells);
        for cell in &cells[..cells.len() - 1] {
            assert!(cell.t_exit > cell.t_enter);
        }
        let shifted = geometry_at([2.0, -3.0, 1.5], 0.25);
        let source = [2.0 + 6.0 * 0.25, -3.0 - 2.0 * 0.25, 1.5 + 9.0 * 0.25];
        let hit = [2.0 - 30.0 * 0.25, -3.0 + 18.0 * 0.25, 1.5 - 3.0 * 0.25];
        let cells = walk(&shifted, source, hit);
        assert_eq!(keys(&cells), crossing_reference(&shifted, source, hit));
        assert_steps_follow_direction(&shifted, source, hit, &cells);
    }

    #[test]
    fn exact_grid_endpoint_adds_no_free_cell_beyond_the_hit() {
        let geometry = geometry([0.0, 0.0, 0.0], 0.25);
        let source = [-0.8125, -0.3125, 1.375];
        let hit = [0.0, -1.0, 0.5];
        let cells = walk(&geometry, source, hit);
        assert_eq!(keys(&cells), crossing_reference(&geometry, source, hit));
        assert_steps_follow_direction(&geometry, source, hit, &cells);
        let end = cells.last().unwrap();
        assert_eq!(end.key, VoxelKey { x: 0, y: -4, z: 2 });
        for cell in &cells[..cells.len() - 1] {
            assert!(cell.key.x <= -1 && cell.key.y >= -4 && cell.key.z >= 2);
        }
    }

    #[test]
    fn exact_grid_rays_match_crossing_reference() {
        let origins = [[0.0, 0.0, 0.0], [2.0, -3.0, 1.5]];
        let mut lcg = Lcg(0x5eed_0000_0000_0001);
        let mut multi_axis_steps = 0usize;
        for index in 0..6_000usize {
            let size = [0.25, 0.5, 1.0][index % 3];
            let geometry = geometry_at(origins[(index / 3) % 2], size);
            let mut quarter = || ((lcg.next_unit() * 48.0 - 24.0) * 4.0).round() / 4.0 * size;
            let source = [quarter(), quarter(), quarter()];
            let hit = [quarter(), quarter(), quarter()];
            let cells = walk(&geometry, source, hit);
            assert_eq!(
                keys(&cells),
                crossing_reference(&geometry, source, hit),
                "source {source:?} hit {hit:?} size {size}"
            );
            assert_steps_follow_direction(&geometry, source, hit, &cells);
            multi_axis_steps += cells
                .windows(2)
                .filter(|pair| axes_changed(pair) > 1)
                .count();
        }
        assert!(multi_axis_steps > 1_000);
    }

    #[test]
    fn long_random_rays_terminate_with_monotone_steps() {
        let sizes = [0.05, 0.1, 0.2, 0.3, 1.7];
        let mut lcg = Lcg(0xfeed_beef_0000_0007);
        for index in 0..4_000usize {
            let size = sizes[index % sizes.len()];
            let origin = random_point(&mut lcg, -50.0, 50.0);
            let geometry = geometry_at(origin, size);
            let source = random_point(&mut lcg, -500.0, 500.0);
            let mut direction = random_point(&mut lcg, -1.0, 1.0);
            let norm = (direction[0] * direction[0]
                + direction[1] * direction[1]
                + direction[2] * direction[2])
                .sqrt()
                .max(1.0e-6);
            for value in &mut direction {
                *value /= norm;
            }
            let length = lcg.next_unit() * 400.0 * size;
            let hit = [
                source[0] + direction[0] * length,
                source[1] + direction[1] * length,
                source[2] + direction[2] * length,
            ];
            let cells = walk(&geometry, source, hit);
            assert_eq!(cells.last().unwrap().key, geometry.voxel_key(hit).unwrap());
            assert_eq!(
                cells
                    .iter()
                    .filter(|cell| cell.role == RayCellRole::Hit)
                    .count(),
                1
            );
            let unique: std::collections::HashSet<_> = cells.iter().map(|cell| cell.key).collect();
            assert_eq!(unique.len(), cells.len());
            assert_steps_follow_direction(&geometry, source, hit, &cells);
            for cell in &cells[..cells.len() - 1] {
                assert!(cell.t_exit > cell.t_enter);
            }
        }
    }

    #[test]
    fn extreme_indices_traverse_exactly() {
        let geometry = geometry([0.0, 0.0, 0.0], 1.0);
        let big = 1_125_899_906_842_624.0;
        let source = [big + 0.5, -big + 0.5, 0.5];
        let hit = [big + 20.5, -big + 7.5, 0.5];
        let cells = walk(&geometry, source, hit);
        assert_eq!(cells.len(), 28);
        assert_eq!(keys(&cells), crossing_reference(&geometry, source, hit));
        let back = walk(&geometry, hit, source);
        assert_eq!(back.len(), 28);
        assert_eq!(keys(&back), crossing_reference(&geometry, hit, source));
        assert_steps_follow_direction(&geometry, hit, source, &back);
    }

    fn geometry_at(origin: [f64; 3], size: f64) -> VoxelGeometry {
        VoxelGeometry::new(origin, size).unwrap()
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
