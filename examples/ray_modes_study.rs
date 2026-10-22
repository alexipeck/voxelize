//! Compares ray-update semantics on synthetic warehouse scans, to inform the open
//! decisions in `docs/live-scene-plan.md`. It does not use the scene writer: each
//! variant updates its own map so the variants can be compared cell by cell.
//!
//!     cargo run --release --example ray_modes_study -- [voxel_m] [grid_origin_z] [scans]
//!
//! Variants, all with the reference log-odds deltas (+0.85 hit, -0.4 free, clamp ±4):
//!   exact            every ray traced as recorded, updates in input order (current behaviour)
//!   pairs-centre     endpoint quantization: one ray per unique (source voxel, hit voxel)
//!                    pair per scan, traced between voxel centres
//!   pairs-centroid   the same pairs, traced between the centroids of their sources and hits
//!   cell-dedup       exact rays, but each cell gets at most one update per scan, and a
//!                    cell that is both hit and crossed in a scan gets only the hit
//!
//! A "surface cell" is any voxel that holds at least one return. The share of surface
//! cells a variant leaves free is how much false free space a planner would see.

#[path = "bench_scan/scene.rs"]
mod scene;

use std::collections::{HashMap, HashSet};
use std::time::Instant;

use voxelaz::live::{RayCellRole, VoxelGeometry, VoxelKey, visit_ray};

const HIT: f32 = 0.85;
const FREE: f32 = -0.4;
const CLAMP: f32 = 4.0;
const STEP_LIMIT: u64 = 1 << 24;

type Map = HashMap<VoxelKey, f32>;

fn apply(map: &mut Map, key: VoxelKey, role: RayCellRole) {
    let delta = if role == RayCellRole::Hit { HIT } else { FREE };
    let cell = map.entry(key).or_insert(0.0);
    *cell = (*cell + delta).clamp(-CLAMP, CLAMP);
}

fn trace(
    geometry: &VoxelGeometry,
    source: [f64; 3],
    hit: [f64; 3],
    mut visit: impl FnMut(VoxelKey, RayCellRole),
) {
    visit_ray(geometry, source, hit, None, STEP_LIMIT, |event| {
        visit(event.key, event.role)
    })
    .expect("finite ray");
}

fn main() {
    let mut args = std::env::args().skip(1);
    let voxel: f64 = args
        .next()
        .map_or(0.2, |value| value.parse().expect("voxel_m"));
    let origin_z: f64 = args
        .next()
        .map_or(0.0, |value| value.parse().expect("grid_origin_z"));
    let scan_count: usize = args
        .next()
        .map_or(10, |value| value.parse().expect("scans"));
    let geometry = VoxelGeometry::new([0.0, 0.0, origin_z], voxel).expect("geometry");

    let world = scene::Warehouse::new(7);
    let lidar = scene::Lidar::for_rate(2.3e6, 10.0, 40.0);
    let per_period = 30;
    let speed = scene::PERIOD_M / per_period as f64 * 10.0;
    let scans = scene::cast_period(&world, &lidar, per_period, [speed, 0.0, 0.0], 0.1, 7);

    let mut exact = Map::new();
    let mut centre = Map::new();
    let mut centroid = Map::new();
    let mut dedup = Map::new();
    let mut surface = HashSet::new();
    let (mut rays, mut exact_events, mut pairs, mut pair_events, mut dedup_updates) =
        (0u64, 0u64, 0u64, 0u64, 0u64);
    let (mut exact_s, mut pair_s) = (0.0, 0.0);

    for scan in scans.iter().take(scan_count) {
        let observed: Vec<([f64; 3], [f64; 3])> = scan
            .offsets
            .iter()
            .zip(&scan.fired)
            .map(|(offset, fired)| {
                let source = [
                    scan.pose[0] + speed * f64::from(*fired) * 0.1,
                    scan.pose[1],
                    scan.pose[2],
                ];
                let hit = [
                    scan.pose[0] + f64::from(offset[0]),
                    scan.pose[1] + f64::from(offset[1]),
                    scan.pose[2] + f64::from(offset[2]),
                ];
                (source, hit)
            })
            .collect();
        rays += observed.len() as u64;

        let started = Instant::now();
        for (source, hit) in &observed {
            trace(&geometry, *source, *hit, |_, _| exact_events += 1);
        }
        exact_s += started.elapsed().as_secs_f64();
        for (source, hit) in &observed {
            surface.insert(geometry.voxel_key(*hit).unwrap());
            trace(&geometry, *source, *hit, |key, role| {
                apply(&mut exact, key, role)
            });
        }

        // Unique voxel pairs in first-seen order, with the sums for their centroids.
        let mut order = Vec::new();
        let mut sums: HashMap<(VoxelKey, VoxelKey), (u32, [f64; 6])> = HashMap::new();
        for (source, hit) in &observed {
            let pair = (
                geometry.voxel_key(*source).unwrap(),
                geometry.voxel_key(*hit).unwrap(),
            );
            let entry = sums.entry(pair).or_insert_with(|| {
                order.push(pair);
                (0, [0.0; 6])
            });
            entry.0 += 1;
            for axis in 0..3 {
                entry.1[axis] += source[axis];
                entry.1[axis + 3] += hit[axis];
            }
        }
        pairs += order.len() as u64;
        let started = Instant::now();
        for (from, to) in &order {
            trace(
                &geometry,
                geometry.voxel_center(*from),
                geometry.voxel_center(*to),
                |_, _| pair_events += 1,
            );
        }
        pair_s += started.elapsed().as_secs_f64();
        for pair in &order {
            trace(
                &geometry,
                geometry.voxel_center(pair.0),
                geometry.voxel_center(pair.1),
                |key, role| apply(&mut centre, key, role),
            );
            let (count, sum) = sums[pair];
            let n = f64::from(count);
            let from = [sum[0] / n, sum[1] / n, sum[2] / n];
            let to = [sum[3] / n, sum[4] / n, sum[5] / n];
            trace(&geometry, from, to, |key, role| {
                apply(&mut centroid, key, role)
            });
        }

        let mut crossed = HashSet::new();
        let mut hit_cells = HashSet::new();
        for (source, hit) in &observed {
            trace(&geometry, *source, *hit, |key, role| {
                if role == RayCellRole::Hit {
                    hit_cells.insert(key);
                } else {
                    crossed.insert(key);
                }
            });
        }
        for key in &hit_cells {
            apply(&mut dedup, *key, RayCellRole::Hit);
        }
        for key in crossed.difference(&hit_cells) {
            apply(&mut dedup, *key, RayCellRole::Free);
        }
        dedup_updates += (crossed.union(&hit_cells).count()) as u64;
    }

    println!(
        "voxel {voxel} m, grid origin z {origin_z} m, {scan_count} scans, {:.0} returns/scan, {} surface cells",
        rays as f64 / scan_count as f64,
        surface.len()
    );
    println!(
        "exact: {:.1} cells/ray; traversal alone {:.2} M rays/s",
        exact_events as f64 / rays as f64,
        rays as f64 / exact_s / 1e6
    );
    println!(
        "pairs: {:.1}x fewer traversals, {:.1}x fewer cell events, traversal time {:.1}x less",
        rays as f64 / pairs as f64,
        exact_events as f64 / pair_events as f64,
        exact_s / pair_s
    );
    println!(
        "cell-dedup: {:.1}x fewer model updates (traversals unchanged)",
        exact_events as f64 / dedup_updates as f64
    );
    for (name, map) in [
        ("exact", &exact),
        ("pairs-centre", &centre),
        ("pairs-centroid", &centroid),
        ("cell-dedup", &dedup),
    ] {
        let free = surface
            .iter()
            .filter(|key| map.get(*key).is_some_and(|value| *value < 0.0))
            .count();
        let occupied = map.values().filter(|value| **value > 0.0).count();
        println!(
            "{name:15} surface cells left free {:5.1}%   occupied cells {occupied}",
            100.0 * free as f64 / surface.len() as f64
        );
    }
}
