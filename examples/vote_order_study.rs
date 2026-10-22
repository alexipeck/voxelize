//! Does ray order matter, and what do per-voxel vote counts buy? Runs on the synthetic
//! warehouse scans and feeds `docs/live-scene-plan.md` (D1, D2).
//!
//!     cargo run --release --example vote_order_study -- [voxel_m] [scans]
//!
//! Compares, on exact rays:
//!   log-odds   the reference clamp (+0.85 hit, -0.4 free, ±4), rays in recorded order and
//!              shuffled within each scan
//!   votes      two integers per voxel (hits, misses), recorded order and shuffled; the
//!              planner decides blocked or free at read time from the two counts
//!   pair-hint  votes, but each unique (source voxel, hit voxel) pair in a scan is traced
//!              once along its first ray's path and that path gets the pair's full count
//!
//! A "surface cell" is any voxel that holds at least one return.

#[path = "bench_scan/scene.rs"]
mod scene;

use std::collections::{HashMap, HashSet};

use voxelaz::live::{RayCellRole, VoxelGeometry, VoxelKey, visit_ray};

use scene::Lcg;

const HIT: f32 = 0.85;
const FREE: f32 = -0.4;
const CLAMP: f32 = 4.0;
const STEP_LIMIT: u64 = 1 << 24;
const RATIOS: [f64; 4] = [0.5, 0.2, 0.1, 0.05];

#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
struct Votes {
    hits: u32,
    misses: u32,
}

impl Votes {
    /// Blocked when at least `ratio` of the votes are hits.
    fn blocked(&self, ratio: f64) -> bool {
        let total = f64::from(self.hits) + f64::from(self.misses);
        total > 0.0 && f64::from(self.hits) >= ratio * total
    }
}

type Rays = Vec<([f64; 3], [f64; 3])>;

fn trace(
    geometry: &VoxelGeometry,
    ray: &([f64; 3], [f64; 3]),
    mut visit: impl FnMut(VoxelKey, RayCellRole),
) {
    visit_ray(geometry, ray.0, ray.1, None, STEP_LIMIT, |event| {
        visit(event.key, event.role)
    })
    .expect("finite ray");
}

fn log_odds(geometry: &VoxelGeometry, scans: &[Rays]) -> HashMap<VoxelKey, f32> {
    let mut map = HashMap::new();
    for rays in scans {
        for ray in rays {
            trace(geometry, ray, |key, role| {
                let delta = if role == RayCellRole::Hit { HIT } else { FREE };
                let cell = map.entry(key).or_insert(0.0f32);
                *cell = (*cell + delta).clamp(-CLAMP, CLAMP);
            });
        }
    }
    map
}

fn votes(geometry: &VoxelGeometry, scans: &[Rays]) -> HashMap<VoxelKey, Votes> {
    let mut map: HashMap<VoxelKey, Votes> = HashMap::new();
    for rays in scans {
        for ray in rays {
            trace(geometry, ray, |key, role| {
                let cell = map.entry(key).or_default();
                match role {
                    RayCellRole::Hit => cell.hits += 1,
                    RayCellRole::Free => cell.misses += 1,
                }
            });
        }
    }
    map
}

/// Votes from one traced path per unique voxel pair, weighted by how many rays share it.
/// Returns the map, the number of traversals and the number of cell writes.
fn pair_hint(geometry: &VoxelGeometry, scans: &[Rays]) -> (HashMap<VoxelKey, Votes>, u64, u64) {
    let mut map: HashMap<VoxelKey, Votes> = HashMap::new();
    let (mut traversals, mut writes) = (0u64, 0u64);
    for rays in scans {
        let mut first: HashMap<(VoxelKey, VoxelKey), (usize, u32)> = HashMap::new();
        let mut order = Vec::new();
        for (index, ray) in rays.iter().enumerate() {
            let pair = (
                geometry.voxel_key(ray.0).unwrap(),
                geometry.voxel_key(ray.1).unwrap(),
            );
            let entry = first.entry(pair).or_insert_with(|| {
                order.push(pair);
                (index, 0)
            });
            entry.1 += 1;
        }
        for pair in &order {
            let (index, count) = first[pair];
            traversals += 1;
            trace(geometry, &rays[index], |key, role| {
                writes += 1;
                let cell = map.entry(key).or_default();
                match role {
                    RayCellRole::Hit => cell.hits += count,
                    RayCellRole::Free => cell.misses += count,
                }
            });
        }
    }
    (map, traversals, writes)
}

fn class(value: Option<&f32>) -> i8 {
    match value {
        None => 2,
        Some(value) if *value > 0.0 => 1,
        Some(value) if *value < 0.0 => -1,
        Some(_) => 0,
    }
}

fn main() {
    let mut args = std::env::args().skip(1);
    let voxel: f64 = args
        .next()
        .map_or(0.1, |value| value.parse().expect("voxel_m"));
    let scan_count: usize = args
        .next()
        .map_or(10, |value| value.parse().expect("scans"));
    let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], voxel).expect("geometry");

    let world = scene::Warehouse::new(7);
    let lidar = scene::Lidar::for_rate(2.3e6, 10.0, 40.0);
    let per_period = 30;
    let speed = scene::PERIOD_M / per_period as f64 * 10.0;
    let cast = scene::cast_period(&world, &lidar, per_period, [speed, 0.0, 0.0], 0.1, 7);
    let recorded: Vec<Rays> = cast
        .iter()
        .take(scan_count)
        .map(|scan| {
            scan.offsets
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
                .collect()
        })
        .collect();
    let mut rng = Lcg::new(99);
    let shuffled: Vec<Rays> = recorded
        .iter()
        .map(|rays| {
            let mut rays = rays.clone();
            for index in (1..rays.len()).rev() {
                let other = (rng.unit() * (index + 1) as f64) as usize;
                rays.swap(index, other.min(index));
            }
            rays
        })
        .collect();
    let surface: HashSet<VoxelKey> = recorded
        .iter()
        .flatten()
        .map(|ray| geometry.voxel_key(ray.1).unwrap())
        .collect();
    let rays: usize = recorded.iter().map(Vec::len).sum();

    let odds = log_odds(&geometry, &recorded);
    let odds_shuffled = log_odds(&geometry, &shuffled);
    let changed = odds
        .iter()
        .filter(|(key, value)| class(Some(value)) != class(odds_shuffled.get(key)))
        .count();
    let largest = odds
        .iter()
        .map(|(key, value)| (value - odds_shuffled[key]).abs())
        .fold(0.0f32, f32::max);
    let odds_free = surface
        .iter()
        .filter(|key| odds.get(*key).is_some_and(|value| *value < 0.0))
        .count();

    let counted = votes(&geometry, &recorded);
    let counted_shuffled = votes(&geometry, &shuffled);
    let (hinted, traversals, writes) = pair_hint(&geometry, &recorded);
    let events: u64 = counted
        .values()
        .map(|votes| u64::from(votes.hits) + u64::from(votes.misses))
        .sum();
    let most_misses = counted
        .values()
        .map(|votes| votes.misses)
        .max()
        .unwrap_or(0);
    let most_hits = counted.values().map(|votes| votes.hits).max().unwrap_or(0);

    println!(
        "voxel {voxel} m, {scan_count} scans, {rays} rays, {} voxels observed, {} surface cells",
        counted.len(),
        surface.len()
    );
    println!(
        "order, log-odds: shuffling rays within each scan changed the class of {changed} of {} voxels ({:.3}%), largest value change {largest:.2}",
        odds.len(),
        100.0 * changed as f64 / odds.len() as f64
    );
    println!(
        "order, votes: shuffled map identical to recorded map: {}",
        counted == counted_shuffled
    );
    println!(
        "votes: largest counts in one voxel after {scan_count} scans: {most_misses} misses, {most_hits} hits"
    );
    println!(
        "surface cells left free: log-odds {:.1}%",
        100.0 * odds_free as f64 / surface.len() as f64
    );
    for ratio in RATIOS {
        let free = |map: &HashMap<VoxelKey, Votes>| {
            surface
                .iter()
                .filter(|key| map.get(*key).is_some_and(|votes| !votes.blocked(ratio)))
                .count()
        };
        let differ = counted
            .iter()
            .filter(|(key, votes)| {
                hinted.get(*key).map(|other| other.blocked(ratio)) != Some(votes.blocked(ratio))
            })
            .count();
        println!(
            "surface cells left free: votes, blocked at hits >= {:>4.0}% of votes: {:5.1}%   pair-hint {:5.1}% (class differs from votes in {:.2}% of voxels)",
            ratio * 100.0,
            100.0 * free(&counted) as f64 / surface.len() as f64,
            100.0 * free(&hinted) as f64 / surface.len() as f64,
            100.0 * differ as f64 / counted.len() as f64
        );
    }
    println!(
        "pair-hint: {:.2}x fewer traversals, {:.2}x fewer cell writes than one write per cell event",
        rays as f64 / traversals as f64,
        events as f64 / writes as f64
    );
}
