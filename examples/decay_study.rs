//! How fast does each decay mode clear an obstacle that has gone, and does it keep the
//! surfaces that are still there? Feeds `docs/live-scene-plan.md` (D7).
//!
//!     cargo run --release --example decay_study -- [voxel_m]
//!
//! A hovering sensor watches the warehouse plus a 1 × 1 × 1.5 m box 5 m away for 2 s
//! (20 passes); then the box is removed and it watches for 6 s more. The box's voxels are
//! the ones blocked at the removal that never get a hit when the same passes are run
//! without the box. For each decay mode and blocked rule it reports how many passes after
//! the removal they take to clear, and how many of the other blocked voxels are still
//! blocked at the end. Decay is the library's `VoteDecay`, applied at each publish, which
//! here is once per pass.

#[path = "bench_scan/scene.rs"]
mod scene;

use std::collections::HashSet;

use voxelaz::live::{
    BlockedRule, RayObservation, RaySceneConfig, RaySceneWriter, VoteDecay, VoteModel, VoxelBounds,
    VoxelGeometry, VoxelKey,
};

use scene::{Lidar, Scan, Warehouse};

const PRESENT: usize = 20;
const AFTER: usize = 60;
const BOX_MIN: [f64; 3] = [5.0, 2.0, 0.0];
const BOX_MAX: [f64; 3] = [6.0, 3.0, 1.5];

fn rules() -> [(&'static str, BlockedRule); 3] {
    [
        ("any hit", BlockedRule::any_hit()),
        ("hits >= 0.5 vote", BlockedRule::new(0.5, 0.0).unwrap()),
        ("hit share >= 10%", BlockedRule::new(0.0, 0.10).unwrap()),
    ]
}

fn cast_all(lidar: &Lidar, pose: [f64; 3], box_present: bool) -> Vec<Scan> {
    let with_box = Warehouse::new(7).with_box(BOX_MIN, BOX_MAX);
    let without = Warehouse::new(7);
    let total = PRESENT + AFTER;
    let mut scans: Vec<Option<Scan>> = (0..total).map(|_| None).collect();
    let workers = std::thread::available_parallelism().map_or(1, usize::from);
    std::thread::scope(|scope| {
        for (worker, slots) in scans.chunks_mut(total.div_ceil(workers)).enumerate() {
            let first = worker * total.div_ceil(workers);
            let (with_box, without) = (&with_box, &without);
            scope.spawn(move || {
                for (offset, slot) in slots.iter_mut().enumerate() {
                    let pass = first + offset;
                    let world = if box_present && pass < PRESENT {
                        with_box
                    } else {
                        without
                    };
                    *slot = Some(scene::cast_scan(
                        world,
                        lidar,
                        pose,
                        [0.0; 3],
                        0.1,
                        1_000 + pass as u64,
                    ));
                }
            });
        }
    });
    scans.into_iter().map(Option::unwrap).collect()
}

fn main() {
    let voxel: f64 = std::env::args()
        .nth(1)
        .map_or(0.25, |value| value.parse().expect("voxel_m"));
    let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], voxel).expect("voxel size");
    let lidar = Lidar::for_rate(2.3e6, 10.0, 40.0);
    let pose = [0.0, 0.4, 1.8];
    let scans = cast_all(&lidar, pose, true);
    let reference = cast_all(&lidar, pose, false);
    let rays_of = |scan: &Scan| -> Vec<RayObservation> {
        scan.offsets
            .iter()
            .map(|offset| RayObservation {
                source: pose,
                hit: [
                    pose[0] + f64::from(offset[0]),
                    pose[1] + f64::from(offset[1]),
                    pose[2] + f64::from(offset[2]),
                ],
            })
            .collect()
    };
    let static_hits: HashSet<VoxelKey> = {
        let (mut writer, reader) = RaySceneWriter::<_, 16>::open(
            geometry,
            &RaySceneConfig::default(),
            VoteModel::default(),
        )
        .unwrap();
        for scan in &reference {
            writer.insert_rays(&rays_of(scan)).unwrap();
        }
        writer.publish().unwrap();
        reader
            .snapshot_all()
            .iter()
            .filter(|(_, votes)| votes.hits > 0)
            .map(|(key, _)| key)
            .collect()
    };
    let near_box = VoxelBounds::try_new(
        geometry
            .voxel_key([BOX_MIN[0] - voxel, BOX_MIN[1] - voxel, BOX_MIN[2]])
            .unwrap(),
        geometry
            .voxel_key([BOX_MAX[0] + voxel, BOX_MAX[1] + voxel, BOX_MAX[2] + voxel])
            .unwrap(),
    )
    .unwrap();

    println!(
        "voxel {voxel} m, hovering sensor, box present for {PRESENT} passes then removed for {AFTER}"
    );
    println!(
        "| decay | rule | box voxels blocked at removal | passes to clear half | passes to clear all | other blocked voxels kept at end |"
    );
    println!("|---|---|---|---|---|---|");
    let modes = [
        ("none", VoteDecay::None),
        ("exp:0.9", VoteDecay::Exponential { keep: 0.9 }),
        ("exp:0.7", VoteDecay::Exponential { keep: 0.7 }),
        ("cap:1000", VoteDecay::CapHalving { cap_votes: 1000 }),
        ("cap:200", VoteDecay::CapHalving { cap_votes: 200 }),
    ];
    let rules = rules();
    for (label, mode) in modes {
        let (mut writer, reader) = RaySceneWriter::<_, 16>::open(
            geometry,
            &RaySceneConfig::default(),
            VoteModel::new(mode).unwrap(),
        )
        .unwrap();
        let mut box_counts: Vec<Vec<usize>> = vec![Vec::new(); rules.len()];
        let mut box_at_removal: Vec<HashSet<VoxelKey>> = vec![HashSet::new(); rules.len()];
        let mut other_at_removal: Vec<HashSet<VoxelKey>> = vec![HashSet::new(); rules.len()];
        for (pass, scan) in scans.iter().enumerate() {
            writer.insert_rays(&rays_of(scan)).unwrap();
            writer.publish().unwrap();
            if pass + 1 == PRESENT {
                let all = reader.snapshot_all();
                for (index, (_, rule)) in rules.iter().enumerate() {
                    for (key, votes) in all.iter() {
                        if rule.is_blocked(votes) {
                            if near_box.contains(key) && !static_hits.contains(&key) {
                                box_at_removal[index].insert(key);
                            } else if !near_box.contains(key) {
                                other_at_removal[index].insert(key);
                            }
                        }
                    }
                }
            }
            if pass + 1 >= PRESENT {
                let region = reader.snapshot_region(near_box).unwrap();
                for (index, (_, rule)) in rules.iter().enumerate() {
                    let still = box_at_removal[index]
                        .iter()
                        .filter(|key| {
                            region
                                .get(**key)
                                .is_some_and(|votes| rule.is_blocked(votes))
                        })
                        .count();
                    box_counts[index].push(still);
                }
            }
        }
        let end = reader.snapshot_all();
        for (index, (name, rule)) in rules.iter().enumerate() {
            let start = box_at_removal[index].len();
            let first_at = |limit: usize| {
                box_counts[index]
                    .iter()
                    .position(|count| *count <= limit)
                    .map_or(format!("more than {AFTER}"), |pass| pass.to_string())
            };
            let kept = other_at_removal[index]
                .iter()
                .filter(|key| end.get(**key).is_some_and(|votes| rule.is_blocked(votes)))
                .count();
            println!(
                "| {label} | {name} | {start} | {} | {} | {:.1}% |",
                first_at(start / 2),
                first_at(0),
                100.0 * kept as f64 / other_at_removal[index].len().max(1) as f64
            );
        }
    }
}
