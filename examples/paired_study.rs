//! Paired maps: how fast does coarse free evidence clear the fine hit-only map after an
//! obstacle leaves, and how much of the fine map that is still real survives? Feeds
//! `docs/live-scene-plan.md` (D14).
//!
//!     cargo run --release --example paired_study -- [coarse_voxel_m]
//!
//! A hovering sensor watches the warehouse plus a 1 × 1 × 1.5 m box 5 m away for 2 s
//! (20 passes); then the box is removed and it watches for 6 s more. The box's fine voxels
//! are those present at the removal that never get a hit when the same passes are run
//! without the box.

#[path = "bench_scan/scene.rs"]
mod scene;

use std::collections::HashSet;

use voxelaz::live::{
    ClearRule, PairedSceneConfig, PairedSceneWriter, VoteDecay, VoteModel, VoxelGeometry, VoxelKey,
};

use scene::{Lidar, Scan, Warehouse};

const PRESENT: usize = 20;
const AFTER: usize = 60;
const BOX_MIN: [f64; 3] = [5.0, 2.0, 0.0];
const BOX_MAX: [f64; 3] = [6.0, 3.0, 1.5];

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
        .map_or(0.2, |value| value.parse().expect("coarse_voxel_m"));
    let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], voxel).expect("voxel size");
    let lidar = Lidar::for_rate(2.3e6, 10.0, 40.0);
    let pose = [0.0, 0.4, 1.8];
    let scans = cast_all(&lidar, pose, true);
    let reference = cast_all(&lidar, pose, false);
    let hits_of = |scan: &Scan| -> Vec<[f64; 3]> {
        scan.offsets
            .iter()
            .map(|offset| {
                [
                    pose[0] + f64::from(offset[0]),
                    pose[1] + f64::from(offset[1]),
                    pose[2] + f64::from(offset[2]),
                ]
            })
            .collect()
    };
    let (static_fine, static_coarse): (HashSet<VoxelKey>, HashSet<VoxelKey>) = {
        let (mut writer, reader) = PairedSceneWriter::open(
            geometry,
            &PairedSceneConfig::default(),
            VoteModel::default(),
        )
        .unwrap();
        for scan in &reference {
            writer
                .insert_rays_from_source(pose, &hits_of(scan))
                .unwrap();
        }
        writer.publish().unwrap();
        let snapshot = reader.snapshot_all();
        let fine = snapshot.fine.iter().map(|(key, _)| key).collect();
        let coarse = snapshot
            .coarse
            .iter()
            .filter(|(_, votes)| votes.hits > 0)
            .map(|(key, _)| key)
            .collect();
        (fine, coarse)
    };
    let parent = |key: VoxelKey| VoxelKey {
        x: key.x.div_euclid(2),
        y: key.y.div_euclid(2),
        z: key.z.div_euclid(2),
    };

    println!(
        "coarse {voxel} m, fine {} m, hovering sensor, box present for {PRESENT} passes then removed for {AFTER}",
        voxel / 2.0
    );
    println!(
        "| decay | clear (max hit, min miss votes) | box fine voxels at removal | passes to clear half | left at end | of those, sharing a coarse voxel with a real surface | real fine voxels present after removal: mean / min | clear events |"
    );
    println!("|---|---|---|---|---|---|---|---|");
    let cases = [
        ("none", VoteDecay::None, Some((0.25, 1.0))),
        ("exp:0.9", VoteDecay::Exponential { keep: 0.9 }, None),
        (
            "exp:0.9",
            VoteDecay::Exponential { keep: 0.9 },
            Some((0.25, 1.0)),
        ),
        (
            "exp:0.9",
            VoteDecay::Exponential { keep: 0.9 },
            Some((0.5, 1.0)),
        ),
        (
            "exp:0.7",
            VoteDecay::Exponential { keep: 0.7 },
            Some((0.25, 1.0)),
        ),
        (
            "exp:0.7",
            VoteDecay::Exponential { keep: 0.7 },
            Some((0.5, 1.0)),
        ),
        (
            "cap:200",
            VoteDecay::CapHalving { cap_votes: 200 },
            Some((0.25, 1.0)),
        ),
    ];
    for (label, decay, clear) in cases {
        let config = PairedSceneConfig {
            clear: clear.map(|(hits, misses)| ClearRule::new(hits, misses).unwrap()),
            ..PairedSceneConfig::default()
        };
        let (mut writer, reader) =
            PairedSceneWriter::open(geometry, &config, VoteModel::new(decay).unwrap()).unwrap();
        let mut box_at_removal = HashSet::new();
        let mut other_at_removal = HashSet::new();
        let mut remaining = Vec::new();
        let mut present = Vec::new();
        let mut cleared = 0u64;
        for (pass, scan) in scans.iter().enumerate() {
            writer
                .insert_rays_from_source(pose, &hits_of(scan))
                .unwrap();
            cleared += writer.publish().unwrap().fine_cleared;
            if pass + 1 == PRESENT {
                for (key, _) in reader.snapshot_all().fine.iter() {
                    if static_fine.contains(&key) {
                        other_at_removal.insert(key);
                    } else {
                        box_at_removal.insert(key);
                    }
                }
            }
            if pass + 1 >= PRESENT {
                let fine: HashSet<VoxelKey> = reader
                    .snapshot_all()
                    .fine
                    .iter()
                    .map(|(key, _)| key)
                    .collect();
                remaining.push(box_at_removal.intersection(&fine).count());
                if pass + 1 > PRESENT {
                    present.push(other_at_removal.intersection(&fine).count() as f64);
                }
            }
        }
        let end: HashSet<VoxelKey> = reader
            .snapshot_all()
            .fine
            .iter()
            .map(|(key, _)| key)
            .collect();
        let start = box_at_removal.len();
        let first_at = |limit: usize| {
            remaining
                .iter()
                .position(|count| *count <= limit)
                .map_or(format!("more than {AFTER}"), |pass| pass.to_string())
        };
        let left: Vec<VoxelKey> = box_at_removal.intersection(&end).copied().collect();
        let beside_surface = left
            .iter()
            .filter(|key| static_coarse.contains(&parent(**key)))
            .count();
        let real = other_at_removal.len().max(1) as f64;
        let mean = present.iter().sum::<f64>() / present.len().max(1) as f64 / real;
        let min = present.iter().copied().fold(f64::INFINITY, f64::min) / real;
        let rule = clear.map_or("none".to_string(), |(hits, misses)| {
            format!("{hits}, {misses}")
        });
        println!(
            "| {label} | {rule} | {start} | {} | {} | {beside_surface} | {:.1}% / {:.1}% | {cleared} |",
            first_at(start / 2),
            left.len(),
            100.0 * mean,
            100.0 * min
        );
    }
}
