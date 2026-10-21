use std::hint::black_box;
use std::time::Instant;

use voxelaz::live::{PointSceneConfig, VoxelGeometry, open_count_scene, open_presence_scene};

fn main() {
    let count = batch_len();
    let points = vec![[0.2, 0.2, 0.2]; count];
    let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], 0.2).unwrap();
    let (mut presence, _) = open_presence_scene(geometry, &PointSceneConfig::default()).unwrap();
    presence.insert_points(&points[..8]).unwrap();
    let presence_rate = time_points(|| presence.insert_points(&points).unwrap().accepted);
    let (mut counts, _) = open_count_scene(geometry, &PointSceneConfig::default()).unwrap();
    counts.insert_points(&points[..8]).unwrap();
    let count_rate = time_points(|| counts.insert_points(&points).unwrap().accepted);
    println!("presence_same_voxel {presence_rate:.0} pts/s");
    println!("count_same_voxel {count_rate:.0} pts/s");
}

fn batch_len() -> usize {
    if std::env::var("VOXELAZ_BENCH").ok().as_deref() == Some("full") {
        65_536
    } else {
        2_048
    }
}

fn time_points(mut work: impl FnMut() -> u64) -> f64 {
    black_box(work());
    let started = Instant::now();
    let mut total = 0u64;
    for _ in 0..4 {
        total += work();
    }
    black_box(total);
    total as f64 / started.elapsed().as_secs_f64().max(1.0e-9)
}
