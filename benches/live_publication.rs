use std::hint::black_box;
use std::time::Instant;

use voxelaz::live::{PointSceneConfig, VoxelGeometry, open_presence_scene};

fn main() {
    let chunks = if std::env::var("VOXELAZ_BENCH").ok().as_deref() == Some("full") {
        4_000
    } else {
        128
    };
    let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], 0.2).unwrap();
    let (mut writer, reader) = open_presence_scene(geometry, &PointSceneConfig::default()).unwrap();
    let mut points = Vec::with_capacity(chunks);
    for index in 0..chunks {
        points.push([index as f64 * 4.0, 0.2, 0.2]);
    }
    writer.insert_points(&points).unwrap();
    let started = Instant::now();
    let stats = writer.publish().unwrap();
    black_box(reader.generation());
    println!(
        "publish_chunks {} duration_ns {} generation {}",
        stats.chunks_published,
        started.elapsed().as_nanos(),
        stats.generation
    );
}
