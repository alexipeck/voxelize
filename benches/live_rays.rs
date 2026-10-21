use std::hint::black_box;
use std::time::Instant;

use voxelaz::live::{
    LogOddsOccupancy, RayObservation, RaySceneConfig, VoxelGeometry, open_ray_scene,
};

fn main() {
    let rays_per_batch = if std::env::var("VOXELAZ_BENCH").ok().as_deref() == Some("full") {
        4_096
    } else {
        256
    };
    let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], 1.0).unwrap();
    for cells in [5u64, 10, 20] {
        let rays = vec![
            RayObservation {
                source: [0.5, 0.5, 0.5],
                hit: [cells as f64 - 0.5, 0.5, 0.5],
            };
            rays_per_batch
        ];
        let config = RaySceneConfig {
            max_cells_per_ray: Some(cells + 8),
            ..RaySceneConfig::default()
        };
        let (mut writer, _) =
            open_ray_scene(geometry, &config, LogOddsOccupancy::default()).unwrap();
        writer.insert_rays(&rays[..8]).unwrap();
        let started = Instant::now();
        let mut accepted = 0u64;
        let mut events = 0u64;
        for _ in 0..3 {
            let stats = writer.insert_rays(&rays).unwrap();
            accepted += stats.accepted;
            events += stats.free_cell_updates + stats.hits_applied;
        }
        black_box(accepted);
        let seconds = started.elapsed().as_secs_f64().max(1.0e-9);
        println!(
            "ray_{cells}_cells {:.0} rays/s {:.0} cell_events/s",
            accepted as f64 / seconds,
            events as f64 / seconds
        );
    }
}
