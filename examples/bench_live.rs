use std::hint::black_box;
use std::time::Instant;

use voxelaz::live::{
    LogOddsOccupancy, Occupied, PointSceneConfig, PointSceneWriter, RayObservation, RaySceneConfig,
    VoxelGeometry, open_count_scene, open_presence_scene, open_ray_scene,
};

fn main() {
    println!("| workload | result |");
    println!("| --- | --- |");
    report("presence same voxel", rate_hz(presence_same));
    report("presence clustered chunks", rate_hz(presence_clustered));
    report(
        "presence expanding",
        format!("{:.0} pts/s", presence_expanding()),
    );
    report("count same voxel", rate_hz(count_same));
    report("presence chunk side 8", side_rate::<8>());
    report("presence chunk side 16", side_rate::<16>());
    report("presence chunk side 32", side_rate::<32>());
    for threshold in [64usize, 128, 256, 512] {
        report(
            &format!("presence promotion {threshold}"),
            format!("{:.0} pts/s", promotion_rate(threshold)),
        );
    }
    for cells in [5u64, 10, 20, 40, 80] {
        let (rays, events) = ray_rate(cells, false);
        report(
            &format!("ray {cells} cells"),
            format!("{rays:.0} rays/s, {events:.0} cell events/s"),
        );
    }
    let (rays, events) = ray_rate(10, true);
    report(
        "ray 10 cells publish each batch",
        format!("{rays:.0} rays/s, {events:.0} cell events/s"),
    );
    for (label, kept) in [
        ("filter keep 100%", 1.0),
        ("filter keep 50%", 0.5),
        ("filter keep 10%", 0.1),
    ] {
        report(label, format!("{:.0} pts/s", filter_rate(kept)));
    }
    report("publish dirty chunks", publish_case());
    report("ingest beside observer", observer_case());
}

fn report(name: &str, value: String) {
    println!("| {name} | {value} |");
}

fn rate_hz(run: fn() -> f64) -> String {
    format!("{:.0} pts/s", run())
}

fn presence_same() -> f64 {
    let points = vec![[0.15, 0.15, 0.15]; 65_536];
    let (mut writer, _) = open_presence_scene(unit(), &PointSceneConfig::default()).unwrap();
    writer.insert_points(&points[..128]).unwrap();
    repeat(|| black_box(writer.insert_points(&points).unwrap().accepted))
}

fn presence_clustered() -> f64 {
    let mut points = Vec::with_capacity(32_768);
    for index in 0..32_768 {
        let lane = (index % 48) as f64;
        points.push([
            0.1 + (lane % 16.0) * 0.2,
            0.1 + ((lane / 16.0) as u32 as f64) * 0.2,
            0.2,
        ]);
    }
    let (mut writer, _) = open_presence_scene(unit(), &PointSceneConfig::default()).unwrap();
    writer.insert_points(&points[..256]).unwrap();
    repeat(|| black_box(writer.insert_points(&points).unwrap().accepted))
}

fn presence_expanding() -> f64 {
    let mut points = Vec::with_capacity(20_000);
    for index in 0..20_000 {
        points.push([index as f64 * 0.37, (index % 17) as f64 * 0.2, 0.1]);
    }
    let (mut writer, _) = open_presence_scene(unit(), &PointSceneConfig::default()).unwrap();
    let started = Instant::now();
    let accepted = writer.insert_points(&points).unwrap().accepted;
    black_box(accepted) as f64 / started.elapsed().as_secs_f64()
}

fn count_same() -> f64 {
    let points = vec![[0.2, 0.2, 0.2]; 65_536];
    let (mut writer, _) = open_count_scene(unit(), &PointSceneConfig::default()).unwrap();
    writer.insert_points(&points[..128]).unwrap();
    repeat(|| black_box(writer.insert_points(&points).unwrap().accepted))
}

fn side_rate<const SIDE: usize>() -> String {
    let points = vec![[0.15, 0.15, 0.15]; 32_768];
    let (mut writer, _) =
        PointSceneWriter::<Occupied, SIDE>::open(unit(), &PointSceneConfig::default()).unwrap();
    writer.insert_points(&points[..64]).unwrap();
    let rate = repeat(|| black_box(writer.insert_points(&points).unwrap().accepted));
    format!("{rate:.0} pts/s")
}

fn promotion_rate(threshold: usize) -> f64 {
    let config = PointSceneConfig {
        promotion_threshold: threshold,
        ..PointSceneConfig::default()
    };
    let mut points = Vec::with_capacity(16_384);
    for index in 0..16_384 {
        let local = (index % 300) as f64;
        points.push([
            0.05 + (local % 15.0) * 0.2,
            0.05 + ((local / 15.0) as u32 as f64) * 0.2,
            0.2,
        ]);
    }
    let (mut writer, _) = open_presence_scene(unit(), &config).unwrap();
    writer.insert_points(&points).unwrap();
    repeat(|| black_box(writer.insert_points(&points).unwrap().accepted))
}

fn ray_rate(cells: u64, publish: bool) -> (f64, f64) {
    let hit_x = cells as f64 - 0.5;
    let rays = vec![
        RayObservation {
            source: [0.5, 0.5, 0.5],
            hit: [hit_x, 0.5, 0.5],
        };
        4_096
    ];
    let config = RaySceneConfig {
        max_cells_per_ray: Some(cells + 8),
        ..RaySceneConfig::default()
    };
    let (mut writer, _) =
        open_ray_scene(unit_size(1.0), &config, LogOddsOccupancy::default()).unwrap();
    writer.insert_rays(&rays[..32]).unwrap();
    if publish {
        writer.publish().unwrap();
    }
    let mut ray_total = 0u64;
    let mut event_total = 0u64;
    let started = Instant::now();
    let mut rounds = 0u32;
    while started.elapsed().as_millis() < 300 || rounds < 2 {
        let stats = writer.insert_rays(&rays).unwrap();
        if publish {
            black_box(writer.publish().unwrap());
        }
        ray_total += stats.accepted;
        event_total += stats.free_cell_updates + stats.hits_applied;
        rounds += 1;
        if started.elapsed().as_millis() > 700 && rounds >= 2 {
            break;
        }
    }
    let seconds = started.elapsed().as_secs_f64();
    (ray_total as f64 / seconds, event_total as f64 / seconds)
}

fn filter_rate(kept: f64) -> f64 {
    let config = PointSceneConfig {
        range: Some(voxelaz::live::RangeFilter::new(0.0, 1.0).unwrap()),
        ..PointSceneConfig::default()
    };
    let mut points = Vec::with_capacity(32_768);
    let near = (32_768.0 * kept) as usize;
    for index in 0..32_768 {
        if index < near {
            points.push([0.2, 0.1, 0.1]);
        } else {
            points.push([40.0 + (index % 10) as f64, 0.1, 0.1]);
        }
    }
    let (mut writer, _) = open_presence_scene(unit(), &config).unwrap();
    repeat(|| {
        black_box(
            writer
                .insert_points_from_source([0.0, 0.0, 0.0], &points)
                .unwrap()
                .input,
        )
    })
}

fn publish_case() -> String {
    let (mut writer, reader) = open_presence_scene(unit(), &PointSceneConfig::default()).unwrap();
    let mut points = Vec::with_capacity(2_000);
    for index in 0..2_000 {
        points.push([index as f64 * 4.0, 0.2, 0.2]);
    }
    writer.insert_points(&points).unwrap();
    let stats = writer.publish().unwrap();
    let again = writer.publish().unwrap();
    format!(
        "{} chunks in {:.3} ms, empty publish gen {}, reader gen {}",
        stats.chunks_published,
        stats.duration_ns as f64 / 1.0e6,
        again.generation,
        reader.generation()
    )
}

fn observer_case() -> String {
    let (mut writer, reader) = open_presence_scene(unit(), &PointSceneConfig::default()).unwrap();
    let points = vec![[0.25, 0.25, 0.25]; 16_384];
    writer.insert_points(&points[..64]).unwrap();
    writer.publish().unwrap();
    let watch = reader.clone();
    let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let flag = std::sync::Arc::clone(&stop);
    let observer = std::thread::spawn(move || {
        let mut snaps = 0u64;
        while !flag.load(std::sync::atomic::Ordering::Relaxed) {
            black_box(watch.snapshot_all());
            snaps += 1;
        }
        snaps
    });
    let rate = repeat(|| {
        let accepted = writer.insert_points(&points).unwrap().accepted;
        black_box(writer.publish().unwrap());
        accepted
    });
    stop.store(true, std::sync::atomic::Ordering::Relaxed);
    let snaps = observer.join().unwrap();
    format!("{rate:.0} pts/s with {snaps} snapshots")
}

fn unit() -> VoxelGeometry {
    unit_size(0.2)
}

fn unit_size(size: f64) -> VoxelGeometry {
    VoxelGeometry::new([0.0, 0.0, 0.0], size).unwrap()
}

fn repeat(mut work: impl FnMut() -> u64) -> f64 {
    black_box(work());
    let started = Instant::now();
    let mut total = 0u64;
    let mut rounds = 0u32;
    while started.elapsed().as_millis() < 250 || rounds < 2 {
        total += work();
        rounds += 1;
        if started.elapsed().as_millis() > 600 && rounds >= 2 {
            break;
        }
    }
    total as f64 / started.elapsed().as_secs_f64()
}
