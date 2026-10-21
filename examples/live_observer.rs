use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::sync_channel;
use std::thread;
use std::time::Duration;

use voxelaz::live::{
    JournalConfig, LogOddsOccupancy, RayObservation, RaySceneConfig, VoxelBounds, VoxelGeometry,
    VoxelKey, open_ray_scene,
};

fn main() {
    let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], 0.5).unwrap();
    let config = RaySceneConfig {
        journal: JournalConfig {
            max_generations: 1,
            max_keys: 65_536,
        },
        max_cells_per_ray: Some(64),
        ..RaySceneConfig::default()
    };
    let (mut writer, reader) =
        open_ray_scene(geometry, &config, LogOddsOccupancy::default()).unwrap();
    writer
        .insert_rays_from_source([0.0, 0.0, 1.5], &[[0.2, 0.2, 0.2], [1.2, 0.2, 0.2]])
        .unwrap();
    writer.publish().unwrap();
    writer
        .insert_ray(RayObservation {
            source: [0.0, 0.0, 1.5],
            hit: [2.4, -0.2, 0.2],
        })
        .unwrap();
    writer.publish().unwrap();

    let observer_reader = reader.clone();
    let done = Arc::new(AtomicBool::new(false));
    let done_flag = Arc::clone(&done);
    let (batch_tx, batch_rx) = sync_channel::<Vec<RayObservation>>(2);
    let observer = thread::spawn(move || {
        let mut cursor = 0u64;
        let mut resyncs = 0u32;
        let mut applied = 0u32;
        loop {
            match observer_reader.snapshot_changes_since(cursor) {
                Ok(snapshot) => {
                    let generation = snapshot.generation();
                    if generation != cursor {
                        if !snapshot.chunks().is_empty() {
                            applied += 1;
                        }
                        cursor = generation;
                    }
                }
                Err(_) => {
                    let snapshot = observer_reader.snapshot_all();
                    cursor = snapshot.generation();
                    resyncs += 1;
                    applied += 1;
                }
            }
            if done_flag.load(Ordering::Acquire) && cursor == observer_reader.generation() {
                break;
            }
            thread::sleep(Duration::from_millis(1));
        }
        (applied, resyncs, cursor)
    });

    let producer = thread::spawn(move || {
        for batch in 0..4 {
            let mut rays = Vec::with_capacity(4);
            for step in 0..4 {
                rays.push(RayObservation {
                    source: [0.1, 0.1, 2.0],
                    hit: [batch as f64 * 0.8 + step as f64 * 0.3, 0.4, 0.2],
                });
            }
            batch_tx.send(rays).unwrap();
        }
    });

    let mut batches = 0u32;
    while let Ok(rays) = batch_rx.recv() {
        writer.insert_rays(&rays).unwrap();
        writer.publish().unwrap();
        batches += 1;
    }
    producer.join().unwrap();
    done.store(true, Ordering::Release);
    let (applied, resyncs, cursor) = observer.join().unwrap();
    assert!(resyncs >= 1);
    assert!(applied >= 1);
    assert_eq!(batches, 4);
    assert_eq!(cursor, reader.generation());
    let region = reader
        .snapshot_region(VoxelBounds {
            min: VoxelKey { x: 0, y: 0, z: 0 },
            max: VoxelKey { x: 4, y: 2, z: 4 },
        })
        .unwrap();
    assert_eq!(region.generation(), reader.generation());
    assert!(region.iter().next().is_some());
    println!(
        "observer applied {applied} resyncs {resyncs} generation {}",
        region.generation()
    );
}
