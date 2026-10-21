use voxelaz::live::{PointSceneConfig, VoxelGeometry, open_presence_scene};

fn main() {
    let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], 0.2).unwrap();
    let (mut writer, reader) = open_presence_scene(geometry, &PointSceneConfig::default()).unwrap();
    let stats = writer
        .insert_points(&[
            [0.11, 0.27, -0.05],
            [0.11, 0.27, -0.05],
            [1.40, -0.30, 0.55],
        ])
        .unwrap();
    assert_eq!(stats.accepted, 3);
    assert_eq!(stats.newly_observed_cells, 2);
    assert!(
        reader
            .get_voxel_world([0.11, 0.27, -0.05])
            .unwrap()
            .is_none()
    );
    let published = writer.publish().unwrap();
    assert!(
        reader
            .get_voxel_world([0.11, 0.27, -0.05])
            .unwrap()
            .is_some()
    );
    let key = geometry.voxel_key([1.40, -0.30, 0.55]).unwrap();
    assert!(reader.get_voxel_key(key).is_some());
    println!(
        "published generation {} chunks {}",
        published.generation, published.chunks_published
    );
}
