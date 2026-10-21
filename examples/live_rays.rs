use voxelaz::live::{
    LogOddsOccupancy, RayObservation, RaySceneConfig, VoxelGeometry, open_ray_scene,
};

fn main() {
    let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], 0.25).unwrap();
    let model = LogOddsOccupancy::default();
    let (mut writer, reader) = open_ray_scene(geometry, &RaySceneConfig::default(), model).unwrap();
    let stats = writer
        .insert_rays(&[
            RayObservation {
                source: [0.0, 0.0, 1.0],
                hit: [1.2, 0.3, 0.1],
            },
            RayObservation {
                source: [0.2, -0.4, 1.0],
                hit: [0.2, -0.4, 0.2],
            },
        ])
        .unwrap();
    assert_eq!(stats.accepted, 2);
    assert_eq!(stats.hits_applied, 2);
    assert!(stats.free_cell_updates > 0);
    assert!(reader.get_voxel_world([1.2, 0.3, 0.1]).unwrap().is_none());
    writer.publish().unwrap();
    let hit = reader.get_voxel_world([1.2, 0.3, 0.1]).unwrap().unwrap();
    println!("ray generation {} hit log-odds {hit}", reader.generation());
}
