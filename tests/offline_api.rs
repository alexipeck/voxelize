use voxelaz::{VoxelGrid, VoxelizeConfig};

#[test]
fn offline_grid_and_config_still_construct() {
    let mut grid = VoxelGrid::new([0.0, 0.0, 0.0], [2.0, 2.0, 2.0], 0.5, true).unwrap();
    grid.insert(0.2, 0.2, 0.2);
    grid.insert(0.2, 0.2, 0.2);
    let config = VoxelizeConfig::new(0.5);
    assert_eq!(config.voxel_size, 0.5);
    assert!(!config.counts_as_intensity);
    let rejected = VoxelGrid::new([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], 0.0, false);
    assert!(rejected.is_err());
}
