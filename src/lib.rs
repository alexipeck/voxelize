mod cancel;
mod classification;
mod error;
pub mod live;
mod pipeline;
mod voxel_grid;

pub use cancel::CancellationToken;
pub use classification::ClassificationFilter;
pub use error::{Result, VoxelizeError};
pub use voxel_grid::{BOUNDED_QUEUE_CAPACITY, CHUNK_SIZE, VoxelGrid, VoxelPoint};

use std::path::Path;

use las::point::Format;
use las::{Color, Point};

use error::{join_consumer, join_producer};
use pipeline::run_pipeline;

pub struct VoxelizeConfig {
    pub voxel_size: f64,
    pub classification_filter: ClassificationFilter,
    pub counts_as_intensity: bool,
    pub cancel: Option<CancellationToken>,
}

impl VoxelizeConfig {
    pub fn new(voxel_size: f64) -> Self {
        Self {
            voxel_size,
            classification_filter: ClassificationFilter::all(),
            counts_as_intensity: false,
            cancel: None,
        }
    }
}

pub fn voxelize(input: &Path, output: &Path, config: &VoxelizeConfig) -> Result<()> {
    if config.voxel_size <= 0.0 {
        return Err(VoxelizeError::InvalidVoxelSize);
    }

    let header = {
        let reader = las::Reader::from_path(input)?;
        reader.header().clone()
    };
    let bounds = header.bounds();
    let origin = [bounds.min.x, bounds.min.y, bounds.min.z];
    let dimensions = [
        bounds.max.x - bounds.min.x,
        bounds.max.y - bounds.min.y,
        bounds.max.z - bounds.min.z,
    ];

    let (producer, consumer) = run_pipeline(
        input,
        output.to_path_buf(),
        header,
        origin,
        dimensions,
        config.voxel_size,
        config.classification_filter,
        config.counts_as_intensity,
        config.cancel.clone(),
    );

    let consumer_result = join_consumer(consumer);
    let producer_result = join_producer(producer);

    let cancelled = matches!(
        (&consumer_result, &producer_result),
        (Err(VoxelizeError::Cancelled), _) | (_, Err(VoxelizeError::Cancelled))
    );

    if cancelled {
        let _ = std::fs::remove_file(output);
        return Err(VoxelizeError::Cancelled);
    }

    consumer_result?;
    producer_result?;

    Ok(())
}

pub(crate) fn voxel_point_to_las_point(
    voxel_point: &VoxelPoint,
    format: &Format,
    counts_as_intensity: bool,
) -> Point {
    let mut point = Point {
        x: voxel_point.x,
        y: voxel_point.y,
        z: voxel_point.z,
        ..Point::default()
    };
    if counts_as_intensity {
        point.intensity = voxel_point.count;
    }
    if format.has_gps_time {
        point.gps_time = Some(0.0);
    }
    if format.has_color {
        point.color = Some(Color::default());
    }
    if format.has_waveform {
        point.waveform = Some(Default::default());
    }
    if format.has_nir {
        point.nir = Some(0);
    }
    if format.extra_bytes > 0 {
        point.extra_bytes = vec![0; format.extra_bytes as usize];
    }
    point
}
