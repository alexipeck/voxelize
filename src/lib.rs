mod classification;
mod pipeline;
mod voxel_grid;

pub use classification::ClassificationFilter;
pub use voxel_grid::{VoxelGrid, VoxelPoint, BOUNDED_QUEUE_CAPACITY, CHUNK_SIZE};

use std::path::Path;

use anyhow::{anyhow, bail, Result};
use las::point::Format;
use las::{Color, Point};

use pipeline::run_pipeline;

pub fn voxelize(
    input: &Path,
    output: &Path,
    voxel_size: f64,
    classification_filter: ClassificationFilter,
) -> Result<()> {
    if voxel_size <= 0.0 {
        bail!("voxel_size must be greater than 0");
    }

    let header = {
        let reader = las::Reader::from_path(input).map_err(anyhow::Error::from)?;
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
        voxel_size,
        classification_filter,
    );

    consumer
        .join()
        .map_err(|_| anyhow!("consumer panicked"))??;
    producer
        .join()
        .map_err(|_| anyhow!("producer panicked"))??;

    Ok(())
}

pub(crate) fn voxel_point_to_las_point(voxel_point: &VoxelPoint, format: &Format) -> Point {
    let mut point = Point::default();
    point.x = voxel_point.x;
    point.y = voxel_point.y;
    point.z = voxel_point.z;
    point.intensity = voxel_point.count;
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
