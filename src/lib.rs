mod pipeline;
mod voxel_grid;

pub use voxel_grid::{VoxelGrid, VoxelPoint, BOUNDED_QUEUE_CAPACITY, CHUNK_SIZE};

use std::path::Path;

use anyhow::{anyhow, bail, Result};
use las::point::Format;
use las::{Color, Header, Point, Writer};

use pipeline::run_pipeline;

pub fn voxelize(input: &Path, output: &Path, voxel_size: f64) -> Result<()> {
    if voxel_size <= 0.0 {
        bail!("voxel_size must be greater than 0");
    }

    let reader = las::Reader::from_path(input).map_err(anyhow::Error::from)?;
    let header = reader.header().clone();
    let bounds = header.bounds();
    let origin = [bounds.min.x, bounds.min.y, bounds.min.z];
    let dimensions = [
        bounds.max.x - bounds.min.x,
        bounds.max.y - bounds.min.y,
        bounds.max.z - bounds.min.z,
    ];

    let (producer, consumer) = run_pipeline(input, origin, dimensions, voxel_size);

    producer
        .join()
        .map_err(|_| anyhow!("producer panicked"))??;
    let voxel_points = consumer
        .join()
        .map_err(|_| anyhow!("consumer panicked"))??;

    write_output(output, header, &voxel_points)?;

    Ok(())
}

fn write_output(output: &Path, header: Header, voxel_points: &[VoxelPoint]) -> Result<()> {
    let point_format = *header.point_format();
    let mut writer = Writer::from_path(output, header).map_err(anyhow::Error::from)?;

    for voxel_point in voxel_points {
        let point = voxel_point_to_las_point(voxel_point, &point_format);
        writer
            .write_point(point)
            .map_err(anyhow::Error::from)?;
    }

    writer.close().map_err(anyhow::Error::from)?;
    Ok(())
}

fn voxel_point_to_las_point(voxel_point: &VoxelPoint, format: &Format) -> Point {
    let mut point = Point::default();
    point.x = voxel_point.x;
    point.y = voxel_point.y;
    point.z = voxel_point.z;
    point.intensity = voxel_point.count.min(u16::MAX as u32) as u16;
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
