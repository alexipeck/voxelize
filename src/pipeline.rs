use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver, SyncSender};
use std::thread::JoinHandle;

use anyhow::{anyhow, Result};
use las::{Header, Point, Writer};

use crate::classification::ClassificationFilter;
use crate::voxel_grid::{VoxelGrid, BOUNDED_QUEUE_CAPACITY, CHUNK_SIZE};
use crate::voxel_point_to_las_point;

struct PipelineChannels {
    a_tx: SyncSender<Vec<Point>>,
    a_rx: Receiver<Vec<Point>>,
    b_tx: mpsc::Sender<Vec<Point>>,
    b_rx: Receiver<Vec<Point>>,
}

fn create_channels() -> PipelineChannels {
    let (a_tx, a_rx) = mpsc::sync_channel(BOUNDED_QUEUE_CAPACITY);
    let (b_tx, b_rx) = mpsc::channel();
    PipelineChannels {
        a_tx,
        a_rx,
        b_tx,
        b_rx,
    }
}

pub fn run_pipeline(
    input: &Path,
    output: PathBuf,
    header: Header,
    origin: [f64; 3],
    dimensions: [f64; 3],
    voxel_size: f64,
    classification_filter: ClassificationFilter,
) -> (
    JoinHandle<Result<()>>,
    JoinHandle<Result<()>>,
) {
    let input = input.to_path_buf();
    let channels = create_channels();

    let producer = std::thread::spawn(move || {
        run_producer(&input, channels.a_tx, channels.b_rx)
    });

    let consumer = std::thread::spawn(move || {
        run_consumer(
            channels.a_rx,
            channels.b_tx,
            output,
            header,
            origin,
            dimensions,
            voxel_size,
            classification_filter,
        )
    });

    (producer, consumer)
}

fn run_producer(
    input: &Path,
    a_tx: SyncSender<Vec<Point>>,
    b_rx: Receiver<Vec<Point>>,
) -> Result<()> {
    let mut reader = las::Reader::from_path(input).map_err(anyhow::Error::from)?;

    loop {
        let mut buffer = match b_rx.try_recv() {
            Ok(mut buf) => {
                buf.clear();
                buf
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => Vec::with_capacity(CHUNK_SIZE),
            Err(std::sync::mpsc::TryRecvError::Disconnected) => Vec::with_capacity(CHUNK_SIZE),
        };

        for _ in 0..CHUNK_SIZE {
            match reader.read_point().map_err(anyhow::Error::from)? {
                Some(point) => buffer.push(point),
                None => break,
            }
        }

        if buffer.is_empty() {
            break;
        }

        let done = buffer.len() < CHUNK_SIZE;
        a_tx
            .send(buffer)
            .map_err(|_| anyhow!("consumer dropped"))?;
        if done {
            break;
        }
    }

    drop(a_tx);
    while b_rx.recv().is_ok() {}

    Ok(())
}

fn run_consumer(
    a_rx: Receiver<Vec<Point>>,
    b_tx: mpsc::Sender<Vec<Point>>,
    output: PathBuf,
    header: Header,
    origin: [f64; 3],
    dimensions: [f64; 3],
    voxel_size: f64,
    classification_filter: ClassificationFilter,
) -> Result<()> {
    let mut grid = VoxelGrid::new(origin, dimensions, voxel_size);

    while let Ok(buffer) = a_rx.recv() {
        for point in &buffer {
            if classification_filter.allows(point.classification) {
                grid.insert(point.x, point.y, point.z);
            }
        }
        let _ = b_tx.send(buffer);
    }

    drop(b_tx);

    let point_format = *header.point_format();
    let mut writer = Writer::from_path(&output, header).map_err(anyhow::Error::from)?;
    grid.drain(|voxel_point| {
        let point = voxel_point_to_las_point(&voxel_point, &point_format);
        writer
            .write_point(point)
            .map_err(anyhow::Error::from)
    })?;
    writer.close().map_err(anyhow::Error::from)?;
    Ok(())
}
