use std::sync::Arc;

use super::config::PointSceneConfig;
use super::error::LiveError;
use super::geometry::VoxelGeometry;
use super::published::{PublishedIndex, SceneReader, publish_dirty};
use super::stats::{BatchStats, InsertStats, PublishStats, SceneMemoryStats};
use super::store::{CellTouch, ChunkStore, DirtySet};
use super::{RangeClass, RangeFilter};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Occupied;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointObservation {
    pub source: [f64; 3],
    pub point: [f64; 3],
}

pub trait PointCell: Copy + Default + Send + Sync + 'static {
    fn observe(cell: &mut Self);
}

impl PointCell for Occupied {
    fn observe(_cell: &mut Self) {}
}

impl PointCell for u16 {
    fn observe(cell: &mut Self) {
        *cell = cell.saturating_add(1);
    }
}

pub struct PointSceneWriter<Cell, const SIDE: usize = 16> {
    geometry: VoxelGeometry,
    range: Option<RangeFilter>,
    max_chunks: Option<u64>,
    store: ChunkStore<Cell, SIDE>,
    dirty: DirtySet,
    published: Arc<PublishedIndex<Cell>>,
}

impl<Cell: PointCell, const SIDE: usize> PointSceneWriter<Cell, SIDE> {
    pub fn open(
        geometry: VoxelGeometry,
        config: &PointSceneConfig,
    ) -> Result<(Self, SceneReader<Cell>), LiveError> {
        config.validate()?;
        let published = Arc::new(PublishedIndex::new(config.journal)?);
        let reader = SceneReader::new(Arc::clone(&published), geometry, SIDE as i64);
        let writer = Self {
            geometry,
            range: config.range,
            max_chunks: config.max_chunks,
            store: ChunkStore::new(config.promotion_threshold),
            dirty: DirtySet::new(),
            published,
        };
        Ok((writer, reader))
    }

    pub fn reader(&self) -> SceneReader<Cell> {
        SceneReader::new(Arc::clone(&self.published), self.geometry, SIDE as i64)
    }

    pub fn insert_point(&mut self, point: [f64; 3]) -> Result<InsertStats, LiveError> {
        self.require_unfiltered()?;
        let mut stats = BatchStats {
            input: 1,
            ..BatchStats::default()
        };
        self.ingest_point(point, None, &mut stats);
        Ok(stats.as_insert())
    }

    pub fn insert_points(&mut self, points: &[[f64; 3]]) -> Result<BatchStats, LiveError> {
        self.require_unfiltered()?;
        let mut stats = BatchStats {
            input: points.len() as u64,
            ..BatchStats::default()
        };
        for point in points {
            self.ingest_point(*point, None, &mut stats);
        }
        Ok(stats)
    }

    pub fn insert_point_from_source(
        &mut self,
        source: [f64; 3],
        point: [f64; 3],
    ) -> Result<InsertStats, LiveError> {
        let mut stats = BatchStats {
            input: 1,
            ..BatchStats::default()
        };
        self.ingest_point(point, Some(source), &mut stats);
        Ok(stats.as_insert())
    }

    pub fn insert_point_observations(
        &mut self,
        observations: &[PointObservation],
    ) -> Result<BatchStats, LiveError> {
        let mut stats = BatchStats {
            input: observations.len() as u64,
            ..BatchStats::default()
        };
        for observation in observations {
            self.ingest_point(observation.point, Some(observation.source), &mut stats);
        }
        Ok(stats)
    }

    pub fn insert_points_from_source(
        &mut self,
        source: [f64; 3],
        points: &[[f64; 3]],
    ) -> Result<BatchStats, LiveError> {
        let mut stats = BatchStats {
            input: points.len() as u64,
            ..BatchStats::default()
        };
        for point in points {
            self.ingest_point(*point, Some(source), &mut stats);
        }
        Ok(stats)
    }

    pub fn publish(&mut self) -> Result<PublishStats, LiveError> {
        let store = &self.store;
        publish_dirty(&mut self.dirty, &self.published, |key| {
            store.to_published(key).ok_or(LiveError::MissingDirtyChunk)
        })
    }

    pub fn observed_cells(&self) -> u64 {
        self.store.observed_cells()
    }

    pub fn chunk_count(&self) -> u64 {
        self.store.chunk_count()
    }

    pub fn memory_stats(&self) -> SceneMemoryStats {
        let working = SceneMemoryStats {
            working_chunks: self.store.chunk_count(),
            observed_cells: self.store.observed_cells(),
            dirty_chunks: self.dirty.len() as u64,
            estimated_working_bytes: self.store.estimated_bytes(),
            ..SceneMemoryStats::default()
        };
        self.published.memory(working)
    }

    pub fn geometry(&self) -> VoxelGeometry {
        self.geometry
    }

    fn require_unfiltered(&self) -> Result<(), LiveError> {
        if self.range.is_some() {
            Err(LiveError::RangeFilterRequiresSource)
        } else {
            Ok(())
        }
    }

    fn ingest_point(&mut self, point: [f64; 3], source: Option<[f64; 3]>, stats: &mut BatchStats) {
        if let Some(source) = source
            && let Some(filter) = self.range
        {
            match filter.classify(source, point) {
                RangeClass::Accept => {}
                RangeClass::Filtered => {
                    stats.filtered_by_range += 1;
                    return;
                }
                RangeClass::Invalid => {
                    stats.invalid += 1;
                    return;
                }
                RangeClass::Overflow => {
                    stats.distance_overflow += 1;
                    return;
                }
            }
        }
        let key = match self.geometry.voxel_key(point) {
            Ok(key) => key,
            Err(_) => {
                stats.invalid += 1;
                return;
            }
        };
        match self
            .store
            .with_voxel(key, self.max_chunks, Cell::default(), Cell::observe)
        {
            CellTouch::OverCapacity => stats.capacity_rejected += 1,
            CellTouch::Updated { chunk_key, newly } => {
                stats.accepted += 1;
                stats.hits_applied += 1;
                if newly {
                    stats.newly_observed_cells += 1;
                }
                if self.dirty.mark(chunk_key) {
                    stats.dirty_chunks += 1;
                }
            }
        }
    }
}

pub fn open_presence_scene(
    geometry: VoxelGeometry,
    config: &PointSceneConfig,
) -> Result<(PointSceneWriter<Occupied, 16>, SceneReader<Occupied>), LiveError> {
    PointSceneWriter::open(geometry, config)
}

pub fn open_count_scene(
    geometry: VoxelGeometry,
    config: &PointSceneConfig,
) -> Result<(PointSceneWriter<u16, 16>, SceneReader<u16>), LiveError> {
    PointSceneWriter::open(geometry, config)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::live::RangeFilter;
    use crate::live::config::JournalConfig;
    use crate::live::error::ChangesError;
    use crate::live::geometry::{ChunkKey, VoxelBounds, VoxelKey};
    use std::sync::{Arc, Barrier};
    use std::time::Duration;

    fn setup<Cell: PointCell, const SIDE: usize>(
        config: PointSceneConfig,
    ) -> (PointSceneWriter<Cell, SIDE>, SceneReader<Cell>) {
        let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], 1.0).unwrap();
        PointSceneWriter::open(geometry, &config).unwrap()
    }

    #[test]
    fn presence_deduplicates_and_is_invisible_until_publish() {
        let (mut writer, reader) = open_presence_scene(
            VoxelGeometry::new([10.0, -2.0, 0.0], 0.5).unwrap(),
            &PointSceneConfig::default(),
        )
        .unwrap();
        let stats = writer
            .insert_points(&[[10.1, -2.1, 0.1], [10.1, -2.1, 0.1], [12.0, -2.1, 0.1]])
            .unwrap();
        assert_eq!(stats.accepted, 3);
        assert_eq!(stats.newly_observed_cells, 2);
        assert_eq!(writer.observed_cells(), 2);
        let occupied = writer.geometry.voxel_key([10.1, -2.1, 0.1]).unwrap();
        assert!(writer.store.working_get(occupied).is_some());
        assert!(reader.get_voxel_key(occupied).is_none());
        let published = writer.publish().unwrap();
        assert_eq!(published.chunks_published, 1);
        assert_eq!(
            reader.get_voxel_world([10.1, -2.1, 0.1]).unwrap(),
            Some(Occupied)
        );
        assert_eq!(writer.publish().unwrap().generation, published.generation);
        let again = writer.insert_point([10.1, -2.1, 0.1]).unwrap();
        assert_eq!(again.newly_observed_cells, 0);
        assert_eq!(again.accepted, 1);
    }

    #[test]
    fn counts_saturate_and_promote_without_losing_values() {
        let config = PointSceneConfig {
            promotion_threshold: 3,
            ..PointSceneConfig::default()
        };
        let (mut writer, reader) = setup::<u16, 16>(config);
        for _ in 0..70_000 {
            writer.insert_point([0.1, 0.1, 0.1]).unwrap();
        }
        writer.insert_point([0.2, 1.2, 0.1]).unwrap();
        writer.insert_point([0.2, 2.2, 0.1]).unwrap();
        writer.insert_point([0.2, 3.2, 0.1]).unwrap();
        assert_eq!(
            writer.store.chunk_is_dense(ChunkKey { x: 0, y: 0, z: 0 }),
            Some(true)
        );
        writer.publish().unwrap();
        assert_eq!(
            reader.get_voxel_key(VoxelKey { x: 0, y: 0, z: 0 }),
            Some(u16::MAX)
        );
        assert_eq!(reader.get_voxel_key(VoxelKey { x: 0, y: 1, z: 0 }), Some(1));
        assert_eq!(reader.get_voxel_key(VoxelKey { x: 0, y: 2, z: 0 }), Some(1));
        assert_eq!(reader.get_voxel_key(VoxelKey { x: 0, y: 3, z: 0 }), Some(1));
        assert_eq!(reader.get_voxel_key(VoxelKey { x: 4, y: 0, z: 0 }), None);
    }

    #[test]
    fn range_filter_requires_a_source_and_capacity_is_deterministic() {
        let config = PointSceneConfig {
            range: Some(RangeFilter::new(0.0, 1.0).unwrap()),
            ..PointSceneConfig::default()
        };
        let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], 1.0).unwrap();
        let (mut plain, plain_reader) =
            open_presence_scene(geometry, &PointSceneConfig::default()).unwrap();
        let far = plain.insert_points(&[[50.0, 0.0, 0.0]]).unwrap();
        assert_eq!(far.accepted, 1);
        assert_eq!(far.filtered_by_range, 0);
        let mixed = plain
            .insert_points(&[[0.4, 0.4, 0.4], [f64::NAN, 0.0, 0.0], [-0.2, -0.2, -0.2]])
            .unwrap();
        assert_eq!(mixed.accepted, 2);
        assert_eq!(mixed.invalid, 1);
        plain.publish().unwrap();
        assert!(
            plain_reader
                .get_voxel_world([50.0, 0.0, 0.0])
                .unwrap()
                .is_some()
        );

        let (mut writer, reader) = open_presence_scene(geometry, &config).unwrap();
        assert_eq!(
            writer.insert_points(&[[50.0, 0.0, 0.0]]).unwrap_err(),
            LiveError::RangeFilterRequiresSource
        );
        assert_eq!(
            writer.insert_point([0.2, 0.2, 0.2]).unwrap_err(),
            LiveError::RangeFilterRequiresSource
        );
        assert_eq!(writer.observed_cells(), 0);
        let filtered = writer
            .insert_points_from_source([0.0, 0.0, 0.0], &[[50.0, 0.0, 0.0], [0.2, 0.2, 0.2]])
            .unwrap();
        assert_eq!(filtered.filtered_by_range, 1);
        assert_eq!(filtered.accepted, 1);
        let per_source = writer
            .insert_point_observations(&[
                PointObservation {
                    source: [0.0, 0.0, 0.0],
                    point: [0.4, 0.4, 0.4],
                },
                PointObservation {
                    source: [0.0, 0.0, 0.0],
                    point: [40.2, 0.0, 0.0],
                },
            ])
            .unwrap();
        assert_eq!(per_source.accepted, 1);
        assert_eq!(per_source.filtered_by_range, 1);
        let overflow = writer
            .insert_points_from_source([0.0, 0.0, 0.0], &[[1.0e308, 1.0e308, 1.0e308]])
            .unwrap();
        assert_eq!(overflow.distance_overflow, 1);
        assert_eq!(overflow.accepted, 0);
        writer.publish().unwrap();
        assert!(reader.get_voxel_world([50.0, 0.0, 0.0]).unwrap().is_none());
        assert!(reader.get_voxel_world([0.2, 0.2, 0.2]).unwrap().is_some());
        assert!(reader.get_voxel_world([0.4, 0.4, 0.4]).unwrap().is_some());
        assert!(reader.get_voxel_world([40.2, 0.0, 0.0]).unwrap().is_none());

        let limited_config = PointSceneConfig {
            max_chunks: Some(1),
            ..PointSceneConfig::default()
        };
        let (mut limited, limited_reader) = open_presence_scene(geometry, &limited_config).unwrap();
        assert_eq!(limited.insert_point([0.2, 0.2, 0.2]).unwrap().accepted, 1);
        let rejected = limited.insert_point([40.2, 0.2, 0.2]).unwrap();
        assert_eq!(rejected.capacity_rejected, 1);
        assert_eq!(limited.chunk_count(), 1);
        limited.publish().unwrap();
        assert!(
            limited_reader
                .get_voxel_world([0.2, 0.2, 0.2])
                .unwrap()
                .is_some()
        );
        assert!(
            limited_reader
                .get_voxel_world([40.2, 0.2, 0.2])
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn scene_expands_and_snapshots_stay_coherent() {
        let config = PointSceneConfig {
            journal: JournalConfig {
                max_generations: 1,
                max_keys: 8,
            },
            ..PointSceneConfig::default()
        };
        let (mut writer, reader) = setup::<Occupied, 16>(config);
        writer.insert_point([0.5, 0.5, 0.5]).unwrap();
        writer.publish().unwrap();
        let first = reader.snapshot_all();
        assert_eq!(first.get(VoxelKey { x: 0, y: 0, z: 0 }), Some(Occupied));
        writer.insert_point([100.5, -20.5, 3.5]).unwrap();
        writer.publish().unwrap();
        assert_eq!(
            first.get(VoxelKey {
                x: 100,
                y: -21,
                z: 3
            }),
            None
        );
        assert_eq!(first.generation(), 1);
        let second = reader.snapshot_all();
        assert_eq!(second.generation(), 2);
        assert_eq!(second.get(VoxelKey { x: 0, y: 0, z: 0 }), Some(Occupied));
        assert!(
            Arc::ptr_eq(&first.chunks()[0].1, &second.chunks()[0].1)
                || first.chunks()[0].0 != second.chunks()[0].0
        );
        let stable_first = reader.snapshot_chunks(&[ChunkKey { x: 0, y: 0, z: 0 }]);
        writer.insert_point([0.6, 0.6, 0.6]).unwrap();
        writer.publish().unwrap();
        let stable_second = reader.snapshot_chunks(&[ChunkKey { x: 0, y: 0, z: 0 }]);
        assert!(!Arc::ptr_eq(
            &stable_first.chunks()[0].1,
            &stable_second.chunks()[0].1
        ));
        writer.insert_point([1.2, 80.2, 0.2]).unwrap();
        writer.publish().unwrap();
        let unchanged = reader.snapshot_chunks(&[ChunkKey { x: 0, y: 0, z: 0 }]);
        let previous = stable_second;
        assert!(Arc::ptr_eq(
            &previous.chunks()[0].1,
            &unchanged.chunks()[0].1
        ));
        assert!(reader.changes_since(0).is_err());
        let cursor = reader.generation() - 1;
        let changes = reader.changes_since(cursor).unwrap();
        assert_eq!(changes.generation, reader.generation());
        assert!(!changes.chunk_keys.is_empty());
        let other = reader.clone();
        let older = other.changes_since(cursor).unwrap();
        assert_eq!(older.chunk_keys, changes.chunk_keys);
        let region = reader
            .snapshot_region(VoxelBounds {
                min: VoxelKey { x: 0, y: 0, z: 0 },
                max: VoxelKey { x: 0, y: 0, z: 0 },
            })
            .unwrap();
        assert_eq!(region.generation(), reader.generation());
        assert!(region.get(VoxelKey { x: 0, y: 0, z: 0 }).is_some());
        assert!(
            reader
                .snapshot_region(VoxelBounds {
                    min: VoxelKey { x: 2, y: 0, z: 0 },
                    max: VoxelKey { x: 1, y: 0, z: 0 },
                })
                .is_err()
        );
        let cells: Vec<_> = region.iter().collect();
        assert!(!cells.is_empty());
        let memory = writer.memory_stats();
        assert!(memory.working_chunks >= 2);
        assert!(memory.estimated_working_bytes > 0);
        assert!(memory.published_chunks >= 1);
    }

    #[test]
    fn publication_does_not_wait_for_a_snapshot_holder() {
        let (mut writer, reader) = setup::<Occupied, 16>(PointSceneConfig::default());
        writer.insert_point([0.2, 0.2, 0.2]).unwrap();
        writer.publish().unwrap();
        let slow = reader.clone();
        let started = Arc::new(Barrier::new(2));
        let started_reader = Arc::clone(&started);
        let handle = std::thread::spawn(move || {
            let snapshot = slow.snapshot_all();
            started_reader.wait();
            std::thread::sleep(Duration::from_millis(150));
            assert_eq!(snapshot.get(VoxelKey { x: 0, y: 0, z: 0 }), Some(Occupied));
        });
        started.wait();
        let began = std::time::Instant::now();
        writer.insert_point([3.2, 0.2, 0.2]).unwrap();
        writer.publish().unwrap();
        assert!(began.elapsed() < Duration::from_millis(100));
        handle.join().unwrap();
        let reader_thread = reader.clone();
        let worker = std::thread::spawn(move || {
            for _ in 0..32 {
                let _ = reader_thread.snapshot_all();
                let _ = reader_thread.get_voxel_key(VoxelKey { x: 0, y: 0, z: 0 });
            }
        });
        for index in 0..64 {
            writer.insert_point([index as f64 + 0.1, 4.1, 0.1]).unwrap();
            if index % 4 == 0 {
                writer.publish().unwrap();
            }
        }
        writer.publish().unwrap();
        worker.join().unwrap();
        assert!(reader.generation() >= 2);
    }

    #[test]
    fn later_partial_snapshot_skips_a_publish() {
        let (mut writer, reader) = setup::<Occupied, 16>(PointSceneConfig::default());
        writer.insert_point([0.5, 0.5, 0.5]).unwrap();
        writer.publish().unwrap();
        let cursor = reader.generation();
        writer.insert_point([1.5, 0.5, 0.5]).unwrap();
        writer.publish().unwrap();
        let changes = reader.changes_since(cursor).unwrap();
        writer.insert_point([100.5, 0.5, 0.5]).unwrap();
        writer.publish().unwrap();
        let partial = reader.snapshot_chunks(&changes.chunk_keys);
        assert!(partial.generation() > changes.generation);
        let skipped = reader.changes_since(partial.generation()).unwrap();
        assert!(skipped.chunk_keys.is_empty());
        assert!(
            reader
                .get_voxel_key(VoxelKey { x: 100, y: 0, z: 0 })
                .is_some()
        );
        let caught = reader.changes_since(changes.generation).unwrap();
        assert!(caught.chunk_keys.iter().any(|key| key.x == 6));
        let observed = reader.snapshot_changes_since(cursor).unwrap();
        assert_eq!(observed.generation(), reader.generation());
        assert!(observed.chunk_keys().any(|key| key.x == 6));
        assert!(
            observed
                .chunk_keys()
                .any(|key| key == ChunkKey { x: 0, y: 0, z: 0 })
        );
        let next = reader.changes_since(observed.generation()).unwrap();
        assert!(next.chunk_keys.is_empty());
        assert!(matches!(
            reader.changes_since(reader.generation() + 1),
            Err(ChangesError::CursorAhead { .. })
        ));
    }

    #[test]
    fn requested_chunk_lookup_does_not_return_the_whole_map() {
        let (mut writer, reader) = setup::<Occupied, 16>(PointSceneConfig::default());
        for index in 0..2_000 {
            writer
                .insert_point([index as f64 * 16.0 + 0.1, 0.1, 0.1])
                .unwrap();
        }
        writer.publish().unwrap();
        let one = reader.snapshot_chunks(&[ChunkKey { x: 0, y: 0, z: 0 }]);
        assert_eq!(one.chunks().len(), 1);
        assert_eq!(one.chunks()[0].0, ChunkKey { x: 0, y: 0, z: 0 });
        let region = reader
            .snapshot_region(VoxelBounds {
                min: VoxelKey { x: 0, y: 0, z: 0 },
                max: VoxelKey { x: 0, y: 0, z: 0 },
            })
            .unwrap();
        assert_eq!(region.chunks().len(), 1);
        assert!(region.get(VoxelKey { x: 0, y: 0, z: 0 }).is_some());
        assert!(region.get(VoxelKey { x: 16, y: 0, z: 0 }).is_none());
        let missing = reader.snapshot_chunks(&[ChunkKey {
            x: 9_999,
            y: 0,
            z: 0,
        }]);
        assert!(missing.chunks().is_empty());
        assert_eq!(missing.generation(), reader.generation());
        assert_eq!(reader.snapshot_all().chunks().len(), 2_000);
    }

    #[test]
    fn reader_and_writer_are_thread_safe_handles() {
        fn assert_send<T: Send>() {}
        fn assert_sync<T: Sync>() {}
        assert_send::<PointSceneWriter<Occupied, 16>>();
        assert_sync::<PointSceneWriter<Occupied, 16>>();
        assert_send::<SceneReader<Occupied>>();
        assert_sync::<SceneReader<Occupied>>();
        assert_send::<PointSceneWriter<u16, 8>>();
        assert_sync::<SceneReader<u16>>();
    }
}
