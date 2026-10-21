use std::sync::Arc;

use super::config::RaySceneConfig;
use super::dda::{RayCellRole, cell_upper_bound, visit_ray};
use super::error::LiveError;
use super::geometry::{VoxelGeometry, chunk_and_local};
use super::model::RayUpdateModel;
use super::published::{PublishedIndex, SceneReader, publish_dirty};
use super::stats::{BatchStats, InsertStats, PublishStats, SceneMemoryStats};
use super::store::{CellTouch, ChunkStore, DirtySet};
use super::{RangeClass, RangeFilter};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayObservation {
    pub source: [f64; 3],
    pub hit: [f64; 3],
}

pub struct RaySceneWriter<M: RayUpdateModel, const SIDE: usize = 16> {
    geometry: VoxelGeometry,
    range: Option<RangeFilter>,
    max_chunks: Option<u64>,
    cell_cap: u64,
    model: M,
    store: ChunkStore<M::Cell, SIDE>,
    dirty: DirtySet,
    published: Arc<PublishedIndex<M::Cell>>,
}

impl<M: RayUpdateModel, const SIDE: usize> RaySceneWriter<M, SIDE> {
    pub fn open(
        geometry: VoxelGeometry,
        config: &RaySceneConfig,
        model: M,
    ) -> Result<(Self, SceneReader<M::Cell>), LiveError> {
        config.validate()?;
        let published = Arc::new(PublishedIndex::new(config.journal)?);
        let reader = SceneReader::new(Arc::clone(&published), geometry, SIDE as i64);
        let writer = Self {
            geometry,
            range: config.range,
            max_chunks: config.max_chunks,
            cell_cap: config.effective_cell_cap(),
            model,
            store: ChunkStore::new(config.promotion_threshold),
            dirty: DirtySet::new(),
            published,
        };
        Ok((writer, reader))
    }

    pub fn reader(&self) -> SceneReader<M::Cell> {
        SceneReader::new(Arc::clone(&self.published), self.geometry, SIDE as i64)
    }

    pub fn model_mut(&mut self) -> &mut M {
        &mut self.model
    }

    pub fn insert_ray(&mut self, ray: RayObservation) -> Result<InsertStats, LiveError> {
        let mut stats = BatchStats {
            input: 1,
            ..BatchStats::default()
        };
        self.ingest_ray(ray, &mut stats)?;
        Ok(stats.as_insert())
    }

    pub fn insert_rays(&mut self, rays: &[RayObservation]) -> Result<BatchStats, LiveError> {
        let mut stats = BatchStats {
            input: rays.len() as u64,
            ..BatchStats::default()
        };
        for ray in rays {
            if let Err(err) = self.ingest_ray(*ray, &mut stats) {
                return Err(attach_stats(err, stats));
            }
        }
        Ok(stats)
    }

    pub fn insert_rays_from_source(
        &mut self,
        source: [f64; 3],
        hits: &[[f64; 3]],
    ) -> Result<BatchStats, LiveError> {
        let mut stats = BatchStats {
            input: hits.len() as u64,
            ..BatchStats::default()
        };
        for hit in hits {
            let ray = RayObservation { source, hit: *hit };
            if let Err(err) = self.ingest_ray(ray, &mut stats) {
                return Err(attach_stats(err, stats));
            }
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

    fn ingest_ray(&mut self, ray: RayObservation, stats: &mut BatchStats) -> Result<(), LiveError> {
        if let Some(filter) = self.range {
            match filter.classify(ray.source, ray.hit) {
                RangeClass::Accept => {}
                RangeClass::Filtered => {
                    stats.filtered_by_range += 1;
                    return Ok(());
                }
                RangeClass::Invalid => {
                    stats.invalid += 1;
                    return Ok(());
                }
                RangeClass::Overflow => {
                    stats.distance_overflow += 1;
                    return Ok(());
                }
            }
        }
        let source_key = match self.geometry.voxel_key(ray.source) {
            Ok(key) => key,
            Err(_) => {
                stats.invalid += 1;
                return Ok(());
            }
        };
        let hit_key = match self.geometry.voxel_key(ray.hit) {
            Ok(key) => key,
            Err(_) => {
                stats.invalid += 1;
                return Ok(());
            }
        };
        let upper = cell_upper_bound(source_key, hit_key);
        if upper > u128::from(self.cell_cap) {
            stats.capacity_rejected += 1;
            return Ok(());
        }
        if let Some(max_chunks) = self.max_chunks
            && self.store.chunk_count() as u128 + upper > u128::from(max_chunks)
        {
            match self.dry_exceeds(ray, max_chunks) {
                Ok(true) => {
                    stats.capacity_rejected += 1;
                    return Ok(());
                }
                Ok(false) => {}
                Err(err) => return Err(attach_stats(err, *stats)),
            }
        }
        let length = if self.model.needs_metric_length() {
            match segment_length(ray.source, ray.hit) {
                Ok(length) => length,
                Err(LiveError::DistanceOverflow) => {
                    stats.distance_overflow += 1;
                    return Ok(());
                }
                Err(_) => {
                    stats.invalid += 1;
                    return Ok(());
                }
            }
        } else {
            0.0
        };
        let step_limit = u64::try_from(upper.saturating_add(8)).unwrap_or(u64::MAX);
        let geometry = self.geometry;
        let max_chunks = self.max_chunks;
        let mut partial = false;
        let walked = {
            let model = &mut self.model;
            let store = &mut self.store;
            let dirty = &mut self.dirty;
            visit_ray(
                &geometry,
                ray.source,
                ray.hit,
                length,
                step_limit,
                |event| {
                    if partial {
                        return;
                    }
                    match store.with_voxel(event.key, max_chunks, |cell| model.update(cell, event))
                    {
                        CellTouch::OverCapacity => partial = true,
                        CellTouch::Updated { chunk_key, newly } => {
                            if newly {
                                stats.newly_observed_cells += 1;
                            }
                            match event.role {
                                RayCellRole::Free => stats.free_cell_updates += 1,
                                RayCellRole::Hit => stats.hits_applied += 1,
                            }
                            if dirty.mark(chunk_key) {
                                stats.dirty_chunks += 1;
                            }
                        }
                    }
                },
            )
        };
        if partial {
            return Err(LiveError::PartialUpdate { stats: *stats });
        }
        match walked {
            Ok(walk) => {
                if walk.hit_cells != 1 {
                    return Err(LiveError::TraversalFailed { stats: *stats });
                }
                stats.accepted += 1;
                Ok(())
            }
            Err(LiveError::InvalidCoordinate) => {
                stats.invalid += 1;
                Ok(())
            }
            Err(err) => Err(attach_stats(err, *stats)),
        }
    }

    fn dry_exceeds(&self, ray: RayObservation, max_chunks: u64) -> Result<bool, LiveError> {
        let source_key = self.geometry.voxel_key(ray.source)?;
        let hit_key = self.geometry.voxel_key(ray.hit)?;
        let step_limit = u64::try_from(cell_upper_bound(source_key, hit_key).saturating_add(8))
            .unwrap_or(u64::MAX);
        let mut fresh = std::collections::HashSet::new();
        let mut exceeded = false;
        visit_ray(
            &self.geometry,
            ray.source,
            ray.hit,
            0.0,
            step_limit,
            |event| {
                if exceeded {
                    return;
                }
                let (chunk_key, _) = chunk_and_local(event.key, SIDE as i64);
                if self.store.contains_key(chunk_key) || !fresh.insert(chunk_key) {
                    return;
                }
                if self.store.chunk_count() + fresh.len() as u64 > max_chunks {
                    exceeded = true;
                }
            },
        )?;
        Ok(exceeded)
    }
}

fn segment_length(source: [f64; 3], hit: [f64; 3]) -> Result<f64, LiveError> {
    let dx = hit[0] - source[0];
    let dy = hit[1] - source[1];
    let dz = hit[2] - source[2];
    let squared = dx * dx + dy * dy + dz * dz;
    if !squared.is_finite() {
        return Err(LiveError::DistanceOverflow);
    }
    Ok(squared.sqrt())
}

fn attach_stats(err: LiveError, stats: BatchStats) -> LiveError {
    match err {
        LiveError::TraversalFailed { .. } => LiveError::TraversalFailed { stats },
        LiveError::PartialUpdate { .. } => LiveError::PartialUpdate { stats },
        other => other,
    }
}

pub fn open_ray_scene<M: RayUpdateModel>(
    geometry: VoxelGeometry,
    config: &RaySceneConfig,
    model: M,
) -> Result<(RaySceneWriter<M, 16>, SceneReader<M::Cell>), LiveError> {
    RaySceneWriter::open(geometry, config, model)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::live::RangeFilter;
    use crate::live::dda::{RayCellContext, RayCellRole, visit_ray};
    use crate::live::geometry::VoxelKey;
    use crate::live::model::{LogOddsOccupancy, RayUpdateModel};
    use std::collections::HashMap;

    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    struct ScoreCell {
        score: i32,
        updates: u32,
    }

    struct ScoreModel;

    impl RayUpdateModel for ScoreModel {
        type Cell = ScoreCell;

        fn update(&mut self, cell: &mut Self::Cell, event: RayCellContext) {
            cell.updates = cell.updates.saturating_add(1);
            let delta = match event.role {
                RayCellRole::Hit => 10,
                RayCellRole::Free => -1,
            };
            cell.score += delta;
        }
    }

    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    struct OrderCell {
        len: u8,
        roles: [u8; 8],
    }

    struct OrderModel;

    impl RayUpdateModel for OrderModel {
        type Cell = OrderCell;

        fn update(&mut self, cell: &mut Self::Cell, event: RayCellContext) {
            if (cell.len as usize) < cell.roles.len() {
                cell.roles[cell.len as usize] = match event.role {
                    RayCellRole::Free => 1,
                    RayCellRole::Hit => 2,
                };
                cell.len += 1;
            }
        }
    }

    struct LengthModel;

    impl RayUpdateModel for LengthModel {
        type Cell = f64;

        fn needs_metric_length(&self) -> bool {
            true
        }

        fn update(&mut self, cell: &mut Self::Cell, event: RayCellContext) {
            if event.role == RayCellRole::Hit {
                *cell = event.ray_length_m;
            }
        }
    }

    fn geometry() -> VoxelGeometry {
        VoxelGeometry::new([0.0, 0.0, 0.0], 1.0).unwrap()
    }

    #[test]
    fn accepted_rays_match_an_independent_update() {
        let model = LogOddsOccupancy::new(0.85, -0.4, -4.0, 4.0).unwrap();
        let (mut writer, reader) =
            open_ray_scene(geometry(), &RaySceneConfig::default(), model).unwrap();
        let rays = [
            RayObservation {
                source: [0.2, 0.2, 0.2],
                hit: [3.4, 0.8, 0.3],
            },
            RayObservation {
                source: [-1.2, 0.4, 0.4],
                hit: [1.2, -0.6, 0.7],
            },
            RayObservation {
                source: [0.6, 0.6, 0.6],
                hit: [0.6, 0.6, 0.6],
            },
        ];
        let stats = writer.insert_rays(&rays).unwrap();
        assert_eq!(stats.accepted, 3);
        assert_eq!(stats.hits_applied, 3);
        assert!(stats.free_cell_updates > 0);
        let mut reference = HashMap::<VoxelKey, f32>::new();
        for ray in rays {
            visit_ray(&geometry(), ray.source, ray.hit, 0.0, 128, |event| {
                let cell = reference.entry(event.key).or_insert(0.0);
                let delta = match event.role {
                    RayCellRole::Hit => 0.85,
                    RayCellRole::Free => -0.4,
                };
                *cell = (*cell + delta).clamp(-4.0, 4.0);
            })
            .unwrap();
        }
        let expected_len = reference.len();
        writer.publish().unwrap();
        for (key, value) in &reference {
            assert_eq!(reader.get_voxel_key(*key), Some(*value));
        }
        assert_eq!(reader.snapshot_all().iter().count(), expected_len);
    }

    #[test]
    fn filtered_and_capped_rays_do_not_touch_state() {
        let config = RaySceneConfig {
            range: Some(RangeFilter::new(0.5, 30.0).unwrap()),
            max_cells_per_ray: Some(3),
            max_chunks: Some(1),
            ..RaySceneConfig::default()
        };
        let (mut writer, reader) =
            open_ray_scene(geometry(), &config, LogOddsOccupancy::default()).unwrap();
        let stats = writer
            .insert_rays(&[
                RayObservation {
                    source: [0.0, 0.0, 0.0],
                    hit: [10.0, 0.0, 0.0],
                },
                RayObservation {
                    source: [0.0, 0.0, 0.0],
                    hit: [0.1, 0.0, 0.0],
                },
                RayObservation {
                    source: [0.0, 0.0, 0.0],
                    hit: [f64::NAN, 0.0, 0.0],
                },
                RayObservation {
                    source: [0.1, 0.1, 0.1],
                    hit: [0.8, 0.1, 0.1],
                },
            ])
            .unwrap();
        assert_eq!(stats.filtered_by_range, 1);
        assert_eq!(stats.capacity_rejected, 1);
        assert_eq!(stats.invalid, 1);
        assert_eq!(stats.accepted, 1);
        assert_eq!(writer.observed_cells(), 1);
        assert_eq!(writer.chunk_count(), 1);
        writer.publish().unwrap();
        assert!(
            reader
                .get_voxel_key(VoxelKey { x: 0, y: 0, z: 0 })
                .is_some()
        );
        assert!(
            reader
                .get_voxel_key(VoxelKey { x: 10, y: 0, z: 0 })
                .is_none()
        );
        let before = writer.observed_cells();
        let crossing = writer
            .insert_ray(RayObservation {
                source: [15.2, 0.2, 0.2],
                hit: [16.2, 0.2, 0.2],
            })
            .unwrap();
        assert_eq!(crossing.capacity_rejected, 1);
        assert_eq!(writer.observed_cells(), before);
        assert_eq!(writer.chunk_count(), 1);
    }

    #[test]
    fn zero_log_odds_stays_observed_and_order_is_preserved() {
        let zeros = LogOddsOccupancy::new(0.0, 0.0, -1.0, 1.0).unwrap();
        let (mut writer, reader) =
            open_ray_scene(geometry(), &RaySceneConfig::default(), zeros).unwrap();
        writer
            .insert_ray(RayObservation {
                source: [0.2, 0.2, 0.2],
                hit: [0.4, 0.2, 0.2],
            })
            .unwrap();
        writer.publish().unwrap();
        assert_eq!(
            reader.get_voxel_key(VoxelKey { x: 0, y: 0, z: 0 }),
            Some(0.0)
        );
        assert_eq!(reader.get_voxel_key(VoxelKey { x: 1, y: 0, z: 0 }), None);

        let (mut writer, reader) =
            open_ray_scene(geometry(), &RaySceneConfig::default(), OrderModel).unwrap();
        writer
            .insert_rays(&[
                RayObservation {
                    source: [0.1, 0.1, 0.1],
                    hit: [1.2, 0.1, 0.1],
                },
                RayObservation {
                    source: [0.2, 0.2, 0.2],
                    hit: [0.3, 0.2, 0.2],
                },
            ])
            .unwrap();
        writer.publish().unwrap();
        let first = reader.get_voxel_key(VoxelKey { x: 0, y: 0, z: 0 }).unwrap();
        assert_eq!(first.roles[0], 1);
        assert_eq!(first.roles[1], 2);
        assert_eq!(first.len, 2);
        let endpoint = reader.get_voxel_key(VoxelKey { x: 1, y: 0, z: 0 }).unwrap();
        assert_eq!(endpoint.roles[0], 2);
        assert_eq!(endpoint.len, 1);
    }

    #[test]
    fn custom_cell_and_metric_length_are_visible() {
        let (mut writer, reader) =
            open_ray_scene(geometry(), &RaySceneConfig::default(), ScoreModel).unwrap();
        let stats = writer
            .insert_ray(RayObservation {
                source: [0.2, 0.2, 0.2],
                hit: [2.2, 0.2, 0.2],
            })
            .unwrap();
        assert_eq!(stats.hits_applied, 1);
        assert_eq!(stats.free_cell_updates, 2);
        writer.publish().unwrap();
        let hit = reader.get_voxel_key(VoxelKey { x: 2, y: 0, z: 0 }).unwrap();
        assert_eq!(hit.score, 10);
        assert_eq!(hit.updates, 1);
        let free = reader.get_voxel_key(VoxelKey { x: 0, y: 0, z: 0 }).unwrap();
        assert_eq!(free.score, -1);
        assert_eq!(free.updates, 1);
        assert!(
            reader
                .get_voxel_key(VoxelKey { x: 3, y: 0, z: 0 })
                .is_none()
        );

        let (mut writer, reader) =
            open_ray_scene(geometry(), &RaySceneConfig::default(), LengthModel).unwrap();
        writer
            .insert_ray(RayObservation {
                source: [0.0, 0.0, 0.0],
                hit: [3.0, 4.0, 0.0],
            })
            .unwrap();
        writer.publish().unwrap();
        let length = reader.get_voxel_key(VoxelKey { x: 3, y: 4, z: 0 }).unwrap();
        assert!((length - 5.0).abs() < 1e-9);
    }

    #[test]
    fn ray_reader_is_shared_across_threads() {
        fn assert_send<T: Send>() {}
        fn assert_sync<T: Sync>() {}
        assert_send::<RaySceneWriter<LogOddsOccupancy, 16>>();
        assert_send::<SceneReader<f32>>();
        assert_sync::<SceneReader<f32>>();
        assert_send::<ScoreCell>();
    }
}
