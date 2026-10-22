use std::collections::HashMap;
use std::sync::{Arc, RwLock};
use std::time::Instant;

use super::chunk::{PublishedChunk, WorkingChunk};
use super::config::{PointSceneConfig, RangeClass, RangeFilter, RaySceneConfig};
use super::error::LiveError;
use super::geometry::{ChunkKey, VoxelBounds, VoxelGeometry, VoxelKey, chunk_and_local};
use super::model::RayUpdateModel;
use super::point::{Occupied, PointSceneWriter};
use super::published::{PublishedSnapshot, unix_ms};
use super::ray::{RayObservation, RaySceneWriter};
use super::stats::{BatchStats, PublishStats, SceneMemoryStats};
use super::votes::{VoteModel, Votes, was_updated};

pub const COARSE_SIDE: usize = 16;
pub const FINE_SIDE: usize = 32;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClearRule {
    max_hit_votes: f64,
    min_miss_votes: f64,
    max_hit_units: u32,
    min_miss_units: u32,
}

impl ClearRule {
    pub fn new(max_hit_votes: f64, min_miss_votes: f64) -> Result<Self, LiveError> {
        let valid = |value: f64| value.is_finite() && value >= 0.0;
        if !valid(max_hit_votes) || !valid(min_miss_votes) {
            return Err(LiveError::InvalidClearRule);
        }
        let unit = f64::from(Votes::UNIT);
        Ok(Self {
            max_hit_votes,
            min_miss_votes,
            max_hit_units: (max_hit_votes * unit).floor().min(f64::from(u32::MAX)) as u32,
            min_miss_units: (min_miss_votes * unit).ceil().min(f64::from(u32::MAX)) as u32,
        })
    }

    pub fn max_hit_votes(&self) -> f64 {
        self.max_hit_votes
    }

    pub fn min_miss_votes(&self) -> f64 {
        self.min_miss_votes
    }

    pub fn clears(&self, votes: Votes) -> bool {
        votes.hits <= self.max_hit_units && votes.misses >= self.min_miss_units
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PairedSceneConfig {
    pub coarse: RaySceneConfig,
    pub fine_promotion_threshold: usize,
    pub clear: Option<ClearRule>,
}

impl Default for PairedSceneConfig {
    fn default() -> Self {
        Self {
            coarse: RaySceneConfig::default(),
            fine_promotion_threshold: 256,
            clear: None,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PairedBatchStats {
    pub coarse: BatchStats,
    pub fine: BatchStats,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PairedPublishStats {
    pub publish: PublishStats,
    pub coarse_chunks: u64,
    pub fine_chunks: u64,
    pub fine_cleared: u64,
}

type Handle<Cell> = (ChunkKey, Arc<PublishedChunk<Cell>>);

struct PairedState {
    coarse: HashMap<ChunkKey, Arc<PublishedChunk<Votes>>>,
    fine: HashMap<ChunkKey, Arc<PublishedChunk<Occupied>>>,
    masks: HashMap<ChunkKey, Arc<PublishedChunk<u8>>>,
    generation: u64,
    published_at: Option<Instant>,
    published_unix_ms: Option<u64>,
}

struct PairedIndex {
    inner: RwLock<PairedState>,
}

impl PairedIndex {
    fn read(&self) -> std::sync::RwLockReadGuard<'_, PairedState> {
        self.inner
            .read()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    fn write(&self) -> std::sync::RwLockWriteGuard<'_, PairedState> {
        self.inner
            .write()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

fn install<Cell>(
    map: &mut HashMap<ChunkKey, Arc<PublishedChunk<Cell>>>,
    prepared: Vec<Handle<Cell>>,
    retired: &mut Vec<Arc<PublishedChunk<Cell>>>,
) {
    for (key, chunk) in prepared {
        if let Some(old) = map.insert(key, chunk) {
            retired.push(old);
        }
    }
}

fn handles<Cell>(map: &HashMap<ChunkKey, Arc<PublishedChunk<Cell>>>) -> Vec<Handle<Cell>> {
    map.iter()
        .map(|(key, chunk)| (*key, Arc::clone(chunk)))
        .collect()
}

fn collect<Cell>(
    map: &HashMap<ChunkKey, Arc<PublishedChunk<Cell>>>,
    min: ChunkKey,
    max: ChunkKey,
) -> Vec<Handle<Cell>> {
    let span = |lo: i64, hi: i64| u128::from(hi.abs_diff(lo)) + 1;
    let region = span(min.x, max.x)
        .checked_mul(span(min.y, max.y))
        .and_then(|area| area.checked_mul(span(min.z, max.z)));
    let mut out = Vec::new();
    if region.is_some_and(|count| count <= map.len() as u128) {
        for z in min.z..=max.z {
            for y in min.y..=max.y {
                for x in min.x..=max.x {
                    let key = ChunkKey { x, y, z };
                    if let Some(chunk) = map.get(&key) {
                        out.push((key, Arc::clone(chunk)));
                    }
                }
            }
        }
    } else {
        for (key, chunk) in map {
            if (min.x..=max.x).contains(&key.x)
                && (min.y..=max.y).contains(&key.y)
                && (min.z..=max.z).contains(&key.z)
            {
                out.push((*key, Arc::clone(chunk)));
            }
        }
    }
    out
}

pub struct PairedSceneWriter {
    coarse: RaySceneWriter<VoteModel, COARSE_SIDE>,
    fine: PointSceneWriter<Occupied, FINE_SIDE>,
    range: Option<RangeFilter>,
    clear: Option<ClearRule>,
    mask_threshold: usize,
    published: Arc<PairedIndex>,
    hits: Vec<[f64; 3]>,
}

impl PairedSceneWriter {
    pub fn open(
        coarse_geometry: VoxelGeometry,
        config: &PairedSceneConfig,
        model: VoteModel,
    ) -> Result<(Self, PairedReader), LiveError> {
        let fine_geometry =
            VoxelGeometry::new(coarse_geometry.origin(), coarse_geometry.voxel_size() / 2.0)?;
        let (coarse, _) = RaySceneWriter::open(coarse_geometry, &config.coarse, model)?;
        let fine_config = PointSceneConfig {
            range: None,
            max_chunks: config.coarse.max_chunks,
            promotion_threshold: config.fine_promotion_threshold,
            journal: config.coarse.journal,
        };
        let (fine, _) = PointSceneWriter::open(fine_geometry, &fine_config)?;
        let published = Arc::new(PairedIndex {
            inner: RwLock::new(PairedState {
                coarse: HashMap::new(),
                fine: HashMap::new(),
                masks: HashMap::new(),
                generation: 0,
                published_at: None,
                published_unix_ms: None,
            }),
        });
        let reader = PairedReader {
            index: Arc::clone(&published),
            coarse_geometry,
            fine_geometry,
        };
        let writer = Self {
            coarse,
            fine,
            range: config.coarse.range,
            clear: config.clear,
            mask_threshold: config.coarse.promotion_threshold,
            published,
            hits: Vec::new(),
        };
        Ok((writer, reader))
    }

    pub fn reader(&self) -> PairedReader {
        PairedReader {
            index: Arc::clone(&self.published),
            coarse_geometry: self.coarse.geometry(),
            fine_geometry: self.fine.geometry(),
        }
    }

    pub fn insert_rays(&mut self, rays: &[RayObservation]) -> Result<PairedBatchStats, LiveError> {
        let coarse = self.coarse.insert_rays(rays)?;
        self.hits.clear();
        for ray in rays {
            if accepts(self.range, ray.source, ray.hit) {
                self.hits.push(ray.hit);
            }
        }
        let fine = self.fine.insert_points(&self.hits)?;
        Ok(PairedBatchStats { coarse, fine })
    }

    pub fn insert_rays_from_source(
        &mut self,
        source: [f64; 3],
        hits: &[[f64; 3]],
    ) -> Result<PairedBatchStats, LiveError> {
        let coarse = self.coarse.insert_rays_from_source(source, hits)?;
        let fine = match self.range {
            None => self.fine.insert_points(hits)?,
            Some(_) => {
                self.hits.clear();
                for hit in hits {
                    if accepts(self.range, source, *hit) {
                        self.hits.push(*hit);
                    }
                }
                self.fine.insert_points(&self.hits)?
            }
        };
        Ok(PairedBatchStats { coarse, fine })
    }

    pub fn publish(&mut self) -> Result<PairedPublishStats, LiveError> {
        let started = Instant::now();
        let clear = self.clear;
        let (coarse_store, coarse_dirty, model) = self.coarse.parts_mut();
        let (fine_store, fine_dirty) = self.fine.parts_mut();
        if coarse_dirty.is_empty() && fine_dirty.is_empty() {
            let state = self.published.read();
            return Ok(PairedPublishStats {
                publish: PublishStats {
                    generation: state.generation,
                    chunks_published: 0,
                    duration_ns: 0,
                    published_unix_ms: state.published_unix_ms.unwrap_or(0),
                },
                coarse_chunks: 0,
                fine_chunks: 0,
                fine_cleared: 0,
            });
        }

        let decays = model.needs_pass_end();
        let mut coarse_keys: Vec<ChunkKey> = coarse_dirty.keys().collect();
        coarse_keys.sort_unstable();
        let mut fine_cleared = 0u64;
        let mut clearable = vec![0u64; CLEARABLE_WORDS];
        let mut to_remove = Vec::new();
        if decays {
            for &key in &coarse_keys {
                let Some(chunk) = coarse_store.chunk_mut(key) else {
                    continue;
                };
                let mut any = false;
                chunk.for_each_observed_mut(|local, votes| {
                    let updated = was_updated(votes);
                    model.pass_end(votes);
                    if updated && clear.is_some_and(|rule| rule.clears(*votes)) {
                        clearable[local as usize / 64] |= 1 << (local % 64);
                        any = true;
                    }
                });
                if !any {
                    continue;
                }
                if let Some(fine_chunk) = fine_store.chunk_mut(key) {
                    to_remove.clear();
                    fine_chunk.for_each_observed_local(|local| {
                        let (parent, _) = parent_of(local);
                        if clearable[parent as usize / 64] & (1 << (parent % 64)) != 0 {
                            to_remove.push(local);
                        }
                    });
                    for &local in &to_remove {
                        fine_chunk.remove_local(local);
                    }
                    if !to_remove.is_empty() {
                        let removed = to_remove.len() as u64;
                        fine_store.forget_cells(removed);
                        fine_dirty.mark(key);
                        fine_cleared += removed;
                    }
                }
                clearable.fill(0);
            }
        }

        let mut fine_keys: Vec<ChunkKey> = fine_dirty.keys().collect();
        fine_keys.sort_unstable();
        let mut coarse_prepared = Vec::with_capacity(coarse_keys.len());
        for &key in &coarse_keys {
            let chunk = coarse_store
                .chunk(key)
                .ok_or(LiveError::MissingDirtyChunk)?;
            coarse_prepared.push((key, Arc::new(chunk.to_published(COARSE_SIDE as u16))));
        }
        let mut fine_prepared = Vec::with_capacity(fine_keys.len());
        let mut mask_prepared = Vec::with_capacity(fine_keys.len());
        for &key in &fine_keys {
            let chunk = fine_store.chunk(key).ok_or(LiveError::MissingDirtyChunk)?;
            fine_prepared.push((key, Arc::new(chunk.to_published(FINE_SIDE as u16))));
            let mut masks = WorkingChunk::<u8, COARSE_SIDE>::new(self.mask_threshold);
            chunk.for_each_observed_local(|local| {
                let (parent, bit) = parent_of(local);
                masks.with_local(parent, 0, |mask, _| *mask |= bit);
            });
            mask_prepared.push((key, Arc::new(masks.to_published(COARSE_SIDE as u16))));
        }

        let coarse_chunks = coarse_prepared.len() as u64;
        let fine_chunks = fine_prepared.len() as u64;
        let published_unix_ms = unix_ms();
        let (mut retired_coarse, mut retired_fine, mut retired_masks) =
            (Vec::new(), Vec::new(), Vec::new());
        let generation = {
            let mut state = self.published.write();
            let generation = state
                .generation
                .checked_add(1)
                .ok_or(LiveError::GenerationOverflow)?;
            install(&mut state.coarse, coarse_prepared, &mut retired_coarse);
            install(&mut state.fine, fine_prepared, &mut retired_fine);
            install(&mut state.masks, mask_prepared, &mut retired_masks);
            state.generation = generation;
            state.published_at = Some(Instant::now());
            state.published_unix_ms = Some(published_unix_ms);
            generation
        };
        drop((retired_coarse, retired_fine, retired_masks));
        coarse_dirty.clear();
        fine_dirty.clear();
        Ok(PairedPublishStats {
            publish: PublishStats {
                generation,
                chunks_published: coarse_chunks + 2 * fine_chunks,
                duration_ns: u64::try_from(started.elapsed().as_nanos()).unwrap_or(u64::MAX),
                published_unix_ms,
            },
            coarse_chunks,
            fine_chunks,
            fine_cleared,
        })
    }

    pub fn coarse_geometry(&self) -> VoxelGeometry {
        self.coarse.geometry()
    }

    pub fn fine_geometry(&self) -> VoxelGeometry {
        self.fine.geometry()
    }

    pub fn memory_stats(&self) -> SceneMemoryStats {
        let coarse = self.coarse.memory_stats();
        let fine = self.fine.memory_stats();
        let state = self.published.read();
        let bytes = |total: u64, chunk: u64| total.saturating_add(chunk);
        let published = state
            .coarse
            .values()
            .map(|chunk| chunk.estimated_bytes())
            .chain(state.fine.values().map(|chunk| chunk.estimated_bytes()))
            .chain(state.masks.values().map(|chunk| chunk.estimated_bytes()))
            .fold(0, bytes);
        SceneMemoryStats {
            working_chunks: coarse.working_chunks + fine.working_chunks,
            observed_cells: coarse.observed_cells + fine.observed_cells,
            dirty_chunks: coarse.dirty_chunks + fine.dirty_chunks,
            estimated_working_bytes: coarse.estimated_working_bytes + fine.estimated_working_bytes,
            published_chunks: (state.coarse.len() + state.fine.len() + state.masks.len()) as u64,
            estimated_published_bytes: published,
            generation: state.generation,
            ..SceneMemoryStats::default()
        }
    }
}

fn accepts(range: Option<RangeFilter>, source: [f64; 3], hit: [f64; 3]) -> bool {
    range.is_none_or(|filter| filter.classify(source, hit) == RangeClass::Accept)
}

const CLEARABLE_WORDS: usize = (COARSE_SIDE * COARSE_SIDE * COARSE_SIDE).div_ceil(64);
const FINE_WIDTH: u16 = FINE_SIDE as u16;
const COARSE_WIDTH: u16 = COARSE_SIDE as u16;

// A fine voxel's bit in its parent's 8-bit mask is x + 2y + 4z of its offset (0 or 1 per
// axis) inside the coarse voxel.
fn parent_of(fine: u16) -> (u16, u8) {
    let (x, y, z) = (
        fine % FINE_WIDTH,
        (fine / FINE_WIDTH) % FINE_WIDTH,
        fine / (FINE_WIDTH * FINE_WIDTH),
    );
    let parent = x / 2 + COARSE_WIDTH * (y / 2) + COARSE_WIDTH * COARSE_WIDTH * (z / 2);
    let bit = (x & 1) | ((y & 1) << 1) | ((z & 1) << 2);
    (parent, 1 << bit)
}

#[derive(Clone)]
pub struct PairedReader {
    index: Arc<PairedIndex>,
    coarse_geometry: VoxelGeometry,
    fine_geometry: VoxelGeometry,
}

impl PairedReader {
    pub fn generation(&self) -> u64 {
        self.index.read().generation
    }

    pub fn coarse_geometry(&self) -> VoxelGeometry {
        self.coarse_geometry
    }

    pub fn fine_geometry(&self) -> VoxelGeometry {
        self.fine_geometry
    }

    pub fn snapshot_region(&self, bounds: VoxelBounds) -> Result<PairedSnapshot, LiveError> {
        let bounds = VoxelBounds::try_new(bounds.min, bounds.max)?;
        let fine_bounds = VoxelBounds::try_new(
            VoxelKey {
                x: bounds.min.x * 2,
                y: bounds.min.y * 2,
                z: bounds.min.z * 2,
            },
            VoxelKey {
                x: bounds.max.x * 2 + 1,
                y: bounds.max.y * 2 + 1,
                z: bounds.max.z * 2 + 1,
            },
        )?;
        let (min, _) = chunk_and_local(bounds.min, COARSE_SIDE as i64);
        let (max, _) = chunk_and_local(bounds.max, COARSE_SIDE as i64);
        let state = self.index.read();
        let coarse = collect(&state.coarse, min, max);
        let fine = collect(&state.fine, min, max);
        let masks = collect(&state.masks, min, max);
        let (generation, at) = (state.generation, state.published_at);
        drop(state);
        Ok(PairedSnapshot {
            coarse: PublishedSnapshot::new(
                generation,
                at,
                coarse,
                COARSE_SIDE as i64,
                Some(bounds),
            ),
            fine: PublishedSnapshot::new(generation, at, fine, FINE_SIDE as i64, Some(fine_bounds)),
            masks: PublishedSnapshot::new(generation, at, masks, COARSE_SIDE as i64, Some(bounds)),
        })
    }

    pub fn snapshot_all(&self) -> PairedSnapshot {
        let state = self.index.read();
        let coarse = handles(&state.coarse);
        let fine = handles(&state.fine);
        let masks = handles(&state.masks);
        let (generation, at) = (state.generation, state.published_at);
        drop(state);
        PairedSnapshot {
            coarse: PublishedSnapshot::new(generation, at, coarse, COARSE_SIDE as i64, None),
            fine: PublishedSnapshot::new(generation, at, fine, FINE_SIDE as i64, None),
            masks: PublishedSnapshot::new(generation, at, masks, COARSE_SIDE as i64, None),
        }
    }
}

pub struct PairedSnapshot {
    pub coarse: PublishedSnapshot<Votes>,
    pub fine: PublishedSnapshot<Occupied>,
    pub masks: PublishedSnapshot<u8>,
}

impl PairedSnapshot {
    pub fn generation(&self) -> u64 {
        self.coarse.generation()
    }

    pub fn sub_voxel_mask(&self, coarse: VoxelKey) -> u8 {
        self.masks.get(coarse).unwrap_or(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::live::VoteDecay;
    use std::collections::HashSet;

    fn children(coarse: u16) -> impl Iterator<Item = u16> {
        let (x, y, z) = (
            coarse % COARSE_WIDTH,
            (coarse / COARSE_WIDTH) % COARSE_WIDTH,
            coarse / (COARSE_WIDTH * COARSE_WIDTH),
        );
        (0..8u16).map(move |bit| {
            let fx = 2 * x + (bit & 1);
            let fy = 2 * y + ((bit >> 1) & 1);
            let fz = 2 * z + ((bit >> 2) & 1);
            fx + FINE_WIDTH * fy + FINE_WIDTH * FINE_WIDTH * fz
        })
    }

    fn random_rays(count: usize, seed: u64) -> Vec<RayObservation> {
        let mut state = seed | 1;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f64 / (1u64 << 53) as f64
        };
        (0..count)
            .map(|_| RayObservation {
                source: [next() * 2.0 - 1.0, next() * 2.0 - 1.0, 1.0 + next()],
                hit: [
                    next() * 24.0 - 12.0,
                    next() * 24.0 - 12.0,
                    next() * 6.0 - 2.0,
                ],
            })
            .collect()
    }

    fn decaying(keep: f64) -> VoteModel {
        VoteModel::new(VoteDecay::Exponential { keep }).unwrap()
    }

    #[test]
    fn children_and_parents_agree() {
        for coarse in 0..(COARSE_WIDTH * COARSE_WIDTH * COARSE_WIDTH) {
            let mut bits = 0u8;
            for child in children(coarse) {
                let (parent, bit) = parent_of(child);
                assert_eq!(parent, coarse);
                assert_eq!(bits & bit, 0);
                bits |= bit;
            }
            assert_eq!(bits, u8::MAX);
        }
    }

    #[test]
    fn fine_grid_nests_exactly_inside_the_coarse_grid() {
        for origin in [[0.0, 0.0, 0.0], [0.3, -1.7, 0.1], [-5.05, 2.2, 7.9]] {
            for size in [0.2, 0.25, 0.1, 0.3] {
                let coarse = VoxelGeometry::new(origin, size).unwrap();
                let fine = VoxelGeometry::new(origin, size / 2.0).unwrap();
                for ray in random_rays(4_000, 0x9e37_79b9) {
                    let parent = coarse.voxel_key(ray.hit).unwrap();
                    let child = fine.voxel_key(ray.hit).unwrap();
                    assert_eq!(
                        child.x.div_euclid(2),
                        parent.x,
                        "{origin:?} {size} {:?}",
                        ray.hit
                    );
                    assert_eq!(child.y.div_euclid(2), parent.y);
                    assert_eq!(child.z.div_euclid(2), parent.z);
                }
                let boundary = [
                    origin[0] + 3.0 * size,
                    origin[1] - 2.0 * size,
                    origin[2] + size / 2.0,
                ];
                let child = fine.voxel_key(boundary).unwrap();
                assert_eq!(child.x.div_euclid(2), coarse.voxel_key(boundary).unwrap().x);
                assert_eq!(child.z.div_euclid(2), coarse.voxel_key(boundary).unwrap().z);
            }
        }
    }

    #[test]
    fn layers_share_one_generation_and_masks_match_the_fine_map() {
        let geometry = VoxelGeometry::new([0.3, -1.7, 0.1], 0.2).unwrap();
        let (mut writer, reader) = PairedSceneWriter::open(
            geometry,
            &PairedSceneConfig::default(),
            VoteModel::default(),
        )
        .unwrap();
        for pass in 0..3 {
            let stats = writer
                .insert_rays(&random_rays(3_000, 0x51 + pass))
                .unwrap();
            assert_eq!(stats.coarse.accepted, 3_000);
            assert_eq!(stats.fine.accepted, 3_000);
            let published = writer.publish().unwrap();
            assert_eq!(published.publish.generation, pass + 1);
            assert!(published.fine_chunks > 0);
        }
        assert_eq!(writer.publish().unwrap().publish.generation, 3);
        let snapshot = reader.snapshot_all();
        assert_eq!(snapshot.generation(), 3);
        assert_eq!(snapshot.fine.generation(), 3);
        assert_eq!(snapshot.masks.generation(), 3);
        let mut expected: HashMap<VoxelKey, u8> = HashMap::new();
        for (child, _) in snapshot.fine.iter() {
            let parent = VoxelKey {
                x: child.x.div_euclid(2),
                y: child.y.div_euclid(2),
                z: child.z.div_euclid(2),
            };
            let bit =
                child.x.rem_euclid(2) | child.y.rem_euclid(2) << 1 | child.z.rem_euclid(2) << 2;
            *expected.entry(parent).or_default() |= 1 << bit;
            assert!(
                snapshot
                    .coarse
                    .get(parent)
                    .is_some_and(|votes| votes.hits > 0)
            );
        }
        let masks: HashMap<VoxelKey, u8> = snapshot.masks.iter().collect();
        assert!(!masks.is_empty());
        assert_eq!(masks, expected);
        for (parent, mask) in &masks {
            assert_eq!(snapshot.sub_voxel_mask(*parent), *mask);
        }
        let memory = writer.memory_stats();
        assert_eq!(memory.generation, 3);
        assert!(memory.estimated_published_bytes > 0);
    }

    #[test]
    fn region_snapshots_bound_both_resolutions() {
        let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], 1.0).unwrap();
        let (mut writer, reader) = PairedSceneWriter::open(
            geometry,
            &PairedSceneConfig::default(),
            VoteModel::default(),
        )
        .unwrap();
        writer
            .insert_rays_from_source([0.5, 0.5, 0.5], &[[3.75, 0.25, 0.25], [40.25, 0.25, 0.25]])
            .unwrap();
        writer.publish().unwrap();
        let near =
            VoxelBounds::try_new(VoxelKey { x: 0, y: 0, z: 0 }, VoxelKey { x: 5, y: 0, z: 0 })
                .unwrap();
        let region = reader.snapshot_region(near).unwrap();
        let fine: Vec<VoxelKey> = region.fine.iter().map(|(key, _)| key).collect();
        assert_eq!(fine, vec![VoxelKey { x: 7, y: 0, z: 0 }]);
        assert_eq!(region.sub_voxel_mask(VoxelKey { x: 3, y: 0, z: 0 }), 0b10);
        assert_eq!(region.sub_voxel_mask(VoxelKey { x: 40, y: 0, z: 0 }), 0);
        assert!(region.coarse.get(VoxelKey { x: 40, y: 0, z: 0 }).is_none());
        assert_eq!(
            reader
                .snapshot_all()
                .sub_voxel_mask(VoxelKey { x: 40, y: 0, z: 0 }),
            0b01
        );
    }

    #[test]
    fn range_filter_applies_to_both_maps() {
        let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], 1.0).unwrap();
        let mut config = PairedSceneConfig::default();
        config.coarse.range = Some(RangeFilter::new(0.0, 5.0).unwrap());
        let (mut writer, reader) =
            PairedSceneWriter::open(geometry, &config, VoteModel::default()).unwrap();
        let stats = writer
            .insert_rays_from_source([0.5, 0.5, 0.5], &[[3.25, 0.5, 0.5], [9.5, 0.5, 0.5]])
            .unwrap();
        assert_eq!(stats.coarse.filtered_by_range, 1);
        assert_eq!(stats.fine.accepted, 1);
        let far = RayObservation {
            source: [0.5, 0.5, 0.5],
            hit: [0.5, 9.5, 0.5],
        };
        assert_eq!(writer.insert_rays(&[far]).unwrap().fine.input, 0);
        writer.publish().unwrap();
        let snapshot = reader.snapshot_all();
        let fine: Vec<VoxelKey> = snapshot.fine.iter().map(|(key, _)| key).collect();
        assert_eq!(fine, vec![VoxelKey { x: 6, y: 1, z: 1 }]);
    }

    #[test]
    fn coarse_free_evidence_clears_fine_hits() {
        let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], 1.0).unwrap();
        let config = PairedSceneConfig {
            clear: Some(ClearRule::new(0.25, 1.0).unwrap()),
            ..PairedSceneConfig::default()
        };
        let source = [0.5, 0.5, 0.5];
        let gone = [3.25, 0.5, 0.5];
        let through = [7.5, 0.5, 0.5];
        let kept = [0.5, 3.25, 0.5];
        let run = |model: VoteModel| {
            let (mut writer, reader) = PairedSceneWriter::open(geometry, &config, model).unwrap();
            writer
                .insert_rays_from_source(source, &[gone, kept])
                .unwrap();
            assert_eq!(writer.publish().unwrap().fine_cleared, 0);
            let mut cleared = 0;
            for _ in 0..3 {
                writer
                    .insert_rays_from_source(source, &[through, through, through, through, kept])
                    .unwrap();
                cleared += writer.publish().unwrap().fine_cleared;
            }
            (cleared, reader.snapshot_all())
        };
        let (cleared, snapshot) = run(decaying(0.5));
        assert_eq!(cleared, 1);
        let fine: HashSet<VoxelKey> = snapshot.fine.iter().map(|(key, _)| key).collect();
        assert!(!fine.contains(&VoxelKey { x: 6, y: 1, z: 1 }));
        assert!(fine.contains(&VoxelKey { x: 1, y: 6, z: 1 }));
        assert_eq!(snapshot.sub_voxel_mask(VoxelKey { x: 3, y: 0, z: 0 }), 0);
        assert_ne!(snapshot.sub_voxel_mask(VoxelKey { x: 0, y: 3, z: 0 }), 0);
        let (cleared, snapshot) = run(VoteModel::default());
        assert_eq!(cleared, 0);
        assert_ne!(snapshot.sub_voxel_mask(VoxelKey { x: 3, y: 0, z: 0 }), 0);
    }

    #[test]
    fn clear_rule_validation() {
        assert!(ClearRule::new(-0.1, 1.0).is_err());
        assert!(ClearRule::new(0.1, f64::NAN).is_err());
        let rule = ClearRule::new(0.25, 1.0).unwrap();
        assert!(rule.clears(Votes {
            hits: 64,
            misses: 256
        }));
        assert!(!rule.clears(Votes {
            hits: 65,
            misses: 256
        }));
        assert!(!rule.clears(Votes {
            hits: 0,
            misses: 255
        }));
        assert_eq!(rule.max_hit_votes(), 0.25);
        assert_eq!(rule.min_miss_votes(), 1.0);
    }
}
