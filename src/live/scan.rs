use std::collections::HashMap;

use super::dda::{GridRay, RayCellContext, RayCellRole, walk};
use super::error::LiveError;
use super::geometry::{ChunkKey, VoxelGeometry, voxel_from_chunk};
use super::model::RayUpdateModel;
use super::ray::{RaySceneWriter, attach_stats, segment_length};
use super::stats::BatchStats;
use super::store::{CellTouch, ChunkStore, DirtySet};
use super::{RangeClass, RangeFilter};

/// Rays counted before the counts are added to the map. A larger call is split into
/// batches of this many, which keeps every count in one `apply_counts` at or below 2^20.
pub const SCAN_BATCH_RAYS: usize = 1 << 20;

// Above this many chunk slots, a scan's box is too large for the lookup table and its
// chunks are found through a hash map instead.
const MAX_TABLE_SLOTS: i64 = 1 << 22;

/// A ray model whose result depends only on how many hit and free events a cell gets,
/// not on their order, so a scan's events can be counted first and applied once.
pub trait CountedRayModel: RayUpdateModel {
    /// Applies `hits` hit events and `misses` free events to `cell` at once.
    fn apply_counts(&mut self, cell: &mut Self::Cell, hits: u32, misses: u32);
}

impl<M: CountedRayModel, const SIDE: usize> RaySceneWriter<M, SIDE> {
    /// Same result and statistics as `insert_rays_from_source`, but each voxel the scan
    /// touches is written to the map once: the rays' events are first counted in a
    /// scan-local accumulator, then each voxel's hit and free counts are applied together.
    /// With `max_chunks` set it falls back to `insert_rays_from_source`.
    pub fn insert_scan_from_source(
        &mut self,
        source: [f64; 3],
        hits: &[[f64; 3]],
    ) -> Result<BatchStats, LiveError> {
        self.insert_scan_from_source_threads(source, hits, 1)
    }

    /// `insert_scan_from_source` with the rays split into `threads` contiguous slices,
    /// each counted on its own thread. The slices are then added to the map in order on
    /// the calling thread, so the map has one writer and the result does not depend on
    /// the thread count.
    pub fn insert_scan_from_source_threads(
        &mut self,
        source: [f64; 3],
        hits: &[[f64; 3]],
        threads: usize,
    ) -> Result<BatchStats, LiveError> {
        if self.max_chunks.is_some() {
            return self.insert_rays_from_source(source, hits);
        }
        let mut stats = BatchStats {
            input: hits.len() as u64,
            ..BatchStats::default()
        };
        let threads = threads.max(1);
        while self.scan.len() < threads {
            self.scan.push(ScanAccumulator::new());
        }
        let ray = RayLimits {
            geometry: self.geometry,
            range: self.range,
            cell_cap: self.cell_cap,
            metric_length: self.model.needs_metric_length(),
        };
        for batch in hits.chunks(SCAN_BATCH_RAYS) {
            let per_slice = batch.len().div_ceil(threads).max(1);
            let mut parts = Vec::with_capacity(threads);
            if threads == 1 {
                parts.push(self.scan[0].accumulate(&ray, source, batch));
            } else {
                std::thread::scope(|scope| {
                    let handles: Vec<_> = self
                        .scan
                        .iter_mut()
                        .zip(batch.chunks(per_slice))
                        .map(|(scan, slice)| {
                            let ray = &ray;
                            scope.spawn(move || scan.accumulate(ray, source, slice))
                        })
                        .collect();
                    for handle in handles {
                        match handle.join() {
                            Ok(part) => parts.push(part),
                            Err(panic) => std::panic::resume_unwind(panic),
                        }
                    }
                });
            }
            // Slices go into the map in ray order. After a failed ray, as in the per-ray
            // path, nothing later is applied.
            let mut failure = None;
            let mut over_capacity = false;
            for (index, (part, error)) in parts.into_iter().enumerate() {
                let scan = &mut self.scan[index];
                if failure.is_some() || over_capacity {
                    scan.discard();
                    continue;
                }
                add_ray_counts(&mut stats, part);
                over_capacity = scan.merge_into(
                    &mut self.store,
                    &mut self.dirty,
                    &mut self.model,
                    &mut stats,
                );
                failure = error;
            }
            if over_capacity {
                return Err(LiveError::PartialUpdate { stats });
            }
            if let Some(err) = failure {
                return Err(attach_stats(err, stats));
            }
        }
        Ok(stats)
    }
}

fn add_ray_counts(total: &mut BatchStats, part: BatchStats) {
    total.accepted += part.accepted;
    total.filtered_by_range += part.filtered_by_range;
    total.invalid += part.invalid;
    total.distance_overflow += part.distance_overflow;
    total.capacity_rejected += part.capacity_rejected;
}

// What the per-ray path checks before it walks a ray.
struct RayLimits {
    geometry: VoxelGeometry,
    range: Option<RangeFilter>,
    cell_cap: u64,
    metric_length: bool,
}

/// Counts of one scan's events per voxel, in blocks aligned with the map's chunks.
/// Blocks are kept between scans and come back zeroed from each merge.
pub(crate) struct ScanAccumulator<const SIDE: usize> {
    // Hit and free event counts, one block of SIDE³ cells per chunk in use, back to back.
    // A merge reads every cell of each block in use: a scan fills its chunks well (about
    // a third of their cells in the warehouse), so that costs less than marking each
    // touched cell as it is counted.
    counts: Vec<[u32; 2]>,
    // Chunk of each block in use, in first-touch order.
    keys: Vec<ChunkKey>,
    // Block index plus one for each chunk in the scan's box, 0 for none.
    table: Vec<u32>,
    spill: HashMap<ChunkKey, u32>,
    low: [i64; 3],
    dims: [i64; 3],
}

impl<const SIDE: usize> ScanAccumulator<SIDE> {
    const CELLS: usize = {
        assert!(SIDE.is_power_of_two());
        SIDE * SIDE * SIDE
    };

    pub(crate) fn new() -> Self {
        Self {
            counts: Vec::new(),
            keys: Vec::new(),
            table: Vec::new(),
            spill: HashMap::new(),
            low: [0; 3],
            dims: [0; 3],
        }
    }

    // Every cell of a ray lies in the box of its two end voxels, so the box of the source
    // and the hits holds the whole scan.
    fn reset_box(&mut self, geometry: &VoxelGeometry, source: [f64; 3], hits: &[[f64; 3]]) {
        let side = SIDE as i64;
        let (mut low, mut high) = ([i64::MAX; 3], [i64::MIN; 3]);
        for point in std::iter::once(&source).chain(hits) {
            let Ok(key) = geometry.voxel_key(*point) else {
                continue;
            };
            for (axis, value) in [key.x, key.y, key.z].into_iter().enumerate() {
                let chunk = value.div_euclid(side);
                low[axis] = low[axis].min(chunk);
                high[axis] = high[axis].max(chunk);
            }
        }
        self.table.clear();
        self.dims = [0; 3];
        let extent = |axis: usize| high[axis].checked_sub(low[axis])?.checked_add(1);
        let slots = (|| extent(0)?.checked_mul(extent(1)?)?.checked_mul(extent(2)?))();
        if let Some(slots) = slots
            && slots > 0
            && slots <= MAX_TABLE_SLOTS
        {
            self.low = low;
            self.dims = [extent(0).unwrap(), extent(1).unwrap(), extent(2).unwrap()];
            self.table.resize(slots as usize, 0);
        }
    }

    /// Counts the events of the rays from `source` to each of `hits`, with the per-ray
    /// path's checks and counters. Stops at a ray whose traversal fails, keeping that
    /// ray's events so far, as the per-ray path does.
    fn accumulate(
        &mut self,
        limits: &RayLimits,
        source: [f64; 3],
        hits: &[[f64; 3]],
    ) -> (BatchStats, Option<LiveError>) {
        self.reset_box(&limits.geometry, source, hits);
        // Separate borrows and copies, so the per-cell code below keeps them in registers
        // instead of reloading them through `self` for every cell.
        let Self {
            counts,
            keys,
            table,
            spill,
            low,
            dims,
        } = self;
        let (low, dims) = (*low, dims.map(|d| d as u64));
        let mut stats = BatchStats::default();
        let side = SIDE as i64;
        // SIDE is a power of two, so a shift gives the chunk (floor division, also below
        // zero) and a mask the cell within it.
        let shift = SIDE.trailing_zeros();
        let mask = side - 1;
        // The last chunk counted into and its block: most cells of a ray share a chunk
        // with the cell before.
        let mut last: Option<(ChunkKey, usize)> = None;
        for hit in hits {
            if let Some(filter) = limits.range {
                match filter.classify(source, *hit) {
                    RangeClass::Accept => {}
                    RangeClass::Filtered => {
                        stats.filtered_by_range += 1;
                        continue;
                    }
                    RangeClass::Invalid => {
                        stats.invalid += 1;
                        continue;
                    }
                    RangeClass::Overflow => {
                        stats.distance_overflow += 1;
                        continue;
                    }
                }
            }
            let Ok(grid) = GridRay::new(&limits.geometry, source, *hit) else {
                stats.invalid += 1;
                continue;
            };
            let upper = grid.cell_upper_bound();
            if upper > u128::from(limits.cell_cap) {
                stats.capacity_rejected += 1;
                continue;
            }
            if limits.metric_length {
                match segment_length(source, *hit) {
                    Ok(_) => {}
                    Err(LiveError::DistanceOverflow) => {
                        stats.distance_overflow += 1;
                        continue;
                    }
                    Err(_) => {
                        stats.invalid += 1;
                        continue;
                    }
                }
            }
            let step_limit = u64::try_from(upper.saturating_add(8)).unwrap_or(u64::MAX);
            let walked = walk(&grid, None, step_limit, |event: RayCellContext| {
                let key = event.key;
                let chunk = ChunkKey {
                    x: key.x >> shift,
                    y: key.y >> shift,
                    z: key.z >> shift,
                };
                let index = match last {
                    Some((key, index)) if key == chunk => index,
                    _ => {
                        let slot = slot_in(chunk, low, dims);
                        let stored = match slot {
                            Some(slot) => table[slot],
                            None => spill.get(&chunk).copied().unwrap_or(0),
                        };
                        let index = match stored {
                            0 => new_block::<SIDE>(counts, keys, table, spill, chunk, slot),
                            stored => stored as usize - 1,
                        };
                        last = Some((chunk, index));
                        index
                    }
                };
                let local =
                    ((key.x & mask) + side * ((key.y & mask) + side * (key.z & mask))) as usize;
                counts[index * Self::CELLS + local][(event.role == RayCellRole::Free) as usize] +=
                    1;
            });
            match walked {
                Ok(_) => stats.accepted += 1,
                Err(err) => return (stats, Some(err)),
            }
        }
        (stats, None)
    }

    /// Applies every counted voxel to the map once, block by block in first-touch order,
    /// and leaves the accumulator empty. Returns true if the store ran out of chunk slots.
    fn merge_into<M: CountedRayModel>(
        &mut self,
        store: &mut ChunkStore<M::Cell, SIDE>,
        dirty: &mut DirtySet,
        model: &mut M,
        stats: &mut BatchStats,
    ) -> bool {
        let side = SIDE as i64;
        let init = model.initial_cell();
        let mut over_capacity = false;
        for (index, chunk) in self.keys.iter().enumerate() {
            let block = &mut self.counts[index * Self::CELLS..(index + 1) * Self::CELLS];
            for (local, counts) in block.iter_mut().enumerate() {
                if *counts == [0; 2] {
                    continue;
                }
                let [hits, misses] = std::mem::take(counts);
                if over_capacity {
                    continue;
                }
                let key = voxel_from_chunk(*chunk, local as u16, side);
                match store.with_voxel(key, None, init, |cell| {
                    model.apply_counts(cell, hits, misses)
                }) {
                    CellTouch::Updated { chunk_key, newly } => {
                        if newly {
                            stats.newly_observed_cells += 1;
                        }
                        stats.hits_applied += u64::from(hits);
                        stats.free_cell_updates += u64::from(misses);
                        if dirty.mark(chunk_key) {
                            stats.dirty_chunks += 1;
                        }
                    }
                    CellTouch::OverCapacity => over_capacity = true,
                }
            }
        }
        self.keys.clear();
        self.spill.clear();
        over_capacity
    }

    /// Drops the counts without applying them.
    fn discard(&mut self) {
        for index in 0..self.keys.len() {
            self.counts[index * Self::CELLS..(index + 1) * Self::CELLS].fill([0; 2]);
        }
        self.keys.clear();
        self.spill.clear();
    }
}

// Index of `chunk` in the box's table. Wrapped differences compared unsigned: a chunk
// below `low` wraps to a huge value.
#[inline]
fn slot_in(chunk: ChunkKey, low: [i64; 3], dims: [u64; 3]) -> Option<usize> {
    let x = chunk.x.wrapping_sub(low[0]) as u64;
    let y = chunk.y.wrapping_sub(low[1]) as u64;
    let z = chunk.z.wrapping_sub(low[2]) as u64;
    if x >= dims[0] || y >= dims[1] || z >= dims[2] {
        return None;
    }
    Some((x + dims[0] * (y + dims[1] * z)) as usize)
}

// Starts a zeroed block for `chunk`, the first time the scan touches it.
#[cold]
#[inline(never)]
fn new_block<const SIDE: usize>(
    counts: &mut Vec<[u32; 2]>,
    keys: &mut Vec<ChunkKey>,
    table: &mut [u32],
    spill: &mut HashMap<ChunkKey, u32>,
    chunk: ChunkKey,
    slot: Option<usize>,
) -> usize {
    let cells = ScanAccumulator::<SIDE>::CELLS;
    let index = keys.len();
    if (index + 1) * cells > counts.len() {
        counts.resize((index + 1) * cells, [0; 2]);
    }
    keys.push(chunk);
    let stored = index as u32 + 1;
    match slot {
        Some(slot) => table[slot] = stored,
        None => {
            spill.insert(chunk, stored);
        }
    }
    index
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::live::{
        RaySceneConfig, SceneReader, VoteDecay, VoteModel, Votes, VoxelKey, open_ray_scene,
    };

    struct Lcg(u64);

    impl Lcg {
        fn unit(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            f64::from((self.0 >> 33) as u32) / f64::from(u32::MAX)
        }

        fn range(&mut self, lo: f64, hi: f64) -> f64 {
            lo + (hi - lo) * self.unit()
        }
    }

    // A few scans from different sources: rays of mixed length in every direction, a
    // zero-length ray, an invalid hit, and one scan with a far diagonal ray whose box is
    // too large for the lookup table.
    fn scans() -> Vec<([f64; 3], Vec<[f64; 3]>)> {
        let mut lcg = Lcg(0x5ca1_ab1e);
        (0..5)
            .map(|index| {
                let source = [
                    lcg.range(-3.0, 3.0),
                    lcg.range(-3.0, 3.0),
                    lcg.range(-1.0, 1.0),
                ];
                let mut hits: Vec<[f64; 3]> = (0..2500)
                    .map(|_| {
                        let z = lcg.range(-1.0, 1.0);
                        let angle = lcg.range(0.0, std::f64::consts::TAU);
                        let radius = (1.0 - z * z).sqrt();
                        let length = lcg.range(0.0, 14.0);
                        [
                            source[0] + length * radius * angle.cos(),
                            source[1] + length * radius * angle.sin(),
                            source[2] + length * z,
                        ]
                    })
                    .collect();
                hits.push(source);
                hits.push([f64::NAN, 0.0, 0.0]);
                hits.push([source[0] + 60.0, source[1], source[2]]);
                if index == 2 {
                    hits.push([source[0] + 900.0, source[1] + 900.0, source[2] + 900.0]);
                }
                (source, hits)
            })
            .collect()
    }

    fn cells(reader: &SceneReader<Votes>) -> Vec<(VoxelKey, Votes)> {
        let mut cells: Vec<_> = reader.snapshot_all().iter().collect();
        cells.sort_by_key(|(key, _)| (key.x, key.y, key.z));
        cells
    }

    // Each scan inserted then published; `threads` None is the per-ray path.
    fn run(
        config: &RaySceneConfig,
        model: VoteModel,
        threads: Option<usize>,
    ) -> (Vec<(VoxelKey, Votes)>, Vec<BatchStats>) {
        let geometry = VoxelGeometry::new([0.1, -0.2, 0.05], 0.25).unwrap();
        let (mut writer, reader) = open_ray_scene(geometry, config, model).unwrap();
        let mut stats = Vec::new();
        for (source, hits) in scans() {
            stats.push(match threads {
                None => writer.insert_rays_from_source(source, &hits).unwrap(),
                Some(threads) => writer
                    .insert_scan_from_source_threads(source, &hits, threads)
                    .unwrap(),
            });
            writer.publish().unwrap();
        }
        (cells(&reader), stats)
    }

    #[test]
    fn scan_insertion_matches_per_ray_insertion() {
        let configs = [
            RaySceneConfig::default(),
            RaySceneConfig {
                range: Some(RangeFilter::new(0.5, 30.0).unwrap()),
                ..RaySceneConfig::default()
            },
            RaySceneConfig {
                max_cells_per_ray: Some(40),
                promotion_threshold: 1,
                ..RaySceneConfig::default()
            },
            RaySceneConfig {
                max_chunks: Some(1_000_000),
                ..RaySceneConfig::default()
            },
        ];
        let models = [
            VoteModel::default(),
            VoteModel::new(VoteDecay::Exponential { keep: 0.7 }).unwrap(),
            VoteModel::new(VoteDecay::CapHalving { cap_votes: 40 }).unwrap(),
        ];
        for config in &configs {
            for model in models {
                let reference = run(config, model, None);
                assert!(reference.0.len() > 10_000);
                for threads in [1, 3, 8] {
                    assert_eq!(
                        run(config, model, Some(threads)),
                        reference,
                        "{threads} threads"
                    );
                }
            }
        }
    }

    #[test]
    fn far_ray_uses_the_spill_map() {
        let geometry = VoxelGeometry::new([0.0; 3], 0.25).unwrap();
        let mut scan = ScanAccumulator::<16>::new();
        scan.reset_box(&geometry, [0.0; 3], &[[900.0, 900.0, 900.0]]);
        assert!(scan.table.is_empty());
        scan.reset_box(&geometry, [0.0; 3], &[[20.0, -20.0, 5.0]]);
        assert!(!scan.table.is_empty());
    }

    #[test]
    fn counted_votes_equal_single_updates_below_the_halving_point() {
        let mut lcg = Lcg(0xc0ffee);
        let decayed = VoteModel::new(VoteDecay::Exponential { keep: 0.5 }).unwrap();
        for mut model in [VoteModel::default(), decayed] {
            for case in 0..2000 {
                let hits = (lcg.unit() * 3000.0) as u32;
                let misses = (lcg.unit() * 3000.0) as u32;
                // Every fifth start is past the halving point, which the first update halves.
                let top = if case % 5 == 0 {
                    1u32 << 30
                } else {
                    (1 << 30) - 3001 * Votes::UNIT
                };
                let start = Votes {
                    hits: (lcg.unit() * f64::from(top)) as u32
                        + if case % 5 == 0 { top } else { 0 },
                    misses: (lcg.unit() * f64::from(top)) as u32,
                };
                let mut counted = start;
                model.apply_counts(&mut counted, hits, misses);
                let mut single = start;
                let event = |role| RayCellContext {
                    key: VoxelKey { x: 0, y: 0, z: 0 },
                    role,
                    ray_length_m: None,
                    t_enter: 0.0,
                    t_exit: 1.0,
                };
                for _ in 0..misses {
                    model.update(&mut single, event(RayCellRole::Free));
                }
                for _ in 0..hits {
                    model.update(&mut single, event(RayCellRole::Hit));
                }
                if hits + misses > 0 {
                    assert_eq!(counted, single, "case {case}");
                }
            }
        }
    }
}
