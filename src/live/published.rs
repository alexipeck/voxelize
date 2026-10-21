use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use super::chunk::PublishedChunk;
use super::config::JournalConfig;
use super::error::{ChangeHistoryLost, ChangesError, LiveError};
use super::geometry::{ChunkKey, VoxelBounds, VoxelGeometry, VoxelKey, chunk_and_local};
use super::stats::{PublishStats, SceneMemoryStats};

struct JournalRecord {
    generation: u64,
    keys: Vec<ChunkKey>,
}

type ChunkHandle<Cell> = (ChunkKey, Arc<PublishedChunk<Cell>>);

struct PublishedState<Cell> {
    chunks: HashMap<ChunkKey, Arc<PublishedChunk<Cell>>>,
    generation: u64,
    published_unix_ms: Option<u64>,
    published_at: Option<Instant>,
    journal: VecDeque<JournalRecord>,
    journal_keys: usize,
    max_generations: usize,
    max_keys: usize,
}

impl<Cell> PublishedState<Cell> {
    fn check_cursor(&self, cursor: u64) -> Result<(), ChangesError> {
        let current = self.generation;
        if cursor > current {
            return Err(ChangesError::CursorAhead {
                requested_generation: cursor,
                current_generation: current,
            });
        }
        let oldest = match self.journal.front() {
            Some(entry) => entry.generation.saturating_sub(1),
            None => current,
        };
        if cursor < oldest {
            return Err(ChangesError::HistoryLost(ChangeHistoryLost {
                requested_generation: cursor,
                oldest_available_generation: oldest,
                current_generation: current,
            }));
        }
        Ok(())
    }

    fn keys_since(&self, cursor: u64, out: &mut Vec<ChunkKey>) {
        for entry in self.journal.iter().rev() {
            if entry.generation <= cursor {
                break;
            }
            out.extend_from_slice(&entry.keys);
        }
    }

    fn push_present(&self, key: ChunkKey, out: &mut Vec<ChunkHandle<Cell>>) {
        if let Some(chunk) = self.chunks.get(&key) {
            out.push((key, Arc::clone(chunk)));
        }
    }
}

pub(crate) struct PublishedIndex<Cell> {
    inner: RwLock<PublishedState<Cell>>,
}

impl<Cell> PublishedIndex<Cell> {
    pub(crate) fn new(journal: JournalConfig) -> Result<Self, LiveError> {
        journal.validate()?;
        Ok(Self {
            inner: RwLock::new(PublishedState {
                chunks: HashMap::new(),
                generation: 0,
                published_unix_ms: None,
                published_at: None,
                journal: VecDeque::new(),
                journal_keys: 0,
                max_generations: journal.max_generations,
                max_keys: journal.max_keys,
            }),
        })
    }

    fn read(&self) -> std::sync::RwLockReadGuard<'_, PublishedState<Cell>> {
        self.inner
            .read()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    fn write(&self) -> std::sync::RwLockWriteGuard<'_, PublishedState<Cell>> {
        self.inner
            .write()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    pub(crate) fn generation(&self) -> u64 {
        self.read().generation
    }

    pub(crate) fn published_unix_ms(&self) -> Option<u64> {
        self.read().published_unix_ms
    }

    pub(crate) fn published_at(&self) -> Option<Instant> {
        self.read().published_at
    }

    pub(crate) fn commit(
        &self,
        prepared: Vec<ChunkHandle<Cell>>,
        started: Instant,
    ) -> Result<PublishStats, LiveError> {
        let chunks_published = prepared.len() as u64;
        let keys: Vec<ChunkKey> = prepared.iter().map(|(key, _)| *key).collect();
        let key_count = keys.len();
        let published_unix_ms = unix_ms();
        let mut retired = Vec::with_capacity(prepared.len());
        let mut trimmed = Vec::new();
        let generation = {
            let mut guard = self.write();
            let generation = guard
                .generation
                .checked_add(1)
                .ok_or(LiveError::GenerationOverflow)?;
            for (key, chunk) in prepared {
                if let Some(old) = guard.chunks.insert(key, chunk) {
                    retired.push(old);
                }
            }
            guard.journal_keys = guard.journal_keys.saturating_add(key_count);
            guard.journal.push_back(JournalRecord { generation, keys });
            while guard.journal.len() > guard.max_generations
                || (guard.journal_keys > guard.max_keys && guard.journal.len() > 1)
            {
                let Some(old) = guard.journal.pop_front() else {
                    break;
                };
                guard.journal_keys = guard.journal_keys.saturating_sub(old.keys.len());
                trimmed.push(old);
            }
            guard.generation = generation;
            guard.published_unix_ms = Some(published_unix_ms);
            guard.published_at = Some(Instant::now());
            generation
        };
        drop(retired);
        drop(trimmed);
        let duration_ns = u64::try_from(started.elapsed().as_nanos()).unwrap_or(u64::MAX);
        Ok(PublishStats {
            generation,
            chunks_published,
            duration_ns,
            published_unix_ms,
        })
    }

    pub(crate) fn changes_since(&self, cursor: u64) -> Result<Changes, ChangesError> {
        let mut chunk_keys = Vec::new();
        let generation = {
            let guard = self.read();
            guard.check_cursor(cursor)?;
            guard.keys_since(cursor, &mut chunk_keys);
            guard.generation
        };
        chunk_keys.sort_unstable();
        chunk_keys.dedup();
        Ok(Changes {
            generation,
            chunk_keys,
        })
    }

    pub(crate) fn memory(&self, working: SceneMemoryStats) -> SceneMemoryStats
    where
        Cell: Copy,
    {
        let guard = self.read();
        let mut estimated_published_bytes = 0u64;
        let mut max_chunk_strong_count = 0u64;
        for chunk in guard.chunks.values() {
            estimated_published_bytes =
                estimated_published_bytes.saturating_add(chunk.estimated_bytes());
            max_chunk_strong_count = max_chunk_strong_count.max(Arc::strong_count(chunk) as u64);
        }
        estimated_published_bytes = estimated_published_bytes
            .saturating_add(guard.journal_keys as u64 * size_of::<ChunkKey>() as u64);
        SceneMemoryStats {
            published_chunks: guard.chunks.len() as u64,
            journal_generations: guard.journal.len() as u64,
            journal_keys: guard.journal_keys as u64,
            estimated_published_bytes,
            max_chunk_strong_count,
            generation: guard.generation,
            ..working
        }
    }

    fn snapshot_keys(&self, keys: &[ChunkKey], side: i64) -> PublishedSnapshot<Cell> {
        let mut chunks = Vec::with_capacity(keys.len());
        let (generation, published_at) = {
            let guard = self.read();
            for key in keys {
                guard.push_present(*key, &mut chunks);
            }
            (guard.generation, guard.published_at)
        };
        PublishedSnapshot::new(generation, published_at, chunks, side, None)
    }

    fn snapshot_region(&self, bounds: VoxelBounds, side: i64) -> PublishedSnapshot<Cell> {
        let (min, _) = chunk_and_local(bounds.min, side);
        let (max, _) = chunk_and_local(bounds.max, side);
        let span = |lo: i64, hi: i64| u128::from(hi.abs_diff(lo)) + 1;
        let region_chunks = span(min.x, max.x)
            .checked_mul(span(min.y, max.y))
            .and_then(|area| area.checked_mul(span(min.z, max.z)));
        let mut chunks = Vec::new();
        let (generation, published_at) = {
            let guard = self.read();
            if region_chunks.is_some_and(|count| count <= guard.chunks.len() as u128) {
                for z in min.z..=max.z {
                    for y in min.y..=max.y {
                        for x in min.x..=max.x {
                            guard.push_present(ChunkKey { x, y, z }, &mut chunks);
                        }
                    }
                }
            } else {
                for (key, chunk) in &guard.chunks {
                    if key.x >= min.x
                        && key.x <= max.x
                        && key.y >= min.y
                        && key.y <= max.y
                        && key.z >= min.z
                        && key.z <= max.z
                    {
                        chunks.push((*key, Arc::clone(chunk)));
                    }
                }
            }
            (guard.generation, guard.published_at)
        };
        PublishedSnapshot::new(generation, published_at, chunks, side, Some(bounds))
    }

    fn snapshot_all(&self, side: i64) -> PublishedSnapshot<Cell> {
        let (generation, published_at, chunks) = {
            let guard = self.read();
            let mut chunks = Vec::with_capacity(guard.chunks.len());
            for (key, chunk) in &guard.chunks {
                chunks.push((*key, Arc::clone(chunk)));
            }
            (guard.generation, guard.published_at, chunks)
        };
        PublishedSnapshot::new(generation, published_at, chunks, side, None)
    }

    fn snapshot_changes_since(
        &self,
        cursor: u64,
        side: i64,
    ) -> Result<PublishedSnapshot<Cell>, ChangesError> {
        let mut keys = Vec::new();
        let mut chunks = Vec::new();
        let (generation, published_at) = {
            let guard = self.read();
            guard.check_cursor(cursor)?;
            guard.keys_since(cursor, &mut keys);
            chunks.reserve(keys.len());
            for key in &keys {
                guard.push_present(*key, &mut chunks);
            }
            (guard.generation, guard.published_at)
        };
        Ok(PublishedSnapshot::new(
            generation,
            published_at,
            chunks,
            side,
            None,
        ))
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Changes {
    pub generation: u64,
    pub chunk_keys: Vec<ChunkKey>,
}

#[derive(Clone)]
pub struct SceneReader<Cell> {
    index: Arc<PublishedIndex<Cell>>,
    geometry: VoxelGeometry,
    side: i64,
}

impl<Cell> SceneReader<Cell> {
    pub(crate) fn new(
        index: Arc<PublishedIndex<Cell>>,
        geometry: VoxelGeometry,
        side: i64,
    ) -> Self {
        Self {
            index,
            geometry,
            side,
        }
    }

    pub fn generation(&self) -> u64 {
        self.index.generation()
    }

    pub fn published_at_unix_ms(&self) -> Option<u64> {
        self.index.published_unix_ms()
    }

    pub fn publication_age(&self) -> Option<Duration> {
        self.index.published_at().map(|at| at.elapsed())
    }

    pub fn geometry(&self) -> VoxelGeometry {
        self.geometry
    }
}

impl<Cell: Copy> SceneReader<Cell> {
    pub fn get_voxel_key(&self, key: VoxelKey) -> Option<Cell> {
        let (chunk_key, local) = chunk_and_local(key, self.side);
        let guard = self.index.read();
        guard.chunks.get(&chunk_key)?.get(local)
    }

    pub fn get_voxel_world(&self, world: [f64; 3]) -> Result<Option<Cell>, LiveError> {
        let key = self.geometry.voxel_key(world)?;
        Ok(self.get_voxel_key(key))
    }

    pub fn snapshot_chunks(&self, keys: &[ChunkKey]) -> PublishedSnapshot<Cell> {
        let mut wanted = keys.to_vec();
        wanted.sort_unstable();
        wanted.dedup();
        self.index.snapshot_keys(&wanted, self.side)
    }

    pub fn snapshot_region(
        &self,
        bounds: VoxelBounds,
    ) -> Result<PublishedSnapshot<Cell>, LiveError> {
        let bounds = VoxelBounds::try_new(bounds.min, bounds.max)?;
        Ok(self.index.snapshot_region(bounds, self.side))
    }

    pub fn snapshot_all(&self) -> PublishedSnapshot<Cell> {
        self.index.snapshot_all(self.side)
    }

    pub fn changes_since(&self, generation: u64) -> Result<Changes, ChangesError> {
        self.index.changes_since(generation)
    }

    pub fn snapshot_changes_since(
        &self,
        generation: u64,
    ) -> Result<PublishedSnapshot<Cell>, ChangesError> {
        self.index.snapshot_changes_since(generation, self.side)
    }
}

pub struct PublishedSnapshot<Cell> {
    generation: u64,
    published_at: Option<Instant>,
    chunks: Vec<ChunkHandle<Cell>>,
    side: i64,
    bounds: Option<VoxelBounds>,
}

impl<Cell> PublishedSnapshot<Cell> {
    fn new(
        generation: u64,
        published_at: Option<Instant>,
        mut chunks: Vec<ChunkHandle<Cell>>,
        side: i64,
        bounds: Option<VoxelBounds>,
    ) -> Self {
        chunks.sort_unstable_by_key(|(key, _)| *key);
        chunks.dedup_by_key(|(key, _)| *key);
        Self {
            generation,
            published_at,
            chunks,
            side,
            bounds,
        }
    }
}

impl<Cell: Copy> PublishedSnapshot<Cell> {
    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn published_at(&self) -> Option<Instant> {
        self.published_at
    }

    pub fn age(&self) -> Option<Duration> {
        self.published_at.map(|at| at.elapsed())
    }

    pub fn bounds(&self) -> Option<VoxelBounds> {
        self.bounds
    }

    pub fn chunks(&self) -> &[ChunkHandle<Cell>] {
        &self.chunks
    }

    pub fn chunk_keys(&self) -> impl Iterator<Item = ChunkKey> + '_ {
        self.chunks.iter().map(|(key, _)| *key)
    }

    pub fn get(&self, key: VoxelKey) -> Option<Cell> {
        if self.bounds.is_some_and(|bounds| !bounds.contains(key)) {
            return None;
        }
        let (chunk_key, local) = chunk_and_local(key, self.side);
        let index = self
            .chunks
            .binary_search_by_key(&chunk_key, |(key, _)| *key)
            .ok()?;
        self.chunks[index].1.get(local)
    }

    pub fn iter(&self) -> impl Iterator<Item = (VoxelKey, Cell)> + '_ {
        let bounds = self.bounds;
        self.chunks
            .iter()
            .flat_map(|(chunk_key, chunk)| chunk.cells(*chunk_key))
            .filter(move |(key, _)| bounds.is_none_or(|bounds| bounds.contains(*key)))
    }
}

pub(crate) fn unix_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_millis() as u64)
        .unwrap_or(0)
}

pub(crate) fn publish_dirty<Cell, F>(
    dirty: &mut super::store::DirtySet,
    index: &PublishedIndex<Cell>,
    mut snapshot: F,
) -> Result<PublishStats, LiveError>
where
    F: FnMut(ChunkKey) -> Result<PublishedChunk<Cell>, LiveError>,
{
    if dirty.is_empty() {
        return Ok(PublishStats {
            generation: index.generation(),
            chunks_published: 0,
            duration_ns: 0,
            published_unix_ms: index.published_unix_ms().unwrap_or(0),
        });
    }
    let started = Instant::now();
    let mut keys: Vec<ChunkKey> = dirty.keys().collect();
    keys.sort_unstable();
    let mut prepared = Vec::with_capacity(keys.len());
    for key in keys {
        prepared.push((key, Arc::new(snapshot(key)?)));
    }
    let stats = index.commit(prepared, started)?;
    dirty.clear();
    Ok(stats)
}
