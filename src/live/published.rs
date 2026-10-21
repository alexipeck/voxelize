use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::{Arc, RwLock};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use super::chunk::PublishedChunk;
use super::config::JournalConfig;
use super::error::{ChangeHistoryLost, LiveError};
use super::geometry::{ChunkKey, VoxelBounds, VoxelGeometry, VoxelKey, chunk_and_local};
use super::stats::{PublishStats, SceneMemoryStats};

struct JournalRecord {
    generation: u64,
    keys: Vec<ChunkKey>,
}

struct PublishedState<Cell> {
    chunks: HashMap<ChunkKey, Arc<PublishedChunk<Cell>>>,
    generation: u64,
    published_unix_ms: Option<u64>,
    journal: VecDeque<JournalRecord>,
    journal_keys: usize,
    max_generations: usize,
    max_keys: usize,
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

    pub(crate) fn commit(
        &self,
        prepared: Vec<(ChunkKey, PublishedChunk<Cell>)>,
        started: std::time::Instant,
    ) -> Result<PublishStats, LiveError> {
        let chunks_published = prepared.len() as u64;
        let mut guard = self.write();
        let generation = guard
            .generation
            .checked_add(1)
            .ok_or(LiveError::GenerationOverflow)?;
        let mut keys = Vec::with_capacity(prepared.len());
        for (key, chunk) in prepared {
            keys.push(key);
            guard.chunks.insert(key, Arc::new(chunk));
        }
        guard.journal_keys = guard.journal_keys.saturating_add(keys.len());
        guard.journal.push_back(JournalRecord { generation, keys });
        while guard.journal.len() > guard.max_generations
            || (guard.journal_keys > guard.max_keys && guard.journal.len() > 1)
        {
            if let Some(old) = guard.journal.pop_front() {
                guard.journal_keys = guard.journal_keys.saturating_sub(old.keys.len());
            } else {
                break;
            }
        }
        guard.generation = generation;
        let published_unix_ms = unix_ms();
        guard.published_unix_ms = Some(published_unix_ms);
        let duration_ns = u64::try_from(started.elapsed().as_nanos()).unwrap_or(u64::MAX);
        Ok(PublishStats {
            generation,
            chunks_published,
            duration_ns,
            published_unix_ms,
        })
    }

    pub(crate) fn changes_since(&self, cursor: u64) -> Result<Changes, ChangeHistoryLost> {
        let guard = self.read();
        let current = guard.generation;
        let oldest = match guard.journal.front() {
            Some(entry) => entry.generation.saturating_sub(1),
            None => current,
        };
        if cursor > current || cursor < oldest {
            return Err(ChangeHistoryLost {
                requested_generation: cursor,
                oldest_available_generation: oldest,
                current_generation: current,
            });
        }
        let mut seen = HashSet::new();
        for entry in &guard.journal {
            if entry.generation > cursor {
                seen.extend(entry.keys.iter().copied());
            }
        }
        let mut chunk_keys: Vec<ChunkKey> = seen.into_iter().collect();
        chunk_keys.sort_unstable();
        Ok(Changes {
            generation: current,
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

    fn collect(
        &self,
        mut select: impl FnMut(&ChunkKey) -> bool,
        side: i64,
    ) -> PublishedSnapshot<Cell> {
        let guard = self.read();
        let mut chunks = Vec::new();
        for (key, chunk) in &guard.chunks {
            if select(key) {
                chunks.push((*key, Arc::clone(chunk)));
            }
        }
        chunks.sort_unstable_by_key(|(key, _)| *key);
        chunks.dedup_by_key(|(key, _)| *key);
        PublishedSnapshot {
            generation: guard.generation,
            chunks,
            side,
        }
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
        let published = self.published_at_unix_ms()?;
        Some(Duration::from_millis(unix_ms().saturating_sub(published)))
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
        let wanted: HashSet<ChunkKey> = keys.iter().copied().collect();
        self.index.collect(|key| wanted.contains(key), self.side)
    }

    pub fn snapshot_region(
        &self,
        bounds: VoxelBounds,
    ) -> Result<PublishedSnapshot<Cell>, LiveError> {
        VoxelBounds::try_new(bounds.min, bounds.max)?;
        let (min_chunk, _) = chunk_and_local(bounds.min, self.side);
        let (max_chunk, _) = chunk_and_local(bounds.max, self.side);
        Ok(self.index.collect(
            |key| {
                key.x >= min_chunk.x
                    && key.x <= max_chunk.x
                    && key.y >= min_chunk.y
                    && key.y <= max_chunk.y
                    && key.z >= min_chunk.z
                    && key.z <= max_chunk.z
            },
            self.side,
        ))
    }

    pub fn snapshot_all(&self) -> PublishedSnapshot<Cell> {
        self.index.collect(|_| true, self.side)
    }

    pub fn changes_since(&self, generation: u64) -> Result<Changes, ChangeHistoryLost> {
        self.index.changes_since(generation)
    }
}

pub struct PublishedSnapshot<Cell> {
    generation: u64,
    chunks: Vec<(ChunkKey, Arc<PublishedChunk<Cell>>)>,
    side: i64,
}

impl<Cell: Copy> PublishedSnapshot<Cell> {
    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn chunks(&self) -> &[(ChunkKey, Arc<PublishedChunk<Cell>>)] {
        &self.chunks
    }

    pub fn get(&self, key: VoxelKey) -> Option<Cell> {
        let (chunk_key, local) = chunk_and_local(key, self.side);
        let index = self
            .chunks
            .binary_search_by_key(&chunk_key, |(key, _)| *key)
            .ok()?;
        self.chunks[index].1.get(local)
    }

    pub fn iter(&self) -> impl Iterator<Item = (VoxelKey, Cell)> + '_ {
        self.chunks.iter().flat_map(|(chunk_key, chunk)| {
            let mut cells = Vec::new();
            chunk.for_each_cell(*chunk_key, |key, cell| cells.push((key, cell)));
            cells
        })
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
    let started = std::time::Instant::now();
    let mut keys: Vec<ChunkKey> = dirty.keys().collect();
    keys.sort_unstable();
    let mut prepared = Vec::with_capacity(keys.len());
    for key in keys {
        prepared.push((key, snapshot(key)?));
    }
    let stats = index.commit(prepared, started)?;
    dirty.clear();
    Ok(stats)
}
