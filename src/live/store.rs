use std::collections::HashMap;

use super::chunk::WorkingChunk;
use super::geometry::{ChunkKey, VoxelKey, chunk_and_local};

#[derive(Clone, Copy)]
struct Cache {
    key: ChunkKey,
    slot: u32,
    origin_x: i64,
    origin_y: i64,
    origin_z: i64,
}

impl Cache {
    fn from_key(key: ChunkKey, slot: u32, side: i64) -> Self {
        Self {
            key,
            slot,
            origin_x: key.x * side,
            origin_y: key.y * side,
            origin_z: key.z * side,
        }
    }
}

pub(crate) struct DirtySet {
    keys: std::collections::HashSet<ChunkKey>,
    cache: Option<ChunkKey>,
}

impl DirtySet {
    pub(crate) fn new() -> Self {
        Self {
            keys: std::collections::HashSet::new(),
            cache: None,
        }
    }

    pub(crate) fn mark(&mut self, key: ChunkKey) -> bool {
        if self.cache == Some(key) {
            return false;
        }
        let inserted = self.keys.insert(key);
        self.cache = Some(key);
        inserted
    }

    pub(crate) fn clear(&mut self) {
        self.keys.clear();
        self.cache = None;
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.keys.is_empty()
    }

    pub(crate) fn len(&self) -> usize {
        self.keys.len()
    }

    pub(crate) fn keys(&self) -> impl Iterator<Item = ChunkKey> + '_ {
        self.keys.iter().copied()
    }
}

pub(crate) enum CellTouch {
    Updated { chunk_key: ChunkKey, newly: bool },
    OverCapacity,
}

pub(crate) struct ChunkStore<Cell, const SIDE: usize> {
    slots: Vec<WorkingChunk<Cell, SIDE>>,
    index: HashMap<ChunkKey, u32>,
    cache: Option<Cache>,
    observed_cells: u64,
    threshold: usize,
}

impl<Cell, const SIDE: usize> ChunkStore<Cell, SIDE>
where
    Cell: Copy + Default,
{
    pub(crate) fn new(threshold: usize) -> Self {
        Self {
            slots: Vec::new(),
            index: HashMap::new(),
            cache: None,
            observed_cells: 0,
            threshold,
        }
    }

    pub(crate) fn chunk_count(&self) -> u64 {
        self.slots.len() as u64
    }

    pub(crate) fn observed_cells(&self) -> u64 {
        self.observed_cells
    }

    pub(crate) fn contains_key(&self, key: ChunkKey) -> bool {
        if self.cache.is_some_and(|cache| cache.key == key) {
            return true;
        }
        self.index.contains_key(&key)
    }

    pub(crate) fn with_voxel(
        &mut self,
        voxel: VoxelKey,
        max_chunks: Option<u64>,
        init: Cell,
        f: impl FnOnce(&mut Cell),
    ) -> CellTouch {
        let (chunk_key, local, slot) = match self.slot_for(voxel, max_chunks) {
            Some(ready) => ready,
            None => return CellTouch::OverCapacity,
        };
        let mut newly = false;
        self.slots[slot as usize].with_local(local, init, |cell, is_new| {
            newly = is_new;
            f(cell);
        });
        if newly {
            self.observed_cells = self.observed_cells.saturating_add(1);
        }
        CellTouch::Updated { chunk_key, newly }
    }

    #[cfg(test)]
    pub(crate) fn working_get(&self, voxel: VoxelKey) -> Option<Cell> {
        let side = SIDE as i64;
        if let Some(cache) = self.cache
            && in_cache(voxel, cache, side)
        {
            let local = local_in_cache(voxel, cache, side);
            return self.slots[cache.slot as usize].get(local);
        }
        let (chunk_key, local) = chunk_and_local(voxel, side);
        let slot = *self.index.get(&chunk_key)?;
        self.slots[slot as usize].get(local)
    }

    #[cfg(test)]
    pub(crate) fn chunk_is_dense(&self, key: ChunkKey) -> Option<bool> {
        let slot = *self.index.get(&key)?;
        Some(self.slots[slot as usize].is_dense())
    }

    pub(crate) fn to_published(&self, key: ChunkKey) -> Option<super::chunk::PublishedChunk<Cell>> {
        let slot = if self.cache.is_some_and(|cache| cache.key == key) {
            self.cache.unwrap().slot
        } else {
            *self.index.get(&key)?
        };
        Some(self.slots[slot as usize].to_published(SIDE as u16))
    }

    pub(crate) fn estimated_bytes(&self) -> u64 {
        let payloads = self.slots.iter().fold(0u64, |sum, chunk| {
            sum.saturating_add(chunk.estimated_bytes())
        });
        let index_bytes = self.index.len() as u64 * (size_of::<ChunkKey>() as u64 + 16);
        payloads.saturating_add(index_bytes)
    }

    fn slot_for(
        &mut self,
        voxel: VoxelKey,
        max_chunks: Option<u64>,
    ) -> Option<(ChunkKey, u16, u32)> {
        let side = SIDE as i64;
        if let Some(cache) = self.cache
            && in_cache(voxel, cache, side)
        {
            return Some((cache.key, local_in_cache(voxel, cache, side), cache.slot));
        }
        let (chunk_key, local) = chunk_and_local(voxel, side);
        if let Some(&slot) = self.index.get(&chunk_key) {
            self.cache = Some(Cache::from_key(chunk_key, slot, side));
            return Some((chunk_key, local, slot));
        }
        if let Some(max_chunks) = max_chunks
            && self.slots.len() as u64 >= max_chunks
        {
            return None;
        }
        if self.slots.len() >= u32::MAX as usize {
            return None;
        }
        let slot = self.slots.len() as u32;
        self.slots.push(WorkingChunk::new(self.threshold));
        self.index.insert(chunk_key, slot);
        self.cache = Some(Cache::from_key(chunk_key, slot, side));
        Some((chunk_key, local, slot))
    }
}

fn in_cache(voxel: VoxelKey, cache: Cache, side: i64) -> bool {
    voxel.x >= cache.origin_x
        && voxel.x < cache.origin_x + side
        && voxel.y >= cache.origin_y
        && voxel.y < cache.origin_y + side
        && voxel.z >= cache.origin_z
        && voxel.z < cache.origin_z + side
}

fn local_in_cache(voxel: VoxelKey, cache: Cache, side: i64) -> u16 {
    let local_x = (voxel.x - cache.origin_x) as u16;
    let local_y = (voxel.y - cache.origin_y) as u16;
    let local_z = (voxel.z - cache.origin_z) as u16;
    let width = side as u16;
    local_x + width * local_y + width * width * local_z
}
