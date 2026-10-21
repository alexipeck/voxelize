#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InsertStats {
    pub accepted: u64,
    pub filtered_by_range: u64,
    pub invalid: u64,
    pub distance_overflow: u64,
    pub capacity_rejected: u64,
    pub hits_applied: u64,
    pub free_cell_updates: u64,
    pub newly_observed_cells: u64,
    pub dirty_chunks: u64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BatchStats {
    pub input: u64,
    pub accepted: u64,
    pub filtered_by_range: u64,
    pub invalid: u64,
    pub distance_overflow: u64,
    pub capacity_rejected: u64,
    pub hits_applied: u64,
    pub free_cell_updates: u64,
    pub newly_observed_cells: u64,
    pub dirty_chunks: u64,
}

impl BatchStats {
    pub(crate) fn as_insert(&self) -> InsertStats {
        InsertStats {
            accepted: self.accepted,
            filtered_by_range: self.filtered_by_range,
            invalid: self.invalid,
            distance_overflow: self.distance_overflow,
            capacity_rejected: self.capacity_rejected,
            hits_applied: self.hits_applied,
            free_cell_updates: self.free_cell_updates,
            newly_observed_cells: self.newly_observed_cells,
            dirty_chunks: self.dirty_chunks,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PublishStats {
    pub generation: u64,
    pub chunks_published: u64,
    pub duration_ns: u64,
    pub published_unix_ms: u64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SceneMemoryStats {
    pub working_chunks: u64,
    pub observed_cells: u64,
    pub dirty_chunks: u64,
    pub estimated_working_bytes: u64,
    pub published_chunks: u64,
    pub journal_generations: u64,
    pub journal_keys: u64,
    pub estimated_published_bytes: u64,
    pub max_chunk_strong_count: u64,
    pub generation: u64,
}
