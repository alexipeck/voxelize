use std::sync::Arc;

use super::bits;
use super::geometry::{ChunkKey, VoxelKey, voxel_from_chunk};

pub(crate) enum Body<Cell> {
    Sparse(Vec<(u16, Cell)>),
    Dense {
        values: Box<[Cell]>,
        observed: Box<[u64]>,
    },
}

pub struct WorkingChunk<Cell, const SIDE: usize> {
    body: Body<Cell>,
    observed: u32,
    threshold: usize,
}

impl<Cell, const SIDE: usize> WorkingChunk<Cell, SIDE>
where
    Cell: Copy + Default,
{
    const VOLUME: usize = SIDE * SIDE * SIDE;
    const WORDS: usize = Self::VOLUME.div_ceil(64);

    pub(crate) fn new(threshold: usize) -> Self {
        const {
            assert!(SIDE >= 1);
            assert!(SIDE <= 40);
            assert!(SIDE * SIDE * SIDE <= 65_536);
        }
        let cap = threshold.clamp(1, 64);
        Self {
            body: Body::Sparse(Vec::with_capacity(cap)),
            observed: 0,
            threshold,
        }
    }

    pub(crate) fn with_local<R>(&mut self, local: u16, f: impl FnOnce(&mut Cell, bool) -> R) -> R {
        let newly = self.ensure(local);
        let cell = match &mut self.body {
            Body::Dense { values, .. } => &mut values[local as usize],
            Body::Sparse(entries) => {
                let index = entries
                    .binary_search_by_key(&local, |entry| entry.0)
                    .expect("observed local index");
                &mut entries[index].1
            }
        };
        f(cell, newly)
    }

    #[cfg(test)]
    pub(crate) fn get(&self, local: u16) -> Option<Cell> {
        match &self.body {
            Body::Sparse(entries) => entries
                .binary_search_by_key(&local, |entry| entry.0)
                .ok()
                .map(|index| entries[index].1),
            Body::Dense { values, observed } => {
                let index = local as usize;
                if index < values.len() && bits::test(observed, index) {
                    Some(values[index])
                } else {
                    None
                }
            }
        }
    }

    pub(crate) fn is_dense(&self) -> bool {
        matches!(self.body, Body::Dense { .. })
    }

    pub(crate) fn estimated_bytes(&self) -> u64 {
        let payload = match &self.body {
            Body::Sparse(entries) => entries.capacity() as u64 * size_of::<(u16, Cell)>() as u64,
            Body::Dense { values, observed } => {
                values.len() as u64 * size_of::<Cell>() as u64 + observed.len() as u64 * 8
            }
        };
        payload.saturating_add(64)
    }

    pub(crate) fn to_published(&self, side: u16) -> PublishedChunk<Cell> {
        let body = match &self.body {
            Body::Sparse(entries) => PublishedBody::Sparse(Arc::from(entries.as_slice())),
            Body::Dense { values, observed } => PublishedBody::Dense {
                values: Arc::from(values.as_ref()),
                observed: Arc::from(observed.as_ref()),
            },
        };
        PublishedChunk {
            side,
            observed: self.observed,
            body,
        }
    }

    fn ensure(&mut self, local: u16) -> bool {
        if self.is_dense() {
            return self.ensure_dense(local);
        }
        if self.sparse_contains(local) {
            return false;
        }
        if self.sparse_len() + 1 >= self.threshold {
            self.promote_with(local);
        } else {
            self.sparse_insert(local);
        }
        true
    }

    fn ensure_dense(&mut self, local: u16) -> bool {
        let Body::Dense { values, observed } = &mut self.body else {
            return false;
        };
        let index = local as usize;
        if bits::test(observed, index) {
            return false;
        }
        bits::set(observed, index);
        values[index] = Cell::default();
        self.observed = self.observed.saturating_add(1);
        true
    }

    fn sparse_contains(&self, local: u16) -> bool {
        let Body::Sparse(entries) = &self.body else {
            return false;
        };
        entries
            .binary_search_by_key(&local, |entry| entry.0)
            .is_ok()
    }

    fn sparse_len(&self) -> usize {
        let Body::Sparse(entries) = &self.body else {
            return 0;
        };
        entries.len()
    }

    fn sparse_insert(&mut self, local: u16) {
        let Body::Sparse(entries) = &mut self.body else {
            return;
        };
        let index = entries
            .binary_search_by_key(&local, |entry| entry.0)
            .unwrap_err();
        entries.insert(index, (local, Cell::default()));
        self.observed = self.observed.saturating_add(1);
    }

    fn promote_with(&mut self, local: u16) {
        let entries = match std::mem::replace(&mut self.body, Body::Sparse(Vec::new())) {
            Body::Sparse(entries) => entries,
            Body::Dense { values, observed } => {
                self.body = Body::Dense { values, observed };
                return;
            }
        };
        let mut values = vec![Cell::default(); Self::VOLUME].into_boxed_slice();
        let mut observed_bits = vec![0u64; Self::WORDS].into_boxed_slice();
        for (index, cell) in entries {
            values[index as usize] = cell;
            bits::set(&mut observed_bits, index as usize);
        }
        values[local as usize] = Cell::default();
        bits::set(&mut observed_bits, local as usize);
        self.observed = self.observed.saturating_add(1);
        self.body = Body::Dense {
            values,
            observed: observed_bits,
        };
    }
}

enum PublishedBody<Cell> {
    Sparse(Arc<[(u16, Cell)]>),
    Dense {
        values: Arc<[Cell]>,
        observed: Arc<[u64]>,
    },
}

pub struct PublishedChunk<Cell> {
    side: u16,
    observed: u32,
    body: PublishedBody<Cell>,
}

impl<Cell: Copy> PublishedChunk<Cell> {
    pub fn get(&self, local: u16) -> Option<Cell> {
        match &self.body {
            PublishedBody::Sparse(entries) => entries
                .binary_search_by_key(&local, |entry| entry.0)
                .ok()
                .map(|index| entries[index].1),
            PublishedBody::Dense { values, observed } => {
                let index = local as usize;
                if index < values.len() && bits::test(observed, index) {
                    Some(values[index])
                } else {
                    None
                }
            }
        }
    }

    pub fn observed_len(&self) -> u32 {
        self.observed
    }

    pub fn for_each_cell(&self, chunk: ChunkKey, mut visit: impl FnMut(VoxelKey, Cell)) {
        let side = i64::from(self.side);
        match &self.body {
            PublishedBody::Sparse(entries) => {
                for &(local, cell) in entries.iter() {
                    visit(voxel_from_chunk(chunk, local, side), cell);
                }
            }
            PublishedBody::Dense { values, observed } => {
                for (word_index, word) in observed.iter().enumerate() {
                    let mut word = *word;
                    while word != 0 {
                        let bit = word.trailing_zeros() as usize;
                        let local = word_index * 64 + bit;
                        if local < values.len() {
                            visit(voxel_from_chunk(chunk, local as u16, side), values[local]);
                        }
                        word &= word - 1;
                    }
                }
            }
        }
    }

    pub(crate) fn estimated_bytes(&self) -> u64 {
        let payload = match &self.body {
            PublishedBody::Sparse(entries) => {
                entries.len() as u64 * size_of::<(u16, Cell)>() as u64
            }
            PublishedBody::Dense { values, observed } => {
                values.len() as u64 * size_of::<Cell>() as u64 + observed.len() as u64 * 8
            }
        };
        payload.saturating_add(32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn promotion_preserves_presence_and_counts() {
        let mut chunk = WorkingChunk::<u16, 16>::new(3);
        let locals = [1u16, 400, 20, 21];
        for (step, local) in locals.iter().copied().enumerate() {
            chunk.with_local(local, |cell, newly| {
                assert!(newly);
                *cell = cell.saturating_add(1);
                if step == 0 {
                    *cell = cell.saturating_add(1);
                }
            });
        }
        assert!(chunk.is_dense());
        assert_eq!(chunk.get(1), Some(2));
        assert_eq!(chunk.get(400), Some(1));
        assert_eq!(chunk.get(20), Some(1));
        assert_eq!(chunk.get(21), Some(1));
        assert_eq!(chunk.get(2), None);
        chunk.with_local(1, |cell, newly| {
            assert!(!newly);
            *cell = cell.saturating_add(1);
        });
        assert_eq!(chunk.get(1), Some(3));
        let published = chunk.to_published(16);
        assert_eq!(published.get(1), Some(3));
        assert_eq!(published.get(2), None);
    }

    #[test]
    fn counts_saturate() {
        let mut chunk = WorkingChunk::<u16, 8>::new(2);
        chunk.with_local(0, |cell, _| *cell = u16::MAX);
        chunk.with_local(0, |cell, newly| {
            assert!(!newly);
            *cell = cell.saturating_add(1);
        });
        assert_eq!(chunk.get(0), Some(u16::MAX));
    }

    #[test]
    fn dense_zero_stays_observed() {
        let mut chunk = WorkingChunk::<f32, 8>::new(1);
        chunk.with_local(3, |cell, newly| {
            assert!(newly);
            assert_eq!(*cell, 0.0);
        });
        assert!(chunk.is_dense());
        assert_eq!(chunk.get(3), Some(0.0));
        assert_eq!(chunk.get(4), None);
    }
}
