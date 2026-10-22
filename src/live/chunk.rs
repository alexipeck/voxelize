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

    pub(crate) fn with_local<R>(
        &mut self,
        local: u16,
        init: Cell,
        f: impl FnOnce(&mut Cell, bool) -> R,
    ) -> R {
        match &mut self.body {
            Body::Dense { values, observed } => {
                let index = local as usize;
                let newly = !bits::test(observed, index);
                if newly {
                    bits::set(observed, index);
                    values[index] = init;
                    self.observed = self.observed.saturating_add(1);
                }
                return f(&mut values[index], newly);
            }
            Body::Sparse(entries) => match entries.binary_search_by_key(&local, |entry| entry.0) {
                Ok(index) => return f(&mut entries[index].1, false),
                Err(index) => {
                    if entries.len() + 1 < self.threshold {
                        entries.insert(index, (local, init));
                        self.observed = self.observed.saturating_add(1);
                        return f(&mut entries[index].1, true);
                    }
                }
            },
        }
        self.promote();
        self.with_local(local, init, f)
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

    #[cfg(test)]
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

    pub(crate) fn for_each_observed_mut(&mut self, mut f: impl FnMut(&mut Cell)) {
        match &mut self.body {
            Body::Sparse(entries) => {
                for (_, cell) in entries.iter_mut() {
                    f(cell);
                }
            }
            Body::Dense { values, observed } => {
                for (word_index, word) in observed.iter().enumerate() {
                    let mut remaining = *word;
                    while remaining != 0 {
                        let index = word_index * 64 + remaining.trailing_zeros() as usize;
                        remaining &= remaining - 1;
                        f(&mut values[index]);
                    }
                }
            }
        }
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

    fn promote(&mut self) {
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
        for (key, cell) in self.cells(chunk) {
            visit(key, cell);
        }
    }

    pub fn cells(&self, chunk: ChunkKey) -> ChunkCells<'_, Cell> {
        let inner = match &self.body {
            PublishedBody::Sparse(entries) => CellsInner::Sparse(entries.iter()),
            PublishedBody::Dense { values, observed } => CellsInner::Dense {
                values,
                observed,
                next_word: 0,
                base: 0,
                word: 0,
            },
        };
        ChunkCells {
            chunk,
            side: i64::from(self.side),
            inner,
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

pub struct ChunkCells<'a, Cell> {
    chunk: ChunkKey,
    side: i64,
    inner: CellsInner<'a, Cell>,
}

enum CellsInner<'a, Cell> {
    Sparse(std::slice::Iter<'a, (u16, Cell)>),
    Dense {
        values: &'a [Cell],
        observed: &'a [u64],
        next_word: usize,
        base: usize,
        word: u64,
    },
}

impl<Cell: Copy> Iterator for ChunkCells<'_, Cell> {
    type Item = (VoxelKey, Cell);

    fn next(&mut self) -> Option<Self::Item> {
        match &mut self.inner {
            CellsInner::Sparse(entries) => {
                let &(local, cell) = entries.next()?;
                Some((voxel_from_chunk(self.chunk, local, self.side), cell))
            }
            CellsInner::Dense {
                values,
                observed,
                next_word,
                base,
                word,
            } => loop {
                if *word != 0 {
                    let bit = word.trailing_zeros() as usize;
                    *word &= *word - 1;
                    let local = *base + bit;
                    if local < values.len() {
                        return Some((
                            voxel_from_chunk(self.chunk, local as u16, self.side),
                            values[local],
                        ));
                    }
                    continue;
                }
                let loaded = *observed.get(*next_word)?;
                *base = *next_word * 64;
                *next_word += 1;
                *word = loaded;
            },
        }
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
            chunk.with_local(local, 0, |cell, newly| {
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
        chunk.with_local(1, 0, |cell, newly| {
            assert!(!newly);
            *cell = cell.saturating_add(1);
        });
        assert_eq!(chunk.get(1), Some(3));
        let published = chunk.to_published(16);
        assert_eq!(published.get(1), Some(3));
        assert_eq!(published.get(2), None);
        let chunk_key = ChunkKey { x: -1, y: 2, z: 0 };
        let mut iterated: Vec<_> = published.cells(chunk_key).collect();
        iterated.sort_unstable_by_key(|(key, _)| *key);
        let mut expected = Vec::new();
        published.for_each_cell(chunk_key, |key, cell| expected.push((key, cell)));
        expected.sort_unstable_by_key(|(key, _)| *key);
        assert_eq!(iterated.len(), 4);
        assert_eq!(iterated, expected);
        assert!(iterated.contains(&(voxel_from_chunk(chunk_key, 400, 16), 1)));
    }

    #[test]
    fn visits_every_observed_cell_across_promotion() {
        let mut chunk = WorkingChunk::<u16, 8>::new(3);
        chunk.with_local(5, 0, |cell, _| *cell += 1);
        chunk.with_local(9, 0, |cell, _| *cell += 2);
        let mut sparse = Vec::new();
        chunk.for_each_observed_mut(|cell| sparse.push(*cell));
        sparse.sort_unstable();
        assert_eq!(sparse, vec![1, 2]);
        assert!(!chunk.is_dense());
        chunk.with_local(300, 0, |cell, _| *cell += 3);
        assert!(chunk.is_dense());
        let mut dense = Vec::new();
        chunk.for_each_observed_mut(|cell| {
            dense.push(*cell);
            *cell *= 10;
        });
        dense.sort_unstable();
        assert_eq!(dense, vec![1, 2, 3]);
        assert_eq!(chunk.get(300), Some(30));
        assert_eq!(chunk.get(6), None);
    }

    #[test]
    fn counts_saturate() {
        let mut chunk = WorkingChunk::<u16, 8>::new(2);
        chunk.with_local(0, 0, |cell, _| *cell = u16::MAX);
        chunk.with_local(0, 0, |cell, newly| {
            assert!(!newly);
            *cell = cell.saturating_add(1);
        });
        assert_eq!(chunk.get(0), Some(u16::MAX));
    }

    #[test]
    fn dense_zero_stays_observed() {
        let mut chunk = WorkingChunk::<f32, 8>::new(1);
        chunk.with_local(3, 0.0, |cell, newly| {
            assert!(newly);
            assert_eq!(*cell, 0.0);
        });
        assert!(chunk.is_dense());
        assert_eq!(chunk.get(3), Some(0.0));
        assert_eq!(chunk.get(4), None);
    }

    #[test]
    fn new_cells_take_the_supplied_initial_value() {
        let mut sparse = WorkingChunk::<f32, 8>::new(64);
        sparse.with_local(5, -1.5, |cell, newly| {
            assert!(newly);
            assert_eq!(*cell, -1.5);
            *cell += 1.0;
        });
        sparse.with_local(5, -1.5, |cell, newly| {
            assert!(!newly);
            assert_eq!(*cell, -0.5);
        });
        let mut dense = WorkingChunk::<f32, 8>::new(2);
        dense.with_local(1, 7.0, |_, _| {});
        dense.with_local(2, 7.0, |cell, newly| {
            assert!(newly);
            assert_eq!(*cell, 7.0);
        });
        assert!(dense.is_dense());
        assert_eq!(dense.get(1), Some(7.0));
        assert_eq!(dense.get(2), Some(7.0));
        assert_eq!(dense.get(3), None);
    }
}
