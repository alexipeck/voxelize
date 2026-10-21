#[inline]
pub(crate) fn test(bits: &[u64], index: usize) -> bool {
    (bits[index / 64] & (1u64 << (index % 64))) != 0
}

#[inline]
pub(crate) fn set(bits: &mut [u64], index: usize) {
    bits[index / 64] |= 1u64 << (index % 64);
}
