use super::error::LiveError;

#[derive(Clone, Copy, Debug)]
pub struct RangeFilter {
    min_distance_m: f64,
    max_distance_m: Option<f64>,
    min_distance_sq: f64,
    max_distance_sq: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RangeClass {
    Accept,
    Filtered,
    Invalid,
    Overflow,
}

impl RangeFilter {
    pub fn new(min_distance_m: f64, max_distance_m: f64) -> Result<Self, LiveError> {
        Self::bounded(min_distance_m, Some(max_distance_m))
    }

    pub fn unbounded_max(min_distance_m: f64) -> Result<Self, LiveError> {
        Self::bounded(min_distance_m, None)
    }

    fn bounded(min_distance_m: f64, max_distance_m: Option<f64>) -> Result<Self, LiveError> {
        if !min_distance_m.is_finite() || min_distance_m < 0.0 {
            return Err(LiveError::InvalidRangeFilter);
        }
        let max_distance_sq = if let Some(max_distance_m) = max_distance_m {
            if !max_distance_m.is_finite() || max_distance_m < min_distance_m {
                return Err(LiveError::InvalidRangeFilter);
            }
            max_distance_m * max_distance_m
        } else {
            f64::INFINITY
        };
        Ok(Self {
            min_distance_m,
            max_distance_m,
            min_distance_sq: min_distance_m * min_distance_m,
            max_distance_sq,
        })
    }

    pub fn min_distance_m(&self) -> f64 {
        self.min_distance_m
    }

    pub fn max_distance_m(&self) -> Option<f64> {
        self.max_distance_m
    }

    #[inline]
    pub fn classify(&self, source: [f64; 3], hit: [f64; 3]) -> RangeClass {
        if !source.iter().copied().all(f64::is_finite) || !hit.iter().copied().all(f64::is_finite) {
            return RangeClass::Invalid;
        }
        let dx = hit[0] - source[0];
        let dy = hit[1] - source[1];
        let dz = hit[2] - source[2];
        let range_sq = dx * dx + dy * dy + dz * dz;
        if !range_sq.is_finite() {
            return RangeClass::Overflow;
        }
        if range_sq >= self.min_distance_sq && range_sq <= self.max_distance_sq {
            RangeClass::Accept
        } else {
            RangeClass::Filtered
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JournalConfig {
    pub max_generations: usize,
    pub max_keys: usize,
}

impl Default for JournalConfig {
    fn default() -> Self {
        Self {
            max_generations: 64,
            max_keys: 65_536,
        }
    }
}

impl JournalConfig {
    pub fn validate(&self) -> Result<(), LiveError> {
        if self.max_generations == 0 || self.max_keys == 0 {
            Err(LiveError::InvalidJournalLimits)
        } else {
            Ok(())
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PointSceneConfig {
    pub range: Option<RangeFilter>,
    pub max_chunks: Option<u64>,
    pub promotion_threshold: usize,
    pub journal: JournalConfig,
}

impl Default for PointSceneConfig {
    fn default() -> Self {
        Self {
            range: None,
            max_chunks: None,
            promotion_threshold: 256,
            journal: JournalConfig::default(),
        }
    }
}

impl PointSceneConfig {
    pub fn validate(&self) -> Result<(), LiveError> {
        if self.promotion_threshold == 0 {
            return Err(LiveError::InvalidPromotionThreshold);
        }
        self.journal.validate()
    }
}

pub const DEFAULT_MAX_CELLS_PER_RAY: u64 = 1_048_576;
pub const ABSOLUTE_MAX_CELLS_PER_RAY: u64 = 16_777_216;

#[derive(Clone, Copy, Debug)]
pub struct RaySceneConfig {
    pub range: Option<RangeFilter>,
    pub max_chunks: Option<u64>,
    pub max_cells_per_ray: Option<u64>,
    pub promotion_threshold: usize,
    pub journal: JournalConfig,
}

impl Default for RaySceneConfig {
    fn default() -> Self {
        Self {
            range: None,
            max_chunks: None,
            max_cells_per_ray: Some(DEFAULT_MAX_CELLS_PER_RAY),
            promotion_threshold: 256,
            journal: JournalConfig::default(),
        }
    }
}

impl RaySceneConfig {
    pub fn validate(&self) -> Result<(), LiveError> {
        if self.promotion_threshold == 0 {
            return Err(LiveError::InvalidPromotionThreshold);
        }
        self.journal.validate()
    }

    pub(crate) fn effective_cell_cap(&self) -> u64 {
        self.max_cells_per_ray
            .unwrap_or(ABSOLUTE_MAX_CELLS_PER_RAY)
            .min(ABSOLUTE_MAX_CELLS_PER_RAY)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn range_filter_uses_inclusive_squared_bounds() {
        let filter = RangeFilter::new(0.5, 5.0).unwrap();
        let source = [0.0, 0.0, 0.0];
        assert_eq!(filter.classify(source, [0.5, 0.0, 0.0]), RangeClass::Accept);
        assert_eq!(filter.classify(source, [3.0, 4.0, 0.0]), RangeClass::Accept);
        assert_eq!(
            filter.classify(source, [0.5 - 1.0e-9, 0.0, 0.0]),
            RangeClass::Filtered
        );
        assert_eq!(
            filter.classify(source, [5.1, 0.0, 0.0]),
            RangeClass::Filtered
        );
        assert_eq!(
            filter.classify(source, [f64::NAN, 0.0, 0.0]),
            RangeClass::Invalid
        );
        assert_eq!(
            filter.classify(source, [1.0e308, 1.0e308, 1.0e308]),
            RangeClass::Overflow
        );
        assert!(RangeFilter::new(-0.1, 1.0).is_err());
        assert!(RangeFilter::new(2.0, 1.0).is_err());
        assert!(RangeFilter::new(0.0, f64::INFINITY).is_err());
        let open = RangeFilter::unbounded_max(1.0).unwrap();
        assert_eq!(
            open.classify(source, [1000.0, 0.0, 0.0]),
            RangeClass::Accept
        );
        assert_eq!(open.classify(source, [0.2, 0.0, 0.0]), RangeClass::Filtered);
    }
}
