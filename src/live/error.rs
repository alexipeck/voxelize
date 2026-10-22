use thiserror::Error;

use super::stats::BatchStats;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum LiveError {
    #[error("voxel size must be finite and greater than 0")]
    InvalidVoxelSize,
    #[error("grid origin must be finite")]
    InvalidOrigin,
    #[error("range filter requires 0 <= min <= max and finite bounds")]
    InvalidRangeFilter,
    #[error("journal limits must retain at least one generation and one key")]
    InvalidJournalLimits,
    #[error("promotion threshold must be greater than 0")]
    InvalidPromotionThreshold,
    #[error("log-odds limits must be finite with min <= max")]
    InvalidModel,
    #[error(
        "blocked rule needs a finite minimum hit count of at least 0 and a hit share between 0 and 1"
    )]
    InvalidBlockedRule,
    #[error("vote decay needs 0 < keep < 1 or a cap of at least one vote")]
    InvalidVoteDecay,
    #[error("coordinate is not a finite representable voxel index")]
    InvalidCoordinate,
    #[error("sensor-to-hit distance is not finite")]
    DistanceOverflow,
    #[error(
        "a range filter is configured, so points need a sensor origin; use insert_points_from_source or insert_point_observations"
    )]
    RangeFilterRequiresSource,
    #[error("voxel bounds are inverted")]
    InvalidBounds,
    #[error("publication generation overflowed")]
    GenerationOverflow,
    #[error("dirty chunk disappeared before publication")]
    MissingDirtyChunk,
    #[error("ray traversal broke an internal invariant; the ray may be partly applied ({stats:?})")]
    TraversalFailed { stats: BatchStats },
    #[error("live scene update was only partially applied ({stats:?})")]
    PartialUpdate { stats: BatchStats },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChangeHistoryLost {
    pub requested_generation: u64,
    pub oldest_available_generation: u64,
    pub current_generation: u64,
}

impl std::fmt::Display for ChangeHistoryLost {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "change history no longer contains generation {}; oldest retained cursor is {}; resync required",
            self.requested_generation, self.oldest_available_generation
        )
    }
}

impl std::error::Error for ChangeHistoryLost {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ChangesError {
    HistoryLost(ChangeHistoryLost),
    CursorAhead {
        requested_generation: u64,
        current_generation: u64,
    },
}

impl std::fmt::Display for ChangesError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::HistoryLost(lost) => lost.fmt(f),
            Self::CursorAhead {
                requested_generation,
                current_generation,
            } => write!(
                f,
                "cursor generation {requested_generation} is ahead of published generation {current_generation}; it did not come from this scene"
            ),
        }
    }
}

impl std::error::Error for ChangesError {}
