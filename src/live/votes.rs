use super::dda::{RayCellContext, RayCellRole};
use super::error::LiveError;
use super::model::RayUpdateModel;

const SHARE_BITS: u32 = 16;
const KEEP_BITS: u32 = 16;
const HALVE_AT: u32 = 1 << 30;
const UPDATED: u32 = 1 << 31;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Votes {
    pub hits: u32,
    pub misses: u32,
}

impl Votes {
    pub const UNIT: u32 = 256;

    pub fn from_votes(hits: u32, misses: u32) -> Self {
        Self {
            hits: hits.saturating_mul(Self::UNIT),
            misses: misses.saturating_mul(Self::UNIT),
        }
    }

    pub fn hit_votes(&self) -> f64 {
        f64::from(self.hits) / f64::from(Self::UNIT)
    }

    pub fn miss_votes(&self) -> f64 {
        f64::from(self.misses) / f64::from(Self::UNIT)
    }

    pub fn total(&self) -> u64 {
        u64::from(self.hits) + u64::from(self.misses)
    }

    pub fn hit_share(&self) -> f64 {
        match self.total() {
            0 => 0.0,
            total => f64::from(self.hits) / total as f64,
        }
    }
}

pub(crate) fn was_updated(votes: &Votes) -> bool {
    votes.misses & UPDATED != 0
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum VoteDecay {
    #[default]
    None,
    Exponential {
        keep: f64,
    },
    CapHalving {
        cap_votes: u32,
    },
}

#[derive(Clone, Copy, Debug)]
pub struct VoteModel {
    decay: VoteDecay,
    keep_fixed: u64,
    cap: u64,
    updated_flag: u32,
}

impl Default for VoteModel {
    fn default() -> Self {
        Self {
            decay: VoteDecay::None,
            keep_fixed: 1 << KEEP_BITS,
            cap: u64::MAX,
            updated_flag: 0,
        }
    }
}

impl VoteModel {
    pub fn new(decay: VoteDecay) -> Result<Self, LiveError> {
        let model = match decay {
            VoteDecay::None => Self::default(),
            VoteDecay::Exponential { keep } => {
                if !(keep > 0.0 && keep < 1.0) {
                    return Err(LiveError::InvalidVoteDecay);
                }
                Self {
                    decay,
                    keep_fixed: ((keep * f64::from(1u32 << KEEP_BITS)).round() as u64)
                        .min((1 << KEEP_BITS) - 1),
                    cap: u64::MAX,
                    updated_flag: UPDATED,
                }
            }
            VoteDecay::CapHalving { cap_votes } => {
                if cap_votes == 0 {
                    return Err(LiveError::InvalidVoteDecay);
                }
                Self {
                    decay,
                    keep_fixed: 1 << KEEP_BITS,
                    cap: u64::from(cap_votes) * u64::from(Votes::UNIT),
                    updated_flag: UPDATED,
                }
            }
        };
        Ok(model)
    }

    pub fn decay(&self) -> VoteDecay {
        self.decay
    }
}

impl RayUpdateModel for VoteModel {
    type Cell = Votes;

    // While decay is on, the top bit of `misses` marks a cell updated since the last
    // publish. Counts stay below 2^30 plus one vote, so the bit is free, and `pass_end`
    // clears it before the chunk is copied, so readers never see it.
    fn update(&mut self, cell: &mut Votes, event: RayCellContext) {
        let mut misses = cell.misses & !UPDATED;
        if cell.hits >= HALVE_AT || misses >= HALVE_AT {
            cell.hits >>= 1;
            misses >>= 1;
        }
        match event.role {
            RayCellRole::Hit => cell.hits += Votes::UNIT,
            RayCellRole::Free => misses += Votes::UNIT,
        }
        cell.misses = misses | self.updated_flag;
    }

    fn needs_pass_end(&self) -> bool {
        self.decay != VoteDecay::None
    }

    fn pass_end(&mut self, cell: &mut Votes) {
        if cell.misses & UPDATED == 0 {
            return;
        }
        cell.misses &= !UPDATED;
        match self.decay {
            VoteDecay::None => {}
            VoteDecay::Exponential { .. } => {
                cell.hits = ((u64::from(cell.hits) * self.keep_fixed) >> KEEP_BITS) as u32;
                cell.misses = ((u64::from(cell.misses) * self.keep_fixed) >> KEEP_BITS) as u32;
            }
            VoteDecay::CapHalving { .. } => {
                if cell.total() > self.cap {
                    cell.hits >>= 1;
                    cell.misses >>= 1;
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BlockedRule {
    min_hits: f64,
    min_hit_share: f64,
    min_hit_units: u32,
    share_fixed: u64,
}

impl BlockedRule {
    pub fn new(min_hits: f64, min_hit_share: f64) -> Result<Self, LiveError> {
        if !(min_hits.is_finite() && min_hits >= 0.0) || !(0.0..=1.0).contains(&min_hit_share) {
            return Err(LiveError::InvalidBlockedRule);
        }
        let min_hit_units = (min_hits * f64::from(Votes::UNIT))
            .ceil()
            .clamp(1.0, f64::from(u32::MAX)) as u32;
        // Rounded down to 1/65536 so the integer test can only block more, never less.
        let share_fixed = (min_hit_share * f64::from(1u32 << SHARE_BITS)).floor() as u64;
        Ok(Self {
            min_hits,
            min_hit_share,
            min_hit_units,
            share_fixed,
        })
    }

    pub fn any_hit() -> Self {
        Self {
            min_hits: 0.0,
            min_hit_share: 0.0,
            min_hit_units: 1,
            share_fixed: 0,
        }
    }

    pub fn min_hits(&self) -> f64 {
        self.min_hits
    }

    pub fn min_hit_share(&self) -> f64 {
        self.min_hit_share
    }

    pub fn is_blocked(&self, votes: Votes) -> bool {
        votes.hits >= self.min_hit_units
            && u64::from(votes.hits) << SHARE_BITS >= self.share_fixed * votes.total()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::live::{RayObservation, RaySceneConfig, VoxelGeometry, VoxelKey, open_ray_scene};

    fn event(role: RayCellRole) -> RayCellContext {
        RayCellContext {
            key: VoxelKey { x: 0, y: 0, z: 0 },
            role,
            ray_length_m: None,
            t_enter: 0.0,
            t_exit: 1.0,
        }
    }

    fn random_rays(count: usize, seed: u64) -> Vec<RayObservation> {
        let mut state = seed;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f64 / (1u64 << 53) as f64
        };
        (0..count)
            .map(|_| RayObservation {
                source: [next() * 2.0, next() * 2.0, 1.0 + next()],
                hit: [next() * 20.0 - 10.0, next() * 20.0 - 10.0, next() * 4.0],
            })
            .collect()
    }

    fn map_after(
        decay: VoteDecay,
        passes: &[Vec<RayObservation>],
        batch: usize,
    ) -> Vec<(VoxelKey, Votes)> {
        let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], 0.25).unwrap();
        let model = VoteModel::new(decay).unwrap();
        let (mut writer, reader) =
            open_ray_scene(geometry, &RaySceneConfig::default(), model).unwrap();
        for pass in passes {
            for rays in pass.chunks(batch) {
                writer.insert_rays(rays).unwrap();
            }
            writer.publish().unwrap();
        }
        let mut cells: Vec<_> = reader.snapshot_all().iter().collect();
        cells.sort_unstable_by_key(|(key, _)| *key);
        cells
    }

    #[test]
    fn rule_thresholds_and_validation() {
        let thin = Votes::from_votes(1, 99);
        let wall = Votes::from_votes(30, 70);
        let free = Votes::from_votes(0, 50);
        assert_eq!(thin.hit_share(), 0.01);
        assert_eq!(thin.hit_votes(), 1.0);
        assert_eq!(wall.miss_votes(), 70.0);
        assert_eq!(Votes::default().hit_share(), 0.0);
        assert!(BlockedRule::any_hit().is_blocked(thin));
        assert!(BlockedRule::any_hit().is_blocked(Votes { hits: 1, misses: 0 }));
        assert!(!BlockedRule::any_hit().is_blocked(free));
        let tenth = BlockedRule::new(1.0, 0.10).unwrap();
        assert!(!tenth.is_blocked(thin));
        assert!(tenth.is_blocked(wall));
        assert!(BlockedRule::new(1.0, 0.30).unwrap().is_blocked(wall));
        let two_hits = BlockedRule::new(2.0, 0.0).unwrap();
        assert!(!two_hits.is_blocked(thin));
        let half = BlockedRule::new(0.5, 0.0).unwrap();
        assert!(half.is_blocked(Votes {
            hits: 128,
            misses: 0
        }));
        assert!(!half.is_blocked(Votes {
            hits: 127,
            misses: 0
        }));
        assert_eq!(BlockedRule::new(0.0, 0.0).unwrap(), BlockedRule::any_hit());
        assert!(BlockedRule::new(-0.1, 0.1).is_err());
        assert!(BlockedRule::new(f64::NAN, 0.1).is_err());
        assert!(BlockedRule::new(1.0, -0.1).is_err());
        assert!(BlockedRule::new(1.0, 1.5).is_err());
        assert!(BlockedRule::new(1.0, f64::NAN).is_err());
        let full = BlockedRule::new(1.0, 1.0).unwrap();
        assert!(full.is_blocked(Votes::from_votes(5, 0)));
        assert!(!full.is_blocked(Votes::from_votes(5, 1)));
        let saturated = Votes {
            hits: u32::MAX,
            misses: u32::MAX,
        };
        assert!(BlockedRule::new(1.0, 0.5).unwrap().is_blocked(saturated));
    }

    #[test]
    fn integer_rule_matches_exact_ratios() {
        for (numerator, denominator) in [(1u64, 10u64), (3, 10), (1, 4), (1, 2), (1, 20)] {
            let rule = BlockedRule::new(1.0, numerator as f64 / denominator as f64).unwrap();
            for hits in 0..300u32 {
                for misses in 0..300u32 {
                    let votes = Votes::from_votes(hits, misses);
                    let exact = hits >= 1
                        && u64::from(hits) * denominator >= numerator * u64::from(hits + misses);
                    assert_eq!(
                        rule.is_blocked(votes),
                        exact,
                        "{numerator}/{denominator} {votes:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn cells_stay_eight_bytes_and_counters_halve_before_overflow() {
        assert_eq!(size_of::<Votes>(), 8);
        let mut model = VoteModel::default();
        let mut cell = Votes {
            hits: HALVE_AT,
            misses: HALVE_AT + 1000,
        };
        model.update(&mut cell, event(RayCellRole::Free));
        assert_eq!(cell.hits, HALVE_AT / 2);
        assert_eq!(cell.misses, (HALVE_AT + 1000) / 2 + Votes::UNIT);
        let mut decaying = VoteModel::new(VoteDecay::CapHalving { cap_votes: 1 }).unwrap();
        let mut cell = Votes {
            hits: HALVE_AT - 1,
            misses: HALVE_AT - 1,
        };
        decaying.update(&mut cell, event(RayCellRole::Hit));
        decaying.update(&mut cell, event(RayCellRole::Free));
        assert!(cell.hits < UPDATED && cell.misses & UPDATED != 0);
        decaying.pass_end(&mut cell);
        assert_eq!(cell.misses & UPDATED, 0);
        let mut cell = Votes::default();
        model.update(&mut cell, event(RayCellRole::Hit));
        model.update(&mut cell, event(RayCellRole::Free));
        assert_eq!(cell, Votes::from_votes(1, 1));
    }

    #[test]
    fn decay_settings_are_validated() {
        assert!(VoteModel::new(VoteDecay::Exponential { keep: 0.0 }).is_err());
        assert!(VoteModel::new(VoteDecay::Exponential { keep: 1.0 }).is_err());
        assert!(VoteModel::new(VoteDecay::Exponential { keep: f64::NAN }).is_err());
        assert!(VoteModel::new(VoteDecay::CapHalving { cap_votes: 0 }).is_err());
        let model = VoteModel::new(VoteDecay::Exponential { keep: 0.5 }).unwrap();
        assert_eq!(model.decay(), VoteDecay::Exponential { keep: 0.5 });
        assert!(model.needs_pass_end());
        assert!(!VoteModel::default().needs_pass_end());
        assert_eq!(VoteModel::default().decay(), VoteDecay::None);
    }

    #[test]
    fn decay_applies_once_per_publish_to_updated_voxels_only() {
        let geometry = VoxelGeometry::new([0.0, 0.0, 0.0], 1.0).unwrap();
        let model = VoteModel::new(VoteDecay::Exponential { keep: 0.5 }).unwrap();
        let (mut writer, reader) =
            open_ray_scene(geometry, &RaySceneConfig::default(), model).unwrap();
        let seen = RayObservation {
            source: [0.5, 0.5, 0.5],
            hit: [3.5, 0.5, 0.5],
        };
        let elsewhere = RayObservation {
            source: [0.5, 5.5, 0.5],
            hit: [0.5, 8.5, 0.5],
        };
        writer.insert_rays(&[seen, seen]).unwrap();
        writer.insert_ray(seen).unwrap();
        writer.publish().unwrap();
        let hit = VoxelKey { x: 3, y: 0, z: 0 };
        let crossed = VoxelKey { x: 1, y: 0, z: 0 };
        assert_eq!(
            reader.get_voxel_key(hit),
            Some(Votes {
                hits: 3 * Votes::UNIT / 2,
                misses: 0
            })
        );
        assert_eq!(
            reader.get_voxel_key(crossed),
            Some(Votes {
                hits: 0,
                misses: 3 * Votes::UNIT / 2
            })
        );
        for _ in 0..4 {
            writer.insert_ray(elsewhere).unwrap();
            writer.publish().unwrap();
        }
        assert_eq!(
            reader.get_voxel_key(hit),
            Some(Votes {
                hits: 3 * Votes::UNIT / 2,
                misses: 0
            })
        );
        writer.insert_ray(seen).unwrap();
        writer.publish().unwrap();
        assert_eq!(
            reader.get_voxel_key(hit),
            Some(Votes {
                hits: (3 * Votes::UNIT / 2 + Votes::UNIT) / 2,
                misses: 0
            })
        );
        let generation = reader.generation();
        assert_eq!(writer.publish().unwrap().generation, generation);
    }

    #[test]
    fn cap_halving_only_above_the_cap() {
        let mut model = VoteModel::new(VoteDecay::CapHalving { cap_votes: 10 }).unwrap();
        let mut cell = Votes::from_votes(4, 5);
        model.update(&mut cell, event(RayCellRole::Free));
        model.pass_end(&mut cell);
        assert_eq!(cell, Votes::from_votes(4, 6));
        model.update(&mut cell, event(RayCellRole::Free));
        model.pass_end(&mut cell);
        assert_eq!(
            cell,
            Votes {
                hits: 2 * Votes::UNIT,
                misses: 7 * Votes::UNIT / 2
            }
        );
        let mut untouched = Votes::from_votes(400, 600);
        model.pass_end(&mut untouched);
        assert_eq!(untouched, Votes::from_votes(400, 600));
        let mut none = VoteModel::default();
        let mut cell = Votes::from_votes(4, 600);
        none.update(&mut cell, event(RayCellRole::Hit));
        none.pass_end(&mut cell);
        assert_eq!(cell, Votes::from_votes(5, 600));
    }

    #[test]
    fn map_is_identical_in_any_ray_order_within_a_pass() {
        let passes: Vec<Vec<RayObservation>> = (0..4)
            .map(|pass| random_rays(1_500, 0x2545_f491_4f6c_dd1d + pass))
            .collect();
        let reversed: Vec<Vec<RayObservation>> = passes
            .iter()
            .map(|pass| pass.iter().rev().copied().collect())
            .collect();
        let interleaved: Vec<Vec<RayObservation>> = passes
            .iter()
            .map(|pass| {
                let mut rays: Vec<_> = pass.iter().step_by(2).copied().collect();
                rays.extend(pass.iter().skip(1).step_by(2).copied());
                rays
            })
            .collect();
        for decay in [
            VoteDecay::None,
            VoteDecay::Exponential { keep: 0.8 },
            VoteDecay::CapHalving { cap_votes: 20 },
        ] {
            let forward = map_after(decay, &passes, 97);
            assert!(forward.iter().all(|(_, votes)| votes.misses & UPDATED == 0));
            assert!(forward.iter().any(|(_, votes)| votes.hits > 0));
            assert!(forward.iter().any(|(_, votes)| votes.misses > 0));
            assert_eq!(forward, map_after(decay, &reversed, 31), "{decay:?}");
            assert_eq!(forward, map_after(decay, &interleaved, 500), "{decay:?}");
        }
        assert_ne!(
            map_after(VoteDecay::None, &passes, 97),
            map_after(VoteDecay::Exponential { keep: 0.8 }, &passes, 97)
        );
    }
}
