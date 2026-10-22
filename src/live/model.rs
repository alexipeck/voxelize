use super::dda::RayCellContext;
use super::error::LiveError;

pub trait RayUpdateModel: Send + 'static {
    type Cell: Copy + Default + Send + Sync + 'static;

    fn update(&mut self, cell: &mut Self::Cell, event: RayCellContext);

    fn needs_metric_length(&self) -> bool {
        false
    }

    fn initial_cell(&self) -> Self::Cell {
        Self::Cell::default()
    }

    fn needs_pass_end(&self) -> bool {
        false
    }

    fn pass_end(&mut self, _cell: &mut Self::Cell) {}
}

#[derive(Clone, Copy, Debug)]
pub struct LogOddsOccupancy {
    hit_delta: f32,
    free_delta: f32,
    min_log_odds: f32,
    max_log_odds: f32,
}

impl LogOddsOccupancy {
    pub fn new(
        hit_delta: f32,
        free_delta: f32,
        min_log_odds: f32,
        max_log_odds: f32,
    ) -> Result<Self, LiveError> {
        if ![hit_delta, free_delta, min_log_odds, max_log_odds]
            .into_iter()
            .all(f32::is_finite)
            || min_log_odds > max_log_odds
        {
            return Err(LiveError::InvalidModel);
        }
        Ok(Self {
            hit_delta,
            free_delta,
            min_log_odds,
            max_log_odds,
        })
    }

    pub fn hit_delta(&self) -> f32 {
        self.hit_delta
    }

    pub fn free_delta(&self) -> f32 {
        self.free_delta
    }

    pub fn min_log_odds(&self) -> f32 {
        self.min_log_odds
    }

    pub fn max_log_odds(&self) -> f32 {
        self.max_log_odds
    }
}

impl Default for LogOddsOccupancy {
    fn default() -> Self {
        Self {
            hit_delta: 0.85,
            free_delta: -0.4,
            min_log_odds: -4.0,
            max_log_odds: 4.0,
        }
    }
}

impl RayUpdateModel for LogOddsOccupancy {
    type Cell = f32;

    fn update(&mut self, cell: &mut Self::Cell, event: RayCellContext) {
        let delta = match event.role {
            super::dda::RayCellRole::Hit => self.hit_delta,
            super::dda::RayCellRole::Free => self.free_delta,
        };
        *cell = (*cell + delta).clamp(self.min_log_odds, self.max_log_odds);
    }
}

pub fn log_odds_to_probability(log_odds: f32) -> f32 {
    if log_odds >= 0.0 {
        let denominator = 1.0 + (-log_odds).exp();
        1.0 / denominator
    } else {
        let numerator = log_odds.exp();
        numerator / (1.0 + numerator)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::live::dda::RayCellRole;
    use crate::live::geometry::VoxelKey;

    fn event(role: RayCellRole) -> RayCellContext {
        RayCellContext {
            key: VoxelKey { x: 0, y: 0, z: 0 },
            role,
            ray_length_m: None,
            t_enter: 0.0,
            t_exit: 1.0,
        }
    }

    #[test]
    fn clamps_repeated_contradictory_updates() {
        let mut model = LogOddsOccupancy::new(1.0, -0.5, -2.0, 2.0).unwrap();
        let mut cell = 0.0;
        model.update(&mut cell, event(RayCellRole::Hit));
        model.update(&mut cell, event(RayCellRole::Hit));
        model.update(&mut cell, event(RayCellRole::Hit));
        assert_eq!(cell, 2.0);
        model.update(&mut cell, event(RayCellRole::Free));
        assert_eq!(cell, 1.5);
        for _ in 0..20 {
            model.update(&mut cell, event(RayCellRole::Free));
        }
        assert_eq!(cell, -2.0);
        for _ in 0..20 {
            model.update(&mut cell, event(RayCellRole::Hit));
        }
        assert_eq!(cell, 2.0);
        assert!((log_odds_to_probability(0.0) - 0.5).abs() < 1.0e-6);
        assert!(log_odds_to_probability(20.0) > 0.99);
        assert!(log_odds_to_probability(-20.0) < 0.01);
        assert!(LogOddsOccupancy::new(0.0, 0.0, 2.0, 1.0).is_err());
    }
}
