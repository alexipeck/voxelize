//! Seeded synthetic LiDAR scans for `bench_scan`.
//!
//! The warehouse repeats along x every `PERIOD_M` metres, so the scan taken at
//! `x + k * PERIOD_M` is the scan taken at `x`, shifted by `k * PERIOD_M`. Scans are
//! cast once at start-up and replayed with that shift, which keeps ray casting out of
//! the timed ingestion path and lets a fly-through run for as long as needed.

pub const PERIOD_M: f64 = 12.0;

pub struct Lcg(u64);

impl Lcg {
    pub fn new(seed: u64) -> Self {
        Self(seed ^ 0x9e37_79b9_7f4a_7c15)
    }

    pub fn unit(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
}

#[derive(Clone, Copy)]
struct Aabb {
    min: [f64; 3],
    max: [f64; 3],
}

/// A corridor 30 m wide and 8 m high with pillars and crates, repeating along x.
pub struct Warehouse {
    half_width: f64,
    height: f64,
    boxes: Vec<Aabb>,
}

impl Warehouse {
    pub fn new(seed: u64) -> Self {
        let mut rng = Lcg::new(seed);
        let mut boxes = vec![
            Aabb {
                min: [2.7, -6.3, 0.0],
                max: [3.3, -5.7, 8.0],
            },
            Aabb {
                min: [8.7, 5.7, 0.0],
                max: [9.3, 6.3, 8.0],
            },
        ];
        for _ in 0..6 {
            let x = rng.range(0.0, PERIOD_M - 2.0);
            let y = rng.range(-13.0, 11.0);
            let size = [
                rng.range(0.5, 2.0),
                rng.range(0.5, 2.0),
                rng.range(0.4, 3.0),
            ];
            boxes.push(Aabb {
                min: [x, y, 0.0],
                max: [x + size[0], y + size[1], size[2]],
            });
        }
        Self {
            half_width: 15.0,
            height: 8.0,
            boxes,
        }
    }

    /// Adds a box to every period, for `decay_study`.
    #[allow(dead_code)]
    pub fn with_box(mut self, min: [f64; 3], max: [f64; 3]) -> Self {
        self.boxes.push(Aabb { min, max });
        self
    }

    /// The boxes in one period as (min, max) corners, for `scene_render`.
    #[allow(dead_code)]
    pub fn boxes(&self) -> Vec<([f64; 3], [f64; 3])> {
        self.boxes.iter().map(|aabb| (aabb.min, aabb.max)).collect()
    }

    /// Corridor half-width and ceiling height, for `scene_render`.
    #[allow(dead_code)]
    pub fn extent(&self) -> (f64, f64) {
        (self.half_width, self.height)
    }

    /// Distance along the unit vector `dir` to the first surface, if it is within `max_range`.
    pub fn cast(&self, origin: [f64; 3], dir: [f64; 3], max_range: f64) -> Option<f64> {
        let mut best = max_range;
        let mut found = false;
        let mut consider = |t: f64, best: &mut f64| {
            if t > 0.0 && t < *best {
                *best = t;
                found = true;
            }
        };
        if dir[2] < 0.0 {
            consider(-origin[2] / dir[2], &mut best);
        } else if dir[2] > 0.0 {
            consider((self.height - origin[2]) / dir[2], &mut best);
        }
        if dir[1] < 0.0 {
            consider((-self.half_width - origin[1]) / dir[1], &mut best);
        } else if dir[1] > 0.0 {
            consider((self.half_width - origin[1]) / dir[1], &mut best);
        }
        let reach = [origin[0] + dir[0] * best, origin[0]];
        let lo = (reach[0].min(reach[1]) / PERIOD_M).floor() as i64 - 1;
        let hi = (reach[0].max(reach[1]) / PERIOD_M).floor() as i64 + 1;
        for k in lo..=hi {
            let shift = k as f64 * PERIOD_M;
            for aabb in &self.boxes {
                if let Some(t) = slab(origin, dir, aabb, shift) {
                    consider(t, &mut best);
                }
            }
        }
        found.then_some(best)
    }
}

fn slab(origin: [f64; 3], dir: [f64; 3], aabb: &Aabb, shift: f64) -> Option<f64> {
    let mut near = f64::NEG_INFINITY;
    let mut far = f64::INFINITY;
    for axis in 0..3 {
        let offset = if axis == 0 { shift } else { 0.0 };
        let (lo, hi) = (aabb.min[axis] + offset, aabb.max[axis] + offset);
        if dir[axis] == 0.0 {
            if origin[axis] < lo || origin[axis] > hi {
                return None;
            }
            continue;
        }
        let a = (lo - origin[axis]) / dir[axis];
        let b = (hi - origin[axis]) / dir[axis];
        near = near.max(a.min(b));
        far = far.min(a.max(b));
    }
    (near <= far && near > 0.0).then_some(near)
}

/// A spinning multi-channel sensor that fires channel-major within each azimuth step.
#[derive(Clone, Copy, Debug)]
pub struct Lidar {
    pub channels: usize,
    pub azimuth_steps: usize,
    pub elevation_deg: (f64, f64),
    pub max_range_m: f64,
    pub range_noise_m: f64,
}

impl Lidar {
    /// 128 channels over ±22.5°, with enough azimuth steps to fire `points_per_second`.
    pub fn for_rate(points_per_second: f64, scan_hz: f64, max_range_m: f64) -> Self {
        let channels = 128;
        let per_scan = (points_per_second / scan_hz).max(channels as f64);
        Self {
            channels,
            azimuth_steps: (per_scan / channels as f64).round().max(1.0) as usize,
            elevation_deg: (-22.5, 22.5),
            max_range_m,
            range_noise_m: 0.02,
        }
    }

    pub fn fired_per_scan(&self) -> usize {
        self.channels * self.azimuth_steps
    }
}

/// One revolution. Returns are stored relative to the scan pose. `fired` is the firing
/// time as a fraction of the scan period, used to move the source during the scan.
#[allow(dead_code)] // decay_study reads only the offsets
pub struct Scan {
    pub pose: [f64; 3],
    pub offsets: Vec<[f32; 3]>,
    pub fired: Vec<f32>,
}

pub fn pose(index: usize, scans_per_period: usize) -> [f64; 3] {
    let phase = index as f64 / scans_per_period as f64;
    [
        phase * PERIOD_M,
        0.4 + 1.5 * (phase * std::f64::consts::TAU).sin(),
        1.5 + 0.3 * (phase * 2.0 * std::f64::consts::TAU).cos(),
    ]
}

/// Casts one scan from `pose`, moving the sensor at `velocity` (m/s) over `period_s`.
pub fn cast_scan(
    world: &Warehouse,
    lidar: &Lidar,
    pose: [f64; 3],
    velocity: [f64; 3],
    period_s: f64,
    seed: u64,
) -> Scan {
    let mut rng = Lcg::new(seed);
    let total = lidar.fired_per_scan();
    let mut offsets = Vec::with_capacity(total);
    let mut fired = Vec::with_capacity(total);
    let (low, high) = lidar.elevation_deg;
    for step in 0..lidar.azimuth_steps {
        let azimuth = step as f64 / lidar.azimuth_steps as f64 * std::f64::consts::TAU;
        for channel in 0..lidar.channels {
            let index = step * lidar.channels + channel;
            let time = index as f64 / total as f64;
            let source = [
                pose[0] + velocity[0] * time * period_s,
                pose[1] + velocity[1] * time * period_s,
                pose[2] + velocity[2] * time * period_s,
            ];
            let elevation = (low
                + (high - low) * channel as f64 / (lidar.channels.max(2) - 1) as f64)
                .to_radians();
            let dir = [
                elevation.cos() * azimuth.cos(),
                elevation.cos() * azimuth.sin(),
                elevation.sin(),
            ];
            let noise = rng.range(-lidar.range_noise_m, lidar.range_noise_m);
            let Some(range) = world.cast(source, dir, lidar.max_range_m) else {
                continue;
            };
            let range = (range + noise).max(0.05);
            offsets.push([
                (source[0] + dir[0] * range - pose[0]) as f32,
                (source[1] + dir[1] * range - pose[1]) as f32,
                (source[2] + dir[2] * range - pose[2]) as f32,
            ]);
            fired.push(time as f32);
        }
    }
    Scan {
        pose,
        offsets,
        fired,
    }
}

/// Casts one period of scans in parallel. Scan `j` is taken at `pose(j, count)`.
#[allow(dead_code)] // export_passes casts one pass at a time
pub fn cast_period(
    world: &Warehouse,
    lidar: &Lidar,
    count: usize,
    velocity: [f64; 3],
    period_s: f64,
    seed: u64,
) -> Vec<Scan> {
    let workers = std::thread::available_parallelism()
        .map(usize::from)
        .unwrap_or(1)
        .min(count.max(1));
    let mut slots: Vec<Option<Scan>> = (0..count).map(|_| None).collect();
    std::thread::scope(|scope| {
        for (worker, chunk) in slots.chunks_mut(count.div_ceil(workers)).enumerate() {
            let first = worker * count.div_ceil(workers);
            scope.spawn(move || {
                for (offset, slot) in chunk.iter_mut().enumerate() {
                    let index = first + offset;
                    *slot = Some(cast_scan(
                        world,
                        lidar,
                        pose(index, count),
                        velocity,
                        period_s,
                        seed.wrapping_add(index as u64),
                    ));
                }
            });
        }
    });
    slots.into_iter().map(|slot| slot.unwrap()).collect()
}
